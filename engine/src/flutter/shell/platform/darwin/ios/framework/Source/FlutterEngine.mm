// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "common/settings.h"
#define FML_USED_ON_EMBEDDER

#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterEngine_Internal.h"

#include <memory>

#include "flutter/common/constants.h"
#include "flutter/fml/message_loop.h"
#include "flutter/fml/platform/darwin/platform_version.h"
#include "flutter/fml/trace_event.h"
#include "flutter/runtime/ptrace_check.h"
#include "flutter/shell/common/engine.h"
#include "flutter/shell/common/platform_view.h"
#include "flutter/shell/common/shell.h"
#include "flutter/shell/common/switches.h"
#include "flutter/shell/common/thread_host.h"
#include "flutter/shell/common/variable_refresh_rate_display.h"
#import "flutter/shell/platform/darwin/common/command_line.h"
#import "flutter/shell/platform/darwin/common/framework/Source/FlutterBinaryMessengerRelay.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterDartProject_Internal.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterDartVMServicePublisher.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterIndirectScribbleDelegate.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterPlatformPlugin.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterSpellCheckPlugin.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterTextInputDelegate.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterTextureRegistryRelay.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterUndoManagerDelegate.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterUndoManagerPlugin.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterViewController_Internal.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/UIViewController+FlutterScreenAndSceneIfLoaded.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/connection_collection.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/platform_message_response_darwin.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/profiler_metrics_ios.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/vsync_waiter_ios.h"
#import "flutter/shell/platform/darwin/ios/platform_view_ios.h"
#import "flutter/shell/platform/darwin/ios/rendering_api_selection.h"
#include "flutter/shell/profiling/sampling_profiler.h"

FLUTTER_ASSERT_ARC

/// Inheriting ThreadConfigurer and use iOS platform thread API to configure the thread priorities
/// Using iOS platform thread API to configure thread priority
static void IOSPlatformThreadConfigSetter(const fml::Thread::ThreadConfig& config) {
  // set thread name
  fml::Thread::SetCurrentThreadName(config);

  // set thread priority
  switch (config.priority) {
    case fml::Thread::ThreadPriority::kBackground: {
      pthread_set_qos_class_self_np(QOS_CLASS_BACKGROUND, 0);
      [[NSThread currentThread] setThreadPriority:0];
      break;
    }
    case fml::Thread::ThreadPriority::kNormal: {
      pthread_set_qos_class_self_np(QOS_CLASS_DEFAULT, 0);
      [[NSThread currentThread] setThreadPriority:0.5];
      break;
    }
    case fml::Thread::ThreadPriority::kRaster:
    case fml::Thread::ThreadPriority::kDisplay: {
      pthread_set_qos_class_self_np(QOS_CLASS_USER_INTERACTIVE, 0);
      [[NSThread currentThread] setThreadPriority:1.0];
      sched_param param;
      int policy;
      pthread_t thread = pthread_self();
      if (!pthread_getschedparam(thread, &policy, &param)) {
        param.sched_priority = 50;
        pthread_setschedparam(thread, policy, &param);
      }
      break;
    }
  }
}

#pragma mark - Public exported constants

NSString* const FlutterDefaultDartEntrypoint = nil;
NSString* const FlutterDefaultInitialRoute = nil;

#pragma mark - Internal constants

NSString* const kFlutterKeyDataChannel = @"flutter/keydata";
static constexpr int kNumProfilerSamplesPerSec = 5;

@interface FlutterEngineRegistrar : NSObject <FlutterPluginRegistrar>
@property(nonatomic, weak) FlutterEngine* flutterEngine;
- (instancetype)initWithPlugin:(NSString*)pluginKey flutterEngine:(FlutterEngine*)flutterEngine;
@end

@interface FlutterEngine () <FlutterIndirectScribbleDelegate,
                             FlutterUndoManagerDelegate,
                             FlutterTextInputDelegate,
                             FlutterBinaryMessenger,
                             FlutterTextureRegistry>

#pragma mark - Properties

@property(nonatomic, readonly) FlutterDartProject* dartProject;
@property(nonatomic, readonly, copy) NSString* labelPrefix;
@property(nonatomic, readonly, assign) BOOL allowHeadlessExecution;
@property(nonatomic, readonly, assign) BOOL restorationEnabled;

@property(nonatomic, strong) FlutterPlatformViewsController* platformViewsController;

// Maintains a dictionary of plugin names that have registered with the engine.  Used by
// FlutterEngineRegistrar to implement a FlutterPluginRegistrar.
@property(nonatomic, readonly) NSMutableDictionary* pluginPublications;
@property(nonatomic, readonly) NSMutableDictionary<NSString*, FlutterEngineRegistrar*>* registrars;

@property(nonatomic, readwrite, copy) NSString* isolateId;
@property(nonatomic, copy) NSString* initialRoute;
@property(nonatomic, strong) id<NSObject> flutterViewControllerWillDeallocObserver;
@property(nonatomic, strong) FlutterDartVMServicePublisher* publisher;
@property(nonatomic, assign) int64_t nextTextureId;

#pragma mark - Channel properties

@property(nonatomic, strong) FlutterPlatformPlugin* platformPlugin;
@property(nonatomic, strong) FlutterTextInputPlugin* textInputPlugin;
@property(nonatomic, strong) FlutterUndoManagerPlugin* undoManagerPlugin;
@property(nonatomic, strong) FlutterSpellCheckPlugin* spellCheckPlugin;
@property(nonatomic, strong) FlutterRestorationPlugin* restorationPlugin;
@property(nonatomic, strong) FlutterMethodChannel* localizationChannel;
@property(nonatomic, strong) FlutterMethodChannel* navigationChannel;
@property(nonatomic, strong) FlutterMethodChannel* restorationChannel;
@property(nonatomic, strong) FlutterMethodChannel* platformChannel;
@property(nonatomic, strong) FlutterMethodChannel* platformViewsChannel;
@property(nonatomic, strong) FlutterMethodChannel* textInputChannel;
@property(nonatomic, strong) FlutterMethodChannel* undoManagerChannel;
@property(nonatomic, strong) FlutterMethodChannel* scribbleChannel;
@property(nonatomic, strong) FlutterMethodChannel* spellCheckChannel;
@property(nonatomic, strong) FlutterBasicMessageChannel* lifecycleChannel;
@property(nonatomic, strong) FlutterBasicMessageChannel* systemChannel;
@property(nonatomic, strong) FlutterBasicMessageChannel* settingsChannel;
@property(nonatomic, strong) FlutterBasicMessageChannel* keyEventChannel;
@property(nonatomic, strong) FlutterMethodChannel* screenshotChannel;

#pragma mark - Embedder API properties

@property(nonatomic, assign) BOOL enableEmbedderAPI;
// Function pointers for interacting with the embedder.h API.
@property(nonatomic) FlutterEngineProcTable& embedderAPI;

@end

@implementation FlutterEngine {
  std::shared_ptr<flutter::ThreadHost> _threadHost;
  std::unique_ptr<flutter::Shell> _shell;

  flutter::IOSRenderingAPI _renderingApi;
  std::shared_ptr<flutter::SamplingProfiler> _profiler;

  FlutterBinaryMessengerRelay* _binaryMessenger;
  FlutterTextureRegistryRelay* _textureRegistry;
  std::unique_ptr<flutter::ConnectionCollection> _connections;
}

- (instancetype)init {
  return [self initWithName:@"FlutterEngine" project:nil allowHeadlessExecution:YES];
}

- (instancetype)initWithName:(NSString*)labelPrefix {
  return [self initWithName:labelPrefix project:nil allowHeadlessExecution:YES];
}

- (instancetype)initWithName:(NSString*)labelPrefix project:(FlutterDartProject*)project {
  return [self initWithName:labelPrefix project:project allowHeadlessExecution:YES];
}

- (instancetype)initWithName:(NSString*)labelPrefix
                     project:(FlutterDartProject*)project
      allowHeadlessExecution:(BOOL)allowHeadlessExecution {
  return [self initWithName:labelPrefix
                     project:project
      allowHeadlessExecution:allowHeadlessExecution
          restorationEnabled:NO];
}

- (instancetype)initWithName:(NSString*)labelPrefix
                     project:(FlutterDartProject*)project
      allowHeadlessExecution:(BOOL)allowHeadlessExecution
          restorationEnabled:(BOOL)restorationEnabled {
  self = [super init];
  NSAssert(self, @"Super init cannot be nil");
  NSAssert(labelPrefix, @"labelPrefix is required");

  _restorationEnabled = restorationEnabled;
  _allowHeadlessExecution = allowHeadlessExecution;
  _labelPrefix = [labelPrefix copy];
  _dartProject = project ?: [[FlutterDartProject alloc] init];

  _enableEmbedderAPI = _dartProject.settings.enable_embedder_api;
  if (_enableEmbedderAPI) {
    NSLog(@"============== iOS: enable_embedder_api is on ==============");
    _embedderAPI.struct_size = sizeof(FlutterEngineProcTable);
    FlutterEngineGetProcAddresses(&_embedderAPI);
  }

  if (!EnableTracingIfNecessary(_dartProject.settings)) {
    NSLog(
        @"Cannot create a FlutterEngine instance in debug mode without Flutter tooling or "
        @"Xcode.\n\nTo launch in debug mode in iOS 14+, run flutter run from Flutter tools, run "
        @"from an IDE with a Flutter IDE plugin or run the iOS project from Xcode.\nAlternatively "
        @"profile and release mode apps can be launched from the home screen.");
    return nil;
  }

  _pluginPublications = [[NSMutableDictionary alloc] init];
  _registrars = [[NSMutableDictionary alloc] init];
  [self recreatePlatformViewsController];
  _binaryMessenger = [[FlutterBinaryMessengerRelay alloc] initWithParent:self];
  _textureRegistry = [[FlutterTextureRegistryRelay alloc] initWithParent:self];
  _connections.reset(new flutter::ConnectionCollection());

  NSNotificationCenter* center = [NSNotificationCenter defaultCenter];
  [center addObserver:self
             selector:@selector(onMemoryWarning:)
                 name:UIApplicationDidReceiveMemoryWarningNotification
               object:nil];

#if APPLICATION_EXTENSION_API_ONLY
  if (@available(iOS 13.0, *)) {
    [self setUpSceneLifecycleNotifications:center];
  } else {
    [self setUpApplicationLifecycleNotifications:center];
  }
#else
  [self setUpApplicationLifecycleNotifications:center];
#endif

  [center addObserver:self
             selector:@selector(onLocaleUpdated:)
                 name:NSCurrentLocaleDidChangeNotification
               object:nil];

  return self;
}

- (void)setUpSceneLifecycleNotifications:(NSNotificationCenter*)center API_AVAILABLE(ios(13.0)) {
  [center addObserver:self
             selector:@selector(sceneWillEnterForeground:)
                 name:UISceneWillEnterForegroundNotification
               object:nil];
  [center addObserver:self
             selector:@selector(sceneDidEnterBackground:)
                 name:UISceneDidEnterBackgroundNotification
               object:nil];
}

- (void)setUpApplicationLifecycleNotifications:(NSNotificationCenter*)center {
  [center addObserver:self
             selector:@selector(applicationWillEnterForeground:)
                 name:UIApplicationWillEnterForegroundNotification
               object:nil];
  [center addObserver:self
             selector:@selector(applicationDidEnterBackground:)
                 name:UIApplicationDidEnterBackgroundNotification
               object:nil];
}

- (void)recreatePlatformViewsController {
  _renderingApi = flutter::GetRenderingAPIForProcess(FlutterView.forceSoftwareRendering);
  _platformViewsController = [[FlutterPlatformViewsController alloc] init];
}

- (flutter::IOSRenderingAPI)platformViewsRenderingAPI {
  return _renderingApi;
}

- (void)dealloc {
  /// Notify plugins of dealloc.  This should happen first in dealloc since the
  /// plugins may be talking to things like the binaryMessenger.
  [_pluginPublications enumerateKeysAndObjectsUsingBlock:^(id key, id object, BOOL* stop) {
    if ([object respondsToSelector:@selector(detachFromEngineForRegistrar:)]) {
      NSObject<FlutterPluginRegistrar>* registrar = self.registrars[key];
      [object detachFromEngineForRegistrar:registrar];
    }
  }];

  // nil out weak references.
  // TODO(cbracken): https://github.com/flutter/flutter/issues/156222
  // Ensure that FlutterEngineRegistrar is using weak pointers, then eliminate this code.
  [_registrars
      enumerateKeysAndObjectsUsingBlock:^(id key, FlutterEngineRegistrar* registrar, BOOL* stop) {
        registrar.flutterEngine = nil;
      }];

  _binaryMessenger.parent = nil;
  _textureRegistry.parent = nil;

  NSNotificationCenter* center = [NSNotificationCenter defaultCenter];
  if (_flutterViewControllerWillDeallocObserver) {
    [center removeObserver:_flutterViewControllerWillDeallocObserver];
  }
  [center removeObserver:self];
}

- (flutter::Shell&)shell {
  FML_DCHECK(_shell);
  return *_shell;
}

- (void)updateViewportMetrics:(flutter::ViewportMetrics)viewportMetrics {
  if (!self.platformView) {
    return;
  }
  self.platformView->SetViewportMetrics(flutter::kFlutterImplicitViewId, viewportMetrics);
}

- (void)dispatchPointerDataPacket:(std::unique_ptr<flutter::PointerDataPacket>)packet {
  if (!self.platformView) {
    return;
  }
  self.platformView->DispatchPointerDataPacket(std::move(packet));
}

- (void)installFirstFrameCallback:(void (^)(void))block {
  if (!self.platformView) {
    return;
  }

  __weak FlutterEngine* weakSelf = self;
  self.platformView->SetNextFrameCallback([weakSelf, block] {
    FlutterEngine* strongSelf = weakSelf;
    if (!strongSelf) {
      return;
    }
    FML_DCHECK(strongSelf.platformTaskRunner);
    FML_DCHECK(strongSelf.rasterTaskRunner);
    FML_DCHECK(strongSelf.rasterTaskRunner->RunsTasksOnCurrentThread());
    // Get callback on raster thread and jump back to platform thread.
    strongSelf.platformTaskRunner->PostTask([block]() { block(); });
  });
}

- (void)enableSemantics:(BOOL)enabled withFlags:(int64_t)flags {
  if (!self.platformView) {
    return;
  }
  self.platformView->SetSemanticsEnabled(enabled);
  self.platformView->SetAccessibilityFeatures(flags);
}

- (void)notifyViewCreated {
  if (!self.platformView) {
    return;
  }
  self.platformView->NotifyCreated();
}

- (void)notifyViewDestroyed {
  if (!self.platformView) {
    return;
  }
  self.platformView->NotifyDestroyed();
}

- (flutter::PlatformViewIOS*)platformView {
  if (!_shell) {
    return nullptr;
  }
  return static_cast<flutter::PlatformViewIOS*>(_shell->GetPlatformView().get());
}

- (fml::RefPtr<fml::TaskRunner>)platformTaskRunner {
  if (!_shell) {
    return {};
  }
  return _shell->GetTaskRunners().GetPlatformTaskRunner();
}

- (fml::RefPtr<fml::TaskRunner>)uiTaskRunner {
  if (!_shell) {
    return {};
  }
  return _shell->GetTaskRunners().GetUITaskRunner();
}

- (fml::RefPtr<fml::TaskRunner>)rasterTaskRunner {
  if (!_shell) {
    return {};
  }
  return _shell->GetTaskRunners().GetRasterTaskRunner();
}

- (void)sendKeyEvent:(const FlutterKeyEvent&)event
            callback:(FlutterKeyEventCallback)callback
            userData:(void*)userData API_AVAILABLE(ios(13.4)) {
  if (@available(iOS 13.4, *)) {
  } else {
    return;
  }
  if (!self.platformView) {
    return;
  }
  const char* character = event.character;

  flutter::KeyData key_data;
  key_data.Clear();
  key_data.timestamp = (uint64_t)event.timestamp;
  switch (event.type) {
    case kFlutterKeyEventTypeUp:
      key_data.type = flutter::KeyEventType::kUp;
      break;
    case kFlutterKeyEventTypeDown:
      key_data.type = flutter::KeyEventType::kDown;
      break;
    case kFlutterKeyEventTypeRepeat:
      key_data.type = flutter::KeyEventType::kRepeat;
      break;
  }
  key_data.physical = event.physical;
  key_data.logical = event.logical;
  key_data.synthesized = event.synthesized;

  auto packet = std::make_unique<flutter::KeyDataPacket>(key_data, character);
  NSData* message = [NSData dataWithBytes:packet->data().data() length:packet->data().size()];

  auto response = ^(NSData* reply) {
    if (callback == nullptr) {
      return;
    }
    BOOL handled = FALSE;
    if (reply.length == 1 && *reinterpret_cast<const uint8_t*>(reply.bytes) == 1) {
      handled = TRUE;
    }
    callback(handled, userData);
  };

  [self sendOnChannel:kFlutterKeyDataChannel message:message binaryReply:response];
}

- (void)ensureSemanticsEnabled {
  if (!self.platformView) {
    return;
  }
  self.platformView->SetSemanticsEnabled(true);
}

- (void)setViewController:(FlutterViewController*)viewController {
  FML_DCHECK(self.platformView);
  _viewController = viewController;
  self.platformView->SetOwnerViewController(_viewController);
  [self maybeSetupPlatformViewChannels];
  [self updateDisplays];
  self.textInputPlugin.viewController = viewController;

  if (viewController) {
    __weak __block FlutterEngine* weakSelf = self;
    self.flutterViewControllerWillDeallocObserver =
        [[NSNotificationCenter defaultCenter] addObserverForName:FlutterViewControllerWillDealloc
                                                          object:viewController
                                                           queue:[NSOperationQueue mainQueue]
                                                      usingBlock:^(NSNotification* note) {
                                                        [weakSelf notifyViewControllerDeallocated];
                                                      }];
  } else {
    self.flutterViewControllerWillDeallocObserver = nil;
    [self notifyLowMemory];
  }
}

- (void)attachView {
  FML_DCHECK(self.platformView);
  self.platformView->attachView();
}

- (void)setFlutterViewControllerWillDeallocObserver:(id<NSObject>)observer {
  if (observer != _flutterViewControllerWillDeallocObserver) {
    if (_flutterViewControllerWillDeallocObserver) {
      [[NSNotificationCenter defaultCenter]
          removeObserver:_flutterViewControllerWillDeallocObserver];
    }
    _flutterViewControllerWillDeallocObserver = observer;
  }
}

- (void)notifyViewControllerDeallocated {
  [self.lifecycleChannel sendMessage:@"AppLifecycleState.detached"];
  self.textInputPlugin.viewController = nil;
  if (!self.allowHeadlessExecution) {
    [self destroyContext];
  } else if (self.platformView) {
    self.platformView->SetOwnerViewController({});
  }
  [self.textInputPlugin resetViewResponder];
  _viewController = nil;
}

- (void)destroyContext {
  [self resetChannels];
  self.isolateId = nil;
  _shell.reset();
  _profiler.reset();
  _threadHost.reset();
  _platformViewsController = nil;
}

- (NSURL*)observatoryUrl {
  return self.publisher.url;
}

- (NSURL*)vmServiceUrl {
  return self.publisher.url;
}

- (void)resetChannels {
  self.localizationChannel = nil;
  self.navigationChannel = nil;
  self.restorationChannel = nil;
  self.platformChannel = nil;
  self.platformViewsChannel = nil;
  self.textInputChannel = nil;
  self.undoManagerChannel = nil;
  self.scribbleChannel = nil;
  self.lifecycleChannel = nil;
  self.systemChannel = nil;
  self.settingsChannel = nil;
  self.keyEventChannel = nil;
  self.spellCheckChannel = nil;
}

- (void)startProfiler {
  FML_DCHECK(!_threadHost->name_prefix.empty());
  _profiler = std::make_shared<flutter::SamplingProfiler>(
      _threadHost->name_prefix.c_str(), _threadHost->profiler_thread->GetTaskRunner(),
      []() {
        flutter::ProfilerMetricsIOS profiler_metrics;
        return profiler_metrics.GenerateSample();
      },
      kNumProfilerSamplesPerSec);
  _profiler->Start();
}

// If you add a channel, be sure to also update `resetChannels`.
// Channels get a reference to the engine, and therefore need manual
// cleanup for proper collection.
- (void)setUpChannels {
  // This will be invoked once the shell is done setting up and the isolate ID
  // for the UI isolate is available.
  __weak FlutterEngine* weakSelf = self;
  [_binaryMessenger setMessageHandlerOnChannel:@"flutter/isolate"
                          binaryMessageHandler:^(NSData* message, FlutterBinaryReply reply) {
                            if (weakSelf) {
                              weakSelf.isolateId =
                                  [[FlutterStringCodec sharedInstance] decode:message];
                            }
                          }];

  self.localizationChannel =
      [[FlutterMethodChannel alloc] initWithName:@"flutter/localization"
                                 binaryMessenger:self.binaryMessenger
                                           codec:[FlutterJSONMethodCodec sharedInstance]];

  self.navigationChannel =
      [[FlutterMethodChannel alloc] initWithName:@"flutter/navigation"
                                 binaryMessenger:self.binaryMessenger
                                           codec:[FlutterJSONMethodCodec sharedInstance]];

  if ([_initialRoute length] > 0) {
    // Flutter isn't ready to receive this method call yet but the channel buffer will cache this.
    [self.navigationChannel invokeMethod:@"setInitialRoute" arguments:_initialRoute];
    _initialRoute = nil;
  }

  self.restorationChannel =
      [[FlutterMethodChannel alloc] initWithName:@"flutter/restoration"
                                 binaryMessenger:self.binaryMessenger
                                           codec:[FlutterStandardMethodCodec sharedInstance]];

  self.platformChannel =
      [[FlutterMethodChannel alloc] initWithName:@"flutter/platform"
                                 binaryMessenger:self.binaryMessenger
                                           codec:[FlutterJSONMethodCodec sharedInstance]];

  self.platformViewsChannel =
      [[FlutterMethodChannel alloc] initWithName:@"flutter/platform_views"
                                 binaryMessenger:self.binaryMessenger
                                           codec:[FlutterStandardMethodCodec sharedInstance]];

  self.textInputChannel =
      [[FlutterMethodChannel alloc] initWithName:@"flutter/textinput"
                                 binaryMessenger:self.binaryMessenger
                                           codec:[FlutterJSONMethodCodec sharedInstance]];

  self.undoManagerChannel =
      [[FlutterMethodChannel alloc] initWithName:@"flutter/undomanager"
                                 binaryMessenger:self.binaryMessenger
                                           codec:[FlutterJSONMethodCodec sharedInstance]];

  self.scribbleChannel =
      [[FlutterMethodChannel alloc] initWithName:@"flutter/scribble"
                                 binaryMessenger:self.binaryMessenger
                                           codec:[FlutterJSONMethodCodec sharedInstance]];

  self.spellCheckChannel =
      [[FlutterMethodChannel alloc] initWithName:@"flutter/spellcheck"
                                 binaryMessenger:self.binaryMessenger
                                           codec:[FlutterStandardMethodCodec sharedInstance]];

  self.lifecycleChannel =
      [[FlutterBasicMessageChannel alloc] initWithName:@"flutter/lifecycle"
                                       binaryMessenger:self.binaryMessenger
                                                 codec:[FlutterStringCodec sharedInstance]];

  self.systemChannel =
      [[FlutterBasicMessageChannel alloc] initWithName:@"flutter/system"
                                       binaryMessenger:self.binaryMessenger
                                                 codec:[FlutterJSONMessageCodec sharedInstance]];

  self.settingsChannel =
      [[FlutterBasicMessageChannel alloc] initWithName:@"flutter/settings"
                                       binaryMessenger:self.binaryMessenger
                                                 codec:[FlutterJSONMessageCodec sharedInstance]];

  self.keyEventChannel =
      [[FlutterBasicMessageChannel alloc] initWithName:@"flutter/keyevent"
                                       binaryMessenger:self.binaryMessenger
                                                 codec:[FlutterJSONMessageCodec sharedInstance]];

  self.textInputPlugin = [[FlutterTextInputPlugin alloc] initWithDelegate:self];
  self.textInputPlugin.indirectScribbleDelegate = self;
  [self.textInputPlugin setUpIndirectScribbleInteraction:self.viewController];

  self.undoManagerPlugin = [[FlutterUndoManagerPlugin alloc] initWithDelegate:self];
  self.platformPlugin = [[FlutterPlatformPlugin alloc] initWithEngine:self];

  self.restorationPlugin =
      [[FlutterRestorationPlugin alloc] initWithChannel:self.restorationChannel
                                     restorationEnabled:self.restorationEnabled];
  self.spellCheckPlugin = [[FlutterSpellCheckPlugin alloc] init];

  self.screenshotChannel =
      [[FlutterMethodChannel alloc] initWithName:@"flutter/screenshot"
                                 binaryMessenger:self.binaryMessenger
                                           codec:[FlutterStandardMethodCodec sharedInstance]];

  [self.screenshotChannel setMethodCallHandler:^(FlutterMethodCall* _Nonnull call,
                                                 FlutterResult _Nonnull result) {
    FlutterEngine* strongSelf = weakSelf;
    if (!(strongSelf && strongSelf->_shell && strongSelf->_shell->IsSetup())) {
      return result([FlutterError
          errorWithCode:@"invalid_state"
                message:@"Requesting screenshot while engine is not running."
                details:nil]);
    }
    flutter::Rasterizer::Screenshot screenshot =
        [strongSelf screenshot:flutter::Rasterizer::ScreenshotType::SurfaceData base64Encode:NO];
    if (!screenshot.data) {
      return result([FlutterError errorWithCode:@"failure"
                                        message:@"Unable to get screenshot."
                                        details:nil]);
    }
    // TODO(gaaclarke): Find way to eliminate this data copy.
    NSData* data = [NSData dataWithBytes:screenshot.data->writable_data()
                                  length:screenshot.data->size()];
    NSString* format = [NSString stringWithUTF8String:screenshot.format.c_str()];
    NSNumber* width = @(screenshot.frame_size.fWidth);
    NSNumber* height = @(screenshot.frame_size.fHeight);
    return result(@[ width, height, format ?: [NSNull null], data ]);
  }];
}

- (void)maybeSetupPlatformViewChannels {
  if (_shell && self.shell.IsSetup()) {
    __weak FlutterEngine* weakSelf = self;

    [self.platformChannel setMethodCallHandler:^(FlutterMethodCall* call, FlutterResult result) {
      [weakSelf.platformPlugin handleMethodCall:call result:result];
    }];

    [self.platformViewsChannel
        setMethodCallHandler:^(FlutterMethodCall* call, FlutterResult result) {
          if (weakSelf) {
            [weakSelf.platformViewsController onMethodCall:call result:result];
          }
        }];

    [self.textInputChannel setMethodCallHandler:^(FlutterMethodCall* call, FlutterResult result) {
      [weakSelf.textInputPlugin handleMethodCall:call result:result];
    }];

    [self.undoManagerChannel setMethodCallHandler:^(FlutterMethodCall* call, FlutterResult result) {
      [weakSelf.undoManagerPlugin handleMethodCall:call result:result];
    }];

    [self.spellCheckChannel setMethodCallHandler:^(FlutterMethodCall* call, FlutterResult result) {
      [weakSelf.spellCheckPlugin handleMethodCall:call result:result];
    }];
  }
}

- (flutter::Rasterizer::Screenshot)screenshot:(flutter::Rasterizer::ScreenshotType)type
                                 base64Encode:(bool)base64Encode {
  return self.shell.Screenshot(type, base64Encode);
}

- (void)launchEngine:(NSString*)entrypoint
          libraryURI:(NSString*)libraryOrNil
      entrypointArgs:(NSArray<NSString*>*)entrypointArgs {
  // Launch the Dart application with the inferred run configuration.
  self.shell.RunEngine([self.dartProject runConfigurationForEntrypoint:entrypoint
                                                          libraryOrNil:libraryOrNil
                                                        entrypointArgs:entrypointArgs]);
}

- (void)setUpShell:(std::unique_ptr<flutter::Shell>)shell
    withVMServicePublication:(BOOL)doesVMServicePublication {
  _shell = std::move(shell);
  [self setUpChannels];
  [self onLocaleUpdated:nil];
  [self updateDisplays];
  self.publisher = [[FlutterDartVMServicePublisher alloc]
      initWithEnableVMServicePublication:doesVMServicePublication];
  [self maybeSetupPlatformViewChannels];
  _shell->SetGpuAvailability(_isGpuDisabled ? flutter::GpuAvailability::kUnavailable
                                            : flutter::GpuAvailability::kAvailable);
}

+ (BOOL)isProfilerEnabled {
  bool profilerEnabled = false;
#if (FLUTTER_RUNTIME_MODE == FLUTTER_RUNTIME_MODE_DEBUG) || \
    (FLUTTER_RUNTIME_MODE == FLUTTER_RUNTIME_MODE_PROFILE)
  profilerEnabled = true;
#endif
  return profilerEnabled;
}

+ (NSString*)generateThreadLabel:(NSString*)labelPrefix {
  static size_t s_shellCount = 0;
  return [NSString stringWithFormat:@"%@.%zu", labelPrefix, ++s_shellCount];
}

static flutter::ThreadHost MakeThreadHost(NSString* thread_label,
                                          const flutter::Settings& settings) {
  // The current thread will be used as the platform thread. Ensure that the message loop is
  // initialized.
  fml::MessageLoop::EnsureInitializedForCurrentThread();

  uint32_t threadHostType = flutter::ThreadHost::Type::kRaster | flutter::ThreadHost::Type::kIo;
  if (!settings.enable_impeller || !settings.merged_platform_ui_thread) {
    threadHostType |= flutter::ThreadHost::Type::kUi;
  }

  if ([FlutterEngine isProfilerEnabled]) {
    threadHostType = threadHostType | flutter::ThreadHost::Type::kProfiler;
  }

  flutter::ThreadHost::ThreadHostConfig host_config(thread_label.UTF8String, threadHostType,
                                                    IOSPlatformThreadConfigSetter);

  host_config.ui_config =
      fml::Thread::ThreadConfig(flutter::ThreadHost::ThreadHostConfig::MakeThreadName(
                                    flutter::ThreadHost::Type::kUi, thread_label.UTF8String),
                                fml::Thread::ThreadPriority::kDisplay);
  host_config.raster_config =
      fml::Thread::ThreadConfig(flutter::ThreadHost::ThreadHostConfig::MakeThreadName(
                                    flutter::ThreadHost::Type::kRaster, thread_label.UTF8String),
                                fml::Thread::ThreadPriority::kRaster);

  host_config.io_config =
      fml::Thread::ThreadConfig(flutter::ThreadHost::ThreadHostConfig::MakeThreadName(
                                    flutter::ThreadHost::Type::kIo, thread_label.UTF8String),
                                fml::Thread::ThreadPriority::kNormal);

  return (flutter::ThreadHost){host_config};
}

static void SetEntryPoint(flutter::Settings* settings, NSString* entrypoint, NSString* libraryURI) {
  if (libraryURI) {
    FML_DCHECK(entrypoint) << "Must specify entrypoint if specifying library";
    settings->advisory_script_entrypoint = entrypoint.UTF8String;
    settings->advisory_script_uri = libraryURI.UTF8String;
  } else if (entrypoint) {
    settings->advisory_script_entrypoint = entrypoint.UTF8String;
    settings->advisory_script_uri = std::string("main.dart");
  } else {
    settings->advisory_script_entrypoint = std::string("main");
    settings->advisory_script_uri = std::string("main.dart");
  }
}

- (BOOL)createShell:(NSString*)entrypoint
         libraryURI:(NSString*)libraryURI
       initialRoute:(NSString*)initialRoute {
  if (_shell != nullptr) {
    FML_LOG(WARNING) << "This FlutterEngine was already invoked.";
    return NO;
  }

  self.initialRoute = initialRoute;

  auto settings = [self.dartProject settings];
  if (initialRoute != nil) {
    self.initialRoute = initialRoute;
  } else if (settings.route.empty() == false) {
    self.initialRoute = [NSString stringWithUTF8String:settings.route.c_str()];
  }

  FlutterView.forceSoftwareRendering = settings.enable_software_rendering;

  auto platformData = [self.dartProject defaultPlatformData];

  SetEntryPoint(&settings, entrypoint, libraryURI);

  NSString* threadLabel = [FlutterEngine generateThreadLabel:self.labelPrefix];
  _threadHost = std::make_shared<flutter::ThreadHost>();
  *_threadHost = MakeThreadHost(threadLabel, settings);

  __weak FlutterEngine* weakSelf = self;
  flutter::Shell::CreateCallback<flutter::PlatformView> on_create_platform_view =
      [weakSelf](flutter::Shell& shell) {
        FlutterEngine* strongSelf = weakSelf;
        if (!strongSelf) {
          return std::unique_ptr<flutter::PlatformViewIOS>();
        }
        [strongSelf recreatePlatformViewsController];
        strongSelf.platformViewsController.taskRunner =
            shell.GetTaskRunners().GetPlatformTaskRunner();
        return std::make_unique<flutter::PlatformViewIOS>(
            shell, strongSelf->_renderingApi, strongSelf.platformViewsController,
            shell.GetTaskRunners(), shell.GetConcurrentWorkerTaskRunner(),
            shell.GetIsGpuDisabledSyncSwitch());
      };

  flutter::Shell::CreateCallback<flutter::Rasterizer> on_create_rasterizer =
      [](flutter::Shell& shell) { return std::make_unique<flutter::Rasterizer>(shell); };

  fml::RefPtr<fml::TaskRunner> ui_runner;
  if (settings.enable_impeller && settings.merged_platform_ui_thread) {
    ui_runner = fml::MessageLoop::GetCurrent().GetTaskRunner();
  } else {
    ui_runner = _threadHost->ui_thread->GetTaskRunner();
  }
  flutter::TaskRunners task_runners(threadLabel.UTF8String,                          // label
                                    fml::MessageLoop::GetCurrent().GetTaskRunner(),  // platform
                                    _threadHost->raster_thread->GetTaskRunner(),     // raster
                                    ui_runner,                                       // ui
                                    _threadHost->io_thread->GetTaskRunner()          // io
  );

#if APPLICATION_EXTENSION_API_ONLY
  if (@available(iOS 13.0, *)) {
    _isGpuDisabled = self.viewController.flutterWindowSceneIfViewLoaded.activationState ==
                     UISceneActivationStateBackground;
  } else {
    // [UIApplication sharedApplication API is not available for app extension.
    // We intialize the shell assuming the GPU is required.
    _isGpuDisabled = NO;
  }
#else
  _isGpuDisabled =
      [UIApplication sharedApplication].applicationState == UIApplicationStateBackground;
#endif

  // Create the shell. This is a blocking operation.
  std::unique_ptr<flutter::Shell> shell = flutter::Shell::Create(
      /*platform_data=*/platformData,
      /*task_runners=*/task_runners,
      /*settings=*/settings,
      /*on_create_platform_view=*/on_create_platform_view,
      /*on_create_rasterizer=*/on_create_rasterizer,
      /*is_gpu_disabled=*/_isGpuDisabled);

  if (shell == nullptr) {
    FML_LOG(ERROR) << "Could not start a shell FlutterEngine with entrypoint: "
                   << entrypoint.UTF8String;
  } else {
    // TODO(vashworth): Remove once done debugging https://github.com/flutter/flutter/issues/129836
    FML_LOG(INFO) << "Enabled VM Service Publication: " << settings.enable_vm_service_publication;
    [self setUpShell:std::move(shell)
        withVMServicePublication:settings.enable_vm_service_publication];
    if ([FlutterEngine isProfilerEnabled]) {
      [self startProfiler];
    }
  }

  return _shell != nullptr;
}

- (void)updateDisplays {
  if (!_shell) {
    // Tests may do this.
    return;
  }
  auto vsync_waiter = _shell->GetVsyncWaiter().lock();
  auto vsync_waiter_ios = std::static_pointer_cast<flutter::VsyncWaiterIOS>(vsync_waiter);
  std::vector<std::unique_ptr<flutter::Display>> displays;
  auto screen_size = UIScreen.mainScreen.nativeBounds.size;
  auto scale = UIScreen.mainScreen.scale;
  displays.push_back(std::make_unique<flutter::VariableRefreshRateDisplay>(
      0, vsync_waiter_ios, screen_size.width, screen_size.height, scale));
  _shell->OnDisplayUpdates(std::move(displays));
}

- (BOOL)run {
  return [self runWithEntrypoint:FlutterDefaultDartEntrypoint
                      libraryURI:nil
                    initialRoute:FlutterDefaultInitialRoute];
}

- (BOOL)runWithEntrypoint:(NSString*)entrypoint libraryURI:(NSString*)libraryURI {
  return [self runWithEntrypoint:entrypoint
                      libraryURI:libraryURI
                    initialRoute:FlutterDefaultInitialRoute];
}

- (BOOL)runWithEntrypoint:(NSString*)entrypoint {
  return [self runWithEntrypoint:entrypoint libraryURI:nil initialRoute:FlutterDefaultInitialRoute];
}

- (BOOL)runWithEntrypoint:(NSString*)entrypoint initialRoute:(NSString*)initialRoute {
  return [self runWithEntrypoint:entrypoint libraryURI:nil initialRoute:initialRoute];
}

- (BOOL)runWithEntrypoint:(NSString*)entrypoint
               libraryURI:(NSString*)libraryURI
             initialRoute:(NSString*)initialRoute {
  return [self runWithEntrypoint:entrypoint
                      libraryURI:libraryURI
                    initialRoute:initialRoute
                  entrypointArgs:nil];
}

- (BOOL)runWithEntrypoint:(NSString*)entrypoint
               libraryURI:(NSString*)libraryURI
             initialRoute:(NSString*)initialRoute
           entrypointArgs:(NSArray<NSString*>*)entrypointArgs {
  if ([self createShell:entrypoint libraryURI:libraryURI initialRoute:initialRoute]) {
    [self launchEngine:entrypoint libraryURI:libraryURI entrypointArgs:entrypointArgs];
  }

  return _shell != nullptr;
}

- (void)notifyLowMemory {
  if (_shell) {
    _shell->NotifyLowMemoryWarning();
  }
  [self.systemChannel sendMessage:@{@"type" : @"memoryPressure"}];
}

#pragma mark - Text input delegate

- (void)flutterTextInputView:(FlutterTextInputView*)textInputView
         updateEditingClient:(int)client
                   withState:(NSDictionary*)state {
  [self.textInputChannel invokeMethod:@"TextInputClient.updateEditingState"
                            arguments:@[ @(client), state ]];
}

- (void)flutterTextInputView:(FlutterTextInputView*)textInputView
         updateEditingClient:(int)client
                   withState:(NSDictionary*)state
                     withTag:(NSString*)tag {
  [self.textInputChannel invokeMethod:@"TextInputClient.updateEditingStateWithTag"
                            arguments:@[ @(client), @{tag : state} ]];
}

- (void)flutterTextInputView:(FlutterTextInputView*)textInputView
         updateEditingClient:(int)client
                   withDelta:(NSDictionary*)delta {
  [self.textInputChannel invokeMethod:@"TextInputClient.updateEditingStateWithDeltas"
                            arguments:@[ @(client), delta ]];
}

- (void)flutterTextInputView:(FlutterTextInputView*)textInputView
        updateFloatingCursor:(FlutterFloatingCursorDragState)state
                  withClient:(int)client
                withPosition:(NSDictionary*)position {
  NSString* stateString;
  switch (state) {
    case FlutterFloatingCursorDragStateStart:
      stateString = @"FloatingCursorDragState.start";
      break;
    case FlutterFloatingCursorDragStateUpdate:
      stateString = @"FloatingCursorDragState.update";
      break;
    case FlutterFloatingCursorDragStateEnd:
      stateString = @"FloatingCursorDragState.end";
      break;
  }
  [self.textInputChannel invokeMethod:@"TextInputClient.updateFloatingCursor"
                            arguments:@[ @(client), stateString, position ]];
}

- (void)flutterTextInputView:(FlutterTextInputView*)textInputView
               performAction:(FlutterTextInputAction)action
                  withClient:(int)client {
  NSString* actionString;
  switch (action) {
    case FlutterTextInputActionUnspecified:
      // Where did the term "unspecified" come from? iOS has a "default" and Android
      // has "unspecified." These 2 terms seem to mean the same thing but we need
      // to pick just one. "unspecified" was chosen because "default" is often a
      // reserved word in languages with switch statements (dart, java, etc).
      actionString = @"TextInputAction.unspecified";
      break;
    case FlutterTextInputActionDone:
      actionString = @"TextInputAction.done";
      break;
    case FlutterTextInputActionGo:
      actionString = @"TextInputAction.go";
      break;
    case FlutterTextInputActionSend:
      actionString = @"TextInputAction.send";
      break;
    case FlutterTextInputActionSearch:
      actionString = @"TextInputAction.search";
      break;
    case FlutterTextInputActionNext:
      actionString = @"TextInputAction.next";
      break;
    case FlutterTextInputActionContinue:
      actionString = @"TextInputAction.continueAction";
      break;
    case FlutterTextInputActionJoin:
      actionString = @"TextInputAction.join";
      break;
    case FlutterTextInputActionRoute:
      actionString = @"TextInputAction.route";
      break;
    case FlutterTextInputActionEmergencyCall:
      actionString = @"TextInputAction.emergencyCall";
      break;
    case FlutterTextInputActionNewline:
      actionString = @"TextInputAction.newline";
      break;
  }
  [self.textInputChannel invokeMethod:@"TextInputClient.performAction"
                            arguments:@[ @(client), actionString ]];
}

- (void)flutterTextInputView:(FlutterTextInputView*)textInputView
    showAutocorrectionPromptRectForStart:(NSUInteger)start
                                     end:(NSUInteger)end
                              withClient:(int)client {
  [self.textInputChannel invokeMethod:@"TextInputClient.showAutocorrectionPromptRect"
                            arguments:@[ @(client), @(start), @(end) ]];
}

- (void)flutterTextInputView:(FlutterTextInputView*)textInputView
    willDismissEditMenuWithTextInputClient:(int)client {
  [self.platformChannel invokeMethod:@"ContextMenu.onDismissSystemContextMenu"
                           arguments:@[ @(client) ]];
}

#pragma mark - FlutterViewEngineDelegate

- (void)flutterTextInputView:(FlutterTextInputView*)textInputView showToolbar:(int)client {
  // TODO(justinmc): Switch from the TextInputClient to Scribble channel when
  // the framework has finished transitioning to the Scribble channel.
  // https://github.com/flutter/flutter/pull/115296
  [self.textInputChannel invokeMethod:@"TextInputClient.showToolbar" arguments:@[ @(client) ]];
}

- (void)flutterTextInputPlugin:(FlutterTextInputPlugin*)textInputPlugin
                  focusElement:(UIScribbleElementIdentifier)elementIdentifier
                       atPoint:(CGPoint)referencePoint
                        result:(FlutterResult)callback {
  // TODO(justinmc): Switch from the TextInputClient to Scribble channel when
  // the framework has finished transitioning to the Scribble channel.
  // https://github.com/flutter/flutter/pull/115296
  [self.textInputChannel
      invokeMethod:@"TextInputClient.focusElement"
         arguments:@[ elementIdentifier, @(referencePoint.x), @(referencePoint.y) ]
            result:callback];
}

- (void)flutterTextInputPlugin:(FlutterTextInputPlugin*)textInputPlugin
         requestElementsInRect:(CGRect)rect
                        result:(FlutterResult)callback {
  // TODO(justinmc): Switch from the TextInputClient to Scribble channel when
  // the framework has finished transitioning to the Scribble channel.
  // https://github.com/flutter/flutter/pull/115296
  [self.textInputChannel
      invokeMethod:@"TextInputClient.requestElementsInRect"
         arguments:@[ @(rect.origin.x), @(rect.origin.y), @(rect.size.width), @(rect.size.height) ]
            result:callback];
}

- (void)flutterTextInputViewScribbleInteractionBegan:(FlutterTextInputView*)textInputView {
  // TODO(justinmc): Switch from the TextInputClient to Scribble channel when
  // the framework has finished transitioning to the Scribble channel.
  // https://github.com/flutter/flutter/pull/115296
  [self.textInputChannel invokeMethod:@"TextInputClient.scribbleInteractionBegan" arguments:nil];
}

- (void)flutterTextInputViewScribbleInteractionFinished:(FlutterTextInputView*)textInputView {
  // TODO(justinmc): Switch from the TextInputClient to Scribble channel when
  // the framework has finished transitioning to the Scribble channel.
  // https://github.com/flutter/flutter/pull/115296
  [self.textInputChannel invokeMethod:@"TextInputClient.scribbleInteractionFinished" arguments:nil];
}

- (void)flutterTextInputView:(FlutterTextInputView*)textInputView
    insertTextPlaceholderWithSize:(CGSize)size
                       withClient:(int)client {
  // TODO(justinmc): Switch from the TextInputClient to Scribble channel when
  // the framework has finished transitioning to the Scribble channel.
  // https://github.com/flutter/flutter/pull/115296
  [self.textInputChannel invokeMethod:@"TextInputClient.insertTextPlaceholder"
                            arguments:@[ @(client), @(size.width), @(size.height) ]];
}

- (void)flutterTextInputView:(FlutterTextInputView*)textInputView
       removeTextPlaceholder:(int)client {
  // TODO(justinmc): Switch from the TextInputClient to Scribble channel when
  // the framework has finished transitioning to the Scribble channel.
  // https://github.com/flutter/flutter/pull/115296
  [self.textInputChannel invokeMethod:@"TextInputClient.removeTextPlaceholder"
                            arguments:@[ @(client) ]];
}

- (void)flutterTextInputView:(FlutterTextInputView*)textInputView
    didResignFirstResponderWithTextInputClient:(int)client {
  // When flutter text input view resign first responder, send a message to
  // framework to ensure the focus state is correct. This is useful when close
  // keyboard from platform side.
  [self.textInputChannel invokeMethod:@"TextInputClient.onConnectionClosed"
                            arguments:@[ @(client) ]];

  // Platform view's first responder detection logic:
  //
  // All text input widgets (e.g. EditableText) are backed by a dummy UITextInput view
  // in the TextInputPlugin. When this dummy UITextInput view resigns first responder,
  // check if any platform view becomes first responder. If any platform view becomes
  // first responder, send a "viewFocused" channel message to inform the framework to un-focus
  // the previously focused text input.
  //
  // Caveat:
  // 1. This detection logic does not cover the scenario when a platform view becomes
  // first responder without any flutter text input resigning its first responder status
  // (e.g. user tapping on platform view first). For now it works fine because the TextInputPlugin
  // does not track the focused platform view id (which is different from Android implementation).
  //
  // 2. This detection logic assumes that all text input widgets are backed by a dummy
  // UITextInput view in the TextInputPlugin, which may not hold true in the future.

  // Have to check in the next run loop, because iOS requests the previous first responder to
  // resign before requesting the next view to become first responder.
  dispatch_async(dispatch_get_main_queue(), ^(void) {
    long platform_view_id = [self.platformViewsController firstResponderPlatformViewId];
    if (platform_view_id == -1) {
      return;
    }

    [self.platformViewsChannel invokeMethod:@"viewFocused" arguments:@(platform_view_id)];
  });
}

#pragma mark - Undo Manager Delegate

- (void)handleUndoWithDirection:(FlutterUndoRedoDirection)direction {
  NSString* action = (direction == FlutterUndoRedoDirectionUndo) ? @"undo" : @"redo";
  [self.undoManagerChannel invokeMethod:@"UndoManagerClient.handleUndo" arguments:@[ action ]];
}

- (UIView<UITextInput>*)activeTextInputView {
  return [[self textInputPlugin] textInputView];
}

- (NSUndoManager*)undoManager {
  return self.viewController.undoManager;
}

#pragma mark - Screenshot Delegate

- (flutter::Rasterizer::Screenshot)takeScreenshot:(flutter::Rasterizer::ScreenshotType)type
                                  asBase64Encoded:(BOOL)base64Encode {
  FML_DCHECK(_shell) << "Cannot takeScreenshot without a shell";
  return _shell->Screenshot(type, base64Encode);
}

- (void)flutterViewAccessibilityDidCall {
  if (self.viewController.view.accessibilityElements == nil) {
    [self ensureSemanticsEnabled];
  }
}

- (NSObject<FlutterBinaryMessenger>*)binaryMessenger {
  return _binaryMessenger;
}

- (NSObject<FlutterTextureRegistry>*)textureRegistry {
  return _textureRegistry;
}

// For test only. Ideally we should create a dependency injector for all dependencies and
// remove this.
- (void)setBinaryMessenger:(FlutterBinaryMessengerRelay*)binaryMessenger {
  // Discard the previous messenger and keep the new one.
  if (binaryMessenger != _binaryMessenger) {
    _binaryMessenger.parent = nil;
    _binaryMessenger = binaryMessenger;
  }
}

#pragma mark - FlutterBinaryMessenger

- (void)sendOnChannel:(NSString*)channel message:(NSData*)message {
  [self sendOnChannel:channel message:message binaryReply:nil];
}

- (void)sendOnChannel:(NSString*)channel
              message:(NSData*)message
          binaryReply:(FlutterBinaryReply)callback {
  NSParameterAssert(channel);
  NSAssert(_shell && _shell->IsSetup(),
           @"Sending a message before the FlutterEngine has been run.");
  fml::RefPtr<flutter::PlatformMessageResponseDarwin> response =
      (callback == nil) ? nullptr
                        : fml::MakeRefCounted<flutter::PlatformMessageResponseDarwin>(
                              ^(NSData* reply) {
                                callback(reply);
                              },
                              _shell->GetTaskRunners().GetPlatformTaskRunner());
  std::unique_ptr<flutter::PlatformMessage> platformMessage =
      (message == nil) ? std::make_unique<flutter::PlatformMessage>(channel.UTF8String, response)
                       : std::make_unique<flutter::PlatformMessage>(
                             channel.UTF8String, flutter::CopyNSDataToMapping(message), response);

  _shell->GetPlatformView()->DispatchPlatformMessage(std::move(platformMessage));
  // platformMessage takes ownership of response.
  // NOLINTNEXTLINE(clang-analyzer-cplusplus.NewDeleteLeaks)
}

- (NSObject<FlutterTaskQueue>*)makeBackgroundTaskQueue {
  return flutter::PlatformMessageHandlerIos::MakeBackgroundTaskQueue();
}

- (FlutterBinaryMessengerConnection)setMessageHandlerOnChannel:(NSString*)channel
                                          binaryMessageHandler:
                                              (FlutterBinaryMessageHandler)handler {
  return [self setMessageHandlerOnChannel:channel binaryMessageHandler:handler taskQueue:nil];
}

- (FlutterBinaryMessengerConnection)
    setMessageHandlerOnChannel:(NSString*)channel
          binaryMessageHandler:(FlutterBinaryMessageHandler)handler
                     taskQueue:(NSObject<FlutterTaskQueue>* _Nullable)taskQueue {
  NSParameterAssert(channel);
  if (_shell && _shell->IsSetup()) {
    self.platformView->GetPlatformMessageHandlerIos()->SetMessageHandler(channel.UTF8String,
                                                                         handler, taskQueue);
    return _connections->AquireConnection(channel.UTF8String);
  } else {
    NSAssert(!handler, @"Setting a message handler before the FlutterEngine has been run.");
    // Setting a handler to nil for a channel that has not yet been set up is a no-op.
    return flutter::ConnectionCollection::MakeErrorConnection(-1);
  }
}

- (void)cleanUpConnection:(FlutterBinaryMessengerConnection)connection {
  if (_shell && _shell->IsSetup()) {
    std::string channel = _connections->CleanupConnection(connection);
    if (!channel.empty()) {
      self.platformView->GetPlatformMessageHandlerIos()->SetMessageHandler(channel.c_str(), nil,
                                                                           nil);
    }
  }
}

#pragma mark - FlutterTextureRegistry

- (int64_t)registerTexture:(NSObject<FlutterTexture>*)texture {
  FML_DCHECK(self.platformView);
  int64_t textureId = self.nextTextureId++;
  self.platformView->RegisterExternalTexture(textureId, texture);
  return textureId;
}

- (void)unregisterTexture:(int64_t)textureId {
  _shell->GetPlatformView()->UnregisterTexture(textureId);
}

- (void)textureFrameAvailable:(int64_t)textureId {
  _shell->GetPlatformView()->MarkTextureFrameAvailable(textureId);
}

- (NSString*)lookupKeyForAsset:(NSString*)asset {
  return [FlutterDartProject lookupKeyForAsset:asset];
}

- (NSString*)lookupKeyForAsset:(NSString*)asset fromPackage:(NSString*)package {
  return [FlutterDartProject lookupKeyForAsset:asset fromPackage:package];
}

- (id<FlutterPluginRegistry>)pluginRegistry {
  return self;
}

#pragma mark - FlutterPluginRegistry

- (NSObject<FlutterPluginRegistrar>*)registrarForPlugin:(NSString*)pluginKey {
  NSAssert(self.pluginPublications[pluginKey] == nil, @"Duplicate plugin key: %@", pluginKey);
  self.pluginPublications[pluginKey] = [NSNull null];
  FlutterEngineRegistrar* result = [[FlutterEngineRegistrar alloc] initWithPlugin:pluginKey
                                                                    flutterEngine:self];
  self.registrars[pluginKey] = result;
  return result;
}

- (BOOL)hasPlugin:(NSString*)pluginKey {
  return _pluginPublications[pluginKey] != nil;
}

- (NSObject*)valuePublishedByPlugin:(NSString*)pluginKey {
  return _pluginPublications[pluginKey];
}

#pragma mark - Notifications

#if APPLICATION_EXTENSION_API_ONLY
- (void)sceneWillEnterForeground:(NSNotification*)notification API_AVAILABLE(ios(13.0)) {
  [self flutterWillEnterForeground:notification];
}

- (void)sceneDidEnterBackground:(NSNotification*)notification API_AVAILABLE(ios(13.0)) {
  [self flutterDidEnterBackground:notification];
}
#else
- (void)applicationWillEnterForeground:(NSNotification*)notification {
  [self flutterWillEnterForeground:notification];
}

- (void)applicationDidEnterBackground:(NSNotification*)notification {
  [self flutterDidEnterBackground:notification];
}
#endif

- (void)flutterWillEnterForeground:(NSNotification*)notification {
  [self setIsGpuDisabled:NO];
}

- (void)flutterDidEnterBackground:(NSNotification*)notification {
  [self setIsGpuDisabled:YES];
  [self notifyLowMemory];
}

- (void)onMemoryWarning:(NSNotification*)notification {
  [self notifyLowMemory];
}

- (void)setIsGpuDisabled:(BOOL)value {
  if (_shell) {
    _shell->SetGpuAvailability(value ? flutter::GpuAvailability::kUnavailable
                                     : flutter::GpuAvailability::kAvailable);
  }
  _isGpuDisabled = value;
}

#pragma mark - Locale updates

- (void)onLocaleUpdated:(NSNotification*)notification {
  // Get and pass the user's preferred locale list to dart:ui.
  NSMutableArray<NSString*>* localeData = [[NSMutableArray alloc] init];
  NSArray<NSString*>* preferredLocales = [NSLocale preferredLanguages];
  for (NSString* localeID in preferredLocales) {
    NSLocale* locale = [[NSLocale alloc] initWithLocaleIdentifier:localeID];
    NSString* languageCode = [locale objectForKey:NSLocaleLanguageCode];
    NSString* countryCode = [locale objectForKey:NSLocaleCountryCode];
    NSString* scriptCode = [locale objectForKey:NSLocaleScriptCode];
    NSString* variantCode = [locale objectForKey:NSLocaleVariantCode];
    if (!languageCode) {
      continue;
    }
    [localeData addObject:languageCode];
    [localeData addObject:(countryCode ? countryCode : @"")];
    [localeData addObject:(scriptCode ? scriptCode : @"")];
    [localeData addObject:(variantCode ? variantCode : @"")];
  }
  if (localeData.count == 0) {
    return;
  }
  [self.localizationChannel invokeMethod:@"setLocale" arguments:localeData];
}

- (void)waitForFirstFrameSync:(NSTimeInterval)timeout
                     callback:(NS_NOESCAPE void (^_Nonnull)(BOOL didTimeout))callback {
  fml::TimeDelta waitTime = fml::TimeDelta::FromMilliseconds(timeout * 1000);
  fml::Status status = self.shell.WaitForFirstFrame(waitTime);
  callback(status.code() == fml::StatusCode::kDeadlineExceeded);
}

- (void)waitForFirstFrame:(NSTimeInterval)timeout
                 callback:(void (^_Nonnull)(BOOL didTimeout))callback {
  dispatch_queue_t queue = dispatch_get_global_queue(QOS_CLASS_BACKGROUND, 0);
  dispatch_group_t group = dispatch_group_create();

  __weak FlutterEngine* weakSelf = self;
  __block BOOL didTimeout = NO;
  dispatch_group_async(group, queue, ^{
    FlutterEngine* strongSelf = weakSelf;
    if (!strongSelf) {
      return;
    }

    fml::TimeDelta waitTime = fml::TimeDelta::FromMilliseconds(timeout * 1000);
    fml::Status status = strongSelf.shell.WaitForFirstFrame(waitTime);
    didTimeout = status.code() == fml::StatusCode::kDeadlineExceeded;
  });

  // Only execute the main queue task once the background task has completely finished executing.
  dispatch_group_notify(group, dispatch_get_main_queue(), ^{
    // Strongly capture self on the task dispatched to the main thread.
    //
    // When we capture weakSelf strongly in the above block on a background thread, we risk the
    // possibility that all other strong references to FlutterEngine go out of scope while the block
    // executes and that the engine is dealloc'ed at the end of the above block on a background
    // thread. FlutterEngine is not safe to release on any thread other than the main thread.
    //
    // self is never nil here since it's a strong reference that's verified non-nil above, but we
    // use a conditional check to avoid an unused expression compiler warning.
    FlutterEngine* strongSelf = self;
    if (!strongSelf) {
      return;
    }
    callback(didTimeout);
  });
}

- (FlutterEngine*)spawnWithEntrypoint:(/*nullable*/ NSString*)entrypoint
                           libraryURI:(/*nullable*/ NSString*)libraryURI
                         initialRoute:(/*nullable*/ NSString*)initialRoute
                       entrypointArgs:(/*nullable*/ NSArray<NSString*>*)entrypointArgs {
  NSAssert(_shell, @"Spawning from an engine without a shell (possibly not run).");
  FlutterEngine* result = [[FlutterEngine alloc] initWithName:self.labelPrefix
                                                      project:self.dartProject
                                       allowHeadlessExecution:self.allowHeadlessExecution];
  flutter::RunConfiguration configuration =
      [self.dartProject runConfigurationForEntrypoint:entrypoint
                                         libraryOrNil:libraryURI
                                       entrypointArgs:entrypointArgs];

  fml::WeakPtr<flutter::PlatformView> platform_view = _shell->GetPlatformView();
  FML_DCHECK(platform_view);
  // Static-cast safe since this class always creates PlatformViewIOS instances.
  flutter::PlatformViewIOS* ios_platform_view =
      static_cast<flutter::PlatformViewIOS*>(platform_view.get());
  std::shared_ptr<flutter::IOSContext> context = ios_platform_view->GetIosContext();
  FML_DCHECK(context);

  // Lambda captures by pointers to ObjC objects are fine here because the
  // create call is synchronous.
  flutter::Shell::CreateCallback<flutter::PlatformView> on_create_platform_view =
      [result, context](flutter::Shell& shell) {
        [result recreatePlatformViewsController];
        result.platformViewsController.taskRunner = shell.GetTaskRunners().GetPlatformTaskRunner();
        return std::make_unique<flutter::PlatformViewIOS>(
            shell, context, result.platformViewsController, shell.GetTaskRunners());
      };

  flutter::Shell::CreateCallback<flutter::Rasterizer> on_create_rasterizer =
      [](flutter::Shell& shell) { return std::make_unique<flutter::Rasterizer>(shell); };

  std::string cppInitialRoute;
  if (initialRoute) {
    cppInitialRoute = [initialRoute UTF8String];
  }

  std::unique_ptr<flutter::Shell> shell = _shell->Spawn(
      std::move(configuration), cppInitialRoute, on_create_platform_view, on_create_rasterizer);

  result->_threadHost = _threadHost;
  result->_profiler = _profiler;
  result->_isGpuDisabled = _isGpuDisabled;
  [result setUpShell:std::move(shell) withVMServicePublication:NO];
  return result;
}

- (const flutter::ThreadHost&)threadHost {
  return *_threadHost;
}

- (FlutterDartProject*)project {
  return self.dartProject;
}

- (BOOL)isUsingImpeller {
  return self.project.isImpellerEnabled;
}

@end

@implementation FlutterEngineRegistrar {
  NSString* _pluginKey;
}

- (instancetype)initWithPlugin:(NSString*)pluginKey flutterEngine:(FlutterEngine*)flutterEngine {
  self = [super init];
  NSAssert(self, @"Super init cannot be nil");
  _pluginKey = [pluginKey copy];
  _flutterEngine = flutterEngine;
  return self;
}

- (NSObject<FlutterBinaryMessenger>*)messenger {
  return _flutterEngine.binaryMessenger;
}

- (NSObject<FlutterTextureRegistry>*)textures {
  return _flutterEngine.textureRegistry;
}

- (void)publish:(NSObject*)value {
  _flutterEngine.pluginPublications[_pluginKey] = value;
}

- (void)addMethodCallDelegate:(NSObject<FlutterPlugin>*)delegate
                      channel:(FlutterMethodChannel*)channel {
  [channel setMethodCallHandler:^(FlutterMethodCall* call, FlutterResult result) {
    [delegate handleMethodCall:call result:result];
  }];
}

- (void)addApplicationDelegate:(NSObject<FlutterPlugin>*)delegate
    NS_EXTENSION_UNAVAILABLE_IOS("Disallowed in plugins used in app extensions") {
  id<UIApplicationDelegate> appDelegate = [[UIApplication sharedApplication] delegate];
  if ([appDelegate conformsToProtocol:@protocol(FlutterAppLifeCycleProvider)]) {
    id<FlutterAppLifeCycleProvider> lifeCycleProvider =
        (id<FlutterAppLifeCycleProvider>)appDelegate;
    [lifeCycleProvider addApplicationLifeCycleDelegate:delegate];
  }
}

- (NSString*)lookupKeyForAsset:(NSString*)asset {
  return [_flutterEngine lookupKeyForAsset:asset];
}

- (NSString*)lookupKeyForAsset:(NSString*)asset fromPackage:(NSString*)package {
  return [_flutterEngine lookupKeyForAsset:asset fromPackage:package];
}

- (void)registerViewFactory:(NSObject<FlutterPlatformViewFactory>*)factory
                     withId:(NSString*)factoryId {
  [self registerViewFactory:factory
                                withId:factoryId
      gestureRecognizersBlockingPolicy:FlutterPlatformViewGestureRecognizersBlockingPolicyEager];
}

- (void)registerViewFactory:(NSObject<FlutterPlatformViewFactory>*)factory
                              withId:(NSString*)factoryId
    gestureRecognizersBlockingPolicy:
        (FlutterPlatformViewGestureRecognizersBlockingPolicy)gestureRecognizersBlockingPolicy {
  [_flutterEngine.platformViewsController registerViewFactory:factory
                                                       withId:factoryId
                             gestureRecognizersBlockingPolicy:gestureRecognizersBlockingPolicy];
}

@end
