// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/macos/framework/Headers/FlutterEngine.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterEngine_Internal.h"

#include <algorithm>
#include <iostream>
#include <vector>

#include "flutter/common/constants.h"
#include "flutter/shell/platform/common/app_lifecycle_state.h"
#include "flutter/shell/platform/common/engine_switches.h"
#include "flutter/shell/platform/embedder/embedder.h"

#import "flutter/shell/platform/darwin/common/framework/Source/FlutterBinaryMessengerRelay.h"
#import "flutter/shell/platform/darwin/macos/framework/Headers/FlutterAppDelegate.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterAppDelegate_Internal.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterCompositor.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterDartProject_Internal.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterDisplayLink.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterMenuPlugin.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterMouseCursorPlugin.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterPlatformViewController.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterRenderer.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterTimeConverter.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterVSyncWaiter.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterViewController_Internal.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterViewEngineProvider.h"

@class FlutterEngineRegistrar;

NSString* const kFlutterPlatformChannel = @"flutter/platform";
NSString* const kFlutterSettingsChannel = @"flutter/settings";
NSString* const kFlutterLifecycleChannel = @"flutter/lifecycle";

using flutter::kFlutterImplicitViewId;

/**
 * Constructs and returns a FlutterLocale struct corresponding to |locale|, which must outlive
 * the returned struct.
 */
static FlutterLocale FlutterLocaleFromNSLocale(NSLocale* locale) {
  FlutterLocale flutterLocale = {};
  flutterLocale.struct_size = sizeof(FlutterLocale);
  flutterLocale.language_code = [[locale objectForKey:NSLocaleLanguageCode] UTF8String];
  flutterLocale.country_code = [[locale objectForKey:NSLocaleCountryCode] UTF8String];
  flutterLocale.script_code = [[locale objectForKey:NSLocaleScriptCode] UTF8String];
  flutterLocale.variant_code = [[locale objectForKey:NSLocaleVariantCode] UTF8String];
  return flutterLocale;
}

/// The private notification for voice over.
static NSString* const kEnhancedUserInterfaceNotification =
    @"NSApplicationDidChangeAccessibilityEnhancedUserInterfaceNotification";
static NSString* const kEnhancedUserInterfaceKey = @"AXEnhancedUserInterface";

/// Clipboard plain text format.
constexpr char kTextPlainFormat[] = "text/plain";

#pragma mark -

// Records an active handler of the messenger (FlutterEngine) that listens to
// platform messages on a given channel.
@interface FlutterEngineHandlerInfo : NSObject

- (instancetype)initWithConnection:(NSNumber*)connection
                           handler:(FlutterBinaryMessageHandler)handler;

@property(nonatomic, readonly) FlutterBinaryMessageHandler handler;
@property(nonatomic, readonly) NSNumber* connection;

@end

@implementation FlutterEngineHandlerInfo
- (instancetype)initWithConnection:(NSNumber*)connection
                           handler:(FlutterBinaryMessageHandler)handler {
  self = [super init];
  NSAssert(self, @"Super init cannot be nil");
  _connection = connection;
  _handler = handler;
  return self;
}
@end

#pragma mark -

/**
 * Private interface declaration for FlutterEngine.
 */
@interface FlutterEngine () <FlutterBinaryMessenger, FlutterMouseCursorPluginDelegate>

/**
 * A mutable array that holds one bool value that determines if responses to platform messages are
 * clear to execute. This value should be read or written only inside of a synchronized block and
 * will return `NO` after the FlutterEngine has been dealloc'd.
 */
@property(nonatomic, strong) NSMutableArray<NSNumber*>* isResponseValid;

/**
 * All delegates added via plugin calls to addApplicationDelegate.
 */
@property(nonatomic, strong) NSPointerArray* pluginAppDelegates;

/**
 * All registrars returned from registrarForPlugin:
 */
@property(nonatomic, readonly)
    NSMutableDictionary<NSString*, FlutterEngineRegistrar*>* pluginRegistrars;

- (nullable FlutterViewController*)viewControllerForIdentifier:
    (FlutterViewIdentifier)viewIdentifier;

/**
 * An internal method that adds the view controller with the given ID.
 *
 * This method assigns the controller with the ID, puts the controller into the
 * map, and does assertions related to the implicit view ID.
 */
- (void)registerViewController:(FlutterViewController*)controller
                 forIdentifier:(FlutterViewIdentifier)viewIdentifier;

/**
 * An internal method that removes the view controller with the given ID.
 *
 * This method clears the ID of the controller, removes the controller from the
 * map. This is an no-op if the view ID is not associated with any view
 * controllers.
 */
- (void)deregisterViewControllerForIdentifier:(FlutterViewIdentifier)viewIdentifier;

/**
 * Shuts down the engine if view requirement is not met, and headless execution
 * is not allowed.
 */
- (void)shutDownIfNeeded;

/**
 * Sends the list of user-preferred locales to the Flutter engine.
 */
- (void)sendUserLocales;

/**
 * Handles a platform message from the engine.
 */
- (void)engineCallbackOnPlatformMessage:(const FlutterPlatformMessage*)message;

/**
 * Invoked right before the engine is restarted.
 *
 * This should reset states to as if the application has just started.  It
 * usually indicates a hot restart (Shift-R in Flutter CLI.)
 */
- (void)engineCallbackOnPreEngineRestart;

/**
 * Requests that the task be posted back the to the Flutter engine at the target time. The target
 * time is in the clock used by the Flutter engine.
 */
- (void)postMainThreadTask:(FlutterTask)task targetTimeInNanoseconds:(uint64_t)targetTime;

/**
 * Loads the AOT snapshots and instructions from the elf bundle (app_elf_snapshot.so) into _aotData,
 * if it is present in the assets directory.
 */
- (void)loadAOTData:(NSString*)assetsDir;

/**
 * Creates a platform view channel and sets up the method handler.
 */
- (void)setUpPlatformViewChannel;

/**
 * Creates an accessibility channel and sets up the message handler.
 */
- (void)setUpAccessibilityChannel;

/**
 * Handles messages received from the Flutter engine on the _*Channel channels.
 */
- (void)handleMethodCall:(FlutterMethodCall*)call result:(FlutterResult)result;

@end

#pragma mark -

@implementation FlutterEngineTerminationHandler {
  __weak FlutterEngine* _engine;
  FlutterTerminationCallback _terminator;
}

- (instancetype)initWithEngine:(FlutterEngine*)engine
                    terminator:(FlutterTerminationCallback)terminator {
  self = [super init];
  _acceptingRequests = NO;
  _engine = engine;
  _terminator = terminator ? terminator : ^(id sender) {
    // Default to actually terminating the application. The terminator exists to
    // allow tests to override it so that an actual exit doesn't occur.
    [[NSApplication sharedApplication] terminate:sender];
  };
  id<NSApplicationDelegate> appDelegate = [[NSApplication sharedApplication] delegate];
  if ([appDelegate respondsToSelector:@selector(setTerminationHandler:)]) {
    FlutterAppDelegate* flutterAppDelegate = reinterpret_cast<FlutterAppDelegate*>(appDelegate);
    flutterAppDelegate.terminationHandler = self;
  }
  return self;
}

// This is called by the method call handler in the engine when the application
// requests termination itself.
- (void)handleRequestAppExitMethodCall:(NSDictionary<NSString*, id>*)arguments
                                result:(FlutterResult)result {
  NSString* type = arguments[@"type"];
  // Ignore the "exitCode" value in the arguments because AppKit doesn't have
  // any good way to set the process exit code other than calling exit(), and
  // that bypasses all of the native applicationShouldExit shutdown events,
  // etc., which we don't want to skip.

  FlutterAppExitType exitType =
      [type isEqualTo:@"cancelable"] ? kFlutterAppExitTypeCancelable : kFlutterAppExitTypeRequired;

  [self requestApplicationTermination:[NSApplication sharedApplication]
                             exitType:exitType
                               result:result];
}

// This is called by the FlutterAppDelegate whenever any termination request is
// received.
- (void)requestApplicationTermination:(id)sender
                             exitType:(FlutterAppExitType)type
                               result:(nullable FlutterResult)result {
  _shouldTerminate = YES;
  if (![self acceptingRequests]) {
    // Until the Dart application has signaled that it is ready to handle
    // termination requests, the app will just terminate when asked.
    type = kFlutterAppExitTypeRequired;
  }
  switch (type) {
    case kFlutterAppExitTypeCancelable: {
      FlutterJSONMethodCodec* codec = [FlutterJSONMethodCodec sharedInstance];
      FlutterMethodCall* methodCall =
          [FlutterMethodCall methodCallWithMethodName:@"System.requestAppExit" arguments:nil];
      [_engine sendOnChannel:kFlutterPlatformChannel
                     message:[codec encodeMethodCall:methodCall]
                 binaryReply:^(NSData* _Nullable reply) {
                   NSAssert(_terminator, @"terminator shouldn't be nil");
                   id decoded_reply = [codec decodeEnvelope:reply];
                   if ([decoded_reply isKindOfClass:[FlutterError class]]) {
                     FlutterError* error = (FlutterError*)decoded_reply;
                     NSLog(@"Method call returned error[%@]: %@ %@", [error code], [error message],
                           [error details]);
                     _terminator(sender);
                     return;
                   }
                   if (![decoded_reply isKindOfClass:[NSDictionary class]]) {
                     NSLog(@"Call to System.requestAppExit returned an unexpected object: %@",
                           decoded_reply);
                     _terminator(sender);
                     return;
                   }
                   NSDictionary* replyArgs = (NSDictionary*)decoded_reply;
                   if ([replyArgs[@"response"] isEqual:@"exit"]) {
                     _terminator(sender);
                   } else if ([replyArgs[@"response"] isEqual:@"cancel"]) {
                     _shouldTerminate = NO;
                   }
                   if (result != nil) {
                     result(replyArgs);
                   }
                 }];
      break;
    }
    case kFlutterAppExitTypeRequired:
      NSAssert(_terminator, @"terminator shouldn't be nil");
      _terminator(sender);
      break;
  }
}

@end

#pragma mark -

@implementation FlutterPasteboard

- (NSInteger)clearContents {
  return [[NSPasteboard generalPasteboard] clearContents];
}

- (NSString*)stringForType:(NSPasteboardType)dataType {
  return [[NSPasteboard generalPasteboard] stringForType:dataType];
}

- (BOOL)setString:(nonnull NSString*)string forType:(nonnull NSPasteboardType)dataType {
  return [[NSPasteboard generalPasteboard] setString:string forType:dataType];
}

@end

#pragma mark -

/**
 * `FlutterPluginRegistrar` implementation handling a single plugin.
 */
@interface FlutterEngineRegistrar : NSObject <FlutterPluginRegistrar>
- (instancetype)initWithPlugin:(nonnull NSString*)pluginKey
                 flutterEngine:(nonnull FlutterEngine*)flutterEngine;

- (nullable NSView*)viewForIdentifier:(FlutterViewIdentifier)viewIdentifier;

/**
 * The value published by this plugin, or NSNull if nothing has been published.
 *
 * The unusual NSNull is for the documented behavior of valuePublishedByPlugin:.
 */
@property(nonatomic, readonly, nonnull) NSObject* publishedValue;
@end

@implementation FlutterEngineRegistrar {
  NSString* _pluginKey;
  __weak FlutterEngine* _flutterEngine;
}

@dynamic view;

- (instancetype)initWithPlugin:(NSString*)pluginKey flutterEngine:(FlutterEngine*)flutterEngine {
  self = [super init];
  if (self) {
    _pluginKey = [pluginKey copy];
    _flutterEngine = flutterEngine;
    _publishedValue = [NSNull null];
  }
  return self;
}

#pragma mark - FlutterPluginRegistrar

- (id<FlutterBinaryMessenger>)messenger {
  return _flutterEngine.binaryMessenger;
}

- (id<FlutterTextureRegistry>)textures {
  return _flutterEngine.renderer;
}

- (NSView*)view {
  return [self viewForIdentifier:kFlutterImplicitViewId];
}

- (NSView*)viewForIdentifier:(FlutterViewIdentifier)viewIdentifier {
  FlutterViewController* controller = [_flutterEngine viewControllerForIdentifier:viewIdentifier];
  if (controller == nil) {
    return nil;
  }
  if (!controller.viewLoaded) {
    [controller loadView];
  }
  return controller.flutterView;
}

- (void)addMethodCallDelegate:(nonnull id<FlutterPlugin>)delegate
                      channel:(nonnull FlutterMethodChannel*)channel {
  [channel setMethodCallHandler:^(FlutterMethodCall* call, FlutterResult result) {
    [delegate handleMethodCall:call result:result];
  }];
}

- (void)addApplicationDelegate:(NSObject<FlutterAppLifecycleDelegate>*)delegate {
  id<NSApplicationDelegate> appDelegate = [[NSApplication sharedApplication] delegate];
  if ([appDelegate conformsToProtocol:@protocol(FlutterAppLifecycleProvider)]) {
    id<FlutterAppLifecycleProvider> lifeCycleProvider =
        static_cast<id<FlutterAppLifecycleProvider>>(appDelegate);
    [lifeCycleProvider addApplicationLifecycleDelegate:delegate];
    [_flutterEngine.pluginAppDelegates addPointer:(__bridge void*)delegate];
  }
}

- (void)registerViewFactory:(nonnull NSObject<FlutterPlatformViewFactory>*)factory
                     withId:(nonnull NSString*)factoryId {
  [[_flutterEngine platformViewController] registerViewFactory:factory withId:factoryId];
}

- (void)publish:(NSObject*)value {
  _publishedValue = value;
}

- (NSString*)lookupKeyForAsset:(NSString*)asset {
  return [FlutterDartProject lookupKeyForAsset:asset];
}

- (NSString*)lookupKeyForAsset:(NSString*)asset fromPackage:(NSString*)package {
  return [FlutterDartProject lookupKeyForAsset:asset fromPackage:package];
}

@end

// Callbacks provided to the engine. See the called methods for documentation.
#pragma mark - Static methods provided to engine configuration

static void OnPlatformMessage(const FlutterPlatformMessage* message, void* user_data) {
  FlutterEngine* engine = (__bridge FlutterEngine*)user_data;
  [engine engineCallbackOnPlatformMessage:message];
}

#pragma mark -

@implementation FlutterEngine {
  // The embedding-API-level engine object.
  FLUTTER_API_SYMBOL(FlutterEngine) _engine;

  // The project being run by this engine.
  FlutterDartProject* _project;

  // A mapping of channel names to the registered information for those channels.
  NSMutableDictionary<NSString*, FlutterEngineHandlerInfo*>* _messengerHandlers;

  // A self-incremental integer to assign to newly assigned channels as
  // identification.
  FlutterBinaryMessengerConnection _currentMessengerConnection;

  // Whether the engine can continue running after the view controller is removed.
  BOOL _allowHeadlessExecution;

  // Pointer to the Dart AOT snapshot and instruction data.
  _FlutterEngineAOTData* _aotData;

  // _macOSCompositor is created when the engine is created and its destruction is handled by ARC
  // when the engine is destroyed.
  std::unique_ptr<flutter::FlutterCompositor> _macOSCompositor;

  // The information of all views attached to this engine mapped from IDs.
  //
  // It can't use NSDictionary, because the values need to be weak references.
  NSMapTable* _viewControllers;

  // FlutterCompositor is copied and used in embedder.cc.
  FlutterCompositor _compositor;

  // Method channel for platform view functions. These functions include creating, disposing and
  // mutating a platform view.
  FlutterMethodChannel* _platformViewsChannel;

  // Used to support creation and deletion of platform views and registering platform view
  // factories. Lifecycle is tied to the engine.
  FlutterPlatformViewController* _platformViewController;

  // A message channel for sending user settings to the flutter engine.
  FlutterBasicMessageChannel* _settingsChannel;

  // A message channel for accessibility.
  FlutterBasicMessageChannel* _accessibilityChannel;

  // A method channel for miscellaneous platform functionality.
  FlutterMethodChannel* _platformChannel;

  FlutterThreadSynchronizer* _threadSynchronizer;

  // Whether the application is currently the active application.
  BOOL _active;

  // Whether any portion of the application is currently visible.
  BOOL _visible;

  // Proxy to allow plugins, channels to hold a weak reference to the binary messenger (self).
  FlutterBinaryMessengerRelay* _binaryMessenger;

  // Map from ViewId to vsync waiter. Note that this is modified on main thread
  // but accessed on UI thread, so access must be @synchronized.
  NSMapTable<NSNumber*, FlutterVSyncWaiter*>* _vsyncWaiters;

  // Weak reference to last view that received a pointer event. This is used to
  // pair cursor change with a view.
  __weak FlutterView* _lastViewWithPointerEvent;
}

- (instancetype)initWithName:(NSString*)labelPrefix project:(FlutterDartProject*)project {
  return [self initWithName:labelPrefix project:project allowHeadlessExecution:YES];
}

static const int kMainThreadPriority = 47;

static void SetThreadPriority(FlutterThreadPriority priority) {
  if (priority == kDisplay || priority == kRaster) {
    pthread_t thread = pthread_self();
    sched_param param;
    int policy;
    if (!pthread_getschedparam(thread, &policy, &param)) {
      param.sched_priority = kMainThreadPriority;
      pthread_setschedparam(thread, policy, &param);
    }
    pthread_set_qos_class_self_np(QOS_CLASS_USER_INTERACTIVE, 0);
  }
}

- (instancetype)initWithName:(NSString*)labelPrefix
                     project:(FlutterDartProject*)project
      allowHeadlessExecution:(BOOL)allowHeadlessExecution {
  self = [super init];
  NSAssert(self, @"Super init cannot be nil");
  _pasteboard = [[FlutterPasteboard alloc] init];
  _active = NO;
  _visible = NO;
  _project = project ?: [[FlutterDartProject alloc] init];
  _messengerHandlers = [[NSMutableDictionary alloc] init];
  _binaryMessenger = [[FlutterBinaryMessengerRelay alloc] initWithParent:self];
  _pluginAppDelegates = [NSPointerArray weakObjectsPointerArray];
  _pluginRegistrars = [[NSMutableDictionary alloc] init];
  _currentMessengerConnection = 1;
  _allowHeadlessExecution = allowHeadlessExecution;
  _semanticsEnabled = NO;
  _binaryMessenger = [[FlutterBinaryMessengerRelay alloc] initWithParent:self];
  _isResponseValid = [[NSMutableArray alloc] initWithCapacity:1];
  [_isResponseValid addObject:@YES];

  _embedderAPI.struct_size = sizeof(FlutterEngineProcTable);
  FlutterEngineGetProcAddresses(&_embedderAPI);

  _viewControllers = [NSMapTable weakToWeakObjectsMapTable];
  _renderer = [[FlutterRenderer alloc] initWithFlutterEngine:self];

  NSNotificationCenter* notificationCenter = [NSNotificationCenter defaultCenter];
  [notificationCenter addObserver:self
                         selector:@selector(sendUserLocales)
                             name:NSCurrentLocaleDidChangeNotification
                           object:nil];

  _platformViewController = [[FlutterPlatformViewController alloc] init];
  _threadSynchronizer = [[FlutterThreadSynchronizer alloc] init];
  // The macOS compositor must be initialized in the initializer because it is
  // used when adding views, which might happen before runWithEntrypoint.
  _macOSCompositor = std::make_unique<flutter::FlutterCompositor>(
      [[FlutterViewEngineProvider alloc] initWithEngine:self],
      [[FlutterTimeConverter alloc] initWithEngine:self], _platformViewController);

  [self setUpPlatformViewChannel];
  [self setUpAccessibilityChannel];
  [self setUpNotificationCenterListeners];
  id<NSApplicationDelegate> appDelegate = [[NSApplication sharedApplication] delegate];
  if ([appDelegate conformsToProtocol:@protocol(FlutterAppLifecycleProvider)]) {
    _terminationHandler = [[FlutterEngineTerminationHandler alloc] initWithEngine:self
                                                                       terminator:nil];
    id<FlutterAppLifecycleProvider> lifecycleProvider =
        static_cast<id<FlutterAppLifecycleProvider>>(appDelegate);
    [lifecycleProvider addApplicationLifecycleDelegate:self];
  } else {
    _terminationHandler = nil;
  }

  _vsyncWaiters = [NSMapTable strongToStrongObjectsMapTable];

  return self;
}

- (void)dealloc {
  id<NSApplicationDelegate> appDelegate = [[NSApplication sharedApplication] delegate];
  if ([appDelegate conformsToProtocol:@protocol(FlutterAppLifecycleProvider)]) {
    id<FlutterAppLifecycleProvider> lifecycleProvider =
        static_cast<id<FlutterAppLifecycleProvider>>(appDelegate);
    [lifecycleProvider removeApplicationLifecycleDelegate:self];

    // Unregister any plugins that registered as app delegates, since they are not guaranteed to
    // live after the engine is destroyed, and their delegation registration is intended to be bound
    // to the engine and its lifetime.
    for (id<FlutterAppLifecycleDelegate> delegate in _pluginAppDelegates) {
      if (delegate) {
        [lifecycleProvider removeApplicationLifecycleDelegate:delegate];
      }
    }
  }
  // Clear any published values, just in case a plugin has created a retain cycle with the
  // registrar.
  for (NSString* pluginName in _pluginRegistrars) {
    [_pluginRegistrars[pluginName] publish:[NSNull null]];
  }
  @synchronized(_isResponseValid) {
    [_isResponseValid removeAllObjects];
    [_isResponseValid addObject:@NO];
  }
  [self shutDownEngine];
  if (_aotData) {
    _embedderAPI.CollectAOTData(_aotData);
  }
}

- (BOOL)runWithEntrypoint:(NSString*)entrypoint {
  if (self.running) {
    return NO;
  }

  if (!_allowHeadlessExecution && [_viewControllers count] == 0) {
    NSLog(@"Attempted to run an engine with no view controller without headless mode enabled.");
    return NO;
  }

  [self addInternalPlugins];

  // The first argument of argv is required to be the executable name.
  std::vector<const char*> argv = {[self.executableName UTF8String]};
  std::vector<std::string> switches = self.switches;

  // Enable Impeller only if specifically asked for from the project or cmdline arguments.
  if (_project.enableImpeller ||
      std::find(switches.begin(), switches.end(), "--enable-impeller=true") != switches.end()) {
    switches.push_back("--enable-impeller=true");
  }

  std::transform(switches.begin(), switches.end(), std::back_inserter(argv),
                 [](const std::string& arg) -> const char* { return arg.c_str(); });

  std::vector<const char*> dartEntrypointArgs;
  for (NSString* argument in [_project dartEntrypointArguments]) {
    dartEntrypointArgs.push_back([argument UTF8String]);
  }

  FlutterProjectArgs flutterArguments = {};
  flutterArguments.struct_size = sizeof(FlutterProjectArgs);
  flutterArguments.assets_path = _project.assetsPath.UTF8String;
  flutterArguments.icu_data_path = _project.ICUDataPath.UTF8String;
  flutterArguments.command_line_argc = static_cast<int>(argv.size());
  flutterArguments.command_line_argv = argv.empty() ? nullptr : argv.data();
  flutterArguments.platform_message_callback = (FlutterPlatformMessageCallback)OnPlatformMessage;
  flutterArguments.update_semantics_callback2 = [](const FlutterSemanticsUpdate2* update,
                                                   void* user_data) {
    // TODO(dkwingsmt): This callback only supports single-view, therefore it
    // only operates on the implicit view. To support multi-view, we need a
    // way to pass in the ID (probably through FlutterSemanticsUpdate).
    FlutterEngine* engine = (__bridge FlutterEngine*)user_data;
    [[engine viewControllerForIdentifier:kFlutterImplicitViewId] updateSemantics:update];
  };
  flutterArguments.custom_dart_entrypoint = entrypoint.UTF8String;
  flutterArguments.shutdown_dart_vm_when_done = true;
  flutterArguments.dart_entrypoint_argc = dartEntrypointArgs.size();
  flutterArguments.dart_entrypoint_argv = dartEntrypointArgs.data();
  flutterArguments.root_isolate_create_callback = _project.rootIsolateCreateCallback;
  flutterArguments.log_message_callback = [](const char* tag, const char* message,
                                             void* user_data) {
    if (tag && tag[0]) {
      std::cout << tag << ": ";
    }
    std::cout << message << std::endl;
  };

  static size_t sTaskRunnerIdentifiers = 0;
  const FlutterTaskRunnerDescription cocoa_task_runner_description = {
      .struct_size = sizeof(FlutterTaskRunnerDescription),
      // Retain for use in post_task_callback. Released in destruction_callback.
      .user_data = (__bridge_retained void*)self,
      .runs_task_on_current_thread_callback = [](void* user_data) -> bool {
        return [[NSThread currentThread] isMainThread];
      },
      .post_task_callback = [](FlutterTask task, uint64_t target_time_nanos,
                               void* user_data) -> void {
        FlutterEngine* engine = (__bridge FlutterEngine*)user_data;
        [engine postMainThreadTask:task targetTimeInNanoseconds:target_time_nanos];
      },
      .identifier = ++sTaskRunnerIdentifiers,
      .destruction_callback =
          [](void* user_data) {
            // Balancing release for the retain when setting user_data above.
            FlutterEngine* engine = (__bridge_transfer FlutterEngine*)user_data;
            engine = nil;
          },
  };
  const FlutterCustomTaskRunners custom_task_runners = {
      .struct_size = sizeof(FlutterCustomTaskRunners),
      .platform_task_runner = &cocoa_task_runner_description,
      .thread_priority_setter = SetThreadPriority};
  flutterArguments.custom_task_runners = &custom_task_runners;

  [self loadAOTData:_project.assetsPath];
  if (_aotData) {
    flutterArguments.aot_data = _aotData;
  }

  flutterArguments.compositor = [self createFlutterCompositor];

  flutterArguments.on_pre_engine_restart_callback = [](void* user_data) {
    FlutterEngine* engine = (__bridge FlutterEngine*)user_data;
    [engine engineCallbackOnPreEngineRestart];
  };

  flutterArguments.vsync_callback = [](void* user_data, intptr_t baton) {
    FlutterEngine* engine = (__bridge FlutterEngine*)user_data;
    [engine onVSync:baton];
  };

  FlutterRendererConfig rendererConfig = [_renderer createRendererConfig];
  FlutterEngineResult result = _embedderAPI.Initialize(
      FLUTTER_ENGINE_VERSION, &rendererConfig, &flutterArguments, (__bridge void*)(self), &_engine);
  if (result != kSuccess) {
    NSLog(@"Failed to initialize Flutter engine: error %d", result);
    return NO;
  }

  result = _embedderAPI.RunInitialized(_engine);
  if (result != kSuccess) {
    NSLog(@"Failed to run an initialized engine: error %d", result);
    return NO;
  }

  [self sendUserLocales];

  // Update window metric for all view controllers.
  NSEnumerator* viewControllerEnumerator = [_viewControllers objectEnumerator];
  FlutterViewController* nextViewController;
  while ((nextViewController = [viewControllerEnumerator nextObject])) {
    [self updateWindowMetricsForViewController:nextViewController];
  }

  [self updateDisplayConfig];
  // Send the initial user settings such as brightness and text scale factor
  // to the engine.
  [self sendInitialSettings];
  return YES;
}

- (void)loadAOTData:(NSString*)assetsDir {
  if (!_embedderAPI.RunsAOTCompiledDartCode()) {
    return;
  }

  BOOL isDirOut = false;  // required for NSFileManager fileExistsAtPath.
  NSFileManager* fileManager = [NSFileManager defaultManager];

  // This is the location where the test fixture places the snapshot file.
  // For applications built by Flutter tool, this is in "App.framework".
  NSString* elfPath = [NSString pathWithComponents:@[ assetsDir, @"app_elf_snapshot.so" ]];

  if (![fileManager fileExistsAtPath:elfPath isDirectory:&isDirOut]) {
    return;
  }

  FlutterEngineAOTDataSource source = {};
  source.type = kFlutterEngineAOTDataSourceTypeElfPath;
  source.elf_path = [elfPath cStringUsingEncoding:NSUTF8StringEncoding];

  auto result = _embedderAPI.CreateAOTData(&source, &_aotData);
  if (result != kSuccess) {
    NSLog(@"Failed to load AOT data from: %@", elfPath);
  }
}

- (void)registerViewController:(FlutterViewController*)controller
                 forIdentifier:(FlutterViewIdentifier)viewIdentifier {
  _macOSCompositor->AddView(viewIdentifier);
  NSAssert(controller != nil, @"The controller must not be nil.");
  NSAssert(controller.engine == nil,
           @"The FlutterViewController is unexpectedly attached to "
           @"engine %@ before initialization.",
           controller.engine);
  NSAssert([_viewControllers objectForKey:@(viewIdentifier)] == nil,
           @"The requested view ID is occupied.");
  [_viewControllers setObject:controller forKey:@(viewIdentifier)];
  [controller setUpWithEngine:self
               viewIdentifier:viewIdentifier
           threadSynchronizer:_threadSynchronizer];
  NSAssert(controller.viewIdentifier == viewIdentifier, @"Failed to assign view ID.");
  // Verify that the controller's property are updated accordingly. Failing the
  // assertions is likely because either the FlutterViewController or the
  // FlutterEngine is mocked. Please subclass these classes instead.
  NSAssert(controller.attached, @"The FlutterViewController should switch to the attached mode "
                                @"after it is added to a FlutterEngine.");
  NSAssert(controller.engine == self,
           @"The FlutterViewController was added to %@, but its engine unexpectedly became %@.",
           self, controller.engine);

  if (controller.viewLoaded) {
    [self viewControllerViewDidLoad:controller];
  }
}

- (void)viewControllerViewDidLoad:(FlutterViewController*)viewController {
  __weak FlutterEngine* weakSelf = self;
  FlutterTimeConverter* timeConverter = [[FlutterTimeConverter alloc] initWithEngine:self];
  FlutterVSyncWaiter* waiter = [[FlutterVSyncWaiter alloc]
      initWithDisplayLink:[FlutterDisplayLink displayLinkWithView:viewController.view]
                    block:^(CFTimeInterval timestamp, CFTimeInterval targetTimestamp,
                            uintptr_t baton) {
                      uint64_t timeNanos = [timeConverter CAMediaTimeToEngineTime:timestamp];
                      uint64_t targetTimeNanos =
                          [timeConverter CAMediaTimeToEngineTime:targetTimestamp];
                      FlutterEngine* engine = weakSelf;
                      if (engine) {
                        // It is a bit unfortunate that embedder requires OnVSync call on
                        // platform thread just to immediately redispatch it to UI thread.
                        // We are already on UI thread right now, but have to do the
                        // extra hop to main thread.
                        [engine->_threadSynchronizer performOnPlatformThread:^{
                          engine->_embedderAPI.OnVsync(_engine, baton, timeNanos, targetTimeNanos);
                        }];
                      }
                    }];
  FML_DCHECK([_vsyncWaiters objectForKey:@(viewController.viewIdentifier)] == nil);
  @synchronized(_vsyncWaiters) {
    [_vsyncWaiters setObject:waiter forKey:@(viewController.viewIdentifier)];
  }
}

- (void)deregisterViewControllerForIdentifier:(FlutterViewIdentifier)viewIdentifier {
  _macOSCompositor->RemoveView(viewIdentifier);
  FlutterViewController* controller = [self viewControllerForIdentifier:viewIdentifier];
  // The controller can be nil. The engine stores only a weak ref, and this
  // method could have been called from the controller's dealloc.
  if (controller != nil) {
    [controller detachFromEngine];
    NSAssert(!controller.attached,
             @"The FlutterViewController unexpectedly stays attached after being removed. "
             @"In unit tests, this is likely because either the FlutterViewController or "
             @"the FlutterEngine is mocked. Please subclass these classes instead.");
  }
  [_viewControllers removeObjectForKey:@(viewIdentifier)];
  @synchronized(_vsyncWaiters) {
    [_vsyncWaiters removeObjectForKey:@(viewIdentifier)];
  }
}

- (void)shutDownIfNeeded {
  if ([_viewControllers count] == 0 && !_allowHeadlessExecution) {
    [self shutDownEngine];
  }
}

- (FlutterViewController*)viewControllerForIdentifier:(FlutterViewIdentifier)viewIdentifier {
  FlutterViewController* controller = [_viewControllers objectForKey:@(viewIdentifier)];
  NSAssert(controller == nil || controller.viewIdentifier == viewIdentifier,
           @"The stored controller has unexpected view ID.");
  return controller;
}

- (void)setViewController:(FlutterViewController*)controller {
  FlutterViewController* currentController =
      [_viewControllers objectForKey:@(kFlutterImplicitViewId)];
  if (currentController == controller) {
    // From nil to nil, or from non-nil to the same controller.
    return;
  }
  if (currentController == nil && controller != nil) {
    // From nil to non-nil.
    NSAssert(controller.engine == nil,
             @"Failed to set view controller to the engine: "
             @"The given FlutterViewController is already attached to an engine %@. "
             @"If you wanted to create an FlutterViewController and set it to an existing engine, "
             @"you should use FlutterViewController#init(engine:, nibName, bundle:) instead.",
             controller.engine);
    [self registerViewController:controller forIdentifier:kFlutterImplicitViewId];
  } else if (currentController != nil && controller == nil) {
    NSAssert(currentController.viewIdentifier == kFlutterImplicitViewId,
             @"The default controller has an unexpected ID %llu", currentController.viewIdentifier);
    // From non-nil to nil.
    [self deregisterViewControllerForIdentifier:kFlutterImplicitViewId];
    [self shutDownIfNeeded];
  } else {
    // From non-nil to a different non-nil view controller.
    NSAssert(NO,
             @"Failed to set view controller to the engine: "
             @"The engine already has an implicit view controller %@. "
             @"If you wanted to make the implicit view render in a different window, "
             @"you should attach the current view controller to the window instead.",
             [_viewControllers objectForKey:@(kFlutterImplicitViewId)]);
  }
}

- (FlutterViewController*)viewController {
  return [self viewControllerForIdentifier:kFlutterImplicitViewId];
}

- (FlutterCompositor*)createFlutterCompositor {
  _compositor = {};
  _compositor.struct_size = sizeof(FlutterCompositor);
  _compositor.user_data = _macOSCompositor.get();

  _compositor.create_backing_store_callback = [](const FlutterBackingStoreConfig* config,  //
                                                 FlutterBackingStore* backing_store_out,   //
                                                 void* user_data                           //
                                              ) {
    return reinterpret_cast<flutter::FlutterCompositor*>(user_data)->CreateBackingStore(
        config, backing_store_out);
  };

  _compositor.collect_backing_store_callback = [](const FlutterBackingStore* backing_store,  //
                                                  void* user_data                            //
                                               ) { return true; };

  _compositor.present_view_callback = [](const FlutterPresentViewInfo* info) {
    return reinterpret_cast<flutter::FlutterCompositor*>(info->user_data)
        ->Present(info->view_id, info->layers, info->layers_count);
  };

  _compositor.avoid_backing_store_cache = true;

  return &_compositor;
}

- (id<FlutterBinaryMessenger>)binaryMessenger {
  return _binaryMessenger;
}

#pragma mark - Framework-internal methods

- (void)addViewController:(FlutterViewController*)controller {
  // FlutterEngine can only handle the implicit view for now. Adding more views
  // throws an assertion.
  NSAssert(self.viewController == nil,
           @"The engine already has a view controller for the implicit view.");
  self.viewController = controller;
}

- (void)removeViewController:(nonnull FlutterViewController*)viewController {
  [self deregisterViewControllerForIdentifier:viewController.viewIdentifier];
  [self shutDownIfNeeded];
}

- (BOOL)running {
  return _engine != nullptr;
}

- (void)updateDisplayConfig:(NSNotification*)notification {
  [self updateDisplayConfig];
}

- (NSArray<NSScreen*>*)screens {
  return [NSScreen screens];
}

- (void)updateDisplayConfig {
  if (!_engine) {
    return;
  }

  std::vector<FlutterEngineDisplay> displays;
  for (NSScreen* screen : [self screens]) {
    CGDirectDisplayID displayID =
        static_cast<CGDirectDisplayID>([screen.deviceDescription[@"NSScreenNumber"] integerValue]);

    double devicePixelRatio = screen.backingScaleFactor;
    FlutterEngineDisplay display;
    display.struct_size = sizeof(display);
    display.display_id = displayID;
    display.single_display = false;
    display.width = static_cast<size_t>(screen.frame.size.width) * devicePixelRatio;
    display.height = static_cast<size_t>(screen.frame.size.height) * devicePixelRatio;
    display.device_pixel_ratio = devicePixelRatio;

    CVDisplayLinkRef displayLinkRef = nil;
    CVReturn error = CVDisplayLinkCreateWithCGDisplay(displayID, &displayLinkRef);

    if (error == 0) {
      CVTime nominal = CVDisplayLinkGetNominalOutputVideoRefreshPeriod(displayLinkRef);
      if (!(nominal.flags & kCVTimeIsIndefinite)) {
        double refreshRate = static_cast<double>(nominal.timeScale) / nominal.timeValue;
        display.refresh_rate = round(refreshRate);
      }
      CVDisplayLinkRelease(displayLinkRef);
    } else {
      display.refresh_rate = 0;
    }

    displays.push_back(display);
  }
  _embedderAPI.NotifyDisplayUpdate(_engine, kFlutterEngineDisplaysUpdateTypeStartup,
                                   displays.data(), displays.size());
}

- (void)onSettingsChanged:(NSNotification*)notification {
  // TODO(jonahwilliams): https://github.com/flutter/flutter/issues/32015.
  NSString* brightness =
      [[NSUserDefaults standardUserDefaults] stringForKey:@"AppleInterfaceStyle"];
  [_settingsChannel sendMessage:@{
    @"platformBrightness" : [brightness isEqualToString:@"Dark"] ? @"dark" : @"light",
    // TODO(jonahwilliams): https://github.com/flutter/flutter/issues/32006.
    @"textScaleFactor" : @1.0,
    @"alwaysUse24HourFormat" : @([FlutterHourFormat isAlwaysUse24HourFormat]),
  }];
}

- (void)sendInitialSettings {
  // TODO(jonahwilliams): https://github.com/flutter/flutter/issues/32015.
  [[NSDistributedNotificationCenter defaultCenter]
      addObserver:self
         selector:@selector(onSettingsChanged:)
             name:@"AppleInterfaceThemeChangedNotification"
           object:nil];
  [self onSettingsChanged:nil];
}

- (FlutterEngineProcTable&)embedderAPI {
  return _embedderAPI;
}

- (nonnull NSString*)executableName {
  return [[[NSProcessInfo processInfo] arguments] firstObject] ?: @"Flutter";
}

- (void)updateWindowMetricsForViewController:(FlutterViewController*)viewController {
  if (!_engine || !viewController || !viewController.viewLoaded) {
    return;
  }
  NSAssert([self viewControllerForIdentifier:viewController.viewIdentifier] == viewController,
           @"The provided view controller is not attached to this engine.");
  NSView* view = viewController.flutterView;
  CGRect scaledBounds = [view convertRectToBacking:view.bounds];
  CGSize scaledSize = scaledBounds.size;
  double pixelRatio = view.bounds.size.width == 0 ? 1 : scaledSize.width / view.bounds.size.width;
  auto displayId = [view.window.screen.deviceDescription[@"NSScreenNumber"] integerValue];
  const FlutterWindowMetricsEvent windowMetricsEvent = {
      .struct_size = sizeof(windowMetricsEvent),
      .width = static_cast<size_t>(scaledSize.width),
      .height = static_cast<size_t>(scaledSize.height),
      .pixel_ratio = pixelRatio,
      .left = static_cast<size_t>(scaledBounds.origin.x),
      .top = static_cast<size_t>(scaledBounds.origin.y),
      .display_id = static_cast<uint64_t>(displayId),
      .view_id = viewController.viewIdentifier,
  };
  _embedderAPI.SendWindowMetricsEvent(_engine, &windowMetricsEvent);
}

- (void)sendPointerEvent:(const FlutterPointerEvent&)event {
  _embedderAPI.SendPointerEvent(_engine, &event, 1);
  _lastViewWithPointerEvent = [self viewControllerForIdentifier:kFlutterImplicitViewId].flutterView;
}

- (void)sendKeyEvent:(const FlutterKeyEvent&)event
            callback:(FlutterKeyEventCallback)callback
            userData:(void*)userData {
  _embedderAPI.SendKeyEvent(_engine, &event, callback, userData);
}

- (void)setSemanticsEnabled:(BOOL)enabled {
  if (_semanticsEnabled == enabled) {
    return;
  }
  _semanticsEnabled = enabled;

  // Update all view controllers' bridges.
  NSEnumerator* viewControllerEnumerator = [_viewControllers objectEnumerator];
  FlutterViewController* nextViewController;
  while ((nextViewController = [viewControllerEnumerator nextObject])) {
    [nextViewController notifySemanticsEnabledChanged];
  }

  _embedderAPI.UpdateSemanticsEnabled(_engine, _semanticsEnabled);
}

- (void)dispatchSemanticsAction:(FlutterSemanticsAction)action
                       toTarget:(uint16_t)target
                       withData:(fml::MallocMapping)data {
  _embedderAPI.DispatchSemanticsAction(_engine, target, action, data.GetMapping(), data.GetSize());
}

- (FlutterPlatformViewController*)platformViewController {
  return _platformViewController;
}

#pragma mark - Private methods

- (void)sendUserLocales {
  if (!self.running) {
    return;
  }

  // Create a list of FlutterLocales corresponding to the preferred languages.
  NSMutableArray<NSLocale*>* locales = [NSMutableArray array];
  std::vector<FlutterLocale> flutterLocales;
  flutterLocales.reserve(locales.count);
  for (NSString* localeID in [NSLocale preferredLanguages]) {
    NSLocale* locale = [[NSLocale alloc] initWithLocaleIdentifier:localeID];
    [locales addObject:locale];
    flutterLocales.push_back(FlutterLocaleFromNSLocale(locale));
  }
  // Convert to a list of pointers, and send to the engine.
  std::vector<const FlutterLocale*> flutterLocaleList;
  flutterLocaleList.reserve(flutterLocales.size());
  std::transform(flutterLocales.begin(), flutterLocales.end(),
                 std::back_inserter(flutterLocaleList),
                 [](const auto& arg) -> const auto* { return &arg; });
  _embedderAPI.UpdateLocales(_engine, flutterLocaleList.data(), flutterLocaleList.size());
}

- (void)engineCallbackOnPlatformMessage:(const FlutterPlatformMessage*)message {
  NSData* messageData = nil;
  if (message->message_size > 0) {
    messageData = [NSData dataWithBytesNoCopy:(void*)message->message
                                       length:message->message_size
                                 freeWhenDone:NO];
  }
  NSString* channel = @(message->channel);
  __block const FlutterPlatformMessageResponseHandle* responseHandle = message->response_handle;
  __block FlutterEngine* weakSelf = self;
  NSMutableArray* isResponseValid = self.isResponseValid;
  FlutterEngineSendPlatformMessageResponseFnPtr sendPlatformMessageResponse =
      _embedderAPI.SendPlatformMessageResponse;
  FlutterBinaryReply binaryResponseHandler = ^(NSData* response) {
    @synchronized(isResponseValid) {
      if (![isResponseValid[0] boolValue]) {
        // Ignore, engine was killed.
        return;
      }
      if (responseHandle) {
        sendPlatformMessageResponse(weakSelf->_engine, responseHandle,
                                    static_cast<const uint8_t*>(response.bytes), response.length);
        responseHandle = NULL;
      } else {
        NSLog(@"Error: Message responses can be sent only once. Ignoring duplicate response "
               "on channel '%@'.",
              channel);
      }
    }
  };

  FlutterEngineHandlerInfo* handlerInfo = _messengerHandlers[channel];
  if (handlerInfo) {
    handlerInfo.handler(messageData, binaryResponseHandler);
  } else {
    binaryResponseHandler(nil);
  }
}

- (void)engineCallbackOnPreEngineRestart {
  NSEnumerator* viewControllerEnumerator = [_viewControllers objectEnumerator];
  FlutterViewController* nextViewController;
  while ((nextViewController = [viewControllerEnumerator nextObject])) {
    [nextViewController onPreEngineRestart];
  }
}

- (void)onVSync:(uintptr_t)baton {
  @synchronized(_vsyncWaiters) {
    // TODO(knopp): Use vsync waiter for correct view.
    // https://github.com/flutter/flutter/issues/142845
    FlutterVSyncWaiter* waiter = [_vsyncWaiters objectForKey:@(kFlutterImplicitViewId)];
    [waiter waitForVSync:baton];
  }
}

/**
 * Note: Called from dealloc. Should not use accessors or other methods.
 */
- (void)shutDownEngine {
  if (_engine == nullptr) {
    return;
  }

  [_threadSynchronizer shutdown];
  _threadSynchronizer = nil;

  FlutterEngineResult result = _embedderAPI.Deinitialize(_engine);
  if (result != kSuccess) {
    NSLog(@"Could not de-initialize the Flutter engine: error %d", result);
  }

  result = _embedderAPI.Shutdown(_engine);
  if (result != kSuccess) {
    NSLog(@"Failed to shut down Flutter engine: error %d", result);
  }
  _engine = nullptr;
}

- (void)setUpPlatformViewChannel {
  _platformViewsChannel =
      [FlutterMethodChannel methodChannelWithName:@"flutter/platform_views"
                                  binaryMessenger:self.binaryMessenger
                                            codec:[FlutterStandardMethodCodec sharedInstance]];

  __weak FlutterEngine* weakSelf = self;
  [_platformViewsChannel setMethodCallHandler:^(FlutterMethodCall* call, FlutterResult result) {
    [[weakSelf platformViewController] handleMethodCall:call result:result];
  }];
}

- (void)setUpAccessibilityChannel {
  _accessibilityChannel = [FlutterBasicMessageChannel
      messageChannelWithName:@"flutter/accessibility"
             binaryMessenger:self.binaryMessenger
                       codec:[FlutterStandardMessageCodec sharedInstance]];
  __weak FlutterEngine* weakSelf = self;
  [_accessibilityChannel setMessageHandler:^(id message, FlutterReply reply) {
    [weakSelf handleAccessibilityEvent:message];
  }];
}
- (void)setUpNotificationCenterListeners {
  NSNotificationCenter* center = [NSNotificationCenter defaultCenter];
  // macOS fires this private message when VoiceOver turns on or off.
  [center addObserver:self
             selector:@selector(onAccessibilityStatusChanged:)
                 name:kEnhancedUserInterfaceNotification
               object:nil];
  [center addObserver:self
             selector:@selector(applicationWillTerminate:)
                 name:NSApplicationWillTerminateNotification
               object:nil];
  [center addObserver:self
             selector:@selector(windowDidChangeScreen:)
                 name:NSWindowDidChangeScreenNotification
               object:nil];
  [center addObserver:self
             selector:@selector(updateDisplayConfig:)
                 name:NSApplicationDidChangeScreenParametersNotification
               object:nil];
}

- (void)addInternalPlugins {
  __weak FlutterEngine* weakSelf = self;
  [FlutterMouseCursorPlugin registerWithRegistrar:[self registrarForPlugin:@"mousecursor"]
                                         delegate:self];
  [FlutterMenuPlugin registerWithRegistrar:[self registrarForPlugin:@"menu"]];
  _settingsChannel =
      [FlutterBasicMessageChannel messageChannelWithName:kFlutterSettingsChannel
                                         binaryMessenger:self.binaryMessenger
                                                   codec:[FlutterJSONMessageCodec sharedInstance]];
  _platformChannel =
      [FlutterMethodChannel methodChannelWithName:kFlutterPlatformChannel
                                  binaryMessenger:self.binaryMessenger
                                            codec:[FlutterJSONMethodCodec sharedInstance]];
  [_platformChannel setMethodCallHandler:^(FlutterMethodCall* call, FlutterResult result) {
    [weakSelf handleMethodCall:call result:result];
  }];
}

- (void)didUpdateMouseCursor:(NSCursor*)cursor {
  // Mouse cursor plugin does not specify which view is responsible for changing the cursor,
  // so the reasonable assumption here is that cursor change is a result of a mouse movement
  // and thus the cursor will be paired with last Flutter view that reveived mouse event.
  [_lastViewWithPointerEvent didUpdateMouseCursor:cursor];
}

- (void)applicationWillTerminate:(NSNotification*)notification {
  [self shutDownEngine];
}

- (void)windowDidChangeScreen:(NSNotification*)notification {
  // Update window metric for all view controllers since the display_id has
  // changed.
  NSEnumerator* viewControllerEnumerator = [_viewControllers objectEnumerator];
  FlutterViewController* nextViewController;
  while ((nextViewController = [viewControllerEnumerator nextObject])) {
    [self updateWindowMetricsForViewController:nextViewController];
  }
}

- (void)onAccessibilityStatusChanged:(NSNotification*)notification {
  BOOL enabled = [notification.userInfo[kEnhancedUserInterfaceKey] boolValue];
  NSEnumerator* viewControllerEnumerator = [_viewControllers objectEnumerator];
  FlutterViewController* nextViewController;
  while ((nextViewController = [viewControllerEnumerator nextObject])) {
    [nextViewController onAccessibilityStatusChanged:enabled];
  }

  self.semanticsEnabled = enabled;
}
- (void)handleAccessibilityEvent:(NSDictionary<NSString*, id>*)annotatedEvent {
  NSString* type = annotatedEvent[@"type"];
  if ([type isEqualToString:@"announce"]) {
    NSString* message = annotatedEvent[@"data"][@"message"];
    NSNumber* assertiveness = annotatedEvent[@"data"][@"assertiveness"];
    if (message == nil) {
      return;
    }

    NSAccessibilityPriorityLevel priority = [assertiveness isEqualToNumber:@1]
                                                ? NSAccessibilityPriorityHigh
                                                : NSAccessibilityPriorityMedium;

    [self announceAccessibilityMessage:message withPriority:priority];
  }
}

- (void)announceAccessibilityMessage:(NSString*)message
                        withPriority:(NSAccessibilityPriorityLevel)priority {
  NSAccessibilityPostNotificationWithUserInfo(
      [self viewControllerForIdentifier:kFlutterImplicitViewId].flutterView,
      NSAccessibilityAnnouncementRequestedNotification,
      @{NSAccessibilityAnnouncementKey : message, NSAccessibilityPriorityKey : @(priority)});
}
- (void)handleMethodCall:(FlutterMethodCall*)call result:(FlutterResult)result {
  if ([call.method isEqualToString:@"SystemNavigator.pop"]) {
    [[NSApplication sharedApplication] terminate:self];
    result(nil);
  } else if ([call.method isEqualToString:@"SystemSound.play"]) {
    [self playSystemSound:call.arguments];
    result(nil);
  } else if ([call.method isEqualToString:@"Clipboard.getData"]) {
    result([self getClipboardData:call.arguments]);
  } else if ([call.method isEqualToString:@"Clipboard.setData"]) {
    [self setClipboardData:call.arguments];
    result(nil);
  } else if ([call.method isEqualToString:@"Clipboard.hasStrings"]) {
    result(@{@"value" : @([self clipboardHasStrings])});
  } else if ([call.method isEqualToString:@"System.exitApplication"]) {
    if ([self terminationHandler] == nil) {
      // If the termination handler isn't set, then either we haven't
      // initialized it yet, or (more likely) the NSApp delegate isn't a
      // FlutterAppDelegate, so it can't cancel requests to exit. So, in that
      // case, just terminate when requested.
      [NSApp terminate:self];
      result(nil);
    } else {
      [[self terminationHandler] handleRequestAppExitMethodCall:call.arguments result:result];
    }
  } else if ([call.method isEqualToString:@"System.initializationComplete"]) {
    if ([self terminationHandler] != nil) {
      [self terminationHandler].acceptingRequests = YES;
    }
    result(nil);
  } else {
    result(FlutterMethodNotImplemented);
  }
}

- (void)playSystemSound:(NSString*)soundType {
  if ([soundType isEqualToString:@"SystemSoundType.alert"]) {
    NSBeep();
  }
}

- (NSDictionary*)getClipboardData:(NSString*)format {
  if ([format isEqualToString:@(kTextPlainFormat)]) {
    NSString* stringInPasteboard = [self.pasteboard stringForType:NSPasteboardTypeString];
    return stringInPasteboard == nil ? nil : @{@"text" : stringInPasteboard};
  }
  return nil;
}

- (void)setClipboardData:(NSDictionary*)data {
  NSString* text = data[@"text"];
  [self.pasteboard clearContents];
  if (text && ![text isEqual:[NSNull null]]) {
    [self.pasteboard setString:text forType:NSPasteboardTypeString];
  }
}

- (BOOL)clipboardHasStrings {
  return [self.pasteboard stringForType:NSPasteboardTypeString].length > 0;
}

- (std::vector<std::string>)switches {
  return flutter::GetSwitchesFromEnvironment();
}

- (FlutterThreadSynchronizer*)testThreadSynchronizer {
  return _threadSynchronizer;
}

#pragma mark - FlutterAppLifecycleDelegate

- (void)setApplicationState:(flutter::AppLifecycleState)state {
  NSString* nextState =
      [[NSString alloc] initWithCString:flutter::AppLifecycleStateToString(state)];
  [self sendOnChannel:kFlutterLifecycleChannel
              message:[nextState dataUsingEncoding:NSUTF8StringEncoding]];
}

/**
 * Called when the |FlutterAppDelegate| gets the applicationWillBecomeActive
 * notification.
 */
- (void)handleWillBecomeActive:(NSNotification*)notification {
  _active = YES;
  if (!_visible) {
    [self setApplicationState:flutter::AppLifecycleState::kHidden];
  } else {
    [self setApplicationState:flutter::AppLifecycleState::kResumed];
  }
}

/**
 * Called when the |FlutterAppDelegate| gets the applicationWillResignActive
 * notification.
 */
- (void)handleWillResignActive:(NSNotification*)notification {
  _active = NO;
  if (!_visible) {
    [self setApplicationState:flutter::AppLifecycleState::kHidden];
  } else {
    [self setApplicationState:flutter::AppLifecycleState::kInactive];
  }
}

/**
 * Called when the |FlutterAppDelegate| gets the applicationDidUnhide
 * notification.
 */
- (void)handleDidChangeOcclusionState:(NSNotification*)notification {
  NSApplicationOcclusionState occlusionState = [[NSApplication sharedApplication] occlusionState];
  if (occlusionState & NSApplicationOcclusionStateVisible) {
    _visible = YES;
    if (_active) {
      [self setApplicationState:flutter::AppLifecycleState::kResumed];
    } else {
      [self setApplicationState:flutter::AppLifecycleState::kInactive];
    }
  } else {
    _visible = NO;
    [self setApplicationState:flutter::AppLifecycleState::kHidden];
  }
}

#pragma mark - FlutterBinaryMessenger

- (void)sendOnChannel:(nonnull NSString*)channel message:(nullable NSData*)message {
  [self sendOnChannel:channel message:message binaryReply:nil];
}

- (void)sendOnChannel:(NSString*)channel
              message:(NSData* _Nullable)message
          binaryReply:(FlutterBinaryReply _Nullable)callback {
  FlutterPlatformMessageResponseHandle* response_handle = nullptr;
  if (callback) {
    struct Captures {
      FlutterBinaryReply reply;
    };
    auto captures = std::make_unique<Captures>();
    captures->reply = callback;
    auto message_reply = [](const uint8_t* data, size_t data_size, void* user_data) {
      auto captures = reinterpret_cast<Captures*>(user_data);
      NSData* reply_data = nil;
      if (data != nullptr && data_size > 0) {
        reply_data = [NSData dataWithBytes:static_cast<const void*>(data) length:data_size];
      }
      captures->reply(reply_data);
      delete captures;
    };

    FlutterEngineResult create_result = _embedderAPI.PlatformMessageCreateResponseHandle(
        _engine, message_reply, captures.get(), &response_handle);
    if (create_result != kSuccess) {
      NSLog(@"Failed to create a FlutterPlatformMessageResponseHandle (%d)", create_result);
      return;
    }
    captures.release();
  }

  FlutterPlatformMessage platformMessage = {
      .struct_size = sizeof(FlutterPlatformMessage),
      .channel = [channel UTF8String],
      .message = static_cast<const uint8_t*>(message.bytes),
      .message_size = message.length,
      .response_handle = response_handle,
  };

  FlutterEngineResult message_result = _embedderAPI.SendPlatformMessage(_engine, &platformMessage);
  if (message_result != kSuccess) {
    NSLog(@"Failed to send message to Flutter engine on channel '%@' (%d).", channel,
          message_result);
  }

  if (response_handle != nullptr) {
    FlutterEngineResult release_result =
        _embedderAPI.PlatformMessageReleaseResponseHandle(_engine, response_handle);
    if (release_result != kSuccess) {
      NSLog(@"Failed to release the response handle (%d).", release_result);
    };
  }
}

- (FlutterBinaryMessengerConnection)setMessageHandlerOnChannel:(nonnull NSString*)channel
                                          binaryMessageHandler:
                                              (nullable FlutterBinaryMessageHandler)handler {
  _currentMessengerConnection += 1;
  _messengerHandlers[channel] =
      [[FlutterEngineHandlerInfo alloc] initWithConnection:@(_currentMessengerConnection)
                                                   handler:[handler copy]];
  return _currentMessengerConnection;
}

- (void)cleanUpConnection:(FlutterBinaryMessengerConnection)connection {
  // Find the _messengerHandlers that has the required connection, and record its
  // channel.
  NSString* foundChannel = nil;
  for (NSString* key in [_messengerHandlers allKeys]) {
    FlutterEngineHandlerInfo* handlerInfo = [_messengerHandlers objectForKey:key];
    if ([handlerInfo.connection isEqual:@(connection)]) {
      foundChannel = key;
      break;
    }
  }
  if (foundChannel) {
    [_messengerHandlers removeObjectForKey:foundChannel];
  }
}

#pragma mark - FlutterPluginRegistry

- (id<FlutterPluginRegistrar>)registrarForPlugin:(NSString*)pluginName {
  id<FlutterPluginRegistrar> registrar = self.pluginRegistrars[pluginName];
  if (!registrar) {
    FlutterEngineRegistrar* registrarImpl =
        [[FlutterEngineRegistrar alloc] initWithPlugin:pluginName flutterEngine:self];
    self.pluginRegistrars[pluginName] = registrarImpl;
    registrar = registrarImpl;
  }
  return registrar;
}

- (nullable NSObject*)valuePublishedByPlugin:(NSString*)pluginName {
  return self.pluginRegistrars[pluginName].publishedValue;
}

#pragma mark - FlutterTextureRegistrar

- (int64_t)registerTexture:(id<FlutterTexture>)texture {
  return [_renderer registerTexture:texture];
}

- (BOOL)registerTextureWithID:(int64_t)textureId {
  return _embedderAPI.RegisterExternalTexture(_engine, textureId) == kSuccess;
}

- (void)textureFrameAvailable:(int64_t)textureID {
  [_renderer textureFrameAvailable:textureID];
}

- (BOOL)markTextureFrameAvailable:(int64_t)textureID {
  return _embedderAPI.MarkExternalTextureFrameAvailable(_engine, textureID) == kSuccess;
}

- (void)unregisterTexture:(int64_t)textureID {
  [_renderer unregisterTexture:textureID];
}

- (BOOL)unregisterTextureWithID:(int64_t)textureID {
  return _embedderAPI.UnregisterExternalTexture(_engine, textureID) == kSuccess;
}

#pragma mark - Task runner integration

- (void)runTaskOnEmbedder:(FlutterTask)task {
  if (_engine) {
    auto result = _embedderAPI.RunTask(_engine, &task);
    if (result != kSuccess) {
      NSLog(@"Could not post a task to the Flutter engine.");
    }
  }
}

- (void)postMainThreadTask:(FlutterTask)task targetTimeInNanoseconds:(uint64_t)targetTime {
  __weak FlutterEngine* weakSelf = self;
  auto worker = ^{
    [weakSelf runTaskOnEmbedder:task];
  };

  const auto engine_time = _embedderAPI.GetCurrentTime();
  if (targetTime <= engine_time) {
    dispatch_async(dispatch_get_main_queue(), worker);

  } else {
    dispatch_after(dispatch_time(DISPATCH_TIME_NOW, targetTime - engine_time),
                   dispatch_get_main_queue(), worker);
  }
}

// Getter used by test harness, only exposed through the FlutterEngine(Test) category
- (flutter::FlutterCompositor*)macOSCompositor {
  return _macOSCompositor.get();
}

@end
