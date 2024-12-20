// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/macos/framework/Headers/FlutterEngine.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterEngine_Internal.h"

#include <objc/objc.h>

#include <algorithm>
#include <functional>
#include <thread>
#include <vector>

#include "flutter/fml/synchronization/waitable_event.h"
#include "flutter/lib/ui/window/platform_message.h"
#include "flutter/shell/platform/common/accessibility_bridge.h"
#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterChannels.h"
#import "flutter/shell/platform/darwin/common/framework/Source/FlutterBinaryMessengerRelay.h"
#import "flutter/shell/platform/darwin/macos/framework/Headers/FlutterAppDelegate.h"
#import "flutter/shell/platform/darwin/macos/framework/Headers/FlutterAppLifecycleDelegate.h"
#import "flutter/shell/platform/darwin/macos/framework/Headers/FlutterPluginMacOS.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterCompositor.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterEngineTestUtils.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterViewControllerTestUtils.h"
#include "flutter/shell/platform/embedder/embedder.h"
#include "flutter/shell/platform/embedder/embedder_engine.h"
#include "flutter/shell/platform/embedder/test_utils/proc_table_replacement.h"
#include "flutter/testing/stream_capture.h"
#include "flutter/testing/test_dart_native_resolver.h"
#include "gtest/gtest.h"

// CREATE_NATIVE_ENTRY and MOCK_ENGINE_PROC are leaky by design
// NOLINTBEGIN(clang-analyzer-core.StackAddressEscape)

constexpr int64_t kImplicitViewId = 0ll;

@interface FlutterEngine (Test)
/**
 * The FlutterCompositor object currently in use by the FlutterEngine.
 *
 * May be nil if the compositor has not been initialized yet.
 */
@property(nonatomic, readonly, nullable) flutter::FlutterCompositor* macOSCompositor;

@end

@interface TestPlatformViewFactory : NSObject <FlutterPlatformViewFactory>
@end

@implementation TestPlatformViewFactory
- (nonnull NSView*)createWithViewIdentifier:(FlutterViewIdentifier)viewIdentifier
                                  arguments:(nullable id)args {
  return viewIdentifier == 42 ? [[NSView alloc] init] : nil;
}

@end

@interface PlainAppDelegate : NSObject <NSApplicationDelegate>
@end

@implementation PlainAppDelegate
- (NSApplicationTerminateReply)applicationShouldTerminate:(NSApplication* _Nonnull)sender {
  // Always cancel, so that the test doesn't exit.
  return NSTerminateCancel;
}
@end

#pragma mark -

@interface FakeLifecycleProvider : NSObject <FlutterAppLifecycleProvider, NSApplicationDelegate>

@property(nonatomic, strong, readonly) NSPointerArray* registeredDelegates;

// True if the given delegate is currently registered.
- (BOOL)hasDelegate:(nonnull NSObject<FlutterAppLifecycleDelegate>*)delegate;
@end

@implementation FakeLifecycleProvider {
  /**
   * All currently registered delegates.
   *
   * This does not use NSPointerArray or any other weak-pointer
   * system, because a weak pointer will be nil'd out at the start of dealloc, which will break
   * queries. E.g., if a delegate is dealloc'd without being unregistered, a weak pointer array
   * would no longer contain that pointer even though removeApplicationLifecycleDelegate: was never
   * called, causing tests to pass incorrectly.
   */
  std::vector<void*> _delegates;
}

- (void)addApplicationLifecycleDelegate:(nonnull NSObject<FlutterAppLifecycleDelegate>*)delegate {
  _delegates.push_back((__bridge void*)delegate);
}

- (void)removeApplicationLifecycleDelegate:
    (nonnull NSObject<FlutterAppLifecycleDelegate>*)delegate {
  auto delegateIndex = std::find(_delegates.begin(), _delegates.end(), (__bridge void*)delegate);
  NSAssert(delegateIndex != _delegates.end(),
           @"Attempting to unregister a delegate that was not registered.");
  _delegates.erase(delegateIndex);
}

- (BOOL)hasDelegate:(nonnull NSObject<FlutterAppLifecycleDelegate>*)delegate {
  return std::find(_delegates.begin(), _delegates.end(), (__bridge void*)delegate) !=
         _delegates.end();
}

@end

#pragma mark -

@interface FakeAppDelegatePlugin : NSObject <FlutterPlugin>
@end

@implementation FakeAppDelegatePlugin
+ (void)registerWithRegistrar:(id<FlutterPluginRegistrar>)registrar {
}
@end

#pragma mark -

@interface MockableFlutterEngine : FlutterEngine
@end

@implementation MockableFlutterEngine
- (NSArray<NSScreen*>*)screens {
  id mockScreen = OCMClassMock([NSScreen class]);
  OCMStub([mockScreen backingScaleFactor]).andReturn(2.0);
  OCMStub([mockScreen deviceDescription]).andReturn(@{
    @"NSScreenNumber" : [NSNumber numberWithInt:10]
  });
  OCMStub([mockScreen frame]).andReturn(NSMakeRect(10, 20, 30, 40));
  return [NSArray arrayWithObject:mockScreen];
}
@end

#pragma mark -

namespace flutter::testing {

TEST_F(FlutterEngineTest, CanLaunch) {
  FlutterEngine* engine = GetFlutterEngine();
  EXPECT_TRUE([engine runWithEntrypoint:@"main"]);
  ASSERT_TRUE(engine.running);
}

TEST_F(FlutterEngineTest, HasNonNullExecutableName) {
  FlutterEngine* engine = GetFlutterEngine();
  std::string executable_name = [[engine executableName] UTF8String];
  ASSERT_FALSE(executable_name.empty());

  // Block until notified by the Dart test of the value of Platform.executable.
  fml::AutoResetWaitableEvent latch;
  AddNativeCallback("NotifyStringValue", CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) {
                      const auto dart_string = tonic::DartConverter<std::string>::FromDart(
                          Dart_GetNativeArgument(args, 0));
                      EXPECT_EQ(executable_name, dart_string);
                      latch.Signal();
                    }));

  // Launch the test entrypoint.
  EXPECT_TRUE([engine runWithEntrypoint:@"executableNameNotNull"]);

  latch.Wait();
}

#ifndef FLUTTER_RELEASE
TEST_F(FlutterEngineTest, Switches) {
  setenv("FLUTTER_ENGINE_SWITCHES", "2", 1);
  setenv("FLUTTER_ENGINE_SWITCH_1", "abc", 1);
  setenv("FLUTTER_ENGINE_SWITCH_2", "foo=\"bar, baz\"", 1);

  FlutterEngine* engine = GetFlutterEngine();
  std::vector<std::string> switches = engine.switches;
  ASSERT_EQ(switches.size(), 2UL);
  EXPECT_EQ(switches[0], "--abc");
  EXPECT_EQ(switches[1], "--foo=\"bar, baz\"");

  unsetenv("FLUTTER_ENGINE_SWITCHES");
  unsetenv("FLUTTER_ENGINE_SWITCH_1");
  unsetenv("FLUTTER_ENGINE_SWITCH_2");
}
#endif  // !FLUTTER_RELEASE

TEST_F(FlutterEngineTest, MessengerSend) {
  FlutterEngine* engine = GetFlutterEngine();
  EXPECT_TRUE([engine runWithEntrypoint:@"main"]);

  NSData* test_message = [@"a message" dataUsingEncoding:NSUTF8StringEncoding];
  bool called = false;

  engine.embedderAPI.SendPlatformMessage = MOCK_ENGINE_PROC(
      SendPlatformMessage, ([&called, test_message](auto engine, auto message) {
        called = true;
        EXPECT_STREQ(message->channel, "test");
        EXPECT_EQ(memcmp(message->message, test_message.bytes, message->message_size), 0);
        return kSuccess;
      }));

  [engine.binaryMessenger sendOnChannel:@"test" message:test_message];
  EXPECT_TRUE(called);
}

TEST_F(FlutterEngineTest, CanLogToStdout) {
  // Block until completion of print statement.
  fml::AutoResetWaitableEvent latch;
  AddNativeCallback("SignalNativeTest",
                    CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) { latch.Signal(); }));

  // Replace stdout stream buffer with our own.
  StreamCapture stdout_capture(&std::cout);

  // Launch the test entrypoint.
  FlutterEngine* engine = GetFlutterEngine();
  EXPECT_TRUE([engine runWithEntrypoint:@"canLogToStdout"]);
  ASSERT_TRUE(engine.running);

  latch.Wait();

  stdout_capture.Stop();

  // Verify hello world was written to stdout.
  EXPECT_TRUE(stdout_capture.GetOutput().find("Hello logging") != std::string::npos);
}

TEST_F(FlutterEngineTest, DISABLED_BackgroundIsBlack) {
  FlutterEngine* engine = GetFlutterEngine();

  // Latch to ensure the entire layer tree has been generated and presented.
  fml::AutoResetWaitableEvent latch;
  AddNativeCallback("SignalNativeTest", CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) {
                      CALayer* rootLayer = engine.viewController.flutterView.layer;
                      EXPECT_TRUE(rootLayer.backgroundColor != nil);
                      if (rootLayer.backgroundColor != nil) {
                        NSColor* actualBackgroundColor =
                            [NSColor colorWithCGColor:rootLayer.backgroundColor];
                        EXPECT_EQ(actualBackgroundColor, [NSColor blackColor]);
                      }
                      latch.Signal();
                    }));

  // Launch the test entrypoint.
  EXPECT_TRUE([engine runWithEntrypoint:@"backgroundTest"]);
  ASSERT_TRUE(engine.running);

  FlutterViewController* viewController = [[FlutterViewController alloc] initWithEngine:engine
                                                                                nibName:nil
                                                                                 bundle:nil];
  [viewController loadView];
  viewController.flutterView.frame = CGRectMake(0, 0, 800, 600);

  latch.Wait();
}

TEST_F(FlutterEngineTest, DISABLED_CanOverrideBackgroundColor) {
  FlutterEngine* engine = GetFlutterEngine();

  // Latch to ensure the entire layer tree has been generated and presented.
  fml::AutoResetWaitableEvent latch;
  AddNativeCallback("SignalNativeTest", CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) {
                      CALayer* rootLayer = engine.viewController.flutterView.layer;
                      EXPECT_TRUE(rootLayer.backgroundColor != nil);
                      if (rootLayer.backgroundColor != nil) {
                        NSColor* actualBackgroundColor =
                            [NSColor colorWithCGColor:rootLayer.backgroundColor];
                        EXPECT_EQ(actualBackgroundColor, [NSColor whiteColor]);
                      }
                      latch.Signal();
                    }));

  // Launch the test entrypoint.
  EXPECT_TRUE([engine runWithEntrypoint:@"backgroundTest"]);
  ASSERT_TRUE(engine.running);

  FlutterViewController* viewController = [[FlutterViewController alloc] initWithEngine:engine
                                                                                nibName:nil
                                                                                 bundle:nil];
  [viewController loadView];
  viewController.flutterView.frame = CGRectMake(0, 0, 800, 600);
  viewController.flutterView.backgroundColor = [NSColor whiteColor];

  latch.Wait();
}

TEST_F(FlutterEngineTest, CanToggleAccessibility) {
  FlutterEngine* engine = GetFlutterEngine();
  // Capture the update callbacks before the embedder API initializes.
  auto original_init = engine.embedderAPI.Initialize;
  std::function<void(const FlutterSemanticsUpdate2*, void*)> update_semantics_callback;
  engine.embedderAPI.Initialize = MOCK_ENGINE_PROC(
      Initialize, ([&update_semantics_callback, &original_init](
                       size_t version, const FlutterRendererConfig* config,
                       const FlutterProjectArgs* args, void* user_data, auto engine_out) {
        update_semantics_callback = args->update_semantics_callback2;
        return original_init(version, config, args, user_data, engine_out);
      }));
  EXPECT_TRUE([engine runWithEntrypoint:@"main"]);
  // Set up view controller.
  FlutterViewController* viewController = [[FlutterViewController alloc] initWithEngine:engine
                                                                                nibName:nil
                                                                                 bundle:nil];
  [viewController loadView];
  // Enable the semantics.
  bool enabled_called = false;
  engine.embedderAPI.UpdateSemanticsEnabled =
      MOCK_ENGINE_PROC(UpdateSemanticsEnabled, ([&enabled_called](auto engine, bool enabled) {
                         enabled_called = enabled;
                         return kSuccess;
                       }));
  engine.semanticsEnabled = YES;
  EXPECT_TRUE(enabled_called);
  // Send flutter semantics updates.
  FlutterSemanticsNode2 root;
  root.id = 0;
  root.flags = static_cast<FlutterSemanticsFlag>(0);
  root.actions = static_cast<FlutterSemanticsAction>(0);
  root.text_selection_base = -1;
  root.text_selection_extent = -1;
  root.label = "root";
  root.hint = "";
  root.value = "";
  root.increased_value = "";
  root.decreased_value = "";
  root.tooltip = "";
  root.child_count = 1;
  int32_t children[] = {1};
  root.children_in_traversal_order = children;
  root.custom_accessibility_actions_count = 0;

  FlutterSemanticsNode2 child1;
  child1.id = 1;
  child1.flags = static_cast<FlutterSemanticsFlag>(0);
  child1.actions = static_cast<FlutterSemanticsAction>(0);
  child1.text_selection_base = -1;
  child1.text_selection_extent = -1;
  child1.label = "child 1";
  child1.hint = "";
  child1.value = "";
  child1.increased_value = "";
  child1.decreased_value = "";
  child1.tooltip = "";
  child1.child_count = 0;
  child1.custom_accessibility_actions_count = 0;

  FlutterSemanticsUpdate2 update;
  update.node_count = 2;
  FlutterSemanticsNode2* nodes[] = {&root, &child1};
  update.nodes = nodes;
  update.custom_action_count = 0;
  update_semantics_callback(&update, (__bridge void*)engine);

  // Verify the accessibility tree is attached to the flutter view.
  EXPECT_EQ([engine.viewController.flutterView.accessibilityChildren count], 1u);
  NSAccessibilityElement* native_root = engine.viewController.flutterView.accessibilityChildren[0];
  std::string root_label = [native_root.accessibilityLabel UTF8String];
  EXPECT_TRUE(root_label == "root");
  EXPECT_EQ(native_root.accessibilityRole, NSAccessibilityGroupRole);
  EXPECT_EQ([native_root.accessibilityChildren count], 1u);
  NSAccessibilityElement* native_child1 = native_root.accessibilityChildren[0];
  std::string child1_value = [native_child1.accessibilityValue UTF8String];
  EXPECT_TRUE(child1_value == "child 1");
  EXPECT_EQ(native_child1.accessibilityRole, NSAccessibilityStaticTextRole);
  EXPECT_EQ([native_child1.accessibilityChildren count], 0u);
  // Disable the semantics.
  bool semanticsEnabled = true;
  engine.embedderAPI.UpdateSemanticsEnabled =
      MOCK_ENGINE_PROC(UpdateSemanticsEnabled, ([&semanticsEnabled](auto engine, bool enabled) {
                         semanticsEnabled = enabled;
                         return kSuccess;
                       }));
  engine.semanticsEnabled = NO;
  EXPECT_FALSE(semanticsEnabled);
  // Verify the accessibility tree is removed from the view.
  EXPECT_EQ([engine.viewController.flutterView.accessibilityChildren count], 0u);

  [engine setViewController:nil];
}

TEST_F(FlutterEngineTest, CanToggleAccessibilityWhenHeadless) {
  FlutterEngine* engine = GetFlutterEngine();
  // Capture the update callbacks before the embedder API initializes.
  auto original_init = engine.embedderAPI.Initialize;
  std::function<void(const FlutterSemanticsUpdate2*, void*)> update_semantics_callback;
  engine.embedderAPI.Initialize = MOCK_ENGINE_PROC(
      Initialize, ([&update_semantics_callback, &original_init](
                       size_t version, const FlutterRendererConfig* config,
                       const FlutterProjectArgs* args, void* user_data, auto engine_out) {
        update_semantics_callback = args->update_semantics_callback2;
        return original_init(version, config, args, user_data, engine_out);
      }));
  EXPECT_TRUE([engine runWithEntrypoint:@"main"]);

  // Enable the semantics without attaching a view controller.
  bool enabled_called = false;
  engine.embedderAPI.UpdateSemanticsEnabled =
      MOCK_ENGINE_PROC(UpdateSemanticsEnabled, ([&enabled_called](auto engine, bool enabled) {
                         enabled_called = enabled;
                         return kSuccess;
                       }));
  engine.semanticsEnabled = YES;
  EXPECT_TRUE(enabled_called);
  // Send flutter semantics updates.
  FlutterSemanticsNode2 root;
  root.id = 0;
  root.flags = static_cast<FlutterSemanticsFlag>(0);
  root.actions = static_cast<FlutterSemanticsAction>(0);
  root.text_selection_base = -1;
  root.text_selection_extent = -1;
  root.label = "root";
  root.hint = "";
  root.value = "";
  root.increased_value = "";
  root.decreased_value = "";
  root.tooltip = "";
  root.child_count = 1;
  int32_t children[] = {1};
  root.children_in_traversal_order = children;
  root.custom_accessibility_actions_count = 0;

  FlutterSemanticsNode2 child1;
  child1.id = 1;
  child1.flags = static_cast<FlutterSemanticsFlag>(0);
  child1.actions = static_cast<FlutterSemanticsAction>(0);
  child1.text_selection_base = -1;
  child1.text_selection_extent = -1;
  child1.label = "child 1";
  child1.hint = "";
  child1.value = "";
  child1.increased_value = "";
  child1.decreased_value = "";
  child1.tooltip = "";
  child1.child_count = 0;
  child1.custom_accessibility_actions_count = 0;

  FlutterSemanticsUpdate2 update;
  update.node_count = 2;
  FlutterSemanticsNode2* nodes[] = {&root, &child1};
  update.nodes = nodes;
  update.custom_action_count = 0;
  // This call updates semantics for the implicit view, which does not exist,
  // and therefore this call is invalid. But the engine should not crash.
  update_semantics_callback(&update, (__bridge void*)engine);

  // No crashes.
  EXPECT_EQ(engine.viewController, nil);

  // Disable the semantics.
  bool semanticsEnabled = true;
  engine.embedderAPI.UpdateSemanticsEnabled =
      MOCK_ENGINE_PROC(UpdateSemanticsEnabled, ([&semanticsEnabled](auto engine, bool enabled) {
                         semanticsEnabled = enabled;
                         return kSuccess;
                       }));
  engine.semanticsEnabled = NO;
  EXPECT_FALSE(semanticsEnabled);
  // Still no crashes
  EXPECT_EQ(engine.viewController, nil);
}

TEST_F(FlutterEngineTest, ProducesAccessibilityTreeWhenAddingViews) {
  FlutterEngine* engine = GetFlutterEngine();
  EXPECT_TRUE([engine runWithEntrypoint:@"main"]);

  // Enable the semantics without attaching a view controller.
  bool enabled_called = false;
  engine.embedderAPI.UpdateSemanticsEnabled =
      MOCK_ENGINE_PROC(UpdateSemanticsEnabled, ([&enabled_called](auto engine, bool enabled) {
                         enabled_called = enabled;
                         return kSuccess;
                       }));
  engine.semanticsEnabled = YES;
  EXPECT_TRUE(enabled_called);

  EXPECT_EQ(engine.viewController, nil);

  // Assign the view controller after enabling semantics
  FlutterViewController* viewController = [[FlutterViewController alloc] initWithEngine:engine
                                                                                nibName:nil
                                                                                 bundle:nil];
  engine.viewController = viewController;

  EXPECT_NE(viewController.accessibilityBridge.lock(), nullptr);
}

TEST_F(FlutterEngineTest, NativeCallbacks) {
  fml::AutoResetWaitableEvent latch;
  bool latch_called = false;
  AddNativeCallback("SignalNativeTest", CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) {
                      latch_called = true;
                      latch.Signal();
                    }));

  FlutterEngine* engine = GetFlutterEngine();
  EXPECT_TRUE([engine runWithEntrypoint:@"nativeCallback"]);
  ASSERT_TRUE(engine.running);

  latch.Wait();
  ASSERT_TRUE(latch_called);
}

TEST_F(FlutterEngineTest, Compositor) {
  NSString* fixtures = @(flutter::testing::GetFixturesPath());
  FlutterDartProject* project = [[FlutterDartProject alloc]
      initWithAssetsPath:fixtures
             ICUDataPath:[fixtures stringByAppendingString:@"/icudtl.dat"]];
  FlutterEngine* engine = [[FlutterEngine alloc] initWithName:@"test" project:project];

  FlutterViewController* viewController = [[FlutterViewController alloc] initWithEngine:engine
                                                                                nibName:nil
                                                                                 bundle:nil];
  [viewController loadView];
  [viewController viewDidLoad];
  viewController.flutterView.frame = CGRectMake(0, 0, 800, 600);

  EXPECT_TRUE([engine runWithEntrypoint:@"canCompositePlatformViews"]);

  [engine.platformViewController registerViewFactory:[[TestPlatformViewFactory alloc] init]
                                              withId:@"factory_id"];
  [engine.platformViewController
      handleMethodCall:[FlutterMethodCall methodCallWithMethodName:@"create"
                                                         arguments:@{
                                                           @"id" : @(42),
                                                           @"viewType" : @"factory_id",
                                                         }]
                result:^(id result){
                }];

  [engine.testThreadSynchronizer blockUntilFrameAvailable];

  CALayer* rootLayer = viewController.flutterView.layer;

  // There are two layers with Flutter contents and one view
  EXPECT_EQ(rootLayer.sublayers.count, 2u);
  EXPECT_EQ(viewController.flutterView.subviews.count, 1u);

  // TODO(gw280): add support for screenshot tests in this test harness

  [engine shutDownEngine];
}

TEST_F(FlutterEngineTest, CompositorIgnoresUnknownView) {
  FlutterEngine* engine = GetFlutterEngine();
  auto original_init = engine.embedderAPI.Initialize;
  ::FlutterCompositor compositor;
  engine.embedderAPI.Initialize = MOCK_ENGINE_PROC(
      Initialize, ([&compositor, &original_init](
                       size_t version, const FlutterRendererConfig* config,
                       const FlutterProjectArgs* args, void* user_data, auto engine_out) {
        compositor = *args->compositor;
        return original_init(version, config, args, user_data, engine_out);
      }));

  FlutterViewController* viewController = [[FlutterViewController alloc] initWithEngine:engine
                                                                                nibName:nil
                                                                                 bundle:nil];
  [viewController loadView];

  EXPECT_TRUE([engine runWithEntrypoint:@"empty"]);

  FlutterBackingStoreConfig config = {
      .struct_size = sizeof(FlutterBackingStoreConfig),
      .size = FlutterSize{10, 10},
  };
  FlutterBackingStore backing_store = {};
  EXPECT_NE(compositor.create_backing_store_callback, nullptr);
  EXPECT_TRUE(
      compositor.create_backing_store_callback(&config, &backing_store, compositor.user_data));

  FlutterLayer layer{
      .type = kFlutterLayerContentTypeBackingStore,
      .backing_store = &backing_store,
  };
  std::vector<FlutterLayer*> layers = {&layer};

  FlutterPresentViewInfo info = {
      .struct_size = sizeof(FlutterPresentViewInfo),
      .view_id = 123,
      .layers = const_cast<const FlutterLayer**>(layers.data()),
      .layers_count = 1,
      .user_data = compositor.user_data,
  };
  EXPECT_NE(compositor.present_view_callback, nullptr);
  EXPECT_FALSE(compositor.present_view_callback(&info));
  EXPECT_TRUE(compositor.collect_backing_store_callback(&backing_store, compositor.user_data));

  (void)viewController;
  [engine shutDownEngine];
}

TEST_F(FlutterEngineTest, DartEntrypointArguments) {
  NSString* fixtures = @(flutter::testing::GetFixturesPath());
  FlutterDartProject* project = [[FlutterDartProject alloc]
      initWithAssetsPath:fixtures
             ICUDataPath:[fixtures stringByAppendingString:@"/icudtl.dat"]];

  project.dartEntrypointArguments = @[ @"arg1", @"arg2" ];
  FlutterEngine* engine = [[FlutterEngine alloc] initWithName:@"test" project:project];

  bool called = false;
  auto original_init = engine.embedderAPI.Initialize;
  engine.embedderAPI.Initialize = MOCK_ENGINE_PROC(
      Initialize, ([&called, &original_init](size_t version, const FlutterRendererConfig* config,
                                             const FlutterProjectArgs* args, void* user_data,
                                             FLUTTER_API_SYMBOL(FlutterEngine) * engine_out) {
        called = true;
        EXPECT_EQ(args->dart_entrypoint_argc, 2);
        NSString* arg1 = [[NSString alloc] initWithCString:args->dart_entrypoint_argv[0]
                                                  encoding:NSUTF8StringEncoding];
        NSString* arg2 = [[NSString alloc] initWithCString:args->dart_entrypoint_argv[1]
                                                  encoding:NSUTF8StringEncoding];

        EXPECT_TRUE([arg1 isEqualToString:@"arg1"]);
        EXPECT_TRUE([arg2 isEqualToString:@"arg2"]);

        return original_init(version, config, args, user_data, engine_out);
      }));

  EXPECT_TRUE([engine runWithEntrypoint:@"main"]);
  EXPECT_TRUE(called);
}

// Verify that the engine is not retained indirectly via the binary messenger held by channels and
// plugins. Previously, FlutterEngine.binaryMessenger returned the engine itself, and thus plugins
// could cause a retain cycle, preventing the engine from being deallocated.
// FlutterEngine.binaryMessenger now returns a FlutterBinaryMessengerRelay whose weak pointer back
// to the engine is cleared when the engine is deallocated.
// Issue: https://github.com/flutter/flutter/issues/116445
TEST_F(FlutterEngineTest, FlutterBinaryMessengerDoesNotRetainEngine) {
  __weak FlutterEngine* weakEngine;
  id<FlutterBinaryMessenger> binaryMessenger = nil;
  @autoreleasepool {
    // Create a test engine.
    NSString* fixtures = @(flutter::testing::GetFixturesPath());
    FlutterDartProject* project = [[FlutterDartProject alloc]
        initWithAssetsPath:fixtures
               ICUDataPath:[fixtures stringByAppendingString:@"/icudtl.dat"]];
    FlutterEngine* engine = [[FlutterEngine alloc] initWithName:@"test"
                                                        project:project
                                         allowHeadlessExecution:YES];
    weakEngine = engine;
    binaryMessenger = engine.binaryMessenger;
  }

  // Once the engine has been deallocated, verify the weak engine pointer is nil, and thus not
  // retained by the relay.
  EXPECT_NE(binaryMessenger, nil);
  EXPECT_EQ(weakEngine, nil);
}

// Verify that the engine is not retained indirectly via the texture registry held by plugins.
// Issue: https://github.com/flutter/flutter/issues/116445
TEST_F(FlutterEngineTest, FlutterTextureRegistryDoesNotReturnEngine) {
  __weak FlutterEngine* weakEngine;
  id<FlutterTextureRegistry> textureRegistry;
  @autoreleasepool {
    // Create a test engine.
    NSString* fixtures = @(flutter::testing::GetFixturesPath());
    FlutterDartProject* project = [[FlutterDartProject alloc]
        initWithAssetsPath:fixtures
               ICUDataPath:[fixtures stringByAppendingString:@"/icudtl.dat"]];
    FlutterEngine* engine = [[FlutterEngine alloc] initWithName:@"test"
                                                        project:project
                                         allowHeadlessExecution:YES];
    id<FlutterPluginRegistrar> registrar = [engine registrarForPlugin:@"MyPlugin"];
    textureRegistry = registrar.textures;
  }

  // Once the engine has been deallocated, verify the weak engine pointer is nil, and thus not
  // retained via the texture registry.
  EXPECT_NE(textureRegistry, nil);
  EXPECT_EQ(weakEngine, nil);
}

TEST_F(FlutterEngineTest, PublishedValueNilForUnknownPlugin) {
  NSString* fixtures = @(flutter::testing::GetFixturesPath());
  FlutterDartProject* project = [[FlutterDartProject alloc]
      initWithAssetsPath:fixtures
             ICUDataPath:[fixtures stringByAppendingString:@"/icudtl.dat"]];
  FlutterEngine* engine = [[FlutterEngine alloc] initWithName:@"test"
                                                      project:project
                                       allowHeadlessExecution:YES];

  EXPECT_EQ([engine valuePublishedByPlugin:@"NoSuchPlugin"], nil);
}

TEST_F(FlutterEngineTest, PublishedValueNSNullIfNoPublishedValue) {
  NSString* fixtures = @(flutter::testing::GetFixturesPath());
  FlutterDartProject* project = [[FlutterDartProject alloc]
      initWithAssetsPath:fixtures
             ICUDataPath:[fixtures stringByAppendingString:@"/icudtl.dat"]];
  FlutterEngine* engine = [[FlutterEngine alloc] initWithName:@"test"
                                                      project:project
                                       allowHeadlessExecution:YES];
  NSString* pluginName = @"MyPlugin";
  // Request the registarar to register the plugin as existing.
  [engine registrarForPlugin:pluginName];

  // The documented behavior is that a plugin that exists but hasn't published
  // anything returns NSNull, rather than nil, as on iOS.
  EXPECT_EQ([engine valuePublishedByPlugin:pluginName], [NSNull null]);
}

TEST_F(FlutterEngineTest, PublishedValueReturnsLastPublished) {
  NSString* fixtures = @(flutter::testing::GetFixturesPath());
  FlutterDartProject* project = [[FlutterDartProject alloc]
      initWithAssetsPath:fixtures
             ICUDataPath:[fixtures stringByAppendingString:@"/icudtl.dat"]];
  FlutterEngine* engine = [[FlutterEngine alloc] initWithName:@"test"
                                                      project:project
                                       allowHeadlessExecution:YES];
  NSString* pluginName = @"MyPlugin";
  id<FlutterPluginRegistrar> registrar = [engine registrarForPlugin:pluginName];

  NSString* firstValue = @"A published value";
  NSArray* secondValue = @[ @"A different published value" ];

  [registrar publish:firstValue];
  EXPECT_EQ([engine valuePublishedByPlugin:pluginName], firstValue);

  [registrar publish:secondValue];
  EXPECT_EQ([engine valuePublishedByPlugin:pluginName], secondValue);
}

// If a channel overrides a previous channel with the same name, cleaning
// the previous channel should not affect the new channel.
//
// This is important when recreating classes that uses a channel, because the
// new instance would create the channel before the first class is deallocated
// and clears the channel.
TEST_F(FlutterEngineTest, MessengerCleanupConnectionWorks) {
  FlutterEngine* engine = GetFlutterEngine();
  EXPECT_TRUE([engine runWithEntrypoint:@"main"]);

  NSString* channel = @"_test_";
  NSData* channel_data = [channel dataUsingEncoding:NSUTF8StringEncoding];

  // Mock SendPlatformMessage so that if a message is sent to
  // "test/send_message", act as if the framework has sent an empty message to
  // the channel marked by the `sendOnChannel:message:` call's message.
  engine.embedderAPI.SendPlatformMessage = MOCK_ENGINE_PROC(
      SendPlatformMessage, ([](auto engine_, auto message_) {
        if (strcmp(message_->channel, "test/send_message") == 0) {
          // The simplest message that is acceptable to a method channel.
          std::string message = R"|({"method": "a"})|";
          std::string channel(reinterpret_cast<const char*>(message_->message),
                              message_->message_size);
          reinterpret_cast<EmbedderEngine*>(engine_)
              ->GetShell()
              .GetPlatformView()
              ->HandlePlatformMessage(std::make_unique<PlatformMessage>(
                  channel.c_str(), fml::MallocMapping::Copy(message.c_str(), message.length()),
                  fml::RefPtr<PlatformMessageResponse>()));
        }
        return kSuccess;
      }));

  __block int record = 0;

  FlutterMethodChannel* channel1 =
      [FlutterMethodChannel methodChannelWithName:channel
                                  binaryMessenger:engine.binaryMessenger
                                            codec:[FlutterJSONMethodCodec sharedInstance]];
  [channel1 setMethodCallHandler:^(FlutterMethodCall* call, FlutterResult result) {
    record += 1;
  }];

  [engine.binaryMessenger sendOnChannel:@"test/send_message" message:channel_data];
  EXPECT_EQ(record, 1);

  FlutterMethodChannel* channel2 =
      [FlutterMethodChannel methodChannelWithName:channel
                                  binaryMessenger:engine.binaryMessenger
                                            codec:[FlutterJSONMethodCodec sharedInstance]];
  [channel2 setMethodCallHandler:^(FlutterMethodCall* call, FlutterResult result) {
    record += 10;
  }];

  [engine.binaryMessenger sendOnChannel:@"test/send_message" message:channel_data];
  EXPECT_EQ(record, 11);

  [channel1 setMethodCallHandler:nil];

  [engine.binaryMessenger sendOnChannel:@"test/send_message" message:channel_data];
  EXPECT_EQ(record, 21);
}

TEST_F(FlutterEngineTest, HasStringsWhenPasteboardEmpty) {
  id engineMock = CreateMockFlutterEngine(nil);

  // Call hasStrings and expect it to be false.
  __block bool calledAfterClear = false;
  __block bool valueAfterClear;
  FlutterResult resultAfterClear = ^(id result) {
    calledAfterClear = true;
    NSNumber* valueNumber = [result valueForKey:@"value"];
    valueAfterClear = [valueNumber boolValue];
  };
  FlutterMethodCall* methodCallAfterClear =
      [FlutterMethodCall methodCallWithMethodName:@"Clipboard.hasStrings" arguments:nil];
  [engineMock handleMethodCall:methodCallAfterClear result:resultAfterClear];
  EXPECT_TRUE(calledAfterClear);
  EXPECT_FALSE(valueAfterClear);
}

TEST_F(FlutterEngineTest, HasStringsWhenPasteboardFull) {
  id engineMock = CreateMockFlutterEngine(@"some string");

  // Call hasStrings and expect it to be true.
  __block bool called = false;
  __block bool value;
  FlutterResult result = ^(id result) {
    called = true;
    NSNumber* valueNumber = [result valueForKey:@"value"];
    value = [valueNumber boolValue];
  };
  FlutterMethodCall* methodCall =
      [FlutterMethodCall methodCallWithMethodName:@"Clipboard.hasStrings" arguments:nil];
  [engineMock handleMethodCall:methodCall result:result];
  EXPECT_TRUE(called);
  EXPECT_TRUE(value);
}

TEST_F(FlutterEngineTest, ResponseAfterEngineDied) {
  FlutterEngine* engine = GetFlutterEngine();
  FlutterBasicMessageChannel* channel = [[FlutterBasicMessageChannel alloc]
         initWithName:@"foo"
      binaryMessenger:engine.binaryMessenger
                codec:[FlutterStandardMessageCodec sharedInstance]];
  __block BOOL didCallCallback = NO;
  [channel setMessageHandler:^(id message, FlutterReply callback) {
    ShutDownEngine();
    callback(nil);
    didCallCallback = YES;
  }];
  EXPECT_TRUE([engine runWithEntrypoint:@"sendFooMessage"]);
  engine = nil;

  while (!didCallCallback) {
    [[NSRunLoop currentRunLoop] runUntilDate:[NSDate dateWithTimeIntervalSinceNow:0.1]];
  }
}

TEST_F(FlutterEngineTest, ResponseFromBackgroundThread) {
  FlutterEngine* engine = GetFlutterEngine();
  FlutterBasicMessageChannel* channel = [[FlutterBasicMessageChannel alloc]
         initWithName:@"foo"
      binaryMessenger:engine.binaryMessenger
                codec:[FlutterStandardMessageCodec sharedInstance]];
  __block BOOL didCallCallback = NO;
  [channel setMessageHandler:^(id message, FlutterReply callback) {
    dispatch_async(dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_DEFAULT, 0), ^{
      callback(nil);
      dispatch_async(dispatch_get_main_queue(), ^{
        didCallCallback = YES;
      });
    });
  }];
  EXPECT_TRUE([engine runWithEntrypoint:@"sendFooMessage"]);

  while (!didCallCallback) {
    [[NSRunLoop currentRunLoop] runUntilDate:[NSDate dateWithTimeIntervalSinceNow:0.1]];
  }
}

TEST_F(FlutterEngineTest, ThreadSynchronizerNotBlockingRasterThreadAfterShutdown) {
  FlutterThreadSynchronizer* threadSynchronizer = [[FlutterThreadSynchronizer alloc] init];
  [threadSynchronizer shutdown];

  std::thread rasterThread([&threadSynchronizer] {
    [threadSynchronizer performCommitForView:kImplicitViewId
                                        size:CGSizeMake(100, 100)
                                      notify:^{
                                      }];
  });

  rasterThread.join();
}

TEST_F(FlutterEngineTest, ManageControllersIfInitiatedByController) {
  NSString* fixtures = @(flutter::testing::GetFixturesPath());
  FlutterDartProject* project = [[FlutterDartProject alloc]
      initWithAssetsPath:fixtures
             ICUDataPath:[fixtures stringByAppendingString:@"/icudtl.dat"]];

  FlutterEngine* engine;
  FlutterViewController* viewController1;

  @autoreleasepool {
    // Create FVC1.
    viewController1 = [[FlutterViewController alloc] initWithProject:project];
    EXPECT_EQ(viewController1.viewIdentifier, 0ll);

    engine = viewController1.engine;
    engine.viewController = nil;

    // Create FVC2 based on the same engine.
    FlutterViewController* viewController2 = [[FlutterViewController alloc] initWithEngine:engine
                                                                                   nibName:nil
                                                                                    bundle:nil];
    EXPECT_EQ(engine.viewController, viewController2);
  }
  // FVC2 is deallocated but FVC1 is retained.

  EXPECT_EQ(engine.viewController, nil);

  engine.viewController = viewController1;
  EXPECT_EQ(engine.viewController, viewController1);
  EXPECT_EQ(viewController1.viewIdentifier, 0ll);
}

TEST_F(FlutterEngineTest, ManageControllersIfInitiatedByEngine) {
  // Don't create the engine with `CreateMockFlutterEngine`, because it adds
  // additional references to FlutterViewControllers, which is crucial to this
  // test case.
  FlutterEngine* engine = [[FlutterEngine alloc] initWithName:@"io.flutter"
                                                      project:nil
                                       allowHeadlessExecution:NO];
  FlutterViewController* viewController1;

  @autoreleasepool {
    viewController1 = [[FlutterViewController alloc] initWithEngine:engine nibName:nil bundle:nil];
    EXPECT_EQ(viewController1.viewIdentifier, 0ll);
    EXPECT_EQ(engine.viewController, viewController1);

    engine.viewController = nil;

    FlutterViewController* viewController2 = [[FlutterViewController alloc] initWithEngine:engine
                                                                                   nibName:nil
                                                                                    bundle:nil];
    EXPECT_EQ(viewController2.viewIdentifier, 0ll);
    EXPECT_EQ(engine.viewController, viewController2);
  }
  // FVC2 is deallocated but FVC1 is retained.

  EXPECT_EQ(engine.viewController, nil);

  engine.viewController = viewController1;
  EXPECT_EQ(engine.viewController, viewController1);
  EXPECT_EQ(viewController1.viewIdentifier, 0ll);
}

TEST_F(FlutterEngineTest, RemovingViewDisposesCompositorResources) {
  NSString* fixtures = @(flutter::testing::GetFixturesPath());
  FlutterDartProject* project = [[FlutterDartProject alloc]
      initWithAssetsPath:fixtures
             ICUDataPath:[fixtures stringByAppendingString:@"/icudtl.dat"]];
  FlutterEngine* engine = [[FlutterEngine alloc] initWithName:@"test" project:project];

  FlutterViewController* viewController = [[FlutterViewController alloc] initWithEngine:engine
                                                                                nibName:nil
                                                                                 bundle:nil];
  [viewController loadView];
  [viewController viewDidLoad];
  viewController.flutterView.frame = CGRectMake(0, 0, 800, 600);

  EXPECT_TRUE([engine runWithEntrypoint:@"drawIntoAllViews"]);
  [engine.testThreadSynchronizer blockUntilFrameAvailable];
  EXPECT_EQ(engine.macOSCompositor->DebugNumViews(), 1u);

  engine.viewController = nil;
  EXPECT_EQ(engine.macOSCompositor->DebugNumViews(), 0u);

  [engine shutDownEngine];
  engine = nil;
}

TEST_F(FlutterEngineTest, HandlesTerminationRequest) {
  id engineMock = CreateMockFlutterEngine(nil);
  __block NSString* nextResponse = @"exit";
  __block BOOL triedToTerminate = NO;
  FlutterEngineTerminationHandler* terminationHandler =
      [[FlutterEngineTerminationHandler alloc] initWithEngine:engineMock
                                                   terminator:^(id sender) {
                                                     triedToTerminate = TRUE;
                                                     // Don't actually terminate, of course.
                                                   }];
  OCMStub([engineMock terminationHandler]).andReturn(terminationHandler);
  id binaryMessengerMock = OCMProtocolMock(@protocol(FlutterBinaryMessenger));
  OCMStub(  // NOLINT(google-objc-avoid-throwing-exception)
      [engineMock binaryMessenger])
      .andReturn(binaryMessengerMock);
  OCMStub([engineMock sendOnChannel:@"flutter/platform"
                            message:[OCMArg any]
                        binaryReply:[OCMArg any]])
      .andDo((^(NSInvocation* invocation) {
        [invocation retainArguments];
        FlutterBinaryReply callback;
        NSData* returnedMessage;
        [invocation getArgument:&callback atIndex:4];
        if ([nextResponse isEqualToString:@"error"]) {
          FlutterError* errorResponse = [FlutterError errorWithCode:@"Error"
                                                            message:@"Failed"
                                                            details:@"Details"];
          returnedMessage =
              [[FlutterJSONMethodCodec sharedInstance] encodeErrorEnvelope:errorResponse];
        } else {
          NSDictionary* responseDict = @{@"response" : nextResponse};
          returnedMessage =
              [[FlutterJSONMethodCodec sharedInstance] encodeSuccessEnvelope:responseDict];
        }
        callback(returnedMessage);
      }));
  __block NSString* calledAfterTerminate = @"";
  FlutterResult appExitResult = ^(id result) {
    NSDictionary* resultDict = result;
    calledAfterTerminate = resultDict[@"response"];
  };
  FlutterMethodCall* methodExitApplication =
      [FlutterMethodCall methodCallWithMethodName:@"System.exitApplication"
                                        arguments:@{@"type" : @"cancelable"}];

  // Always terminate when the binding isn't ready (which is the default).
  triedToTerminate = NO;
  calledAfterTerminate = @"";
  nextResponse = @"cancel";
  [engineMock handleMethodCall:methodExitApplication result:appExitResult];
  EXPECT_STREQ([calledAfterTerminate UTF8String], "");
  EXPECT_TRUE(triedToTerminate);

  // Once the binding is ready, handle the request.
  terminationHandler.acceptingRequests = YES;
  triedToTerminate = NO;
  calledAfterTerminate = @"";
  nextResponse = @"exit";
  [engineMock handleMethodCall:methodExitApplication result:appExitResult];
  EXPECT_STREQ([calledAfterTerminate UTF8String], "exit");
  EXPECT_TRUE(triedToTerminate);

  triedToTerminate = NO;
  calledAfterTerminate = @"";
  nextResponse = @"cancel";
  [engineMock handleMethodCall:methodExitApplication result:appExitResult];
  EXPECT_STREQ([calledAfterTerminate UTF8String], "cancel");
  EXPECT_FALSE(triedToTerminate);

  // Check that it doesn't crash on error.
  triedToTerminate = NO;
  calledAfterTerminate = @"";
  nextResponse = @"error";
  [engineMock handleMethodCall:methodExitApplication result:appExitResult];
  EXPECT_STREQ([calledAfterTerminate UTF8String], "");
  EXPECT_TRUE(triedToTerminate);
}

TEST_F(FlutterEngineTest, IgnoresTerminationRequestIfNotFlutterAppDelegate) {
  id<NSApplicationDelegate> previousDelegate = [[NSApplication sharedApplication] delegate];
  id<NSApplicationDelegate> plainDelegate = [[PlainAppDelegate alloc] init];
  [NSApplication sharedApplication].delegate = plainDelegate;

  // Creating the engine shouldn't fail here, even though the delegate isn't a
  // FlutterAppDelegate.
  CreateMockFlutterEngine(nil);

  // Asking to terminate the app should cancel.
  EXPECT_EQ([[[NSApplication sharedApplication] delegate] applicationShouldTerminate:NSApp],
            NSTerminateCancel);

  [NSApplication sharedApplication].delegate = previousDelegate;
}

TEST_F(FlutterEngineTest, HandleAccessibilityEvent) {
  __block BOOL announced = NO;
  id engineMock = CreateMockFlutterEngine(nil);

  OCMStub([engineMock announceAccessibilityMessage:[OCMArg any]
                                      withPriority:NSAccessibilityPriorityMedium])
      .andDo((^(NSInvocation* invocation) {
        announced = TRUE;
        [invocation retainArguments];
        NSString* message;
        [invocation getArgument:&message atIndex:2];
        EXPECT_EQ(message, @"error message");
      }));

  NSDictionary<NSString*, id>* annotatedEvent =
      @{@"type" : @"announce",
        @"data" : @{@"message" : @"error message"}};

  [engineMock handleAccessibilityEvent:annotatedEvent];

  EXPECT_TRUE(announced);
}

TEST_F(FlutterEngineTest, HandleLifecycleStates) API_AVAILABLE(macos(10.9)) {
  __block flutter::AppLifecycleState sentState;
  id engineMock = CreateMockFlutterEngine(nil);

  // Have to enumerate all the values because OCMStub can't capture
  // non-Objective-C object arguments.
  OCMStub([engineMock setApplicationState:flutter::AppLifecycleState::kDetached])
      .andDo((^(NSInvocation* invocation) {
        sentState = flutter::AppLifecycleState::kDetached;
      }));
  OCMStub([engineMock setApplicationState:flutter::AppLifecycleState::kResumed])
      .andDo((^(NSInvocation* invocation) {
        sentState = flutter::AppLifecycleState::kResumed;
      }));
  OCMStub([engineMock setApplicationState:flutter::AppLifecycleState::kInactive])
      .andDo((^(NSInvocation* invocation) {
        sentState = flutter::AppLifecycleState::kInactive;
      }));
  OCMStub([engineMock setApplicationState:flutter::AppLifecycleState::kHidden])
      .andDo((^(NSInvocation* invocation) {
        sentState = flutter::AppLifecycleState::kHidden;
      }));
  OCMStub([engineMock setApplicationState:flutter::AppLifecycleState::kPaused])
      .andDo((^(NSInvocation* invocation) {
        sentState = flutter::AppLifecycleState::kPaused;
      }));

  __block NSApplicationOcclusionState visibility = NSApplicationOcclusionStateVisible;
  id mockApplication = OCMPartialMock([NSApplication sharedApplication]);
  OCMStub((NSApplicationOcclusionState)[mockApplication occlusionState])
      .andDo(^(NSInvocation* invocation) {
        [invocation setReturnValue:&visibility];
      });

  NSNotification* willBecomeActive =
      [[NSNotification alloc] initWithName:NSApplicationWillBecomeActiveNotification
                                    object:nil
                                  userInfo:nil];
  NSNotification* willResignActive =
      [[NSNotification alloc] initWithName:NSApplicationWillResignActiveNotification
                                    object:nil
                                  userInfo:nil];

  NSNotification* didChangeOcclusionState;
  didChangeOcclusionState =
      [[NSNotification alloc] initWithName:NSApplicationDidChangeOcclusionStateNotification
                                    object:nil
                                  userInfo:nil];

  [engineMock handleDidChangeOcclusionState:didChangeOcclusionState];
  EXPECT_EQ(sentState, flutter::AppLifecycleState::kInactive);

  [engineMock handleWillBecomeActive:willBecomeActive];
  EXPECT_EQ(sentState, flutter::AppLifecycleState::kResumed);

  [engineMock handleWillResignActive:willResignActive];
  EXPECT_EQ(sentState, flutter::AppLifecycleState::kInactive);

  visibility = 0;
  [engineMock handleDidChangeOcclusionState:didChangeOcclusionState];
  EXPECT_EQ(sentState, flutter::AppLifecycleState::kHidden);

  [engineMock handleWillBecomeActive:willBecomeActive];
  EXPECT_EQ(sentState, flutter::AppLifecycleState::kHidden);

  [engineMock handleWillResignActive:willResignActive];
  EXPECT_EQ(sentState, flutter::AppLifecycleState::kHidden);

  [mockApplication stopMocking];
}

TEST_F(FlutterEngineTest, ForwardsPluginDelegateRegistration) {
  id<NSApplicationDelegate> previousDelegate = [[NSApplication sharedApplication] delegate];
  FakeLifecycleProvider* fakeAppDelegate = [[FakeLifecycleProvider alloc] init];
  [NSApplication sharedApplication].delegate = fakeAppDelegate;

  FakeAppDelegatePlugin* plugin = [[FakeAppDelegatePlugin alloc] init];
  FlutterEngine* engine = CreateMockFlutterEngine(nil);

  [[engine registrarForPlugin:@"TestPlugin"] addApplicationDelegate:plugin];

  EXPECT_TRUE([fakeAppDelegate hasDelegate:plugin]);

  [NSApplication sharedApplication].delegate = previousDelegate;
}

TEST_F(FlutterEngineTest, UnregistersPluginsOnEngineDestruction) {
  id<NSApplicationDelegate> previousDelegate = [[NSApplication sharedApplication] delegate];
  FakeLifecycleProvider* fakeAppDelegate = [[FakeLifecycleProvider alloc] init];
  [NSApplication sharedApplication].delegate = fakeAppDelegate;

  FakeAppDelegatePlugin* plugin = [[FakeAppDelegatePlugin alloc] init];

  @autoreleasepool {
    FlutterEngine* engine = [[FlutterEngine alloc] initWithName:@"test" project:nil];

    [[engine registrarForPlugin:@"TestPlugin"] addApplicationDelegate:plugin];
    EXPECT_TRUE([fakeAppDelegate hasDelegate:plugin]);
  }

  // When the engine is released, it should unregister any plugins it had
  // registered on its behalf.
  EXPECT_FALSE([fakeAppDelegate hasDelegate:plugin]);

  [NSApplication sharedApplication].delegate = previousDelegate;
}

TEST_F(FlutterEngineTest, RunWithEntrypointUpdatesDisplayConfig) {
  BOOL updated = NO;
  FlutterEngine* engine = GetFlutterEngine();
  auto original_update_displays = engine.embedderAPI.NotifyDisplayUpdate;
  engine.embedderAPI.NotifyDisplayUpdate = MOCK_ENGINE_PROC(
      NotifyDisplayUpdate, ([&updated, &original_update_displays](
                                auto engine, auto update_type, auto* displays, auto display_count) {
        updated = YES;
        return original_update_displays(engine, update_type, displays, display_count);
      }));

  EXPECT_TRUE([engine runWithEntrypoint:@"main"]);
  EXPECT_TRUE(updated);

  updated = NO;
  [[NSNotificationCenter defaultCenter]
      postNotificationName:NSApplicationDidChangeScreenParametersNotification
                    object:nil];
  EXPECT_TRUE(updated);
}

TEST_F(FlutterEngineTest, NotificationsUpdateDisplays) {
  BOOL updated = NO;
  FlutterEngine* engine = GetFlutterEngine();
  auto original_set_viewport_metrics = engine.embedderAPI.SendWindowMetricsEvent;
  engine.embedderAPI.SendWindowMetricsEvent = MOCK_ENGINE_PROC(
      SendWindowMetricsEvent,
      ([&updated, &original_set_viewport_metrics](auto engine, auto* window_metrics) {
        updated = YES;
        return original_set_viewport_metrics(engine, window_metrics);
      }));

  EXPECT_TRUE([engine runWithEntrypoint:@"main"]);

  updated = NO;
  [[NSNotificationCenter defaultCenter] postNotificationName:NSWindowDidChangeScreenNotification
                                                      object:nil];
  // No VC.
  EXPECT_FALSE(updated);

  FlutterViewController* viewController = [[FlutterViewController alloc] initWithEngine:engine
                                                                                nibName:nil
                                                                                 bundle:nil];
  [viewController loadView];
  viewController.flutterView.frame = CGRectMake(0, 0, 800, 600);

  [[NSNotificationCenter defaultCenter] postNotificationName:NSWindowDidChangeScreenNotification
                                                      object:nil];
  EXPECT_TRUE(updated);
}

TEST_F(FlutterEngineTest, DisplaySizeIsInPhysicalPixel) {
  NSString* fixtures = @(testing::GetFixturesPath());
  FlutterDartProject* project = [[FlutterDartProject alloc]
      initWithAssetsPath:fixtures
             ICUDataPath:[fixtures stringByAppendingString:@"/icudtl.dat"]];
  project.rootIsolateCreateCallback = FlutterEngineTest::IsolateCreateCallback;
  MockableFlutterEngine* engine = [[MockableFlutterEngine alloc] initWithName:@"foobar"
                                                                      project:project
                                                       allowHeadlessExecution:true];
  BOOL updated = NO;
  auto original_update_displays = engine.embedderAPI.NotifyDisplayUpdate;
  engine.embedderAPI.NotifyDisplayUpdate = MOCK_ENGINE_PROC(
      NotifyDisplayUpdate, ([&updated, &original_update_displays](
                                auto engine, auto update_type, auto* displays, auto display_count) {
        EXPECT_EQ(display_count, 1UL);
        EXPECT_EQ(displays->display_id, 10UL);
        EXPECT_EQ(displays->width, 60UL);
        EXPECT_EQ(displays->height, 80UL);
        EXPECT_EQ(displays->device_pixel_ratio, 2UL);
        updated = YES;
        return original_update_displays(engine, update_type, displays, display_count);
      }));
  EXPECT_TRUE([engine runWithEntrypoint:@"main"]);
  EXPECT_TRUE(updated);
  [engine shutDownEngine];
  engine = nil;
}

TEST_F(FlutterEngineTest, ReportsHourFormat) {
  __block BOOL expectedValue;

  // Set up mocks.
  id channelMock = OCMClassMock([FlutterBasicMessageChannel class]);
  OCMStub([channelMock messageChannelWithName:@"flutter/settings"
                              binaryMessenger:[OCMArg any]
                                        codec:[OCMArg any]])
      .andReturn(channelMock);
  OCMStub([channelMock sendMessage:[OCMArg any]]).andDo((^(NSInvocation* invocation) {
    __weak id message;
    [invocation getArgument:&message atIndex:2];
    EXPECT_EQ(message[@"alwaysUse24HourFormat"], @(expectedValue));
  }));

  id mockHourFormat = OCMClassMock([FlutterHourFormat class]);
  OCMStub([mockHourFormat isAlwaysUse24HourFormat]).andDo((^(NSInvocation* invocation) {
    [invocation setReturnValue:&expectedValue];
  }));

  id engineMock = CreateMockFlutterEngine(nil);

  // Verify the YES case.
  expectedValue = YES;
  EXPECT_TRUE([engineMock runWithEntrypoint:@"main"]);
  [engineMock shutDownEngine];

  // Verify the NO case.
  expectedValue = NO;
  EXPECT_TRUE([engineMock runWithEntrypoint:@"main"]);
  [engineMock shutDownEngine];

  // Clean up mocks.
  [mockHourFormat stopMocking];
  [engineMock stopMocking];
  [channelMock stopMocking];
}

}  // namespace flutter::testing

// NOLINTEND(clang-analyzer-core.StackAddressEscape)
