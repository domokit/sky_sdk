// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import <OCMock/OCMock.h>
#import <XCTest/XCTest.h>

#import "flutter/fml/thread.h"
#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterMacros.h"
#import "flutter/shell/platform/darwin/ios/framework/Headers/FlutterPlatformViews.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterPlatformViews_Internal.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterSemanticsScrollView.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterViewController_Internal.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/accessibility_bridge.h"
#import "flutter/shell/platform/darwin/ios/platform_view_ios.h"

FLUTTER_ASSERT_ARC

@class MockPlatformView;
__weak static MockPlatformView* gMockPlatformView = nil;

@interface MockPlatformView : UIView
@end
@implementation MockPlatformView

- (instancetype)init {
  self = [super init];
  if (self) {
    gMockPlatformView = self;
  }
  return self;
}

- (void)dealloc {
  gMockPlatformView = nil;
}

@end

@interface MockFlutterPlatformView : NSObject <FlutterPlatformView>
@property(nonatomic, strong) UIView* view;
@end

@implementation MockFlutterPlatformView

- (instancetype)init {
  if (self = [super init]) {
    _view = [[MockPlatformView alloc] init];
  }
  return self;
}

@end

@interface MockFlutterPlatformFactory : NSObject <FlutterPlatformViewFactory>
@end

@implementation MockFlutterPlatformFactory
- (NSObject<FlutterPlatformView>*)createWithFrame:(CGRect)frame
                                   viewIdentifier:(int64_t)viewId
                                        arguments:(id _Nullable)args {
  return [[MockFlutterPlatformView alloc] init];
}

@end

namespace flutter {
namespace {
class MockDelegate : public PlatformView::Delegate {
 public:
  void OnPlatformViewCreated(std::unique_ptr<Surface> surface) override {}
  void OnPlatformViewDestroyed() override {}
  void OnPlatformViewScheduleFrame() override {}
  void OnPlatformViewAddView(int64_t view_id,
                             const ViewportMetrics& viewport_metrics,
                             AddViewCallback callback) override {}
  void OnPlatformViewRemoveView(int64_t view_id, RemoveViewCallback callback) override {}
  void OnPlatformViewSetNextFrameCallback(const fml::closure& closure) override {}
  void OnPlatformViewSetViewportMetrics(int64_t view_id, const ViewportMetrics& metrics) override {}
  const flutter::Settings& OnPlatformViewGetSettings() const override { return settings_; }
  void OnPlatformViewDispatchPlatformMessage(std::unique_ptr<PlatformMessage> message) override {}
  void OnPlatformViewDispatchPointerDataPacket(std::unique_ptr<PointerDataPacket> packet) override {
  }
  void OnPlatformViewDispatchSemanticsAction(int32_t id,
                                             SemanticsAction action,
                                             fml::MallocMapping args) override {}
  void OnPlatformViewSetSemanticsEnabled(bool enabled) override {}
  void OnPlatformViewSetAccessibilityFeatures(int32_t flags) override {}
  void OnPlatformViewRegisterTexture(std::shared_ptr<Texture> texture) override {}
  void OnPlatformViewUnregisterTexture(int64_t texture_id) override {}
  void OnPlatformViewMarkTextureFrameAvailable(int64_t texture_id) override {}

  void LoadDartDeferredLibrary(intptr_t loading_unit_id,
                               std::unique_ptr<const fml::Mapping> snapshot_data,
                               std::unique_ptr<const fml::Mapping> snapshot_instructions) override {
  }
  void LoadDartDeferredLibraryError(intptr_t loading_unit_id,
                                    const std::string error_message,
                                    bool transient) override {}
  void UpdateAssetResolverByType(std::unique_ptr<flutter::AssetResolver> updated_asset_resolver,
                                 flutter::AssetResolver::AssetResolverType type) override {}

  flutter::Settings settings_;
};

class MockIosDelegate : public AccessibilityBridge::IosDelegate {
 public:
  bool IsFlutterViewControllerPresentingModalViewController(
      FlutterViewController* view_controller) override {
    return result_IsFlutterViewControllerPresentingModalViewController_;
  };

  void PostAccessibilityNotification(UIAccessibilityNotifications notification,
                                     id argument) override {
    if (on_PostAccessibilityNotification_) {
      on_PostAccessibilityNotification_(notification, argument);
    }
  }
  std::function<void(UIAccessibilityNotifications, id)> on_PostAccessibilityNotification_;
  bool result_IsFlutterViewControllerPresentingModalViewController_ = false;
};
}  // namespace
}  // namespace flutter

namespace {
fml::RefPtr<fml::TaskRunner> CreateNewThread(const std::string& name) {
  auto thread = std::make_unique<fml::Thread>(name);
  auto runner = thread->GetTaskRunner();
  return runner;
}
}  // namespace

@interface AccessibilityBridgeTest : XCTestCase
@end

@implementation AccessibilityBridgeTest

- (void)testCreate {
  flutter::MockDelegate mock_delegate;
  auto thread_task_runner = CreateNewThread("AccessibilityBridgeTest");
  flutter::TaskRunners runners(/*label=*/self.name.UTF8String,
                               /*platform=*/thread_task_runner,
                               /*raster=*/thread_task_runner,
                               /*ui=*/thread_task_runner,
                               /*io=*/thread_task_runner);
  auto platform_view = std::make_unique<flutter::PlatformViewIOS>(
      /*delegate=*/mock_delegate,
      /*rendering_api=*/mock_delegate.settings_.enable_impeller
          ? flutter::IOSRenderingAPI::kMetal
          : flutter::IOSRenderingAPI::kSoftware,
      /*platform_views_controller=*/nil,
      /*task_runners=*/runners,
      /*worker_task_runner=*/nil,
      /*is_gpu_disabled_sync_switch=*/std::make_shared<fml::SyncSwitch>());
  auto bridge =
      std::make_unique<flutter::AccessibilityBridge>(/*view=*/nil,
                                                     /*platform_view=*/platform_view.get(),
                                                     /*platform_views_controller=*/nil);
  XCTAssertTrue(bridge.get());
}

- (void)testUpdateSemanticsEmpty {
  flutter::MockDelegate mock_delegate;
  auto thread_task_runner = CreateNewThread("AccessibilityBridgeTest");
  flutter::TaskRunners runners(/*label=*/self.name.UTF8String,
                               /*platform=*/thread_task_runner,
                               /*raster=*/thread_task_runner,
                               /*ui=*/thread_task_runner,
                               /*io=*/thread_task_runner);
  auto platform_view = std::make_unique<flutter::PlatformViewIOS>(
      /*delegate=*/mock_delegate,
      /*rendering_api=*/mock_delegate.settings_.enable_impeller
          ? flutter::IOSRenderingAPI::kMetal
          : flutter::IOSRenderingAPI::kSoftware,
      /*platform_views_controller=*/nil,
      /*task_runners=*/runners,
      /*worker_task_runner=*/nil,
      /*is_gpu_disabled_sync_switch=*/std::make_shared<fml::SyncSwitch>());
  id mockFlutterView = OCMClassMock([FlutterView class]);
  id mockFlutterViewController = OCMClassMock([FlutterViewController class]);
  OCMStub([mockFlutterViewController viewIfLoaded]).andReturn(mockFlutterView);
  OCMExpect([mockFlutterView setAccessibilityElements:[OCMArg isNil]]);
  auto bridge =
      std::make_unique<flutter::AccessibilityBridge>(/*view_controller=*/mockFlutterViewController,
                                                     /*platform_view=*/platform_view.get(),
                                                     /*platform_views_controller=*/nil);
  flutter::SemanticsNodeUpdates nodes;
  flutter::CustomAccessibilityActionUpdates actions;
  bridge->UpdateSemantics(/*nodes=*/nodes, /*actions=*/actions);
  OCMVerifyAll(mockFlutterView);
}

- (void)testUpdateSemanticsOneNode {
  flutter::MockDelegate mock_delegate;
  auto thread_task_runner = CreateNewThread("AccessibilityBridgeTest");
  flutter::TaskRunners runners(/*label=*/self.name.UTF8String,
                               /*platform=*/thread_task_runner,
                               /*raster=*/thread_task_runner,
                               /*ui=*/thread_task_runner,
                               /*io=*/thread_task_runner);
  auto platform_view = std::make_unique<flutter::PlatformViewIOS>(
      /*delegate=*/mock_delegate,
      /*rendering_api=*/mock_delegate.settings_.enable_impeller
          ? flutter::IOSRenderingAPI::kMetal
          : flutter::IOSRenderingAPI::kSoftware,
      /*platform_views_controller=*/nil,
      /*task_runners=*/runners,
      /*worker_task_runner=*/nil,
      /*is_gpu_disabled_sync_switch=*/std::make_shared<fml::SyncSwitch>());
  id mockFlutterView = OCMClassMock([FlutterView class]);
  id mockFlutterViewController = OCMClassMock([FlutterViewController class]);
  OCMStub([mockFlutterViewController view]).andReturn(mockFlutterView);
  std::string label = "some label";

  __block auto bridge =
      std::make_unique<flutter::AccessibilityBridge>(/*view_controller=*/mockFlutterViewController,
                                                     /*platform_view=*/platform_view.get(),
                                                     /*platform_views_controller=*/nil);

  OCMExpect([mockFlutterView setAccessibilityElements:[OCMArg checkWithBlock:^BOOL(NSArray* value) {
                               if ([value count] != 1) {
                                 return NO;
                               } else {
                                 SemanticsObjectContainer* container = value[0];
                                 SemanticsObject* object = container.semanticsObject;
                                 return object.uid == kRootNodeId &&
                                        object.bridge.get() == bridge.get() &&
                                        object.node.label == label;
                               }
                             }]]);

  flutter::SemanticsNodeUpdates nodes;
  flutter::SemanticsNode semantics_node;
  semantics_node.id = kRootNodeId;
  semantics_node.label = label;
  nodes[kRootNodeId] = semantics_node;
  flutter::CustomAccessibilityActionUpdates actions;
  bridge->UpdateSemantics(/*nodes=*/nodes, /*actions=*/actions);
  OCMVerifyAll(mockFlutterView);
}

- (void)testIsVoiceOverRunning {
  flutter::MockDelegate mock_delegate;
  auto thread_task_runner = CreateNewThread("AccessibilityBridgeTest");
  flutter::TaskRunners runners(/*label=*/self.name.UTF8String,
                               /*platform=*/thread_task_runner,
                               /*raster=*/thread_task_runner,
                               /*ui=*/thread_task_runner,
                               /*io=*/thread_task_runner);
  auto platform_view = std::make_unique<flutter::PlatformViewIOS>(
      /*delegate=*/mock_delegate,
      /*rendering_api=*/mock_delegate.settings_.enable_impeller
          ? flutter::IOSRenderingAPI::kMetal
          : flutter::IOSRenderingAPI::kSoftware,
      /*platform_views_controller=*/nil,
      /*task_runners=*/runners,
      /*worker_task_runner=*/nil,
      /*is_gpu_disabled_sync_switch=*/std::make_shared<fml::SyncSwitch>());
  id mockFlutterView = OCMClassMock([FlutterView class]);
  id mockFlutterViewController = OCMClassMock([FlutterViewController class]);
  OCMStub([mockFlutterViewController view]).andReturn(mockFlutterView);
  OCMStub([mockFlutterViewController isVoiceOverRunning]).andReturn(YES);

  __block auto bridge =
      std::make_unique<flutter::AccessibilityBridge>(/*view_controller=*/mockFlutterViewController,
                                                     /*platform_view=*/platform_view.get(),
                                                     /*platform_views_controller=*/nil);

  XCTAssertTrue(bridge->isVoiceOverRunning());
}

- (void)testSemanticsDeallocated {
  @autoreleasepool {
    flutter::MockDelegate mock_delegate;
    auto thread_task_runner = CreateNewThread("AccessibilityBridgeTest");
    flutter::TaskRunners runners(/*label=*/self.name.UTF8String,
                                 /*platform=*/thread_task_runner,
                                 /*raster=*/thread_task_runner,
                                 /*ui=*/thread_task_runner,
                                 /*io=*/thread_task_runner);

    FlutterPlatformViewsController* flutterPlatformViewsController =
        [[FlutterPlatformViewsController alloc] init];
    flutterPlatformViewsController.taskRunner = thread_task_runner;
    auto platform_view = std::make_unique<flutter::PlatformViewIOS>(
        /*delegate=*/mock_delegate,
        /*rendering_api=*/mock_delegate.settings_.enable_impeller
            ? flutter::IOSRenderingAPI::kMetal
            : flutter::IOSRenderingAPI::kSoftware,
        /*platform_views_controller=*/flutterPlatformViewsController,
        /*task_runners=*/runners,
        /*worker_task_runner=*/nil,
        /*is_gpu_disabled_sync_switch=*/std::make_shared<fml::SyncSwitch>());
    id mockFlutterView = OCMClassMock([FlutterView class]);
    id mockFlutterViewController = OCMClassMock([FlutterViewController class]);
    OCMStub([mockFlutterViewController view]).andReturn(mockFlutterView);
    std::string label = "some label";
    flutterPlatformViewsController.flutterView = mockFlutterView;

    MockFlutterPlatformFactory* factory = [[MockFlutterPlatformFactory alloc] init];
    [flutterPlatformViewsController
                     registerViewFactory:factory
                                  withId:@"MockFlutterPlatformView"
        gestureRecognizersBlockingPolicy:FlutterPlatformViewGestureRecognizersBlockingPolicyEager];
    FlutterResult result = ^(id result) {
    };
    [flutterPlatformViewsController
        onMethodCall:[FlutterMethodCall methodCallWithMethodName:@"create"
                                                       arguments:@{
                                                         @"id" : @2,
                                                         @"viewType" : @"MockFlutterPlatformView"
                                                       }]
              result:result];

    auto bridge = std::make_unique<flutter::AccessibilityBridge>(
        /*view_controller=*/mockFlutterViewController,
        /*platform_view=*/platform_view.get(),
        /*platform_views_controller=*/flutterPlatformViewsController);

    flutter::SemanticsNodeUpdates nodes;
    flutter::SemanticsNode semantics_node;
    semantics_node.id = 2;
    semantics_node.platformViewId = 2;
    semantics_node.label = label;
    nodes[kRootNodeId] = semantics_node;
    flutter::CustomAccessibilityActionUpdates actions;
    bridge->UpdateSemantics(/*nodes=*/nodes, /*actions=*/actions);
    XCTAssertNotNil(gMockPlatformView);
    [flutterPlatformViewsController reset];
  }
  XCTAssertNil(gMockPlatformView);
}

- (void)testSemanticsDeallocatedWithoutLoadingView {
  id engine = OCMClassMock([FlutterEngine class]);
  FlutterViewController* flutterViewController =
      [[FlutterViewController alloc] initWithEngine:engine nibName:nil bundle:nil];
  @autoreleasepool {
    flutter::MockDelegate mock_delegate;
    auto thread_task_runner = CreateNewThread("AccessibilityBridgeTest");
    flutter::TaskRunners runners(/*label=*/self.name.UTF8String,
                                 /*platform=*/thread_task_runner,
                                 /*raster=*/thread_task_runner,
                                 /*ui=*/thread_task_runner,
                                 /*io=*/thread_task_runner);

    FlutterPlatformViewsController* flutterPlatformViewsController =
        [[FlutterPlatformViewsController alloc] init];
    flutterPlatformViewsController.taskRunner = thread_task_runner;
    auto platform_view = std::make_unique<flutter::PlatformViewIOS>(
        /*delegate=*/mock_delegate,
        /*rendering_api=*/mock_delegate.settings_.enable_impeller
            ? flutter::IOSRenderingAPI::kMetal
            : flutter::IOSRenderingAPI::kSoftware,
        /*platform_views_controller=*/flutterPlatformViewsController,
        /*task_runners=*/runners,
        /*worker_task_runner=*/nil,
        /*is_gpu_disabled_sync_switch=*/std::make_shared<fml::SyncSwitch>());

    MockFlutterPlatformFactory* factory = [[MockFlutterPlatformFactory alloc] init];
    [flutterPlatformViewsController
                     registerViewFactory:factory
                                  withId:@"MockFlutterPlatformView"
        gestureRecognizersBlockingPolicy:FlutterPlatformViewGestureRecognizersBlockingPolicyEager];
    FlutterResult result = ^(id result) {
    };
    [flutterPlatformViewsController
        onMethodCall:[FlutterMethodCall methodCallWithMethodName:@"create"
                                                       arguments:@{
                                                         @"id" : @2,
                                                         @"viewType" : @"MockFlutterPlatformView"
                                                       }]
              result:result];

    auto bridge = std::make_unique<flutter::AccessibilityBridge>(
        /*view_controller=*/flutterViewController,
        /*platform_view=*/platform_view.get(),
        /*platform_views_controller=*/flutterPlatformViewsController);

    XCTAssertNotNil(gMockPlatformView);
    [flutterPlatformViewsController reset];
    platform_view->NotifyDestroyed();
  }
  XCTAssertNil(gMockPlatformView);
  XCTAssertNil(flutterViewController.viewIfLoaded);
  [flutterViewController deregisterNotifications];
}

- (void)testReplacedSemanticsDoesNotCleanupChildren {
  flutter::MockDelegate mock_delegate;
  auto thread_task_runner = CreateNewThread("AccessibilityBridgeTest");
  flutter::TaskRunners runners(/*label=*/self.name.UTF8String,
                               /*platform=*/thread_task_runner,
                               /*raster=*/thread_task_runner,
                               /*ui=*/thread_task_runner,
                               /*io=*/thread_task_runner);

  FlutterPlatformViewsController* flutterPlatformViewsController =
      [[FlutterPlatformViewsController alloc] init];
  flutterPlatformViewsController.taskRunner = thread_task_runner;
  auto platform_view = std::make_unique<flutter::PlatformViewIOS>(
      /*delegate=*/mock_delegate,
      /*rendering_api=*/mock_delegate.settings_.enable_impeller
          ? flutter::IOSRenderingAPI::kMetal
          : flutter::IOSRenderingAPI::kSoftware,
      /*platform_views_controller=*/flutterPlatformViewsController,
      /*task_runners=*/runners,
      /*worker_task_runner=*/nil,
      /*is_gpu_disabled_sync_switch=*/std::make_shared<fml::SyncSwitch>());
  id engine = OCMClassMock([FlutterEngine class]);
  id mockFlutterViewController = OCMClassMock([FlutterViewController class]);
  FlutterView* flutterView = [[FlutterView alloc] initWithDelegate:engine
                                                            opaque:YES
                                                   enableWideGamut:NO];
  OCMStub([mockFlutterViewController view]).andReturn(flutterView);
  std::string label = "some label";
  auto bridge = std::make_unique<flutter::AccessibilityBridge>(
      /*view_controller=*/mockFlutterViewController,
      /*platform_view=*/platform_view.get(),
      /*platform_views_controller=*/flutterPlatformViewsController);
  @autoreleasepool {
    flutter::SemanticsNodeUpdates nodes;
    flutter::SemanticsNode parent;
    parent.id = 0;
    parent.rect = SkRect::MakeXYWH(0, 0, 100, 200);
    parent.label = "label";
    parent.value = "value";
    parent.hint = "hint";

    flutter::SemanticsNode node;
    node.id = 1;
    node.rect = SkRect::MakeXYWH(0, 0, 100, 200);
    node.label = "label";
    node.value = "value";
    node.hint = "hint";
    node.scrollExtentMax = 100.0;
    node.scrollPosition = 0.0;
    parent.childrenInTraversalOrder.push_back(1);
    parent.childrenInHitTestOrder.push_back(1);

    flutter::SemanticsNode child;
    child.id = 2;
    child.rect = SkRect::MakeXYWH(0, 0, 100, 200);
    child.label = "label";
    child.value = "value";
    child.hint = "hint";
    node.childrenInTraversalOrder.push_back(2);
    node.childrenInHitTestOrder.push_back(2);

    nodes[0] = parent;
    nodes[1] = node;
    nodes[2] = child;
    flutter::CustomAccessibilityActionUpdates actions;
    bridge->UpdateSemantics(/*nodes=*/nodes, /*actions=*/actions);

    // Add implicit scroll from node 1 to cause replacement.
    flutter::SemanticsNodeUpdates new_nodes;
    flutter::SemanticsNode new_node;
    new_node.id = 1;
    new_node.rect = SkRect::MakeXYWH(0, 0, 100, 200);
    new_node.flags = static_cast<int32_t>(flutter::SemanticsFlags::kHasImplicitScrolling);
    new_node.actions = flutter::kHorizontalScrollSemanticsActions;
    new_node.label = "label";
    new_node.value = "value";
    new_node.hint = "hint";
    new_node.scrollExtentMax = 100.0;
    new_node.scrollPosition = 0.0;
    new_node.childrenInTraversalOrder.push_back(2);
    new_node.childrenInHitTestOrder.push_back(2);

    new_nodes[1] = new_node;
    bridge->UpdateSemantics(/*nodes=*/new_nodes, /*actions=*/actions);
  }
  /// The old node should be deallocated at this moment. Procced to check
  /// accessibility tree integrity.
  id rootContainer = flutterView.accessibilityElements[0];
  XCTAssertTrue([rootContainer accessibilityElementCount] ==
                2);  // one for root, one for scrollable.
  id scrollableContainer = [rootContainer accessibilityElementAtIndex:1];
  XCTAssertTrue([scrollableContainer accessibilityElementCount] ==
                2);  // one for scrollable, one for scrollable child.
  id child = [scrollableContainer accessibilityElementAtIndex:1];
  /// Replacing node 1 should not accidentally clean up its child's container.
  XCTAssertNotNil([child accessibilityContainer]);
}

- (void)testScrollableSemanticsDeallocated {
  flutter::MockDelegate mock_delegate;
  auto thread_task_runner = CreateNewThread("AccessibilityBridgeTest");
  flutter::TaskRunners runners(/*label=*/self.name.UTF8String,
                               /*platform=*/thread_task_runner,
                               /*raster=*/thread_task_runner,
                               /*ui=*/thread_task_runner,
                               /*io=*/thread_task_runner);

  FlutterPlatformViewsController* flutterPlatformViewsController =
      [[FlutterPlatformViewsController alloc] init];
  flutterPlatformViewsController.taskRunner = thread_task_runner;
  auto platform_view = std::make_unique<flutter::PlatformViewIOS>(
      /*delegate=*/mock_delegate,
      /*rendering_api=*/mock_delegate.settings_.enable_impeller
          ? flutter::IOSRenderingAPI::kMetal
          : flutter::IOSRenderingAPI::kSoftware,
      /*platform_views_controller=*/flutterPlatformViewsController,
      /*task_runners=*/runners,
      /*worker_task_runner=*/nil,
      /*is_gpu_disabled_sync_switch=*/std::make_shared<fml::SyncSwitch>());
  id engine = OCMClassMock([FlutterEngine class]);
  id mockFlutterViewController = OCMClassMock([FlutterViewController class]);
  FlutterView* flutterView = [[FlutterView alloc] initWithDelegate:engine
                                                            opaque:YES
                                                   enableWideGamut:NO];
  OCMStub([mockFlutterViewController view]).andReturn(flutterView);
  std::string label = "some label";
  @autoreleasepool {
    auto bridge = std::make_unique<flutter::AccessibilityBridge>(
        /*view_controller=*/mockFlutterViewController,
        /*platform_view=*/platform_view.get(),
        /*platform_views_controller=*/flutterPlatformViewsController);

    flutter::SemanticsNodeUpdates nodes;
    flutter::SemanticsNode parent;
    parent.id = 0;
    parent.rect = SkRect::MakeXYWH(0, 0, 100, 200);
    parent.label = "label";
    parent.value = "value";
    parent.hint = "hint";

    flutter::SemanticsNode node;
    node.id = 1;
    node.flags = static_cast<int32_t>(flutter::SemanticsFlags::kHasImplicitScrolling);
    node.actions = flutter::kHorizontalScrollSemanticsActions;
    node.rect = SkRect::MakeXYWH(0, 0, 100, 200);
    node.label = "label";
    node.value = "value";
    node.hint = "hint";
    node.scrollExtentMax = 100.0;
    node.scrollPosition = 0.0;
    parent.childrenInTraversalOrder.push_back(1);
    parent.childrenInHitTestOrder.push_back(1);
    nodes[0] = parent;
    nodes[1] = node;
    flutter::CustomAccessibilityActionUpdates actions;
    bridge->UpdateSemantics(/*nodes=*/nodes, /*actions=*/actions);
    XCTAssertTrue([flutterView.subviews count] == 1);
    XCTAssertTrue([flutterView.subviews[0] isKindOfClass:[FlutterSemanticsScrollView class]]);
    XCTAssertTrue([flutterView.subviews[0].accessibilityLabel isEqualToString:@"label"]);

    // Remove the scrollable from the tree.
    flutter::SemanticsNodeUpdates new_nodes;
    flutter::SemanticsNode new_parent;
    new_parent.id = 0;
    new_parent.rect = SkRect::MakeXYWH(0, 0, 100, 200);
    new_parent.label = "label";
    new_parent.value = "value";
    new_parent.hint = "hint";
    new_nodes[0] = new_parent;
    bridge->UpdateSemantics(/*nodes=*/new_nodes, /*actions=*/actions);
  }
  XCTAssertTrue([flutterView.subviews count] == 0);
}

- (void)testBridgeReplacesSemanticsNode {
  flutter::MockDelegate mock_delegate;
  auto thread_task_runner = CreateNewThread("AccessibilityBridgeTest");
  flutter::TaskRunners runners(/*label=*/self.name.UTF8String,
                               /*platform=*/thread_task_runner,
                               /*raster=*/thread_task_runner,
                               /*ui=*/thread_task_runner,
                               /*io=*/thread_task_runner);

  FlutterPlatformViewsController* flutterPlatformViewsController =
      [[FlutterPlatformViewsController alloc] init];
  auto platform_view = std::make_unique<flutter::PlatformViewIOS>(
      /*delegate=*/mock_delegate,
      /*rendering_api=*/mock_delegate.settings_.enable_impeller
          ? flutter::IOSRenderingAPI::kMetal
          : flutter::IOSRenderingAPI::kSoftware,
      /*platform_views_controller=*/flutterPlatformViewsController,
      /*task_runners=*/runners,
      /*worker_task_runner=*/nil,
      /*is_gpu_disabled_sync_switch=*/std::make_shared<fml::SyncSwitch>());
  id engine = OCMClassMock([FlutterEngine class]);
  id mockFlutterViewController = OCMClassMock([FlutterViewController class]);
  FlutterView* flutterView = [[FlutterView alloc] initWithDelegate:engine
                                                            opaque:YES
                                                   enableWideGamut:NO];
  OCMStub([mockFlutterViewController view]).andReturn(flutterView);
  std::string label = "some label";
  @autoreleasepool {
    auto bridge = std::make_unique<flutter::AccessibilityBridge>(
        /*view_controller=*/mockFlutterViewController,
        /*platform_view=*/platform_view.get(),
        /*platform_views_controller=*/flutterPlatformViewsController);

    flutter::SemanticsNodeUpdates nodes;
    flutter::SemanticsNode parent;
    parent.id = 0;
    parent.rect = SkRect::MakeXYWH(0, 0, 100, 200);
    parent.label = "label";
    parent.value = "value";
    parent.hint = "hint";

    flutter::SemanticsNode node;
    node.id = 1;
    node.flags = static_cast<int32_t>(flutter::SemanticsFlags::kHasImplicitScrolling);
    node.actions = flutter::kHorizontalScrollSemanticsActions;
    node.rect = SkRect::MakeXYWH(0, 0, 100, 200);
    node.label = "label";
    node.value = "value";
    node.hint = "hint";
    node.scrollExtentMax = 100.0;
    node.scrollPosition = 0.0;
    parent.childrenInTraversalOrder.push_back(1);
    parent.childrenInHitTestOrder.push_back(1);
    nodes[0] = parent;
    nodes[1] = node;
    flutter::CustomAccessibilityActionUpdates actions;
    bridge->UpdateSemantics(/*nodes=*/nodes, /*actions=*/actions);
    XCTAssertTrue([flutterView.subviews count] == 1);
    XCTAssertTrue([flutterView.subviews[0] isKindOfClass:[FlutterSemanticsScrollView class]]);
    XCTAssertTrue([flutterView.subviews[0].accessibilityLabel isEqualToString:@"label"]);

    // Remove implicit scroll from node 1.
    flutter::SemanticsNodeUpdates new_nodes;
    flutter::SemanticsNode new_node;
    new_node.id = 1;
    new_node.rect = SkRect::MakeXYWH(0, 0, 100, 200);
    new_node.label = "label";
    new_node.value = "value";
    new_node.hint = "hint";
    new_node.scrollExtentMax = 100.0;
    new_node.scrollPosition = 0.0;
    new_nodes[1] = new_node;
    bridge->UpdateSemantics(/*nodes=*/new_nodes, /*actions=*/actions);
  }
  XCTAssertTrue([flutterView.subviews count] == 0);
}

- (void)testAnnouncesRouteChanges {
  flutter::MockDelegate mock_delegate;
  auto thread_task_runner = CreateNewThread("AccessibilityBridgeTest");
  flutter::TaskRunners runners(/*label=*/self.name.UTF8String,
                               /*platform=*/thread_task_runner,
                               /*raster=*/thread_task_runner,
                               /*ui=*/thread_task_runner,
                               /*io=*/thread_task_runner);
  auto platform_view = std::make_unique<flutter::PlatformViewIOS>(
      /*delegate=*/mock_delegate,
      /*rendering_api=*/mock_delegate.settings_.enable_impeller
          ? flutter::IOSRenderingAPI::kMetal
          : flutter::IOSRenderingAPI::kSoftware,
      /*platform_views_controller=*/nil,
      /*task_runners=*/runners,
      /*worker_task_runner=*/nil,
      /*is_gpu_disabled_sync_switch=*/std::make_shared<fml::SyncSwitch>());
  id mockFlutterView = OCMClassMock([FlutterView class]);
  id mockFlutterViewController = OCMClassMock([FlutterViewController class]);
  OCMStub([mockFlutterViewController view]).andReturn(mockFlutterView);

  NSMutableArray<NSDictionary<NSString*, id>*>* accessibility_notifications =
      [[NSMutableArray alloc] init];
  auto ios_delegate = std::make_unique<flutter::MockIosDelegate>();
  ios_delegate->on_PostAccessibilityNotification_ =
      [accessibility_notifications](UIAccessibilityNotifications notification, id argument) {
        [accessibility_notifications addObject:@{
          @"notification" : @(notification),
          @"argument" : argument ? argument : [NSNull null],
        }];
      };
  __block auto bridge =
      std::make_unique<flutter::AccessibilityBridge>(/*view_controller=*/mockFlutterViewController,
                                                     /*platform_view=*/platform_view.get(),
                                                     /*platform_views_controller=*/nil,
                                                     /*ios_delegate=*/std::move(ios_delegate));

  flutter::CustomAccessibilityActionUpdates actions;
  flutter::SemanticsNodeUpdates nodes;

  flutter::SemanticsNode node1;
  node1.id = 1;
  node1.label = "node1";
  node1.flags = static_cast<int32_t>(flutter::SemanticsFlags::kScopesRoute);
  node1.childrenInTraversalOrder = {2, 3};
  node1.childrenInHitTestOrder = {2, 3};
  nodes[node1.id] = node1;
  flutter::SemanticsNode node2;
  node2.id = 2;
  node2.label = "node2";
  nodes[node2.id] = node2;
  flutter::SemanticsNode node3;
  node3.id = 3;
  node3.flags = static_cast<int32_t>(flutter::SemanticsFlags::kNamesRoute);
  node3.label = "node3";
  nodes[node3.id] = node3;
  flutter::SemanticsNode root_node;
  root_node.id = kRootNodeId;
  root_node.flags = static_cast<int32_t>(flutter::SemanticsFlags::kScopesRoute);
  root_node.childrenInTraversalOrder = {1};
  root_node.childrenInHitTestOrder = {1};
  nodes[root_node.id] = root_node;
  bridge->UpdateSemantics(/*nodes=*/nodes, /*actions=*/actions);

  XCTAssertEqual([accessibility_notifications count], 1ul);
  XCTAssertEqualObjects(accessibility_notifications[0][@"argument"], @"node3");
  XCTAssertEqual([accessibility_notifications[0][@"notification"] unsignedIntValue],
                 UIAccessibilityScreenChangedNotification);
}

- (void)testRadioButtonIsNotSwitchButton {
  flutter::MockDelegate mock_delegate;
  auto thread_task_runner = CreateNewThread("AccessibilityBridgeTest");
  flutter::TaskRunners runners(/*label=*/self.name.UTF8String,
                               /*platform=*/thread_task_runner,
                               /*raster=*/thread_task_runner,
                               /*ui=*/thread_task_runner,
                               /*io=*/thread_task_runner);
  auto platform_view = std::make_unique<flutter::PlatformViewIOS>(
      /*delegate=*/mock_delegate,
      /*rendering_api=*/mock_delegate.settings_.enable_impeller
          ? flutter::IOSRenderingAPI::kMetal
          : flutter::IOSRenderingAPI::kSoftware,
      /*platform_views_controller=*/nil,
      /*task_runners=*/runners,
      /*worker_task_runner=*/nil,
      /*is_gpu_disabled_sync_switch=*/std::make_shared<fml::SyncSwitch>());
  id engine = OCMClassMock([FlutterEngine class]);
  id mockFlutterViewController = OCMClassMock([FlutterViewController class]);
  FlutterView* flutterView = [[FlutterView alloc] initWithDelegate:engine
                                                            opaque:YES
                                                   enableWideGamut:NO];
  OCMStub([mockFlutterViewController view]).andReturn(flutterView);
  auto ios_delegate = std::make_unique<flutter::MockIosDelegate>();
  __block auto bridge =
      std::make_unique<flutter::AccessibilityBridge>(/*view_controller=*/mockFlutterViewController,
                                                     /*platform_view=*/platform_view.get(),
                                                     /*platform_views_controller=*/nil,
                                                     /*ios_delegate=*/std::move(ios_delegate));

  flutter::CustomAccessibilityActionUpdates actions;
  flutter::SemanticsNodeUpdates nodes;

  flutter::SemanticsNode root_node;
  root_node.id = kRootNodeId;
  root_node.flags = static_cast<int32_t>(flutter::SemanticsFlags::kIsInMutuallyExclusiveGroup) |
                    static_cast<int32_t>(flutter::SemanticsFlags::kIsEnabled) |
                    static_cast<int32_t>(flutter::SemanticsFlags::kHasCheckedState) |
                    static_cast<int32_t>(flutter::SemanticsFlags::kHasEnabledState);
  nodes[root_node.id] = root_node;
  bridge->UpdateSemantics(/*nodes=*/nodes, /*actions=*/actions);

  SemanticsObjectContainer* rootContainer = flutterView.accessibilityElements[0];
  FlutterSemanticsObject* rootNode = [rootContainer accessibilityElementAtIndex:0];

  XCTAssertTrue((rootNode.accessibilityTraits & UIAccessibilityTraitButton) > 0);
  XCTAssertNil(rootNode.accessibilityValue);
}

- (void)testLayoutChangeWithNonAccessibilityElement {
  flutter::MockDelegate mock_delegate;
  auto thread_task_runner = CreateNewThread("AccessibilityBridgeTest");
  flutter::TaskRunners runners(/*label=*/self.name.UTF8String,
                               /*platform=*/thread_task_runner,
                               /*raster=*/thread_task_runner,
                               /*ui=*/thread_task_runner,
                               /*io=*/thread_task_runner);
  auto platform_view = std::make_unique<flutter::PlatformViewIOS>(
      /*delegate=*/mock_delegate,
      /*rendering_api=*/mock_delegate.settings_.enable_impeller
          ? flutter::IOSRenderingAPI::kMetal
          : flutter::IOSRenderingAPI::kSoftware,
      /*platform_views_controller=*/nil,
      /*task_runners=*/runners,
      /*worker_task_runner=*/nil,
      /*is_gpu_disabled_sync_switch=*/std::make_shared<fml::SyncSwitch>());
  id mockFlutterView = OCMClassMock([FlutterView class]);
  id mockFlutterViewController = OCMClassMock([FlutterViewController class]);
  OCMStub([mockFlutterViewController view]).andReturn(mockFlutterView);

  NSMutableArray<NSDictionary<NSString*, id>*>* accessibility_notifications =
      [[NSMutableArray alloc] init];
  auto ios_delegate = std::make_unique<flutter::MockIosDelegate>();
  ios_delegate->on_PostAccessibilityNotification_ =
      [accessibility_notifications](UIAccessibilityNotifications notification, id argument) {
        [accessibility_notifications addObject:@{
          @"notification" : @(notification),
          @"argument" : argument ? argument : [NSNull null],
        }];
      };
  __block auto bridge =
      std::make_unique<flutter::AccessibilityBridge>(/*view_controller=*/mockFlutterViewController,
                                                     /*platform_view=*/platform_view.get(),
                                                     /*platform_views_controller=*/nil,
                                                     /*ios_delegate=*/std::move(ios_delegate));

  flutter::CustomAccessibilityActionUpdates actions;
  flutter::SemanticsNodeUpdates nodes;

  flutter::SemanticsNode node1;
  node1.id = 1;
  node1.label = "node1";
  node1.childrenInTraversalOrder = {2, 3};
  node1.childrenInHitTestOrder = {2, 3};
  nodes[node1.id] = node1;
  flutter::SemanticsNode node2;
  node2.id = 2;
  node2.label = "node2";
  nodes[node2.id] = node2;
  flutter::SemanticsNode node3;
  node3.id = 3;
  node3.label = "node3";
  nodes[node3.id] = node3;
  flutter::SemanticsNode root_node;
  root_node.id = kRootNodeId;
  root_node.label = "root";
  root_node.childrenInTraversalOrder = {1};
  root_node.childrenInHitTestOrder = {1};
  nodes[root_node.id] = root_node;
  bridge->UpdateSemantics(/*nodes=*/nodes, /*actions=*/actions);

  // Simulates the focusing on the node 1.
  bridge->AccessibilityObjectDidBecomeFocused(1);

  // In this update, we make node 1 unfocusable and trigger the
  // layout change. The accessibility bridge should send layoutchange
  // notification with the first focusable node under node 1
  flutter::CustomAccessibilityActionUpdates new_actions;
  flutter::SemanticsNodeUpdates new_nodes;

  flutter::SemanticsNode new_node1;
  new_node1.id = 1;
  new_node1.childrenInTraversalOrder = {2};
  new_node1.childrenInHitTestOrder = {2};
  new_nodes[new_node1.id] = new_node1;
  bridge->UpdateSemantics(/*nodes=*/new_nodes, /*actions=*/new_actions);

  XCTAssertEqual([accessibility_notifications count], 1ul);
  SemanticsObject* focusObject = accessibility_notifications[0][@"argument"];
  // Since node 1 is no longer focusable (no label), it will focus node 2 instead.
  XCTAssertEqual([focusObject uid], 2);
  XCTAssertEqual([accessibility_notifications[0][@"notification"] unsignedIntValue],
                 UIAccessibilityLayoutChangedNotification);
}

- (void)testLayoutChangeDoesCallNativeAccessibility {
  flutter::MockDelegate mock_delegate;
  auto thread_task_runner = CreateNewThread("AccessibilityBridgeTest");
  flutter::TaskRunners runners(/*label=*/self.name.UTF8String,
                               /*platform=*/thread_task_runner,
                               /*raster=*/thread_task_runner,
                               /*ui=*/thread_task_runner,
                               /*io=*/thread_task_runner);
  auto platform_view = std::make_unique<flutter::PlatformViewIOS>(
      /*delegate=*/mock_delegate,
      /*rendering_api=*/mock_delegate.settings_.enable_impeller
          ? flutter::IOSRenderingAPI::kMetal
          : flutter::IOSRenderingAPI::kSoftware,
      /*platform_views_controller=*/nil,
      /*task_runners=*/runners,
      /*worker_task_runner=*/nil,
      /*is_gpu_disabled_sync_switch=*/std::make_shared<fml::SyncSwitch>());
  id mockFlutterView = OCMClassMock([FlutterView class]);
  id mockFlutterViewController = OCMClassMock([FlutterViewController class]);
  OCMStub([mockFlutterViewController view]).andReturn(mockFlutterView);

  NSMutableArray<NSDictionary<NSString*, id>*>* accessibility_notifications =
      [[NSMutableArray alloc] init];
  auto ios_delegate = std::make_unique<flutter::MockIosDelegate>();
  ios_delegate->on_PostAccessibilityNotification_ =
      [accessibility_notifications](UIAccessibilityNotifications notification, id argument) {
        [accessibility_notifications addObject:@{
          @"notification" : @(notification),
          @"argument" : argument ? argument : [NSNull null],
        }];
      };
  __block auto bridge =
      std::make_unique<flutter::AccessibilityBridge>(/*view_controller=*/mockFlutterViewController,
                                                     /*platform_view=*/platform_view.get(),
                                                     /*platform_views_controller=*/nil,
                                                     /*ios_delegate=*/std::move(ios_delegate));

  flutter::CustomAccessibilityActionUpdates actions;
  flutter::SemanticsNodeUpdates nodes;

  flutter::SemanticsNode node1;
  node1.id = 1;
  node1.label = "node1";
  nodes[node1.id] = node1;
  flutter::SemanticsNode root_node;
  root_node.id = kRootNodeId;
  root_node.label = "root";
  root_node.flags = static_cast<int32_t>(flutter::SemanticsFlags::kHasImplicitScrolling);
  root_node.childrenInTraversalOrder = {1};
  root_node.childrenInHitTestOrder = {1};
  nodes[root_node.id] = root_node;
  bridge->UpdateSemantics(/*nodes=*/nodes, /*actions=*/actions);

  // Simulates the focusing on the node 0.
  bridge->AccessibilityObjectDidBecomeFocused(0);

  // Remove node 1 to trigger a layout change notification
  flutter::CustomAccessibilityActionUpdates new_actions;
  flutter::SemanticsNodeUpdates new_nodes;

  flutter::SemanticsNode new_root_node;
  new_root_node.id = kRootNodeId;
  new_root_node.label = "root";
  new_root_node.flags = static_cast<int32_t>(flutter::SemanticsFlags::kHasImplicitScrolling);
  new_nodes[new_root_node.id] = new_root_node;
  bridge->UpdateSemantics(/*nodes=*/new_nodes, /*actions=*/new_actions);

  XCTAssertEqual([accessibility_notifications count], 1ul);
  id focusObject = accessibility_notifications[0][@"argument"];

  // Make sure the focused item is not specificed when it stays the same.
  // See: https://github.com/flutter/flutter/issues/104176
  XCTAssertEqualObjects(focusObject, [NSNull null]);
  XCTAssertEqual([accessibility_notifications[0][@"notification"] unsignedIntValue],
                 UIAccessibilityLayoutChangedNotification);
}

- (void)testLayoutChangeDoesCallNativeAccessibilityWhenFocusChanged {
  flutter::MockDelegate mock_delegate;
  auto thread_task_runner = CreateNewThread("AccessibilityBridgeTest");
  flutter::TaskRunners runners(/*label=*/self.name.UTF8String,
                               /*platform=*/thread_task_runner,
                               /*raster=*/thread_task_runner,
                               /*ui=*/thread_task_runner,
                               /*io=*/thread_task_runner);
  auto platform_view = std::make_unique<flutter::PlatformViewIOS>(
      /*delegate=*/mock_delegate,
      /*rendering_api=*/mock_delegate.settings_.enable_impeller
          ? flutter::IOSRenderingAPI::kMetal
          : flutter::IOSRenderingAPI::kSoftware,
      /*platform_views_controller=*/nil,
      /*task_runners=*/runners,
      /*worker_task_runner=*/nil,
      /*is_gpu_disabled_sync_switch=*/std::make_shared<fml::SyncSwitch>());
  id mockFlutterView = OCMClassMock([FlutterView class]);
  id mockFlutterViewController = OCMClassMock([FlutterViewController class]);
  OCMStub([mockFlutterViewController view]).andReturn(mockFlutterView);

  NSMutableArray<NSDictionary<NSString*, id>*>* accessibility_notifications =
      [[NSMutableArray alloc] init];
  auto ios_delegate = std::make_unique<flutter::MockIosDelegate>();
  ios_delegate->on_PostAccessibilityNotification_ =
      [accessibility_notifications](UIAccessibilityNotifications notification, id argument) {
        [accessibility_notifications addObject:@{
          @"notification" : @(notification),
          @"argument" : argument ? argument : [NSNull null],
        }];
      };
  __block auto bridge =
      std::make_unique<flutter::AccessibilityBridge>(/*view_controller=*/mockFlutterViewController,
                                                     /*platform_view=*/platform_view.get(),
                                                     /*platform_views_controller=*/nil,
                                                     /*ios_delegate=*/std::move(ios_delegate));

  flutter::CustomAccessibilityActionUpdates actions;
  flutter::SemanticsNodeUpdates nodes;

  flutter::SemanticsNode node1;
  node1.id = 1;
  node1.label = "node1";
  nodes[node1.id] = node1;
  flutter::SemanticsNode root_node;
  root_node.id = kRootNodeId;
  root_node.label = "root";
  root_node.flags = static_cast<int32_t>(flutter::SemanticsFlags::kHasImplicitScrolling);
  root_node.childrenInTraversalOrder = {1};
  root_node.childrenInHitTestOrder = {1};
  nodes[root_node.id] = root_node;
  bridge->UpdateSemantics(/*nodes=*/nodes, /*actions=*/actions);

  // Simulates the focusing on the node 1.
  bridge->AccessibilityObjectDidBecomeFocused(1);

  // Remove node 1 to trigger a layout change notification, and focus should be one root
  flutter::CustomAccessibilityActionUpdates new_actions;
  flutter::SemanticsNodeUpdates new_nodes;

  flutter::SemanticsNode new_root_node;
  new_root_node.id = kRootNodeId;
  new_root_node.label = "root";
  new_root_node.flags = static_cast<int32_t>(flutter::SemanticsFlags::kHasImplicitScrolling);
  new_nodes[new_root_node.id] = new_root_node;
  bridge->UpdateSemantics(/*nodes=*/new_nodes, /*actions=*/new_actions);

  XCTAssertEqual([accessibility_notifications count], 1ul);
  SemanticsObject* focusObject2 = accessibility_notifications[0][@"argument"];

  // Bridge should ask accessibility to focus on root because node 1 is moved from screen.
  XCTAssertTrue([focusObject2 isKindOfClass:[FlutterSemanticsScrollView class]]);
  XCTAssertEqual([accessibility_notifications[0][@"notification"] unsignedIntValue],
                 UIAccessibilityLayoutChangedNotification);
}

- (void)testScrollableSemanticsContainerReturnsCorrectChildren {
  flutter::MockDelegate mock_delegate;
  auto thread_task_runner = CreateNewThread("AccessibilityBridgeTest");
  flutter::TaskRunners runners(/*label=*/self.name.UTF8String,
                               /*platform=*/thread_task_runner,
                               /*raster=*/thread_task_runner,
                               /*ui=*/thread_task_runner,
                               /*io=*/thread_task_runner);
  auto platform_view = std::make_unique<flutter::PlatformViewIOS>(
      /*delegate=*/mock_delegate,
      /*rendering_api=*/mock_delegate.settings_.enable_impeller
          ? flutter::IOSRenderingAPI::kMetal
          : flutter::IOSRenderingAPI::kSoftware,
      /*platform_views_controller=*/nil,
      /*task_runners=*/runners,
      /*worker_task_runner=*/nil,
      /*is_gpu_disabled_sync_switch=*/std::make_shared<fml::SyncSwitch>());
  id mockFlutterView = OCMClassMock([FlutterView class]);
  id mockFlutterViewController = OCMClassMock([FlutterViewController class]);
  OCMStub([mockFlutterViewController view]).andReturn(mockFlutterView);

  OCMExpect([mockFlutterView
      setAccessibilityElements:[OCMArg checkWithBlock:^BOOL(NSArray* value) {
        if ([value count] != 1) {
          return NO;
        }
        SemanticsObjectContainer* container = value[0];
        SemanticsObject* object = container.semanticsObject;
        FlutterScrollableSemanticsObject* scrollable =
            (FlutterScrollableSemanticsObject*)object.children[0];
        id nativeScrollable = scrollable.nativeAccessibility;
        SemanticsObjectContainer* scrollableContainer = [nativeScrollable accessibilityContainer];
        return [scrollableContainer indexOfAccessibilityElement:nativeScrollable] == 1;
      }]]);
  auto ios_delegate = std::make_unique<flutter::MockIosDelegate>();
  __block auto bridge =
      std::make_unique<flutter::AccessibilityBridge>(/*view_controller=*/mockFlutterViewController,
                                                     /*platform_view=*/platform_view.get(),
                                                     /*platform_views_controller=*/nil,
                                                     /*ios_delegate=*/std::move(ios_delegate));

  flutter::CustomAccessibilityActionUpdates actions;
  flutter::SemanticsNodeUpdates nodes;

  flutter::SemanticsNode node1;
  node1.id = 1;
  node1.label = "node1";
  node1.flags = static_cast<int32_t>(flutter::SemanticsFlags::kHasImplicitScrolling);
  nodes[node1.id] = node1;
  flutter::SemanticsNode root_node;
  root_node.id = kRootNodeId;
  root_node.label = "root";
  root_node.childrenInTraversalOrder = {1};
  root_node.childrenInHitTestOrder = {1};
  nodes[root_node.id] = root_node;
  bridge->UpdateSemantics(/*nodes=*/nodes, /*actions=*/actions);
  OCMVerifyAll(mockFlutterView);
}

- (void)testAnnouncesRouteChangesAndLayoutChangeInOneUpdate {
  flutter::MockDelegate mock_delegate;
  auto thread_task_runner = CreateNewThread("AccessibilityBridgeTest");
  flutter::TaskRunners runners(/*label=*/self.name.UTF8String,
                               /*platform=*/thread_task_runner,
                               /*raster=*/thread_task_runner,
                               /*ui=*/thread_task_runner,
                               /*io=*/thread_task_runner);
  auto platform_view = std::make_unique<flutter::PlatformViewIOS>(
      /*delegate=*/mock_delegate,
      /*rendering_api=*/mock_delegate.settings_.enable_impeller
          ? flutter::IOSRenderingAPI::kMetal
          : flutter::IOSRenderingAPI::kSoftware,
      /*platform_views_controller=*/nil,
      /*task_runners=*/runners,
      /*worker_task_runner=*/nil,
      /*is_gpu_disabled_sync_switch=*/std::make_shared<fml::SyncSwitch>());
  id mockFlutterView = OCMClassMock([FlutterView class]);
  id mockFlutterViewController = OCMClassMock([FlutterViewController class]);
  OCMStub([mockFlutterViewController view]).andReturn(mockFlutterView);

  NSMutableArray<NSDictionary<NSString*, id>*>* accessibility_notifications =
      [[NSMutableArray alloc] init];
  auto ios_delegate = std::make_unique<flutter::MockIosDelegate>();
  ios_delegate->on_PostAccessibilityNotification_ =
      [accessibility_notifications](UIAccessibilityNotifications notification, id argument) {
        [accessibility_notifications addObject:@{
          @"notification" : @(notification),
          @"argument" : argument ? argument : [NSNull null],
        }];
      };
  __block auto bridge =
      std::make_unique<flutter::AccessibilityBridge>(/*view_controller=*/mockFlutterViewController,
                                                     /*platform_view=*/platform_view.get(),
                                                     /*platform_views_controller=*/nil,
                                                     /*ios_delegate=*/std::move(ios_delegate));

  flutter::CustomAccessibilityActionUpdates actions;
  flutter::SemanticsNodeUpdates nodes;

  flutter::SemanticsNode node1;
  node1.id = 1;
  node1.label = "node1";
  node1.flags = static_cast<int32_t>(flutter::SemanticsFlags::kScopesRoute) |
                static_cast<int32_t>(flutter::SemanticsFlags::kNamesRoute);
  nodes[node1.id] = node1;
  flutter::SemanticsNode node3;
  node3.id = 3;
  node3.label = "node3";
  nodes[node3.id] = node3;
  flutter::SemanticsNode root_node;
  root_node.id = kRootNodeId;
  root_node.label = "root";
  root_node.childrenInTraversalOrder = {1, 3};
  root_node.childrenInHitTestOrder = {1, 3};
  nodes[root_node.id] = root_node;
  bridge->UpdateSemantics(/*nodes=*/nodes, /*actions=*/actions);

  XCTAssertEqual([accessibility_notifications count], 1ul);
  XCTAssertEqualObjects(accessibility_notifications[0][@"argument"], @"node1");
  XCTAssertEqual([accessibility_notifications[0][@"notification"] unsignedIntValue],
                 UIAccessibilityScreenChangedNotification);

  // Simulates the focusing on the node 0.
  bridge->AccessibilityObjectDidBecomeFocused(0);

  flutter::SemanticsNodeUpdates new_nodes;

  flutter::SemanticsNode new_node1;
  new_node1.id = 1;
  new_node1.label = "new_node1";
  new_node1.flags = static_cast<int32_t>(flutter::SemanticsFlags::kScopesRoute) |
                    static_cast<int32_t>(flutter::SemanticsFlags::kNamesRoute);
  new_node1.childrenInTraversalOrder = {2};
  new_node1.childrenInHitTestOrder = {2};
  new_nodes[new_node1.id] = new_node1;
  flutter::SemanticsNode new_node2;
  new_node2.id = 2;
  new_node2.label = "new_node2";
  new_node2.flags = static_cast<int32_t>(flutter::SemanticsFlags::kScopesRoute) |
                    static_cast<int32_t>(flutter::SemanticsFlags::kNamesRoute);
  new_nodes[new_node2.id] = new_node2;
  flutter::SemanticsNode new_root_node;
  new_root_node.id = kRootNodeId;
  new_root_node.label = "root";
  new_root_node.childrenInTraversalOrder = {1};
  new_root_node.childrenInHitTestOrder = {1};
  new_nodes[new_root_node.id] = new_root_node;
  bridge->UpdateSemantics(/*nodes=*/new_nodes, /*actions=*/actions);
  XCTAssertEqual([accessibility_notifications count], 3ul);
  XCTAssertEqualObjects(accessibility_notifications[1][@"argument"], @"new_node2");
  XCTAssertEqual([accessibility_notifications[1][@"notification"] unsignedIntValue],
                 UIAccessibilityScreenChangedNotification);
  SemanticsObject* focusObject = accessibility_notifications[2][@"argument"];
  XCTAssertEqual([focusObject uid], 0);
  XCTAssertEqual([accessibility_notifications[2][@"notification"] unsignedIntValue],
                 UIAccessibilityLayoutChangedNotification);
}

- (void)testAnnouncesRouteChangesWhenAddAdditionalRoute {
  flutter::MockDelegate mock_delegate;
  auto thread_task_runner = CreateNewThread("AccessibilityBridgeTest");
  flutter::TaskRunners runners(/*label=*/self.name.UTF8String,
                               /*platform=*/thread_task_runner,
                               /*raster=*/thread_task_runner,
                               /*ui=*/thread_task_runner,
                               /*io=*/thread_task_runner);
  auto platform_view = std::make_unique<flutter::PlatformViewIOS>(
      /*delegate=*/mock_delegate,
      /*rendering_api=*/mock_delegate.settings_.enable_impeller
          ? flutter::IOSRenderingAPI::kMetal
          : flutter::IOSRenderingAPI::kSoftware,
      /*platform_views_controller=*/nil,
      /*task_runners=*/runners,
      /*worker_task_runner=*/nil,
      /*is_gpu_disabled_sync_switch=*/std::make_shared<fml::SyncSwitch>());
  id mockFlutterView = OCMClassMock([FlutterView class]);
  id mockFlutterViewController = OCMClassMock([FlutterViewController class]);
  OCMStub([mockFlutterViewController view]).andReturn(mockFlutterView);

  NSMutableArray<NSDictionary<NSString*, id>*>* accessibility_notifications =
      [[NSMutableArray alloc] init];
  auto ios_delegate = std::make_unique<flutter::MockIosDelegate>();
  ios_delegate->on_PostAccessibilityNotification_ =
      [accessibility_notifications](UIAccessibilityNotifications notification, id argument) {
        [accessibility_notifications addObject:@{
          @"notification" : @(notification),
          @"argument" : argument ? argument : [NSNull null],
        }];
      };
  __block auto bridge =
      std::make_unique<flutter::AccessibilityBridge>(/*view_controller=*/mockFlutterViewController,
                                                     /*platform_view=*/platform_view.get(),
                                                     /*platform_views_controller=*/nil,
                                                     /*ios_delegate=*/std::move(ios_delegate));

  flutter::CustomAccessibilityActionUpdates actions;
  flutter::SemanticsNodeUpdates nodes;

  flutter::SemanticsNode node1;
  node1.id = 1;
  node1.label = "node1";
  node1.flags = static_cast<int32_t>(flutter::SemanticsFlags::kScopesRoute) |
                static_cast<int32_t>(flutter::SemanticsFlags::kNamesRoute);
  nodes[node1.id] = node1;
  flutter::SemanticsNode root_node;
  root_node.id = kRootNodeId;
  root_node.flags = static_cast<int32_t>(flutter::SemanticsFlags::kScopesRoute);
  root_node.childrenInTraversalOrder = {1};
  root_node.childrenInHitTestOrder = {1};
  nodes[root_node.id] = root_node;
  bridge->UpdateSemantics(/*nodes=*/nodes, /*actions=*/actions);

  XCTAssertEqual([accessibility_notifications count], 1ul);
  XCTAssertEqualObjects(accessibility_notifications[0][@"argument"], @"node1");
  XCTAssertEqual([accessibility_notifications[0][@"notification"] unsignedIntValue],
                 UIAccessibilityScreenChangedNotification);

  flutter::SemanticsNodeUpdates new_nodes;

  flutter::SemanticsNode new_node1;
  new_node1.id = 1;
  new_node1.label = "new_node1";
  new_node1.flags = static_cast<int32_t>(flutter::SemanticsFlags::kScopesRoute) |
                    static_cast<int32_t>(flutter::SemanticsFlags::kNamesRoute);
  new_node1.childrenInTraversalOrder = {2};
  new_node1.childrenInHitTestOrder = {2};
  new_nodes[new_node1.id] = new_node1;
  flutter::SemanticsNode new_node2;
  new_node2.id = 2;
  new_node2.label = "new_node2";
  new_node2.flags = static_cast<int32_t>(flutter::SemanticsFlags::kScopesRoute) |
                    static_cast<int32_t>(flutter::SemanticsFlags::kNamesRoute);
  new_nodes[new_node2.id] = new_node2;
  flutter::SemanticsNode new_root_node;
  new_root_node.id = kRootNodeId;
  new_root_node.flags = static_cast<int32_t>(flutter::SemanticsFlags::kScopesRoute);
  new_root_node.childrenInTraversalOrder = {1};
  new_root_node.childrenInHitTestOrder = {1};
  new_nodes[new_root_node.id] = new_root_node;
  bridge->UpdateSemantics(/*nodes=*/new_nodes, /*actions=*/actions);
  XCTAssertEqual([accessibility_notifications count], 2ul);
  XCTAssertEqualObjects(accessibility_notifications[1][@"argument"], @"new_node2");
  XCTAssertEqual([accessibility_notifications[1][@"notification"] unsignedIntValue],
                 UIAccessibilityScreenChangedNotification);
}

- (void)testAnnouncesRouteChangesRemoveRouteInMiddle {
  flutter::MockDelegate mock_delegate;
  auto thread_task_runner = CreateNewThread("AccessibilityBridgeTest");
  flutter::TaskRunners runners(/*label=*/self.name.UTF8String,
                               /*platform=*/thread_task_runner,
                               /*raster=*/thread_task_runner,
                               /*ui=*/thread_task_runner,
                               /*io=*/thread_task_runner);
  auto platform_view = std::make_unique<flutter::PlatformViewIOS>(
      /*delegate=*/mock_delegate,
      /*rendering_api=*/mock_delegate.settings_.enable_impeller
          ? flutter::IOSRenderingAPI::kMetal
          : flutter::IOSRenderingAPI::kSoftware,
      /*platform_views_controller=*/nil,
      /*task_runners=*/runners,
      /*worker_task_runner=*/nil,
      /*is_gpu_disabled_sync_switch=*/std::make_shared<fml::SyncSwitch>());
  id mockFlutterView = OCMClassMock([FlutterView class]);
  id mockFlutterViewController = OCMClassMock([FlutterViewController class]);
  OCMStub([mockFlutterViewController view]).andReturn(mockFlutterView);

  NSMutableArray<NSDictionary<NSString*, id>*>* accessibility_notifications =
      [[NSMutableArray alloc] init];
  auto ios_delegate = std::make_unique<flutter::MockIosDelegate>();
  ios_delegate->on_PostAccessibilityNotification_ =
      [accessibility_notifications](UIAccessibilityNotifications notification, id argument) {
        [accessibility_notifications addObject:@{
          @"notification" : @(notification),
          @"argument" : argument ? argument : [NSNull null],
        }];
      };
  __block auto bridge =
      std::make_unique<flutter::AccessibilityBridge>(/*view_controller=*/mockFlutterViewController,
                                                     /*platform_view=*/platform_view.get(),
                                                     /*platform_views_controller=*/nil,
                                                     /*ios_delegate=*/std::move(ios_delegate));

  flutter::CustomAccessibilityActionUpdates actions;
  flutter::SemanticsNodeUpdates nodes;

  flutter::SemanticsNode node1;
  node1.id = 1;
  node1.label = "node1";
  node1.flags = static_cast<int32_t>(flutter::SemanticsFlags::kScopesRoute) |
                static_cast<int32_t>(flutter::SemanticsFlags::kNamesRoute);
  node1.childrenInTraversalOrder = {2};
  node1.childrenInHitTestOrder = {2};
  nodes[node1.id] = node1;
  flutter::SemanticsNode node2;
  node2.id = 2;
  node2.label = "node2";
  node2.flags = static_cast<int32_t>(flutter::SemanticsFlags::kScopesRoute) |
                static_cast<int32_t>(flutter::SemanticsFlags::kNamesRoute);
  nodes[node2.id] = node2;
  flutter::SemanticsNode root_node;
  root_node.id = kRootNodeId;
  root_node.flags = static_cast<int32_t>(flutter::SemanticsFlags::kScopesRoute);
  root_node.childrenInTraversalOrder = {1};
  root_node.childrenInHitTestOrder = {1};
  nodes[root_node.id] = root_node;
  bridge->UpdateSemantics(/*nodes=*/nodes, /*actions=*/actions);

  XCTAssertEqual([accessibility_notifications count], 1ul);
  XCTAssertEqualObjects(accessibility_notifications[0][@"argument"], @"node2");
  XCTAssertEqual([accessibility_notifications[0][@"notification"] unsignedIntValue],
                 UIAccessibilityScreenChangedNotification);

  flutter::SemanticsNodeUpdates new_nodes;

  flutter::SemanticsNode new_node1;
  new_node1.id = 1;
  new_node1.label = "new_node1";
  new_node1.childrenInTraversalOrder = {2};
  new_node1.childrenInHitTestOrder = {2};
  new_nodes[new_node1.id] = new_node1;
  flutter::SemanticsNode new_node2;
  new_node2.id = 2;
  new_node2.label = "new_node2";
  new_node2.flags = static_cast<int32_t>(flutter::SemanticsFlags::kScopesRoute) |
                    static_cast<int32_t>(flutter::SemanticsFlags::kNamesRoute);
  new_nodes[new_node2.id] = new_node2;
  flutter::SemanticsNode new_root_node;
  new_root_node.id = kRootNodeId;
  new_root_node.flags = static_cast<int32_t>(flutter::SemanticsFlags::kScopesRoute);
  new_root_node.childrenInTraversalOrder = {1};
  new_root_node.childrenInHitTestOrder = {1};
  new_nodes[new_root_node.id] = new_root_node;
  bridge->UpdateSemantics(/*nodes=*/new_nodes, /*actions=*/actions);
  XCTAssertEqual([accessibility_notifications count], 2ul);
  XCTAssertEqualObjects(accessibility_notifications[1][@"argument"], @"new_node2");
  XCTAssertEqual([accessibility_notifications[1][@"notification"] unsignedIntValue],
                 UIAccessibilityScreenChangedNotification);
}

- (void)testHandleEvent {
  flutter::MockDelegate mock_delegate;
  auto thread_task_runner = CreateNewThread("AccessibilityBridgeTest");
  flutter::TaskRunners runners(/*label=*/self.name.UTF8String,
                               /*platform=*/thread_task_runner,
                               /*raster=*/thread_task_runner,
                               /*ui=*/thread_task_runner,
                               /*io=*/thread_task_runner);
  auto platform_view = std::make_unique<flutter::PlatformViewIOS>(
      /*delegate=*/mock_delegate,
      /*rendering_api=*/mock_delegate.settings_.enable_impeller
          ? flutter::IOSRenderingAPI::kMetal
          : flutter::IOSRenderingAPI::kSoftware,
      /*platform_views_controller=*/nil,
      /*task_runners=*/runners,
      /*worker_task_runner=*/nil,
      /*is_gpu_disabled_sync_switch=*/std::make_shared<fml::SyncSwitch>());
  id mockFlutterView = OCMClassMock([FlutterView class]);
  id mockFlutterViewController = OCMClassMock([FlutterViewController class]);
  OCMStub([mockFlutterViewController view]).andReturn(mockFlutterView);

  NSMutableArray<NSDictionary<NSString*, id>*>* accessibility_notifications =
      [[NSMutableArray alloc] init];
  auto ios_delegate = std::make_unique<flutter::MockIosDelegate>();
  ios_delegate->on_PostAccessibilityNotification_ =
      [accessibility_notifications](UIAccessibilityNotifications notification, id argument) {
        [accessibility_notifications addObject:@{
          @"notification" : @(notification),
          @"argument" : argument ? argument : [NSNull null],
        }];
      };
  __block auto bridge =
      std::make_unique<flutter::AccessibilityBridge>(/*view_controller=*/mockFlutterViewController,
                                                     /*platform_view=*/platform_view.get(),
                                                     /*platform_views_controller=*/nil,
                                                     /*ios_delegate=*/std::move(ios_delegate));

  NSDictionary<NSString*, id>* annotatedEvent = @{@"type" : @"focus", @"nodeId" : @123};

  bridge->HandleEvent(annotatedEvent);

  XCTAssertEqual([accessibility_notifications count], 1ul);
  XCTAssertEqual([accessibility_notifications[0][@"notification"] unsignedIntValue],
                 UIAccessibilityLayoutChangedNotification);
}

- (void)testAccessibilityObjectDidBecomeFocused {
  flutter::MockDelegate mock_delegate;
  auto thread = std::make_unique<fml::Thread>("AccessibilityBridgeTest");
  auto thread_task_runner = thread->GetTaskRunner();
  flutter::TaskRunners runners(/*label=*/self.name.UTF8String,
                               /*platform=*/thread_task_runner,
                               /*raster=*/thread_task_runner,
                               /*ui=*/thread_task_runner,
                               /*io=*/thread_task_runner);
  id messenger = OCMProtocolMock(@protocol(FlutterBinaryMessenger));
  id engine = OCMClassMock([FlutterEngine class]);
  id flutterViewController = OCMClassMock([FlutterViewController class]);

  OCMStub([flutterViewController engine]).andReturn(engine);
  OCMStub([engine binaryMessenger]).andReturn(messenger);
  FlutterBinaryMessengerConnection connection = 123;
  OCMStub([messenger setMessageHandlerOnChannel:@"flutter/accessibility"
                           binaryMessageHandler:[OCMArg any]])
      .andReturn(connection);

  auto platform_view = std::make_unique<flutter::PlatformViewIOS>(
      /*delegate=*/mock_delegate,
      /*rendering_api=*/mock_delegate.settings_.enable_impeller
          ? flutter::IOSRenderingAPI::kMetal
          : flutter::IOSRenderingAPI::kSoftware,
      /*platform_views_controller=*/nil,
      /*task_runners=*/runners,
      /*worker_task_runner=*/nil,
      /*is_gpu_disabled_sync_switch=*/std::make_shared<fml::SyncSwitch>());
  fml::AutoResetWaitableEvent latch;
  thread_task_runner->PostTask([&] {
    platform_view->SetOwnerViewController(flutterViewController);
    auto bridge =
        std::make_unique<flutter::AccessibilityBridge>(/*view=*/nil,
                                                       /*platform_view=*/platform_view.get(),
                                                       /*platform_views_controller=*/nil);
    XCTAssertTrue(bridge.get());
    OCMVerify([messenger setMessageHandlerOnChannel:@"flutter/accessibility"
                               binaryMessageHandler:[OCMArg isNotNil]]);

    bridge->AccessibilityObjectDidBecomeFocused(123);

    NSDictionary<NSString*, id>* annotatedEvent = @{@"type" : @"didGainFocus", @"nodeId" : @123};
    NSData* encodedMessage = [[FlutterStandardMessageCodec sharedInstance] encode:annotatedEvent];

    OCMVerify([messenger sendOnChannel:@"flutter/accessibility" message:encodedMessage]);
    latch.Signal();
  });
  latch.Wait();

  [engine stopMocking];
}

- (void)testAnnouncesRouteChangesWhenNoNamesRoute {
  flutter::MockDelegate mock_delegate;
  auto thread_task_runner = CreateNewThread("AccessibilityBridgeTest");
  flutter::TaskRunners runners(/*label=*/self.name.UTF8String,
                               /*platform=*/thread_task_runner,
                               /*raster=*/thread_task_runner,
                               /*ui=*/thread_task_runner,
                               /*io=*/thread_task_runner);
  auto platform_view = std::make_unique<flutter::PlatformViewIOS>(
      /*delegate=*/mock_delegate,
      /*rendering_api=*/mock_delegate.settings_.enable_impeller
          ? flutter::IOSRenderingAPI::kMetal
          : flutter::IOSRenderingAPI::kSoftware,
      /*platform_views_controller=*/nil,
      /*task_runners=*/runners,
      /*worker_task_runner=*/nil,
      /*is_gpu_disabled_sync_switch=*/std::make_shared<fml::SyncSwitch>());
  id mockFlutterView = OCMClassMock([FlutterView class]);
  id mockFlutterViewController = OCMClassMock([FlutterViewController class]);
  OCMStub([mockFlutterViewController view]).andReturn(mockFlutterView);

  NSMutableArray<NSDictionary<NSString*, id>*>* accessibility_notifications =
      [[NSMutableArray alloc] init];
  auto ios_delegate = std::make_unique<flutter::MockIosDelegate>();
  ios_delegate->on_PostAccessibilityNotification_ =
      [accessibility_notifications](UIAccessibilityNotifications notification, id argument) {
        [accessibility_notifications addObject:@{
          @"notification" : @(notification),
          @"argument" : argument ? argument : [NSNull null],
        }];
      };
  __block auto bridge =
      std::make_unique<flutter::AccessibilityBridge>(/*view_controller=*/mockFlutterViewController,
                                                     /*platform_view=*/platform_view.get(),
                                                     /*platform_views_controller=*/nil,
                                                     /*ios_delegate=*/std::move(ios_delegate));

  flutter::CustomAccessibilityActionUpdates actions;
  flutter::SemanticsNodeUpdates nodes;

  flutter::SemanticsNode node1;
  node1.id = 1;
  node1.label = "node1";
  node1.flags = static_cast<int32_t>(flutter::SemanticsFlags::kScopesRoute) |
                static_cast<int32_t>(flutter::SemanticsFlags::kNamesRoute);
  node1.childrenInTraversalOrder = {2, 3};
  node1.childrenInHitTestOrder = {2, 3};
  nodes[node1.id] = node1;
  flutter::SemanticsNode node2;
  node2.id = 2;
  node2.label = "node2";
  nodes[node2.id] = node2;
  flutter::SemanticsNode node3;
  node3.id = 3;
  node3.label = "node3";
  nodes[node3.id] = node3;
  flutter::SemanticsNode root_node;
  root_node.id = kRootNodeId;
  root_node.childrenInTraversalOrder = {1};
  root_node.childrenInHitTestOrder = {1};
  nodes[root_node.id] = root_node;
  bridge->UpdateSemantics(/*nodes=*/nodes, /*actions=*/actions);

  // Notification should focus first focusable node, which is node1.
  XCTAssertEqual([accessibility_notifications count], 1ul);
  id focusObject = accessibility_notifications[0][@"argument"];
  XCTAssertTrue([focusObject isKindOfClass:[NSString class]]);
  XCTAssertEqualObjects(focusObject, @"node1");
  XCTAssertEqual([accessibility_notifications[0][@"notification"] unsignedIntValue],
                 UIAccessibilityScreenChangedNotification);
}

- (void)testAnnouncesLayoutChangeWithNilIfLastFocusIsRemoved {
  flutter::MockDelegate mock_delegate;
  auto thread_task_runner = CreateNewThread("AccessibilityBridgeTest");
  flutter::TaskRunners runners(/*label=*/self.name.UTF8String,
                               /*platform=*/thread_task_runner,
                               /*raster=*/thread_task_runner,
                               /*ui=*/thread_task_runner,
                               /*io=*/thread_task_runner);
  auto platform_view = std::make_unique<flutter::PlatformViewIOS>(
      /*delegate=*/mock_delegate,
      /*rendering_api=*/mock_delegate.settings_.enable_impeller
          ? flutter::IOSRenderingAPI::kMetal
          : flutter::IOSRenderingAPI::kSoftware,
      /*platform_views_controller=*/nil,
      /*task_runners=*/runners,
      /*worker_task_runner=*/nil,
      /*is_gpu_disabled_sync_switch=*/std::make_shared<fml::SyncSwitch>());
  id mockFlutterViewController = OCMClassMock([FlutterViewController class]);
  id mockFlutterView = OCMClassMock([FlutterView class]);
  OCMStub([mockFlutterViewController view]).andReturn(mockFlutterView);

  NSMutableArray<NSDictionary<NSString*, id>*>* accessibility_notifications =
      [[NSMutableArray alloc] init];
  auto ios_delegate = std::make_unique<flutter::MockIosDelegate>();
  ios_delegate->on_PostAccessibilityNotification_ =
      [accessibility_notifications](UIAccessibilityNotifications notification, id argument) {
        [accessibility_notifications addObject:@{
          @"notification" : @(notification),
          @"argument" : argument ? argument : [NSNull null],
        }];
      };
  __block auto bridge =
      std::make_unique<flutter::AccessibilityBridge>(/*view_controller=*/mockFlutterViewController,
                                                     /*platform_view=*/platform_view.get(),
                                                     /*platform_views_controller=*/nil,
                                                     /*ios_delegate=*/std::move(ios_delegate));

  flutter::CustomAccessibilityActionUpdates actions;
  flutter::SemanticsNodeUpdates first_update;

  flutter::SemanticsNode route_node;
  route_node.id = 1;
  route_node.label = "route";
  first_update[route_node.id] = route_node;
  flutter::SemanticsNode root_node;
  root_node.id = kRootNodeId;
  root_node.label = "root";
  root_node.childrenInTraversalOrder = {1};
  root_node.childrenInHitTestOrder = {1};
  first_update[root_node.id] = root_node;
  bridge->UpdateSemantics(/*nodes=*/first_update, /*actions=*/actions);

  XCTAssertEqual([accessibility_notifications count], 0ul);
  // Simulates the focusing on the node 1.
  bridge->AccessibilityObjectDidBecomeFocused(1);

  flutter::SemanticsNodeUpdates second_update;
  // Simulates the removal of the node 1
  flutter::SemanticsNode new_root_node;
  new_root_node.id = kRootNodeId;
  new_root_node.label = "root";
  second_update[root_node.id] = new_root_node;
  bridge->UpdateSemantics(/*nodes=*/second_update, /*actions=*/actions);
  SemanticsObject* focusObject = accessibility_notifications[0][@"argument"];
  // The node 1 was removed, so the bridge will set the focus object to root.
  XCTAssertEqual([focusObject uid], 0);
  XCTAssertEqualObjects([focusObject accessibilityLabel], @"root");
  XCTAssertEqual([accessibility_notifications[0][@"notification"] unsignedIntValue],
                 UIAccessibilityLayoutChangedNotification);
}

- (void)testAnnouncesLayoutChangeWithTheSameItemFocused {
  flutter::MockDelegate mock_delegate;
  auto thread_task_runner = CreateNewThread("AccessibilityBridgeTest");
  flutter::TaskRunners runners(/*label=*/self.name.UTF8String,
                               /*platform=*/thread_task_runner,
                               /*raster=*/thread_task_runner,
                               /*ui=*/thread_task_runner,
                               /*io=*/thread_task_runner);
  auto platform_view = std::make_unique<flutter::PlatformViewIOS>(
      /*delegate=*/mock_delegate,
      /*rendering_api=*/mock_delegate.settings_.enable_impeller
          ? flutter::IOSRenderingAPI::kMetal
          : flutter::IOSRenderingAPI::kSoftware,
      /*platform_views_controller=*/nil,
      /*task_runners=*/runners,
      /*worker_task_runner=*/nil,
      /*is_gpu_disabled_sync_switch=*/std::make_shared<fml::SyncSwitch>());
  id mockFlutterViewController = OCMClassMock([FlutterViewController class]);
  id mockFlutterView = OCMClassMock([FlutterView class]);
  OCMStub([mockFlutterViewController view]).andReturn(mockFlutterView);

  NSMutableArray<NSDictionary<NSString*, id>*>* accessibility_notifications =
      [[NSMutableArray alloc] init];
  auto ios_delegate = std::make_unique<flutter::MockIosDelegate>();
  ios_delegate->on_PostAccessibilityNotification_ =
      [accessibility_notifications](UIAccessibilityNotifications notification, id argument) {
        [accessibility_notifications addObject:@{
          @"notification" : @(notification),
          @"argument" : argument ? argument : [NSNull null],
        }];
      };
  __block auto bridge =
      std::make_unique<flutter::AccessibilityBridge>(/*view_controller=*/mockFlutterViewController,
                                                     /*platform_view=*/platform_view.get(),
                                                     /*platform_views_controller=*/nil,
                                                     /*ios_delegate=*/std::move(ios_delegate));

  flutter::CustomAccessibilityActionUpdates actions;
  flutter::SemanticsNodeUpdates first_update;

  flutter::SemanticsNode node_one;
  node_one.id = 1;
  node_one.label = "route1";
  first_update[node_one.id] = node_one;
  flutter::SemanticsNode node_two;
  node_two.id = 2;
  node_two.label = "route2";
  first_update[node_two.id] = node_two;
  flutter::SemanticsNode root_node;
  root_node.id = kRootNodeId;
  root_node.label = "root";
  root_node.childrenInTraversalOrder = {1, 2};
  root_node.childrenInHitTestOrder = {1, 2};
  first_update[root_node.id] = root_node;
  bridge->UpdateSemantics(/*nodes=*/first_update, /*actions=*/actions);

  XCTAssertEqual([accessibility_notifications count], 0ul);
  // Simulates the focusing on the node 1.
  bridge->AccessibilityObjectDidBecomeFocused(1);

  flutter::SemanticsNodeUpdates second_update;
  // Simulates the removal of the node 2.
  flutter::SemanticsNode new_root_node;
  new_root_node.id = kRootNodeId;
  new_root_node.label = "root";
  new_root_node.childrenInTraversalOrder = {1};
  new_root_node.childrenInHitTestOrder = {1};
  second_update[root_node.id] = new_root_node;
  bridge->UpdateSemantics(/*nodes=*/second_update, /*actions=*/actions);
  id focusObject = accessibility_notifications[0][@"argument"];
  // Since we have focused on the node 1 right before the layout changed, the bridge should not ask
  // to refocus again on the same node.
  XCTAssertEqualObjects(focusObject, [NSNull null]);
  XCTAssertEqual([accessibility_notifications[0][@"notification"] unsignedIntValue],
                 UIAccessibilityLayoutChangedNotification);
}

- (void)testAnnouncesLayoutChangeWhenFocusMovedOutside {
  flutter::MockDelegate mock_delegate;
  auto thread_task_runner = CreateNewThread("AccessibilityBridgeTest");
  flutter::TaskRunners runners(/*label=*/self.name.UTF8String,
                               /*platform=*/thread_task_runner,
                               /*raster=*/thread_task_runner,
                               /*ui=*/thread_task_runner,
                               /*io=*/thread_task_runner);
  auto platform_view = std::make_unique<flutter::PlatformViewIOS>(
      /*delegate=*/mock_delegate,
      /*rendering_api=*/mock_delegate.settings_.enable_impeller
          ? flutter::IOSRenderingAPI::kMetal
          : flutter::IOSRenderingAPI::kSoftware,
      /*platform_views_controller=*/nil,
      /*task_runners=*/runners,
      /*worker_task_runner=*/nil,
      /*is_gpu_disabled_sync_switch=*/std::make_shared<fml::SyncSwitch>());
  id mockFlutterViewController = OCMClassMock([FlutterViewController class]);
  id mockFlutterView = OCMClassMock([FlutterView class]);
  OCMStub([mockFlutterViewController view]).andReturn(mockFlutterView);

  NSMutableArray<NSDictionary<NSString*, id>*>* accessibility_notifications =
      [[NSMutableArray alloc] init];
  auto ios_delegate = std::make_unique<flutter::MockIosDelegate>();
  ios_delegate->on_PostAccessibilityNotification_ =
      [accessibility_notifications](UIAccessibilityNotifications notification, id argument) {
        [accessibility_notifications addObject:@{
          @"notification" : @(notification),
          @"argument" : argument ? argument : [NSNull null],
        }];
      };
  __block auto bridge =
      std::make_unique<flutter::AccessibilityBridge>(/*view_controller=*/mockFlutterViewController,
                                                     /*platform_view=*/platform_view.get(),
                                                     /*platform_views_controller=*/nil,
                                                     /*ios_delegate=*/std::move(ios_delegate));

  flutter::CustomAccessibilityActionUpdates actions;
  flutter::SemanticsNodeUpdates first_update;

  flutter::SemanticsNode node_one;
  node_one.id = 1;
  node_one.label = "route1";
  first_update[node_one.id] = node_one;
  flutter::SemanticsNode node_two;
  node_two.id = 2;
  node_two.label = "route2";
  first_update[node_two.id] = node_two;
  flutter::SemanticsNode root_node;
  root_node.id = kRootNodeId;
  root_node.label = "root";
  root_node.childrenInTraversalOrder = {1, 2};
  root_node.childrenInHitTestOrder = {1, 2};
  first_update[root_node.id] = root_node;
  bridge->UpdateSemantics(/*nodes=*/first_update, /*actions=*/actions);

  XCTAssertEqual([accessibility_notifications count], 0ul);
  // Simulates the focusing on the node 1.
  bridge->AccessibilityObjectDidBecomeFocused(1);
  // Simulates that the focus move outside of flutter.
  bridge->AccessibilityObjectDidLoseFocus(1);

  flutter::SemanticsNodeUpdates second_update;
  // Simulates the removal of the node 2.
  flutter::SemanticsNode new_root_node;
  new_root_node.id = kRootNodeId;
  new_root_node.label = "root";
  new_root_node.childrenInTraversalOrder = {1};
  new_root_node.childrenInHitTestOrder = {1};
  second_update[root_node.id] = new_root_node;
  bridge->UpdateSemantics(/*nodes=*/second_update, /*actions=*/actions);
  NSNull* focusObject = accessibility_notifications[0][@"argument"];
  // Since the focus is moved outside of the app right before the layout
  // changed, the bridge should not try to refocus anything .
  XCTAssertEqual(focusObject, [NSNull null]);
  XCTAssertEqual([accessibility_notifications[0][@"notification"] unsignedIntValue],
                 UIAccessibilityLayoutChangedNotification);
}

- (void)testAnnouncesScrollChangeWithLastFocused {
  flutter::MockDelegate mock_delegate;
  auto thread_task_runner = CreateNewThread("AccessibilityBridgeTest");
  flutter::TaskRunners runners(/*label=*/self.name.UTF8String,
                               /*platform=*/thread_task_runner,
                               /*raster=*/thread_task_runner,
                               /*ui=*/thread_task_runner,
                               /*io=*/thread_task_runner);
  auto platform_view = std::make_unique<flutter::PlatformViewIOS>(
      /*delegate=*/mock_delegate,
      /*rendering_api=*/mock_delegate.settings_.enable_impeller
          ? flutter::IOSRenderingAPI::kMetal
          : flutter::IOSRenderingAPI::kSoftware,
      /*platform_views_controller=*/nil,
      /*task_runners=*/runners,
      /*worker_task_runner=*/nil,
      /*is_gpu_disabled_sync_switch=*/std::make_shared<fml::SyncSwitch>());
  id mockFlutterViewController = OCMClassMock([FlutterViewController class]);
  id mockFlutterView = OCMClassMock([FlutterView class]);
  OCMStub([mockFlutterViewController view]).andReturn(mockFlutterView);

  NSMutableArray<NSDictionary<NSString*, id>*>* accessibility_notifications =
      [[NSMutableArray alloc] init];
  auto ios_delegate = std::make_unique<flutter::MockIosDelegate>();
  ios_delegate->on_PostAccessibilityNotification_ =
      [accessibility_notifications](UIAccessibilityNotifications notification, id argument) {
        [accessibility_notifications addObject:@{
          @"notification" : @(notification),
          @"argument" : argument ? argument : [NSNull null],
        }];
      };
  __block auto bridge =
      std::make_unique<flutter::AccessibilityBridge>(/*view_controller=*/mockFlutterViewController,
                                                     /*platform_view=*/platform_view.get(),
                                                     /*platform_views_controller=*/nil,
                                                     /*ios_delegate=*/std::move(ios_delegate));

  flutter::CustomAccessibilityActionUpdates actions;
  flutter::SemanticsNodeUpdates first_update;

  flutter::SemanticsNode node_one;
  node_one.id = 1;
  node_one.label = "route1";
  node_one.scrollPosition = 0.0;
  first_update[node_one.id] = node_one;
  flutter::SemanticsNode root_node;
  root_node.id = kRootNodeId;
  root_node.label = "root";
  root_node.childrenInTraversalOrder = {1};
  root_node.childrenInHitTestOrder = {1};
  first_update[root_node.id] = root_node;
  bridge->UpdateSemantics(/*nodes=*/first_update, /*actions=*/actions);

  // The first update will trigger a scroll announcement, but we are not interested in it.
  [accessibility_notifications removeAllObjects];

  // Simulates the focusing on the node 1.
  bridge->AccessibilityObjectDidBecomeFocused(1);

  flutter::SemanticsNodeUpdates second_update;
  // Simulates the scrolling on the node 1.
  flutter::SemanticsNode new_node_one;
  new_node_one.id = 1;
  new_node_one.label = "route1";
  new_node_one.scrollPosition = 1.0;
  second_update[new_node_one.id] = new_node_one;
  bridge->UpdateSemantics(/*nodes=*/second_update, /*actions=*/actions);
  SemanticsObject* focusObject = accessibility_notifications[0][@"argument"];
  // Since we have focused on the node 1 right before the scrolling, the bridge should refocus the
  // node 1.
  XCTAssertEqual([focusObject uid], 1);
  XCTAssertEqual([accessibility_notifications[0][@"notification"] unsignedIntValue],
                 UIAccessibilityPageScrolledNotification);
}

- (void)testAnnouncesScrollChangeDoesCallNativeAccessibility {
  flutter::MockDelegate mock_delegate;
  auto thread_task_runner = CreateNewThread("AccessibilityBridgeTest");
  flutter::TaskRunners runners(/*label=*/self.name.UTF8String,
                               /*platform=*/thread_task_runner,
                               /*raster=*/thread_task_runner,
                               /*ui=*/thread_task_runner,
                               /*io=*/thread_task_runner);
  auto platform_view = std::make_unique<flutter::PlatformViewIOS>(
      /*delegate=*/mock_delegate,
      /*rendering_api=*/mock_delegate.settings_.enable_impeller
          ? flutter::IOSRenderingAPI::kMetal
          : flutter::IOSRenderingAPI::kSoftware,
      /*platform_views_controller=*/nil,
      /*task_runners=*/runners,
      /*worker_task_runner=*/nil,
      /*is_gpu_disabled_sync_switch=*/std::make_shared<fml::SyncSwitch>());
  id mockFlutterViewController = OCMClassMock([FlutterViewController class]);
  id mockFlutterView = OCMClassMock([FlutterView class]);
  OCMStub([mockFlutterViewController view]).andReturn(mockFlutterView);

  NSMutableArray<NSDictionary<NSString*, id>*>* accessibility_notifications =
      [[NSMutableArray alloc] init];
  auto ios_delegate = std::make_unique<flutter::MockIosDelegate>();
  ios_delegate->on_PostAccessibilityNotification_ =
      [accessibility_notifications](UIAccessibilityNotifications notification, id argument) {
        [accessibility_notifications addObject:@{
          @"notification" : @(notification),
          @"argument" : argument ? argument : [NSNull null],
        }];
      };
  __block auto bridge =
      std::make_unique<flutter::AccessibilityBridge>(/*view_controller=*/mockFlutterViewController,
                                                     /*platform_view=*/platform_view.get(),
                                                     /*platform_views_controller=*/nil,
                                                     /*ios_delegate=*/std::move(ios_delegate));

  flutter::CustomAccessibilityActionUpdates actions;
  flutter::SemanticsNodeUpdates first_update;

  flutter::SemanticsNode node_one;
  node_one.id = 1;
  node_one.label = "route1";
  node_one.flags = static_cast<int32_t>(flutter::SemanticsFlags::kHasImplicitScrolling);
  node_one.scrollPosition = 0.0;
  first_update[node_one.id] = node_one;
  flutter::SemanticsNode root_node;
  root_node.id = kRootNodeId;
  root_node.label = "root";
  root_node.childrenInTraversalOrder = {1};
  root_node.childrenInHitTestOrder = {1};
  first_update[root_node.id] = root_node;
  bridge->UpdateSemantics(/*nodes=*/first_update, /*actions=*/actions);

  // The first update will trigger a scroll announcement, but we are not interested in it.
  [accessibility_notifications removeAllObjects];

  // Simulates the focusing on the node 1.
  bridge->AccessibilityObjectDidBecomeFocused(1);

  flutter::SemanticsNodeUpdates second_update;
  // Simulates the scrolling on the node 1.
  flutter::SemanticsNode new_node_one;
  new_node_one.id = 1;
  new_node_one.label = "route1";
  new_node_one.flags = static_cast<int32_t>(flutter::SemanticsFlags::kHasImplicitScrolling);
  new_node_one.scrollPosition = 1.0;
  second_update[new_node_one.id] = new_node_one;
  bridge->UpdateSemantics(/*nodes=*/second_update, /*actions=*/actions);
  SemanticsObject* focusObject = accessibility_notifications[0][@"argument"];
  // Make sure refocus event is sent with the nativeAccessibility of node_one
  // which is a FlutterSemanticsScrollView.
  XCTAssertTrue([focusObject isKindOfClass:[FlutterSemanticsScrollView class]]);
  XCTAssertEqual([accessibility_notifications[0][@"notification"] unsignedIntValue],
                 UIAccessibilityPageScrolledNotification);
}

- (void)testAnnouncesIgnoresRouteChangesWhenModal {
  flutter::MockDelegate mock_delegate;
  auto thread_task_runner = CreateNewThread("AccessibilityBridgeTest");
  flutter::TaskRunners runners(/*label=*/self.name.UTF8String,
                               /*platform=*/thread_task_runner,
                               /*raster=*/thread_task_runner,
                               /*ui=*/thread_task_runner,
                               /*io=*/thread_task_runner);
  auto platform_view = std::make_unique<flutter::PlatformViewIOS>(
      /*delegate=*/mock_delegate,
      /*rendering_api=*/mock_delegate.settings_.enable_impeller
          ? flutter::IOSRenderingAPI::kMetal
          : flutter::IOSRenderingAPI::kSoftware,
      /*platform_views_controller=*/nil,
      /*task_runners=*/runners,
      /*worker_task_runner=*/nil,
      /*is_gpu_disabled_sync_switch=*/std::make_shared<fml::SyncSwitch>());
  id mockFlutterView = OCMClassMock([FlutterView class]);
  id mockFlutterViewController = OCMClassMock([FlutterViewController class]);
  OCMStub([mockFlutterViewController view]).andReturn(mockFlutterView);
  std::string label = "some label";

  NSMutableArray<NSDictionary<NSString*, id>*>* accessibility_notifications =
      [[NSMutableArray alloc] init];
  auto ios_delegate = std::make_unique<flutter::MockIosDelegate>();
  ios_delegate->on_PostAccessibilityNotification_ =
      [accessibility_notifications](UIAccessibilityNotifications notification, id argument) {
        [accessibility_notifications addObject:@{
          @"notification" : @(notification),
          @"argument" : argument ? argument : [NSNull null],
        }];
      };
  ios_delegate->result_IsFlutterViewControllerPresentingModalViewController_ = true;
  __block auto bridge =
      std::make_unique<flutter::AccessibilityBridge>(/*view_controller=*/mockFlutterViewController,
                                                     /*platform_view=*/platform_view.get(),
                                                     /*platform_views_controller=*/nil,
                                                     /*ios_delegate=*/std::move(ios_delegate));

  flutter::CustomAccessibilityActionUpdates actions;
  flutter::SemanticsNodeUpdates nodes;

  flutter::SemanticsNode route_node;
  route_node.id = 1;
  route_node.flags = static_cast<int32_t>(flutter::SemanticsFlags::kScopesRoute) |
                     static_cast<int32_t>(flutter::SemanticsFlags::kNamesRoute);
  route_node.label = "route";
  nodes[route_node.id] = route_node;
  flutter::SemanticsNode root_node;
  root_node.id = kRootNodeId;
  root_node.label = label;
  root_node.childrenInTraversalOrder = {1};
  root_node.childrenInHitTestOrder = {1};
  nodes[root_node.id] = root_node;
  bridge->UpdateSemantics(/*nodes=*/nodes, /*actions=*/actions);

  XCTAssertEqual([accessibility_notifications count], 0ul);
}

- (void)testAnnouncesIgnoresLayoutChangeWhenModal {
  flutter::MockDelegate mock_delegate;
  auto thread_task_runner = CreateNewThread("AccessibilityBridgeTest");
  flutter::TaskRunners runners(/*label=*/self.name.UTF8String,
                               /*platform=*/thread_task_runner,
                               /*raster=*/thread_task_runner,
                               /*ui=*/thread_task_runner,
                               /*io=*/thread_task_runner);
  auto platform_view = std::make_unique<flutter::PlatformViewIOS>(
      /*delegate=*/mock_delegate,
      /*rendering_api=*/mock_delegate.settings_.enable_impeller
          ? flutter::IOSRenderingAPI::kMetal
          : flutter::IOSRenderingAPI::kSoftware,
      /*platform_views_controller=*/nil,
      /*task_runners=*/runners,
      /*worker_task_runner=*/nil,
      /*is_gpu_disabled_sync_switch=*/std::make_shared<fml::SyncSwitch>());
  id mockFlutterView = OCMClassMock([FlutterView class]);
  id mockFlutterViewController = OCMClassMock([FlutterViewController class]);
  OCMStub([mockFlutterViewController view]).andReturn(mockFlutterView);

  NSMutableArray<NSDictionary<NSString*, id>*>* accessibility_notifications =
      [[NSMutableArray alloc] init];
  auto ios_delegate = std::make_unique<flutter::MockIosDelegate>();
  ios_delegate->on_PostAccessibilityNotification_ =
      [accessibility_notifications](UIAccessibilityNotifications notification, id argument) {
        [accessibility_notifications addObject:@{
          @"notification" : @(notification),
          @"argument" : argument ? argument : [NSNull null],
        }];
      };
  ios_delegate->result_IsFlutterViewControllerPresentingModalViewController_ = true;
  __block auto bridge =
      std::make_unique<flutter::AccessibilityBridge>(/*view_controller=*/mockFlutterViewController,
                                                     /*platform_view=*/platform_view.get(),
                                                     /*platform_views_controller=*/nil,
                                                     /*ios_delegate=*/std::move(ios_delegate));

  flutter::CustomAccessibilityActionUpdates actions;
  flutter::SemanticsNodeUpdates nodes;

  flutter::SemanticsNode child_node;
  child_node.id = 1;
  child_node.label = "child_node";
  nodes[child_node.id] = child_node;
  flutter::SemanticsNode root_node;
  root_node.id = kRootNodeId;
  root_node.label = "root";
  root_node.childrenInTraversalOrder = {1};
  root_node.childrenInHitTestOrder = {1};
  nodes[root_node.id] = root_node;
  bridge->UpdateSemantics(/*nodes=*/nodes, /*actions=*/actions);

  // Removes child_node to simulate a layout change.
  flutter::SemanticsNodeUpdates new_nodes;
  flutter::SemanticsNode new_root_node;
  new_root_node.id = kRootNodeId;
  new_root_node.label = "root";
  new_nodes[new_root_node.id] = new_root_node;
  bridge->UpdateSemantics(/*nodes=*/new_nodes, /*actions=*/actions);

  XCTAssertEqual([accessibility_notifications count], 0ul);
}

- (void)testAnnouncesIgnoresScrollChangeWhenModal {
  flutter::MockDelegate mock_delegate;
  auto thread_task_runner = CreateNewThread("AccessibilityBridgeTest");
  flutter::TaskRunners runners(/*label=*/self.name.UTF8String,
                               /*platform=*/thread_task_runner,
                               /*raster=*/thread_task_runner,
                               /*ui=*/thread_task_runner,
                               /*io=*/thread_task_runner);
  auto platform_view = std::make_unique<flutter::PlatformViewIOS>(
      /*delegate=*/mock_delegate,
      /*rendering_api=*/mock_delegate.settings_.enable_impeller
          ? flutter::IOSRenderingAPI::kMetal
          : flutter::IOSRenderingAPI::kSoftware,
      /*platform_views_controller=*/nil,
      /*task_runners=*/runners,
      /*worker_task_runner=*/nil,
      /*is_gpu_disabled_sync_switch=*/std::make_shared<fml::SyncSwitch>());
  id mockFlutterView = OCMClassMock([FlutterView class]);
  id mockFlutterViewController = OCMClassMock([FlutterViewController class]);
  OCMStub([mockFlutterViewController view]).andReturn(mockFlutterView);

  NSMutableArray<NSDictionary<NSString*, id>*>* accessibility_notifications =
      [[NSMutableArray alloc] init];
  auto ios_delegate = std::make_unique<flutter::MockIosDelegate>();
  ios_delegate->on_PostAccessibilityNotification_ =
      [accessibility_notifications](UIAccessibilityNotifications notification, id argument) {
        [accessibility_notifications addObject:@{
          @"notification" : @(notification),
          @"argument" : argument ? argument : [NSNull null],
        }];
      };
  ios_delegate->result_IsFlutterViewControllerPresentingModalViewController_ = true;
  __block auto bridge =
      std::make_unique<flutter::AccessibilityBridge>(/*view_controller=*/mockFlutterViewController,
                                                     /*platform_view=*/platform_view.get(),
                                                     /*platform_views_controller=*/nil,
                                                     /*ios_delegate=*/std::move(ios_delegate));

  flutter::CustomAccessibilityActionUpdates actions;
  flutter::SemanticsNodeUpdates nodes;

  flutter::SemanticsNode root_node;
  root_node.id = kRootNodeId;
  root_node.label = "root";
  root_node.scrollPosition = 1;
  nodes[root_node.id] = root_node;
  bridge->UpdateSemantics(/*nodes=*/nodes, /*actions=*/actions);

  // Removes child_node to simulate a layout change.
  flutter::SemanticsNodeUpdates new_nodes;
  flutter::SemanticsNode new_root_node;
  new_root_node.id = kRootNodeId;
  new_root_node.label = "root";
  new_root_node.scrollPosition = 2;
  new_nodes[new_root_node.id] = new_root_node;
  bridge->UpdateSemantics(/*nodes=*/new_nodes, /*actions=*/actions);

  XCTAssertEqual([accessibility_notifications count], 0ul);
}

- (void)testAccessibilityMessageAfterDeletion {
  flutter::MockDelegate mock_delegate;
  auto thread = std::make_unique<fml::Thread>("AccessibilityBridgeTest");
  auto thread_task_runner = thread->GetTaskRunner();
  flutter::TaskRunners runners(/*label=*/self.name.UTF8String,
                               /*platform=*/thread_task_runner,
                               /*raster=*/thread_task_runner,
                               /*ui=*/thread_task_runner,
                               /*io=*/thread_task_runner);
  id messenger = OCMProtocolMock(@protocol(FlutterBinaryMessenger));
  id engine = OCMClassMock([FlutterEngine class]);
  id flutterViewController = OCMClassMock([FlutterViewController class]);

  OCMStub([flutterViewController engine]).andReturn(engine);
  OCMStub([engine binaryMessenger]).andReturn(messenger);
  FlutterBinaryMessengerConnection connection = 123;
  OCMStub([messenger setMessageHandlerOnChannel:@"flutter/accessibility"
                           binaryMessageHandler:[OCMArg any]])
      .andReturn(connection);

  auto platform_view = std::make_unique<flutter::PlatformViewIOS>(
      /*delegate=*/mock_delegate,
      /*rendering_api=*/mock_delegate.settings_.enable_impeller
          ? flutter::IOSRenderingAPI::kMetal
          : flutter::IOSRenderingAPI::kSoftware,
      /*platform_views_controller=*/nil,
      /*task_runners=*/runners,
      /*worker_task_runner=*/nil,
      /*is_gpu_disabled_sync_switch=*/std::make_shared<fml::SyncSwitch>());
  fml::AutoResetWaitableEvent latch;
  thread_task_runner->PostTask([&] {
    platform_view->SetOwnerViewController(flutterViewController);
    auto bridge =
        std::make_unique<flutter::AccessibilityBridge>(/*view=*/nil,
                                                       /*platform_view=*/platform_view.get(),
                                                       /*platform_views_controller=*/nil);
    XCTAssertTrue(bridge.get());
    OCMVerify([messenger setMessageHandlerOnChannel:@"flutter/accessibility"
                               binaryMessageHandler:[OCMArg isNotNil]]);
    bridge.reset();
    latch.Signal();
  });
  latch.Wait();
  OCMVerify([messenger cleanUpConnection:connection]);
  [engine stopMocking];
}

- (void)testFlutterSemanticsScrollViewManagedObjectLifecycleCorrectly {
  flutter::MockDelegate mock_delegate;
  auto thread_task_runner = CreateNewThread("AccessibilityBridgeTest");
  flutter::TaskRunners runners(/*label=*/self.name.UTF8String,
                               /*platform=*/thread_task_runner,
                               /*raster=*/thread_task_runner,
                               /*ui=*/thread_task_runner,
                               /*io=*/thread_task_runner);
  auto platform_view = std::make_unique<flutter::PlatformViewIOS>(
      /*delegate=*/mock_delegate,
      /*rendering_api=*/mock_delegate.settings_.enable_impeller
          ? flutter::IOSRenderingAPI::kMetal
          : flutter::IOSRenderingAPI::kSoftware,
      /*platform_views_controller=*/nil,
      /*task_runners=*/runners,
      /*worker_task_runner=*/nil,
      /*is_gpu_disabled_sync_switch=*/std::make_shared<fml::SyncSwitch>());
  id mockFlutterView = OCMClassMock([FlutterView class]);
  id mockFlutterViewController = OCMClassMock([FlutterViewController class]);
  OCMStub([mockFlutterViewController view]).andReturn(mockFlutterView);

  auto ios_delegate = std::make_unique<flutter::MockIosDelegate>();
  __block auto bridge =
      std::make_unique<flutter::AccessibilityBridge>(/*view_controller=*/mockFlutterViewController,
                                                     /*platform_view=*/platform_view.get(),
                                                     /*platform_views_controller=*/nil,
                                                     /*ios_delegate=*/std::move(ios_delegate));

  FlutterSemanticsScrollView* flutterSemanticsScrollView;
  @autoreleasepool {
    FlutterScrollableSemanticsObject* semanticsObject =
        [[FlutterScrollableSemanticsObject alloc] initWithBridge:bridge->GetWeakPtr() uid:1234];

    flutterSemanticsScrollView = semanticsObject.nativeAccessibility;
  }
  XCTAssertTrue(flutterSemanticsScrollView);
  // If the _semanticsObject is not a weak pointer this (or any other method on
  // flutterSemanticsScrollView) will cause an EXC_BAD_ACCESS.
  XCTAssertFalse([flutterSemanticsScrollView isAccessibilityElement]);
}

- (void)testPlatformViewDestructorDoesNotCallSemanticsAPIs {
  class TestDelegate : public flutter::MockDelegate {
   public:
    void OnPlatformViewSetSemanticsEnabled(bool enabled) override { set_semantics_enabled_calls++; }
    int set_semantics_enabled_calls = 0;
  };

  TestDelegate test_delegate;
  auto thread = std::make_unique<fml::Thread>("AccessibilityBridgeTest");
  auto thread_task_runner = thread->GetTaskRunner();
  flutter::TaskRunners runners(/*label=*/self.name.UTF8String,
                               /*platform=*/thread_task_runner,
                               /*raster=*/thread_task_runner,
                               /*ui=*/thread_task_runner,
                               /*io=*/thread_task_runner);

  fml::AutoResetWaitableEvent latch;
  thread_task_runner->PostTask([&] {
    auto platform_view = std::make_unique<flutter::PlatformViewIOS>(
        /*delegate=*/test_delegate,
        /*rendering_api=*/test_delegate.settings_.enable_impeller
            ? flutter::IOSRenderingAPI::kMetal
            : flutter::IOSRenderingAPI::kSoftware,
        /*platform_views_controller=*/nil,
        /*task_runners=*/runners,
        /*worker_task_runner=*/nil,
        /*is_gpu_disabled_sync_switch=*/std::make_shared<fml::SyncSwitch>());

    id mockFlutterViewController = OCMClassMock([FlutterViewController class]);
    FlutterPlatformViewsController* flutterPlatformViewsController =
        [[FlutterPlatformViewsController alloc] init];
    flutterPlatformViewsController.taskRunner = thread_task_runner;

    OCMStub([mockFlutterViewController platformViewsController])
        .andReturn(flutterPlatformViewsController);
    platform_view->SetOwnerViewController(mockFlutterViewController);

    platform_view->SetSemanticsEnabled(true);
    XCTAssertNotEqual(test_delegate.set_semantics_enabled_calls, 0);

    // Deleting PlatformViewIOS should not call OnPlatformViewSetSemanticsEnabled
    test_delegate.set_semantics_enabled_calls = 0;
    platform_view.reset();
    XCTAssertEqual(test_delegate.set_semantics_enabled_calls, 0);

    latch.Signal();
  });
  latch.Wait();
}

@end
