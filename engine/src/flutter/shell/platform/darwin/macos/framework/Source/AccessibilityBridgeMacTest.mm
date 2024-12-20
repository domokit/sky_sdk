// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/macos/framework/Source/AccessibilityBridgeMac.h"

#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterDartProject_Internal.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterEngine_Internal.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterViewController_Internal.h"
#include "flutter/testing/autoreleasepool_test.h"
#include "flutter/testing/testing.h"

namespace flutter::testing {

namespace {

class AccessibilityBridgeMacSpy : public AccessibilityBridgeMac {
 public:
  using AccessibilityBridgeMac::OnAccessibilityEvent;

  AccessibilityBridgeMacSpy(__weak FlutterEngine* flutter_engine,
                            __weak FlutterViewController* view_controller)
      : AccessibilityBridgeMac(flutter_engine, view_controller) {}

  std::unordered_map<std::string, gfx::NativeViewAccessible> actual_notifications;

 private:
  void DispatchMacOSNotification(gfx::NativeViewAccessible native_node,
                                 NSAccessibilityNotificationName mac_notification) override {
    actual_notifications[[mac_notification UTF8String]] = native_node;
  }
};

}  // namespace
}  // namespace flutter::testing

@interface AccessibilityBridgeTestViewController : FlutterViewController
- (std::shared_ptr<flutter::AccessibilityBridgeMac>)createAccessibilityBridgeWithEngine:
    (nonnull FlutterEngine*)engine;
@end

@implementation AccessibilityBridgeTestViewController
- (std::shared_ptr<flutter::AccessibilityBridgeMac>)createAccessibilityBridgeWithEngine:
    (nonnull FlutterEngine*)engine {
  return std::make_shared<flutter::testing::AccessibilityBridgeMacSpy>(engine, self);
}
@end

namespace flutter::testing {

namespace {

// Returns an engine configured for the text fixture resource configuration.
FlutterViewController* CreateTestViewController() {
  NSString* fixtures = @(testing::GetFixturesPath());
  FlutterDartProject* project = [[FlutterDartProject alloc]
      initWithAssetsPath:fixtures
             ICUDataPath:[fixtures stringByAppendingString:@"/icudtl.dat"]];
  return [[AccessibilityBridgeTestViewController alloc] initWithProject:project];
}

// Test fixture that instantiates and re-uses a single NSWindow across multiple tests.
//
// Works around: http://www.openradar.me/FB13291861
class AccessibilityBridgeMacWindowTest : public AutoreleasePoolTest {
 public:
  AccessibilityBridgeMacWindowTest() {
    if (!gWindow_) {
      gWindow_ = [[NSWindow alloc] initWithContentRect:NSMakeRect(0, 0, 800, 600)
                                             styleMask:NSBorderlessWindowMask
                                               backing:NSBackingStoreBuffered
                                                 defer:NO];
    }
  }

  NSWindow* GetWindow() const { return gWindow_; }

 private:
  static NSWindow* gWindow_;
  FML_DISALLOW_COPY_AND_ASSIGN(AccessibilityBridgeMacWindowTest);
};

NSWindow* AccessibilityBridgeMacWindowTest::gWindow_ = nil;

// Test-specific name for AutoreleasePoolTest fixture.
using AccessibilityBridgeMacTest = AutoreleasePoolTest;

}  // namespace

TEST_F(AccessibilityBridgeMacWindowTest, SendsAccessibilityCreateNotificationFlutterViewWindow) {
  FlutterViewController* viewController = CreateTestViewController();
  FlutterEngine* engine = viewController.engine;
  NSWindow* expectedTarget = GetWindow();
  expectedTarget.contentView = viewController.view;

  // Setting up bridge so that the AccessibilityBridgeMacDelegateSpy
  // can query semantics information from.
  engine.semanticsEnabled = YES;
  auto bridge = std::static_pointer_cast<AccessibilityBridgeMacSpy>(
      viewController.accessibilityBridge.lock());
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
  root.child_count = 0;
  root.custom_accessibility_actions_count = 0;
  bridge->AddFlutterSemanticsNodeUpdate(root);

  bridge->CommitUpdates();
  auto platform_node_delegate = bridge->GetFlutterPlatformNodeDelegateFromID(0).lock();

  // Creates a targeted event.
  ui::AXTree tree;
  ui::AXNode ax_node(&tree, nullptr, 0, 0);
  ui::AXNodeData node_data;
  node_data.id = 0;
  ax_node.SetData(node_data);
  std::vector<ui::AXEventIntent> intent;
  ui::AXEventGenerator::EventParams event_params(ui::AXEventGenerator::Event::CHILDREN_CHANGED,
                                                 ax::mojom::EventFrom::kNone, intent);
  ui::AXEventGenerator::TargetedEvent targeted_event(&ax_node, event_params);

  bridge->OnAccessibilityEvent(targeted_event);

  ASSERT_EQ(bridge->actual_notifications.size(), 1u);
  auto target = bridge->actual_notifications.find([NSAccessibilityCreatedNotification UTF8String]);
  ASSERT_NE(target, bridge->actual_notifications.end());
  EXPECT_EQ(target->second, expectedTarget);
  [engine shutDownEngine];
}

// Flutter used to assume that the accessibility root had ID 0.
// In a multi-view world, each view has its own accessibility root
// with a globally unique node ID.
//
//        node1
//          |
//        node2
TEST_F(AccessibilityBridgeMacWindowTest, NonZeroRootNodeId) {
  FlutterViewController* viewController = CreateTestViewController();
  FlutterEngine* engine = viewController.engine;
  NSWindow* expectedTarget = GetWindow();
  expectedTarget.contentView = viewController.view;

  // Setting up bridge so that the AccessibilityBridgeMacDelegateSpy
  // can query semantics information from.
  engine.semanticsEnabled = YES;
  auto bridge = std::static_pointer_cast<AccessibilityBridgeMacSpy>(
      viewController.accessibilityBridge.lock());

  FlutterSemanticsNode2 node1;
  std::vector<int32_t> node1_children{2};
  node1.id = 1;
  node1.flags = static_cast<FlutterSemanticsFlag>(0);
  node1.actions = static_cast<FlutterSemanticsAction>(0);
  node1.text_selection_base = -1;
  node1.text_selection_extent = -1;
  node1.label = "node1";
  node1.hint = "";
  node1.value = "";
  node1.increased_value = "";
  node1.decreased_value = "";
  node1.tooltip = "";
  node1.child_count = node1_children.size();
  node1.children_in_traversal_order = node1_children.data();
  node1.children_in_hit_test_order = node1_children.data();
  node1.custom_accessibility_actions_count = 0;

  FlutterSemanticsNode2 node2;
  node2.id = 2;
  node2.flags = static_cast<FlutterSemanticsFlag>(0);
  node2.actions = static_cast<FlutterSemanticsAction>(0);
  node2.text_selection_base = -1;
  node2.text_selection_extent = -1;
  node2.label = "node2";
  node2.hint = "";
  node2.value = "";
  node2.increased_value = "";
  node2.decreased_value = "";
  node2.tooltip = "";
  node2.child_count = 0;
  node2.custom_accessibility_actions_count = 0;

  bridge->AddFlutterSemanticsNodeUpdate(node1);
  bridge->AddFlutterSemanticsNodeUpdate(node2);
  bridge->CommitUpdates();

  // Look up the root node delegate.
  auto root_delegate = bridge->GetFlutterPlatformNodeDelegateFromID(1).lock();
  ASSERT_TRUE(root_delegate);
  ASSERT_EQ(root_delegate->GetChildCount(), 1);

  // Look up the child node delegate.
  auto child_delegate = bridge->GetFlutterPlatformNodeDelegateFromID(2).lock();
  ASSERT_TRUE(child_delegate);
  ASSERT_EQ(child_delegate->GetChildCount(), 0);

  // Ensure a node with ID 0 does not exist.
  auto invalid_delegate = bridge->GetFlutterPlatformNodeDelegateFromID(0).lock();
  ASSERT_FALSE(invalid_delegate);

  [engine shutDownEngine];
}

TEST_F(AccessibilityBridgeMacTest, DoesNotSendAccessibilityCreateNotificationWhenHeadless) {
  FlutterViewController* viewController = CreateTestViewController();
  FlutterEngine* engine = viewController.engine;

  // Setting up bridge so that the AccessibilityBridgeMacDelegateSpy
  // can query semantics information from.
  engine.semanticsEnabled = YES;
  auto bridge = std::static_pointer_cast<AccessibilityBridgeMacSpy>(
      viewController.accessibilityBridge.lock());
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
  root.child_count = 0;
  root.custom_accessibility_actions_count = 0;
  bridge->AddFlutterSemanticsNodeUpdate(root);

  bridge->CommitUpdates();
  auto platform_node_delegate = bridge->GetFlutterPlatformNodeDelegateFromID(0).lock();

  // Creates a targeted event.
  ui::AXTree tree;
  ui::AXNode ax_node(&tree, nullptr, 0, 0);
  ui::AXNodeData node_data;
  node_data.id = 0;
  ax_node.SetData(node_data);
  std::vector<ui::AXEventIntent> intent;
  ui::AXEventGenerator::EventParams event_params(ui::AXEventGenerator::Event::CHILDREN_CHANGED,
                                                 ax::mojom::EventFrom::kNone, intent);
  ui::AXEventGenerator::TargetedEvent targeted_event(&ax_node, event_params);

  bridge->OnAccessibilityEvent(targeted_event);

  // Does not send any notification if the engine is headless.
  EXPECT_EQ(bridge->actual_notifications.size(), 0u);
  [engine shutDownEngine];
}

TEST_F(AccessibilityBridgeMacTest, DoesNotSendAccessibilityCreateNotificationWhenNoWindow) {
  FlutterViewController* viewController = CreateTestViewController();
  FlutterEngine* engine = viewController.engine;

  // Setting up bridge so that the AccessibilityBridgeMacDelegateSpy
  // can query semantics information from.
  engine.semanticsEnabled = YES;
  auto bridge = std::static_pointer_cast<AccessibilityBridgeMacSpy>(
      viewController.accessibilityBridge.lock());
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
  root.child_count = 0;
  root.custom_accessibility_actions_count = 0;
  bridge->AddFlutterSemanticsNodeUpdate(root);

  bridge->CommitUpdates();
  auto platform_node_delegate = bridge->GetFlutterPlatformNodeDelegateFromID(0).lock();

  // Creates a targeted event.
  ui::AXTree tree;
  ui::AXNode ax_node(&tree, nullptr, 0, 0);
  ui::AXNodeData node_data;
  node_data.id = 0;
  ax_node.SetData(node_data);
  std::vector<ui::AXEventIntent> intent;
  ui::AXEventGenerator::EventParams event_params(ui::AXEventGenerator::Event::CHILDREN_CHANGED,
                                                 ax::mojom::EventFrom::kNone, intent);
  ui::AXEventGenerator::TargetedEvent targeted_event(&ax_node, event_params);

  bridge->OnAccessibilityEvent(targeted_event);

  // Does not send any notification if the flutter view is not attached to a NSWindow.
  EXPECT_EQ(bridge->actual_notifications.size(), 0u);
  [engine shutDownEngine];
}

}  // namespace flutter::testing
