// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.
#include "flutter/testing/testing.h"

#import "flutter/shell/platform/darwin/macos/framework/Headers/FlutterEngine.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/AccessibilityBridgeMac.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterDartProject_Internal.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterEngine_Internal.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterPlatformNodeDelegateMac.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterTextInputSemanticsObject.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterViewControllerTestUtils.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterViewController_Internal.h"

#include "flutter/shell/platform/common/accessibility_bridge.h"
#include "flutter/shell/platform/embedder/test_utils/proc_table_replacement.h"
#include "flutter/third_party/accessibility/ax/ax_action_data.h"

namespace flutter::testing {

namespace {
// Returns a view controller configured for the text fixture resource configuration.
FlutterViewController* CreateTestViewController() {
  NSString* fixtures = @(testing::GetFixturesPath());
  FlutterDartProject* project = [[FlutterDartProject alloc]
      initWithAssetsPath:fixtures
             ICUDataPath:[fixtures stringByAppendingString:@"/icudtl.dat"]];
  return [[FlutterViewController alloc] initWithProject:project];
}
}  // namespace

TEST(FlutterPlatformNodeDelegateMac, Basics) {
  FlutterViewController* viewController = CreateTestViewController();
  FlutterEngine* engine = viewController.engine;
  engine.semanticsEnabled = YES;
  auto bridge = viewController.accessibilityBridge.lock();
  // Initialize ax node data.
  FlutterSemanticsNode2 root;
  root.id = 0;
  root.flags = static_cast<FlutterSemanticsFlag>(0);
  ;
  root.actions = static_cast<FlutterSemanticsAction>(0);
  root.text_selection_base = -1;
  root.text_selection_extent = -1;
  root.label = "accessibility";
  root.hint = "";
  root.value = "";
  root.increased_value = "";
  root.decreased_value = "";
  root.tooltip = "";
  root.child_count = 0;
  root.custom_accessibility_actions_count = 0;
  bridge->AddFlutterSemanticsNodeUpdate(root);

  bridge->CommitUpdates();

  auto root_platform_node_delegate = bridge->GetFlutterPlatformNodeDelegateFromID(0).lock();
  // Verify the accessibility attribute matches.
  NSAccessibilityElement* native_accessibility =
      root_platform_node_delegate->GetNativeViewAccessible();
  std::string value = [native_accessibility.accessibilityValue UTF8String];
  EXPECT_TRUE(value == "accessibility");
  EXPECT_EQ(native_accessibility.accessibilityRole, NSAccessibilityStaticTextRole);
  EXPECT_EQ([native_accessibility.accessibilityChildren count], 0u);
  [engine shutDownEngine];
}

TEST(FlutterPlatformNodeDelegateMac, SelectableTextHasCorrectSemantics) {
  FlutterViewController* viewController = CreateTestViewController();
  FlutterEngine* engine = viewController.engine;
  engine.semanticsEnabled = YES;
  auto bridge = viewController.accessibilityBridge.lock();
  // Initialize ax node data.
  FlutterSemanticsNode2 root;
  root.id = 0;
  root.flags =
      static_cast<FlutterSemanticsFlag>(FlutterSemanticsFlag::kFlutterSemanticsFlagIsTextField |
                                        FlutterSemanticsFlag::kFlutterSemanticsFlagIsReadOnly);
  root.actions = static_cast<FlutterSemanticsAction>(0);
  root.text_selection_base = 1;
  root.text_selection_extent = 3;
  root.label = "";
  root.hint = "";
  // Selectable text store its text in value
  root.value = "selectable text";
  root.increased_value = "";
  root.decreased_value = "";
  root.tooltip = "";
  root.child_count = 0;
  root.custom_accessibility_actions_count = 0;
  bridge->AddFlutterSemanticsNodeUpdate(root);

  bridge->CommitUpdates();

  auto root_platform_node_delegate = bridge->GetFlutterPlatformNodeDelegateFromID(0).lock();
  // Verify the accessibility attribute matches.
  NSAccessibilityElement* native_accessibility =
      root_platform_node_delegate->GetNativeViewAccessible();
  std::string value = [native_accessibility.accessibilityValue UTF8String];
  EXPECT_EQ(value, "selectable text");
  EXPECT_EQ(native_accessibility.accessibilityRole, NSAccessibilityStaticTextRole);
  EXPECT_EQ([native_accessibility.accessibilityChildren count], 0u);
  NSRange selection = native_accessibility.accessibilitySelectedTextRange;
  EXPECT_EQ(selection.location, 1u);
  EXPECT_EQ(selection.length, 2u);
  std::string selected_text = [native_accessibility.accessibilitySelectedText UTF8String];
  EXPECT_EQ(selected_text, "el");
}

TEST(FlutterPlatformNodeDelegateMac, SelectableTextWithoutSelectionReturnZeroRange) {
  FlutterViewController* viewController = CreateTestViewController();
  FlutterEngine* engine = viewController.engine;
  engine.semanticsEnabled = YES;
  auto bridge = viewController.accessibilityBridge.lock();
  // Initialize ax node data.
  FlutterSemanticsNode2 root;
  root.id = 0;
  root.flags =
      static_cast<FlutterSemanticsFlag>(FlutterSemanticsFlag::kFlutterSemanticsFlagIsTextField |
                                        FlutterSemanticsFlag::kFlutterSemanticsFlagIsReadOnly);
  root.actions = static_cast<FlutterSemanticsAction>(0);
  root.text_selection_base = -1;
  root.text_selection_extent = -1;
  root.label = "";
  root.hint = "";
  // Selectable text store its text in value
  root.value = "selectable text";
  root.increased_value = "";
  root.decreased_value = "";
  root.tooltip = "";
  root.child_count = 0;
  root.custom_accessibility_actions_count = 0;
  bridge->AddFlutterSemanticsNodeUpdate(root);

  bridge->CommitUpdates();

  auto root_platform_node_delegate = bridge->GetFlutterPlatformNodeDelegateFromID(0).lock();
  // Verify the accessibility attribute matches.
  NSAccessibilityElement* native_accessibility =
      root_platform_node_delegate->GetNativeViewAccessible();
  NSRange selection = native_accessibility.accessibilitySelectedTextRange;
  EXPECT_TRUE(selection.location == NSNotFound);
  EXPECT_EQ(selection.length, 0u);
}

// MOCK_ENGINE_PROC is leaky by design
// NOLINTBEGIN(clang-analyzer-core.StackAddressEscape)

TEST(FlutterPlatformNodeDelegateMac, CanPerformAction) {
  FlutterViewController* viewController = CreateTestViewController();
  FlutterEngine* engine = viewController.engine;

  // Attach the view to a NSWindow.
  NSWindow* window = [[NSWindow alloc] initWithContentRect:NSMakeRect(0, 0, 800, 600)
                                                 styleMask:NSBorderlessWindowMask
                                                   backing:NSBackingStoreBuffered
                                                     defer:NO];
  window.contentView = viewController.view;

  engine.semanticsEnabled = YES;
  auto bridge = viewController.accessibilityBridge.lock();
  // Initialize ax node data.
  FlutterSemanticsNode2 root;
  root.id = 0;
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
  bridge->AddFlutterSemanticsNodeUpdate(root);

  FlutterSemanticsNode2 child1;
  child1.id = 1;
  child1.label = "child 1";
  child1.hint = "";
  child1.value = "";
  child1.increased_value = "";
  child1.decreased_value = "";
  child1.tooltip = "";
  child1.child_count = 0;
  child1.custom_accessibility_actions_count = 0;
  bridge->AddFlutterSemanticsNodeUpdate(child1);

  bridge->CommitUpdates();

  auto root_platform_node_delegate = bridge->GetFlutterPlatformNodeDelegateFromID(1).lock();

  // Set up embedder API mock.
  FlutterSemanticsAction called_action;
  uint64_t called_id;

  engine.embedderAPI.DispatchSemanticsAction = MOCK_ENGINE_PROC(
      DispatchSemanticsAction,
      ([&called_id, &called_action](auto engine, uint64_t id, FlutterSemanticsAction action,
                                    const uint8_t* data, size_t data_length) {
        called_id = id;
        called_action = action;
        return kSuccess;
      }));

  // Performs an AXAction.
  ui::AXActionData action_data;
  action_data.action = ax::mojom::Action::kDoDefault;
  root_platform_node_delegate->AccessibilityPerformAction(action_data);

  EXPECT_EQ(called_action, FlutterSemanticsAction::kFlutterSemanticsActionTap);
  EXPECT_EQ(called_id, 1u);

  [engine setViewController:nil];
  [engine shutDownEngine];
}

// NOLINTEND(clang-analyzer-core.StackAddressEscape)

TEST(FlutterPlatformNodeDelegateMac, TextFieldUsesFlutterTextField) {
  FlutterViewController* viewController = CreateTestViewController();
  FlutterEngine* engine = viewController.engine;
  [viewController loadView];

  // Unit test localization is unnecessary.
  // NOLINTNEXTLINE(clang-analyzer-optin.osx.cocoa.localizability.NonLocalizedStringChecker)
  viewController.textInputPlugin.string = @"textfield";
  // Creates a NSWindow so that the native text field can become first responder.
  NSWindow* window = [[NSWindow alloc] initWithContentRect:NSMakeRect(0, 0, 800, 600)
                                                 styleMask:NSBorderlessWindowMask
                                                   backing:NSBackingStoreBuffered
                                                     defer:NO];
  window.contentView = viewController.view;
  engine.semanticsEnabled = YES;

  auto bridge = viewController.accessibilityBridge.lock();
  // Initialize ax node data.
  FlutterSemanticsNode2 root;
  root.id = 0;
  root.flags = static_cast<FlutterSemanticsFlag>(0);
  root.actions = static_cast<FlutterSemanticsAction>(0);
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
  root.rect = {0, 0, 100, 100};  // LTRB
  root.transform = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  bridge->AddFlutterSemanticsNodeUpdate(root);

  double rectSize = 50;
  double transformFactor = 0.5;

  FlutterSemanticsNode2 child1;
  child1.id = 1;
  child1.flags = FlutterSemanticsFlag::kFlutterSemanticsFlagIsTextField;
  child1.actions = static_cast<FlutterSemanticsAction>(0);
  child1.label = "";
  child1.hint = "";
  child1.value = "textfield";
  child1.increased_value = "";
  child1.decreased_value = "";
  child1.tooltip = "";
  child1.text_selection_base = -1;
  child1.text_selection_extent = -1;
  child1.child_count = 0;
  child1.custom_accessibility_actions_count = 0;
  child1.rect = {0, 0, rectSize, rectSize};  // LTRB
  child1.transform = {transformFactor, 0, 0, 0, transformFactor, 0, 0, 0, 1};
  bridge->AddFlutterSemanticsNodeUpdate(child1);

  bridge->CommitUpdates();

  auto child_platform_node_delegate = bridge->GetFlutterPlatformNodeDelegateFromID(1).lock();
  // Verify the accessibility attribute matches.
  id native_accessibility = child_platform_node_delegate->GetNativeViewAccessible();
  EXPECT_EQ([native_accessibility isKindOfClass:[FlutterTextField class]], YES);
  FlutterTextField* native_text_field = (FlutterTextField*)native_accessibility;

  NSView* view = viewController.flutterView;
  CGRect scaledBounds = [view convertRectToBacking:view.bounds];
  CGSize scaledSize = scaledBounds.size;
  double pixelRatio = view.bounds.size.width == 0 ? 1 : scaledSize.width / view.bounds.size.width;

  double expectedFrameSize = rectSize * transformFactor / pixelRatio;
  EXPECT_EQ(NSEqualRects(native_text_field.frame, NSMakeRect(0, 600 - expectedFrameSize,
                                                             expectedFrameSize, expectedFrameSize)),
            YES);

  [native_text_field startEditing];
  EXPECT_EQ([native_text_field.stringValue isEqualToString:@"textfield"], YES);
}

TEST(FlutterPlatformNodeDelegateMac, ChangingFlagsUpdatesNativeViewAccessible) {
  FlutterViewController* viewController = CreateTestViewController();
  FlutterEngine* engine = viewController.engine;
  [viewController loadView];

  // Creates a NSWindow so that the native text field can become first responder.
  NSWindow* window = [[NSWindow alloc] initWithContentRect:NSMakeRect(0, 0, 800, 600)
                                                 styleMask:NSBorderlessWindowMask
                                                   backing:NSBackingStoreBuffered
                                                     defer:NO];
  window.contentView = viewController.view;
  engine.semanticsEnabled = YES;

  auto bridge = viewController.accessibilityBridge.lock();
  // Initialize ax node data.
  FlutterSemanticsNode2 root;
  root.id = 0;
  root.flags = static_cast<FlutterSemanticsFlag>(0);
  root.actions = static_cast<FlutterSemanticsAction>(0);
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
  root.rect = {0, 0, 100, 100};  // LTRB
  root.transform = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  bridge->AddFlutterSemanticsNodeUpdate(root);

  double rectSize = 50;
  double transformFactor = 0.5;

  FlutterSemanticsNode2 child1;
  child1.id = 1;
  child1.flags = static_cast<FlutterSemanticsFlag>(0);
  child1.actions = static_cast<FlutterSemanticsAction>(0);
  child1.label = "";
  child1.hint = "";
  child1.value = "textfield";
  child1.increased_value = "";
  child1.decreased_value = "";
  child1.tooltip = "";
  child1.text_selection_base = -1;
  child1.text_selection_extent = -1;
  child1.child_count = 0;
  child1.custom_accessibility_actions_count = 0;
  child1.rect = {0, 0, rectSize, rectSize};  // LTRB
  child1.transform = {transformFactor, 0, 0, 0, transformFactor, 0, 0, 0, 1};
  bridge->AddFlutterSemanticsNodeUpdate(child1);

  bridge->CommitUpdates();

  auto child_platform_node_delegate = bridge->GetFlutterPlatformNodeDelegateFromID(1).lock();
  // Verify the accessibility attribute matches.
  id native_accessibility = child_platform_node_delegate->GetNativeViewAccessible();
  EXPECT_TRUE([[native_accessibility className] isEqualToString:@"AXPlatformNodeCocoa"]);

  // Converting child to text field should produce `FlutterTextField` native view accessible.
  child1.flags = FlutterSemanticsFlag::kFlutterSemanticsFlagIsTextField;
  bridge->AddFlutterSemanticsNodeUpdate(child1);
  bridge->CommitUpdates();

  native_accessibility = child_platform_node_delegate->GetNativeViewAccessible();
  EXPECT_TRUE([native_accessibility isKindOfClass:[FlutterTextField class]]);

  child1.flags = static_cast<FlutterSemanticsFlag>(0);
  bridge->AddFlutterSemanticsNodeUpdate(child1);
  bridge->CommitUpdates();

  native_accessibility = child_platform_node_delegate->GetNativeViewAccessible();
  EXPECT_TRUE([[native_accessibility className] isEqualToString:@"AXPlatformNodeCocoa"]);
}

}  // namespace flutter::testing
