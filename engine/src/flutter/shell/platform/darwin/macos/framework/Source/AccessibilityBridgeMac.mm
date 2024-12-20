// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/macos/framework/Source/AccessibilityBridgeMac.h"

#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterEngine_Internal.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterPlatformNodeDelegateMac.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterTextInputSemanticsObject.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterViewController_Internal.h"
#include "flutter/shell/platform/embedder/embedder.h"

namespace flutter {

// Native mac notifications fired. These notifications are not publicly documented.
static NSString* const kAccessibilityLoadCompleteNotification = @"AXLoadComplete";
static NSString* const kAccessibilityInvalidStatusChangedNotification = @"AXInvalidStatusChanged";
static NSString* const kAccessibilityLiveRegionCreatedNotification = @"AXLiveRegionCreated";
static NSString* const kAccessibilityLiveRegionChangedNotification = @"AXLiveRegionChanged";
static NSString* const kAccessibilityExpandedChanged = @"AXExpandedChanged";
static NSString* const kAccessibilityMenuItemSelectedNotification = @"AXMenuItemSelected";

AccessibilityBridgeMac::AccessibilityBridgeMac(__weak FlutterEngine* flutter_engine,
                                               __weak FlutterViewController* view_controller)
    : flutter_engine_(flutter_engine), view_controller_(view_controller) {}

void AccessibilityBridgeMac::OnAccessibilityEvent(
    ui::AXEventGenerator::TargetedEvent targeted_event) {
  if (!view_controller_.viewLoaded || !view_controller_.view.window) {
    // Don't need to send accessibility events if the there is no view or window.
    return;
  }
  ui::AXNode* ax_node = targeted_event.node;
  std::vector<AccessibilityBridgeMac::NSAccessibilityEvent> events =
      MacOSEventsFromAXEvent(targeted_event.event_params.event, *ax_node);
  for (const AccessibilityBridgeMac::NSAccessibilityEvent& event : events) {
    if (event.user_info != nil) {
      DispatchMacOSNotificationWithUserInfo(event.target, event.name, event.user_info);
    } else {
      DispatchMacOSNotification(event.target, event.name);
    }
  }
}

std::vector<AccessibilityBridgeMac::NSAccessibilityEvent>
AccessibilityBridgeMac::MacOSEventsFromAXEvent(ui::AXEventGenerator::Event event_type,
                                               const ui::AXNode& ax_node) const {
  // Gets the native_node with the node_id.
  NSCAssert(flutter_engine_, @"Flutter engine should not be deallocated");
  auto platform_node_delegate = GetFlutterPlatformNodeDelegateFromID(ax_node.id()).lock();
  NSCAssert(platform_node_delegate, @"Event target must exist in accessibility bridge.");
  auto mac_platform_node_delegate =
      std::static_pointer_cast<FlutterPlatformNodeDelegateMac>(platform_node_delegate);
  gfx::NativeViewAccessible native_node = mac_platform_node_delegate->GetNativeViewAccessible();

  std::vector<AccessibilityBridgeMac::NSAccessibilityEvent> events;
  switch (event_type) {
    case ui::AXEventGenerator::Event::ACTIVE_DESCENDANT_CHANGED:
      if (ax_node.data().role == ax::mojom::Role::kTree) {
        events.push_back({
            .name = NSAccessibilitySelectedRowsChangedNotification,
            .target = native_node,
            .user_info = nil,
        });
      } else if (ax_node.data().role == ax::mojom::Role::kTextFieldWithComboBox) {
        // Even though the selected item in the combo box has changed, don't
        // post a focus change because this will take the focus out of
        // the combo box where the user might be typing.
        events.push_back({
            .name = NSAccessibilitySelectedChildrenChangedNotification,
            .target = native_node,
            .user_info = nil,
        });
      }
      // In all other cases, this delegate should post
      // |NSAccessibilityFocusedUIElementChangedNotification|, but this is
      // handled elsewhere.
      break;
    case ui::AXEventGenerator::Event::LOAD_COMPLETE:
      events.push_back({
          .name = kAccessibilityLoadCompleteNotification,
          .target = native_node,
          .user_info = nil,
      });
      break;
    case ui::AXEventGenerator::Event::INVALID_STATUS_CHANGED:
      events.push_back({
          .name = kAccessibilityInvalidStatusChangedNotification,
          .target = native_node,
          .user_info = nil,
      });
      break;
    case ui::AXEventGenerator::Event::SELECTED_CHILDREN_CHANGED:
      if (ui::IsTableLike(ax_node.data().role)) {
        events.push_back({
            .name = NSAccessibilitySelectedRowsChangedNotification,
            .target = native_node,
            .user_info = nil,
        });
      } else {
        // VoiceOver does not read anything if selection changes on the
        // currently focused object, and the focus did not move. Fire a
        // selection change if the focus did not change.
        NSAccessibilityElement* native_accessibility_node = (NSAccessibilityElement*)native_node;
        if (native_accessibility_node.accessibilityFocusedUIElement &&
            ax_node.data().HasState(ax::mojom::State::kMultiselectable) &&
            !HasPendingEvent(ui::AXEventGenerator::Event::ACTIVE_DESCENDANT_CHANGED) &&
            !HasPendingEvent(ui::AXEventGenerator::Event::FOCUS_CHANGED)) {
          // Don't fire selected children change, it will sometimes override
          // announcement of current focus.
          break;
        }
        events.push_back({
            .name = NSAccessibilitySelectedChildrenChangedNotification,
            .target = native_node,
            .user_info = nil,
        });
      }
      break;
    case ui::AXEventGenerator::Event::DOCUMENT_SELECTION_CHANGED: {
      id focused = mac_platform_node_delegate->GetFocus();
      if ([focused isKindOfClass:[FlutterTextField class]]) {
        // If it is a text field, the selection notifications are handled by
        // the FlutterTextField directly. Only need to make sure it is the
        // first responder.
        FlutterTextField* native_text_field = (FlutterTextField*)focused;
        if (native_text_field == mac_platform_node_delegate->GetFocus()) {
          [native_text_field startEditing];
        }
        break;
      }
      // This event always fires at root
      events.push_back({
          .name = NSAccessibilitySelectedTextChangedNotification,
          .target = native_node,
          .user_info = nil,
      });
      // WebKit fires a notification both on the focused object and the page
      // root.
      const ui::AXTreeData& tree_data = GetAXTreeData();
      int32_t focus = tree_data.focus_id;
      if (focus == ui::AXNode::kInvalidAXID || focus != tree_data.sel_anchor_object_id) {
        break;  // Just fire a notification on the root.
      }
      auto focus_node = GetFlutterPlatformNodeDelegateFromID(focus).lock();
      if (!focus_node) {
        break;  // Just fire a notification on the root.
      }
      events.push_back({
          .name = NSAccessibilitySelectedTextChangedNotification,
          .target = focus_node->GetNativeViewAccessible(),
          .user_info = nil,
      });
      break;
    }
    case ui::AXEventGenerator::Event::CHECKED_STATE_CHANGED:
      events.push_back({
          .name = NSAccessibilityValueChangedNotification,
          .target = native_node,
          .user_info = nil,
      });
      break;
    case ui::AXEventGenerator::Event::VALUE_CHANGED: {
      if (ax_node.data().role == ax::mojom::Role::kTextField) {
        // If it is a text field, the value change notifications are handled by
        // the FlutterTextField directly. Only need to make sure it is the
        // first responder.
        id native_text_field = mac_platform_node_delegate->GetNativeViewAccessible();
        FML_DCHECK([native_text_field isKindOfClass:FlutterTextField.class]);
        id focused = mac_platform_node_delegate->GetFocus();
        if (!focused || native_text_field == focused) {
          [(FlutterTextField*)native_text_field startEditing];
        }
        break;
      }
      events.push_back({
          .name = NSAccessibilityValueChangedNotification,
          .target = native_node,
          .user_info = nil,
      });
      if (ax_node.data().HasState(ax::mojom::State::kEditable)) {
        events.push_back({
            .name = NSAccessibilityValueChangedNotification,
            .target = RootDelegate()->GetNativeViewAccessible(),
            .user_info = nil,
        });
      }
      break;
    }
    case ui::AXEventGenerator::Event::LIVE_REGION_CREATED:
      events.push_back({
          .name = kAccessibilityLiveRegionCreatedNotification,
          .target = native_node,
          .user_info = nil,
      });
      break;
    case ui::AXEventGenerator::Event::ALERT: {
      events.push_back({
          .name = kAccessibilityLiveRegionCreatedNotification,
          .target = native_node,
          .user_info = nil,
      });
      // VoiceOver requires a live region changed notification to actually
      // announce the live region.
      auto live_region_events =
          MacOSEventsFromAXEvent(ui::AXEventGenerator::Event::LIVE_REGION_CHANGED, ax_node);
      events.insert(events.end(), live_region_events.begin(), live_region_events.end());
      break;
    }
    case ui::AXEventGenerator::Event::LIVE_REGION_CHANGED: {
      // Uses native VoiceOver support for live regions.
      events.push_back({
          .name = kAccessibilityLiveRegionChangedNotification,
          .target = native_node,
          .user_info = nil,
      });
      break;
    }
    case ui::AXEventGenerator::Event::ROW_COUNT_CHANGED:
      events.push_back({
          .name = NSAccessibilityRowCountChangedNotification,
          .target = native_node,
          .user_info = nil,
      });
      break;
    case ui::AXEventGenerator::Event::EXPANDED: {
      NSAccessibilityNotificationName mac_notification;
      if (ax_node.data().role == ax::mojom::Role::kRow ||
          ax_node.data().role == ax::mojom::Role::kTreeItem) {
        mac_notification = NSAccessibilityRowExpandedNotification;
      } else {
        mac_notification = kAccessibilityExpandedChanged;
      }
      events.push_back({
          .name = mac_notification,
          .target = native_node,
          .user_info = nil,
      });
      break;
    }
    case ui::AXEventGenerator::Event::COLLAPSED: {
      NSAccessibilityNotificationName mac_notification;
      if (ax_node.data().role == ax::mojom::Role::kRow ||
          ax_node.data().role == ax::mojom::Role::kTreeItem) {
        mac_notification = NSAccessibilityRowCollapsedNotification;
      } else {
        mac_notification = kAccessibilityExpandedChanged;
      }
      events.push_back({
          .name = mac_notification,
          .target = native_node,
          .user_info = nil,
      });
      break;
    }
    case ui::AXEventGenerator::Event::MENU_ITEM_SELECTED:
      events.push_back({
          .name = kAccessibilityMenuItemSelectedNotification,
          .target = native_node,
          .user_info = nil,
      });
      break;
    case ui::AXEventGenerator::Event::CHILDREN_CHANGED: {
      // NSAccessibilityCreatedNotification seems to be the only way to let
      // Voiceover pick up layout changes.
      events.push_back({
          .name = NSAccessibilityCreatedNotification,
          .target = view_controller_.view.window,
          .user_info = nil,
      });
      break;
    }
    case ui::AXEventGenerator::Event::SUBTREE_CREATED:
    case ui::AXEventGenerator::Event::ACCESS_KEY_CHANGED:
    case ui::AXEventGenerator::Event::ATK_TEXT_OBJECT_ATTRIBUTE_CHANGED:
    case ui::AXEventGenerator::Event::ATOMIC_CHANGED:
    case ui::AXEventGenerator::Event::AUTO_COMPLETE_CHANGED:
    case ui::AXEventGenerator::Event::BUSY_CHANGED:
    case ui::AXEventGenerator::Event::CONTROLS_CHANGED:
    case ui::AXEventGenerator::Event::CLASS_NAME_CHANGED:
    case ui::AXEventGenerator::Event::DESCRIBED_BY_CHANGED:
    case ui::AXEventGenerator::Event::DESCRIPTION_CHANGED:
    case ui::AXEventGenerator::Event::DOCUMENT_TITLE_CHANGED:
    case ui::AXEventGenerator::Event::DROPEFFECT_CHANGED:
    case ui::AXEventGenerator::Event::ENABLED_CHANGED:
    case ui::AXEventGenerator::Event::FOCUS_CHANGED:
    case ui::AXEventGenerator::Event::FLOW_FROM_CHANGED:
    case ui::AXEventGenerator::Event::FLOW_TO_CHANGED:
    case ui::AXEventGenerator::Event::GRABBED_CHANGED:
    case ui::AXEventGenerator::Event::HASPOPUP_CHANGED:
    case ui::AXEventGenerator::Event::HIERARCHICAL_LEVEL_CHANGED:
    case ui::AXEventGenerator::Event::IGNORED_CHANGED:
    case ui::AXEventGenerator::Event::IMAGE_ANNOTATION_CHANGED:
    case ui::AXEventGenerator::Event::KEY_SHORTCUTS_CHANGED:
    case ui::AXEventGenerator::Event::LABELED_BY_CHANGED:
    case ui::AXEventGenerator::Event::LANGUAGE_CHANGED:
    case ui::AXEventGenerator::Event::LAYOUT_INVALIDATED:
    case ui::AXEventGenerator::Event::LIVE_REGION_NODE_CHANGED:
    case ui::AXEventGenerator::Event::LIVE_RELEVANT_CHANGED:
    case ui::AXEventGenerator::Event::LIVE_STATUS_CHANGED:
    case ui::AXEventGenerator::Event::LOAD_START:
    case ui::AXEventGenerator::Event::MULTILINE_STATE_CHANGED:
    case ui::AXEventGenerator::Event::MULTISELECTABLE_STATE_CHANGED:
    case ui::AXEventGenerator::Event::NAME_CHANGED:
    case ui::AXEventGenerator::Event::OBJECT_ATTRIBUTE_CHANGED:
    case ui::AXEventGenerator::Event::OTHER_ATTRIBUTE_CHANGED:
    case ui::AXEventGenerator::Event::PLACEHOLDER_CHANGED:
    case ui::AXEventGenerator::Event::PORTAL_ACTIVATED:
    case ui::AXEventGenerator::Event::POSITION_IN_SET_CHANGED:
    case ui::AXEventGenerator::Event::READONLY_CHANGED:
    case ui::AXEventGenerator::Event::RELATED_NODE_CHANGED:
    case ui::AXEventGenerator::Event::REQUIRED_STATE_CHANGED:
    case ui::AXEventGenerator::Event::ROLE_CHANGED:
    case ui::AXEventGenerator::Event::SCROLL_HORIZONTAL_POSITION_CHANGED:
    case ui::AXEventGenerator::Event::SCROLL_VERTICAL_POSITION_CHANGED:
    case ui::AXEventGenerator::Event::SELECTED_CHANGED:
    case ui::AXEventGenerator::Event::SET_SIZE_CHANGED:
    case ui::AXEventGenerator::Event::SORT_CHANGED:
    case ui::AXEventGenerator::Event::STATE_CHANGED:
    case ui::AXEventGenerator::Event::TEXT_ATTRIBUTE_CHANGED:
    case ui::AXEventGenerator::Event::VALUE_MAX_CHANGED:
    case ui::AXEventGenerator::Event::VALUE_MIN_CHANGED:
    case ui::AXEventGenerator::Event::VALUE_STEP_CHANGED:
    case ui::AXEventGenerator::Event::WIN_IACCESSIBLE_STATE_CHANGED:
      // There are some notifications that aren't meaningful on Mac.
      // It's okay to skip them.
      break;
  }
  return events;
}

void AccessibilityBridgeMac::DispatchAccessibilityAction(ui::AXNode::AXID target,
                                                         FlutterSemanticsAction action,
                                                         fml::MallocMapping data) {
  NSCAssert(flutter_engine_, @"Flutter engine should not be deallocated");
  NSCAssert(view_controller_.viewLoaded && view_controller_.view.window,
            @"The accessibility bridge should not receive accessibility actions if the flutter view"
            @"is not loaded or attached to a NSWindow.");
  [flutter_engine_ dispatchSemanticsAction:action toTarget:target withData:std::move(data)];
}

std::shared_ptr<FlutterPlatformNodeDelegate>
AccessibilityBridgeMac::CreateFlutterPlatformNodeDelegate() {
  return std::make_shared<FlutterPlatformNodeDelegateMac>(weak_from_this(), view_controller_);
}

// Private method
void AccessibilityBridgeMac::DispatchMacOSNotification(
    gfx::NativeViewAccessible native_node,
    NSAccessibilityNotificationName mac_notification) {
  NSCAssert(mac_notification, @"The notification must not be null.");
  NSCAssert(native_node, @"The notification target must not be null.");
  NSAccessibilityPostNotification(native_node, mac_notification);
}

void AccessibilityBridgeMac::DispatchMacOSNotificationWithUserInfo(
    gfx::NativeViewAccessible native_node,
    NSAccessibilityNotificationName mac_notification,
    NSDictionary* user_info) {
  NSCAssert(mac_notification, @"The notification must not be null.");
  NSCAssert(native_node, @"The notification target must not be null.");
  NSCAssert(user_info, @"The notification data must not be null.");
  NSAccessibilityPostNotificationWithUserInfo(native_node, mac_notification, user_info);
}

bool AccessibilityBridgeMac::HasPendingEvent(ui::AXEventGenerator::Event event) const {
  NSCAssert(flutter_engine_, @"Flutter engine should not be deallocated");
  std::vector<ui::AXEventGenerator::TargetedEvent> pending_events = GetPendingEvents();
  for (const auto& pending_event : GetPendingEvents()) {
    if (pending_event.event_params.event == event) {
      return true;
    }
  }
  return false;
}

}  // namespace flutter
