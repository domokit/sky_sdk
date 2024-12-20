// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/windows/accessibility_bridge_windows.h"

#include "flutter/fml/logging.h"
#include "flutter/shell/platform/windows/flutter_platform_node_delegate_windows.h"
#include "flutter/third_party/accessibility/ax/platform/ax_platform_node_delegate_base.h"

namespace flutter {

AccessibilityBridgeWindows::AccessibilityBridgeWindows(FlutterWindowsView* view)
    : view_(view) {
  FML_DCHECK(view_);
}

void AccessibilityBridgeWindows::OnAccessibilityEvent(
    ui::AXEventGenerator::TargetedEvent targeted_event) {
  ui::AXNode* ax_node = targeted_event.node;
  ui::AXEventGenerator::Event event_type = targeted_event.event_params.event;

  auto node_delegate =
      GetFlutterPlatformNodeDelegateFromID(ax_node->id()).lock();
  FML_DCHECK(node_delegate)
      << "No FlutterPlatformNodeDelegate found for node ID " << ax_node->id();
  std::shared_ptr<FlutterPlatformNodeDelegateWindows> win_delegate =
      std::static_pointer_cast<FlutterPlatformNodeDelegateWindows>(
          node_delegate);

  switch (event_type) {
    case ui::AXEventGenerator::Event::ALERT:
      DispatchWinAccessibilityEvent(win_delegate, ax::mojom::Event::kAlert);
      break;
    case ui::AXEventGenerator::Event::CHECKED_STATE_CHANGED:
      DispatchWinAccessibilityEvent(win_delegate,
                                    ax::mojom::Event::kValueChanged);
      break;
    case ui::AXEventGenerator::Event::CHILDREN_CHANGED:
      DispatchWinAccessibilityEvent(win_delegate,
                                    ax::mojom::Event::kChildrenChanged);
      break;
    case ui::AXEventGenerator::Event::DOCUMENT_SELECTION_CHANGED: {
      // An event indicating a change in document selection should be fired
      // only for the focused node whose selection has changed. If a valid
      // caret and selection exist in the app tree, they must both be within
      // the focus node.
      auto focus_delegate = GetFocusedNode().lock();
      if (focus_delegate) {
        win_delegate =
            std::static_pointer_cast<FlutterPlatformNodeDelegateWindows>(
                focus_delegate);
      }
      DispatchWinAccessibilityEvent(
          win_delegate, ax::mojom::Event::kDocumentSelectionChanged);
      break;
    }
    case ui::AXEventGenerator::Event::FOCUS_CHANGED:
      DispatchWinAccessibilityEvent(win_delegate, ax::mojom::Event::kFocus);
      SetFocus(win_delegate);
      break;
    case ui::AXEventGenerator::Event::IGNORED_CHANGED:
      if (ax_node->IsIgnored()) {
        DispatchWinAccessibilityEvent(win_delegate, ax::mojom::Event::kHide);
      }
      break;
    case ui::AXEventGenerator::Event::IMAGE_ANNOTATION_CHANGED:
      DispatchWinAccessibilityEvent(win_delegate,
                                    ax::mojom::Event::kTextChanged);
      break;
    case ui::AXEventGenerator::Event::LIVE_REGION_CHANGED:
      DispatchWinAccessibilityEvent(win_delegate,
                                    ax::mojom::Event::kLiveRegionChanged);
      break;
    case ui::AXEventGenerator::Event::NAME_CHANGED:
      DispatchWinAccessibilityEvent(win_delegate,
                                    ax::mojom::Event::kTextChanged);
      break;
    case ui::AXEventGenerator::Event::SCROLL_HORIZONTAL_POSITION_CHANGED:
      DispatchWinAccessibilityEvent(win_delegate,
                                    ax::mojom::Event::kScrollPositionChanged);
      break;
    case ui::AXEventGenerator::Event::SCROLL_VERTICAL_POSITION_CHANGED:
      DispatchWinAccessibilityEvent(win_delegate,
                                    ax::mojom::Event::kScrollPositionChanged);
      break;
    case ui::AXEventGenerator::Event::SELECTED_CHANGED:
      DispatchWinAccessibilityEvent(win_delegate,
                                    ax::mojom::Event::kValueChanged);
      break;
    case ui::AXEventGenerator::Event::SELECTED_CHILDREN_CHANGED:
      DispatchWinAccessibilityEvent(win_delegate,
                                    ax::mojom::Event::kSelectedChildrenChanged);
      break;
    case ui::AXEventGenerator::Event::SUBTREE_CREATED:
      DispatchWinAccessibilityEvent(win_delegate, ax::mojom::Event::kShow);
      break;
    case ui::AXEventGenerator::Event::VALUE_CHANGED:
      DispatchWinAccessibilityEvent(win_delegate,
                                    ax::mojom::Event::kValueChanged);
      break;
    case ui::AXEventGenerator::Event::WIN_IACCESSIBLE_STATE_CHANGED:
      DispatchWinAccessibilityEvent(win_delegate,
                                    ax::mojom::Event::kStateChanged);
      break;
    case ui::AXEventGenerator::Event::ACCESS_KEY_CHANGED:
    case ui::AXEventGenerator::Event::ACTIVE_DESCENDANT_CHANGED:
    case ui::AXEventGenerator::Event::ATK_TEXT_OBJECT_ATTRIBUTE_CHANGED:
    case ui::AXEventGenerator::Event::ATOMIC_CHANGED:
    case ui::AXEventGenerator::Event::AUTO_COMPLETE_CHANGED:
    case ui::AXEventGenerator::Event::BUSY_CHANGED:
    case ui::AXEventGenerator::Event::CLASS_NAME_CHANGED:
    case ui::AXEventGenerator::Event::COLLAPSED:
    case ui::AXEventGenerator::Event::CONTROLS_CHANGED:
    case ui::AXEventGenerator::Event::DESCRIBED_BY_CHANGED:
    case ui::AXEventGenerator::Event::DESCRIPTION_CHANGED:
    case ui::AXEventGenerator::Event::DOCUMENT_TITLE_CHANGED:
    case ui::AXEventGenerator::Event::DROPEFFECT_CHANGED:
    case ui::AXEventGenerator::Event::ENABLED_CHANGED:
    case ui::AXEventGenerator::Event::EXPANDED:
    case ui::AXEventGenerator::Event::FLOW_FROM_CHANGED:
    case ui::AXEventGenerator::Event::FLOW_TO_CHANGED:
    case ui::AXEventGenerator::Event::GRABBED_CHANGED:
    case ui::AXEventGenerator::Event::HASPOPUP_CHANGED:
    case ui::AXEventGenerator::Event::HIERARCHICAL_LEVEL_CHANGED:
    case ui::AXEventGenerator::Event::INVALID_STATUS_CHANGED:
    case ui::AXEventGenerator::Event::KEY_SHORTCUTS_CHANGED:
    case ui::AXEventGenerator::Event::LABELED_BY_CHANGED:
    case ui::AXEventGenerator::Event::LANGUAGE_CHANGED:
    case ui::AXEventGenerator::Event::LAYOUT_INVALIDATED:
    case ui::AXEventGenerator::Event::LIVE_REGION_CREATED:
    case ui::AXEventGenerator::Event::LIVE_REGION_NODE_CHANGED:
    case ui::AXEventGenerator::Event::LIVE_RELEVANT_CHANGED:
    case ui::AXEventGenerator::Event::LIVE_STATUS_CHANGED:
    case ui::AXEventGenerator::Event::LOAD_COMPLETE:
    case ui::AXEventGenerator::Event::LOAD_START:
    case ui::AXEventGenerator::Event::MENU_ITEM_SELECTED:
    case ui::AXEventGenerator::Event::MULTILINE_STATE_CHANGED:
    case ui::AXEventGenerator::Event::MULTISELECTABLE_STATE_CHANGED:
    case ui::AXEventGenerator::Event::OBJECT_ATTRIBUTE_CHANGED:
    case ui::AXEventGenerator::Event::OTHER_ATTRIBUTE_CHANGED:
    case ui::AXEventGenerator::Event::PLACEHOLDER_CHANGED:
    case ui::AXEventGenerator::Event::PORTAL_ACTIVATED:
    case ui::AXEventGenerator::Event::POSITION_IN_SET_CHANGED:
    case ui::AXEventGenerator::Event::READONLY_CHANGED:
    case ui::AXEventGenerator::Event::RELATED_NODE_CHANGED:
    case ui::AXEventGenerator::Event::REQUIRED_STATE_CHANGED:
    case ui::AXEventGenerator::Event::ROLE_CHANGED:
    case ui::AXEventGenerator::Event::ROW_COUNT_CHANGED:
    case ui::AXEventGenerator::Event::SET_SIZE_CHANGED:
    case ui::AXEventGenerator::Event::SORT_CHANGED:
    case ui::AXEventGenerator::Event::STATE_CHANGED:
    case ui::AXEventGenerator::Event::TEXT_ATTRIBUTE_CHANGED:
    case ui::AXEventGenerator::Event::VALUE_MAX_CHANGED:
    case ui::AXEventGenerator::Event::VALUE_MIN_CHANGED:
    case ui::AXEventGenerator::Event::VALUE_STEP_CHANGED:
      // Unhandled event type.
      break;
  }
}

void AccessibilityBridgeWindows::DispatchAccessibilityAction(
    AccessibilityNodeId target,
    FlutterSemanticsAction action,
    fml::MallocMapping data) {
  view_->GetEngine()->DispatchSemanticsAction(target, action, std::move(data));
}

std::shared_ptr<FlutterPlatformNodeDelegate>
AccessibilityBridgeWindows::CreateFlutterPlatformNodeDelegate() {
  return std::make_shared<FlutterPlatformNodeDelegateWindows>(
      shared_from_this(), view_);
}

void AccessibilityBridgeWindows::DispatchWinAccessibilityEvent(
    std::shared_ptr<FlutterPlatformNodeDelegateWindows> node_delegate,
    ax::mojom::Event event_type) {
  node_delegate->DispatchWinAccessibilityEvent(event_type);
}

void AccessibilityBridgeWindows::SetFocus(
    std::shared_ptr<FlutterPlatformNodeDelegateWindows> node_delegate) {
  node_delegate->SetFocus();
}

gfx::NativeViewAccessible
AccessibilityBridgeWindows::GetChildOfAXFragmentRoot() {
  ui::AXPlatformNodeDelegate* root_delegate = RootDelegate();
  if (!root_delegate) {
    return nullptr;
  }

  return root_delegate->GetNativeViewAccessible();
}

gfx::NativeViewAccessible
AccessibilityBridgeWindows::GetParentOfAXFragmentRoot() {
  return nullptr;
}

bool AccessibilityBridgeWindows::IsAXFragmentRootAControlElement() {
  return true;
}

std::weak_ptr<FlutterPlatformNodeDelegate>
AccessibilityBridgeWindows::GetFocusedNode() {
  ui::AXNode::AXID focus_id = GetAXTreeData().sel_focus_object_id;
  return GetFlutterPlatformNodeDelegateFromID(focus_id);
}

}  // namespace flutter
