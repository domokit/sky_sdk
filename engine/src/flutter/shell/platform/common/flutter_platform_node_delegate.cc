// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter_platform_node_delegate.h"

#include <utility>

#include "flutter/shell/platform/common/accessibility_bridge.h"
#include "flutter/third_party/accessibility/ax/ax_action_data.h"
#include "flutter/third_party/accessibility/ax/ax_tree_manager_map.h"
#include "flutter/third_party/accessibility/gfx/geometry/rect_conversions.h"

namespace flutter {

FlutterPlatformNodeDelegate::FlutterPlatformNodeDelegate() = default;

FlutterPlatformNodeDelegate::~FlutterPlatformNodeDelegate() = default;

void FlutterPlatformNodeDelegate::Init(std::weak_ptr<OwnerBridge> bridge,
                                       ui::AXNode* node) {
  bridge_ = std::move(bridge);
  ax_node_ = node;
}

ui::AXNode* FlutterPlatformNodeDelegate::GetAXNode() const {
  return ax_node_;
}

bool FlutterPlatformNodeDelegate::AccessibilityPerformAction(
    const ui::AXActionData& data) {
  AccessibilityNodeId target = ax_node_->id();
  auto bridge_ptr = bridge_.lock();
  BASE_DCHECK(bridge_ptr);
  switch (data.action) {
    case ax::mojom::Action::kDoDefault:
      bridge_ptr->DispatchAccessibilityAction(
          target, FlutterSemanticsAction::kFlutterSemanticsActionTap, {});
      return true;
    case ax::mojom::Action::kFocus:
      bridge_ptr->SetLastFocusedId(target);
      bridge_ptr->DispatchAccessibilityAction(
          target,
          FlutterSemanticsAction::
              kFlutterSemanticsActionDidGainAccessibilityFocus,
          {});
      return true;
    case ax::mojom::Action::kScrollToMakeVisible:
      bridge_ptr->DispatchAccessibilityAction(
          target, FlutterSemanticsAction::kFlutterSemanticsActionShowOnScreen,
          {});
      return true;
    // TODO(chunhtai): support more actions.
    default:
      return false;
  }
  return false;
}

const ui::AXNodeData& FlutterPlatformNodeDelegate::GetData() const {
  return ax_node_->data();
}

gfx::NativeViewAccessible FlutterPlatformNodeDelegate::GetParent() {
  if (!ax_node_->parent()) {
    return nullptr;
  }
  auto bridge_ptr = bridge_.lock();
  BASE_DCHECK(bridge_ptr);
  return bridge_ptr->GetNativeAccessibleFromId(ax_node_->parent()->id());
}

gfx::NativeViewAccessible FlutterPlatformNodeDelegate::GetFocus() {
  auto bridge_ptr = bridge_.lock();
  BASE_DCHECK(bridge_ptr);
  AccessibilityNodeId last_focused = bridge_ptr->GetLastFocusedId();
  if (last_focused == ui::AXNode::kInvalidAXID) {
    return nullptr;
  }
  return bridge_ptr->GetNativeAccessibleFromId(last_focused);
}

int FlutterPlatformNodeDelegate::GetChildCount() const {
  return static_cast<int>(ax_node_->GetUnignoredChildCount());
}

gfx::NativeViewAccessible FlutterPlatformNodeDelegate::ChildAtIndex(int index) {
  auto bridge_ptr = bridge_.lock();
  BASE_DCHECK(bridge_ptr);
  AccessibilityNodeId child = ax_node_->GetUnignoredChildAtIndex(index)->id();
  return bridge_ptr->GetNativeAccessibleFromId(child);
}

gfx::Rect FlutterPlatformNodeDelegate::GetBoundsRect(
    const ui::AXCoordinateSystem coordinate_system,
    const ui::AXClippingBehavior clipping_behavior,
    ui::AXOffscreenResult* offscreen_result) const {
  auto bridge_ptr = bridge_.lock();
  BASE_DCHECK(bridge_ptr);
  // TODO(chunhtai): We need to apply screen dpr in here.
  // https://github.com/flutter/flutter/issues/74283
  const bool clip_bounds =
      clipping_behavior == ui::AXClippingBehavior::kClipped;
  bool offscreen = false;
  gfx::RectF bounds =
      bridge_ptr->RelativeToGlobalBounds(ax_node_, offscreen, clip_bounds);
  if (offscreen_result != nullptr) {
    *offscreen_result = offscreen ? ui::AXOffscreenResult::kOffscreen
                                  : ui::AXOffscreenResult::kOnscreen;
  }
  return gfx::ToEnclosingRect(bounds);
}

std::weak_ptr<FlutterPlatformNodeDelegate::OwnerBridge>
FlutterPlatformNodeDelegate::GetOwnerBridge() const {
  return bridge_;
}

ui::AXPlatformNode* FlutterPlatformNodeDelegate::GetPlatformNode() const {
  return nullptr;
}

gfx::NativeViewAccessible
FlutterPlatformNodeDelegate::GetLowestPlatformAncestor() const {
  auto bridge_ptr = bridge_.lock();
  BASE_DCHECK(bridge_ptr);
  auto lowest_platform_ancestor = ax_node_->GetLowestPlatformAncestor();
  if (lowest_platform_ancestor) {
    return bridge_ptr->GetNativeAccessibleFromId(
        ax_node_->GetLowestPlatformAncestor()->id());
  }
  return nullptr;
}

ui::AXNodePosition::AXPositionInstance
FlutterPlatformNodeDelegate::CreateTextPositionAt(int offset) const {
  return ui::AXNodePosition::CreatePosition(*ax_node_, offset);
}

ui::AXPlatformNode* FlutterPlatformNodeDelegate::GetFromNodeID(
    int32_t node_id) {
  ui::AXTreeManager* tree_manager =
      ui::AXTreeManagerMap::GetInstance().GetManager(
          ax_node_->tree()->GetAXTreeID());
  AccessibilityBridge* platform_manager =
      static_cast<AccessibilityBridge*>(tree_manager);
  return platform_manager->GetPlatformNodeFromTree(node_id);
}

ui::AXPlatformNode* FlutterPlatformNodeDelegate::GetFromTreeIDAndNodeID(
    const ui::AXTreeID& tree_id,
    int32_t node_id) {
  ui::AXTreeManager* tree_manager =
      ui::AXTreeManagerMap::GetInstance().GetManager(tree_id);
  AccessibilityBridge* platform_manager =
      static_cast<AccessibilityBridge*>(tree_manager);
  return platform_manager->GetPlatformNodeFromTree(node_id);
}

const ui::AXTree::Selection FlutterPlatformNodeDelegate::GetUnignoredSelection()
    const {
  return ax_node_->GetUnignoredSelection();
}

}  // namespace flutter
