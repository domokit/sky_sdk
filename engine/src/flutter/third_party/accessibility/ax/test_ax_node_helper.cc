// Copyright 2020 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "test_ax_node_helper.h"

#include <map>
#include <utility>

#include "ax_action_data.h"
#include "ax_role_properties.h"
#include "ax_table_info.h"
#include "ax_tree_observer.h"
#include "base/numerics/ranges.h"
#include "gfx/geometry/rect_conversions.h"

namespace ui {

namespace {

// A global map from AXNodes to TestAXNodeHelpers.
std::map<AXNode::AXID, TestAXNodeHelper*> g_node_id_to_helper_map;

// A simple implementation of AXTreeObserver to catch when AXNodes are
// deleted so we can delete their helpers.
class TestAXTreeObserver : public AXTreeObserver {
 private:
  void OnNodeDeleted(AXTree* tree, int32_t node_id) override {
    const auto iter = g_node_id_to_helper_map.find(node_id);
    if (iter != g_node_id_to_helper_map.end()) {
      TestAXNodeHelper* helper = iter->second;
      delete helper;
      g_node_id_to_helper_map.erase(node_id);
    }
  }
};

TestAXTreeObserver g_ax_tree_observer;

}  // namespace

// static
TestAXNodeHelper* TestAXNodeHelper::GetOrCreate(AXTree* tree, AXNode* node) {
  if (!tree || !node)
    return nullptr;

  if (!tree->HasObserver(&g_ax_tree_observer))
    tree->AddObserver(&g_ax_tree_observer);
  auto iter = g_node_id_to_helper_map.find(node->id());
  if (iter != g_node_id_to_helper_map.end())
    return iter->second;
  TestAXNodeHelper* helper = new TestAXNodeHelper(tree, node);
  g_node_id_to_helper_map[node->id()] = helper;
  return helper;
}

TestAXNodeHelper::TestAXNodeHelper(AXTree* tree, AXNode* node)
    : tree_(tree), node_(node) {}

TestAXNodeHelper::~TestAXNodeHelper() = default;

gfx::Rect TestAXNodeHelper::GetBoundsRect(
    const AXCoordinateSystem coordinate_system,
    const AXClippingBehavior clipping_behavior,
    AXOffscreenResult* offscreen_result) const {
  switch (coordinate_system) {
    case AXCoordinateSystem::kScreenPhysicalPixels:
      // For unit testing purposes, assume a device scale factor of 1 and fall
      // through.
    case AXCoordinateSystem::kScreenDIPs: {
      // We could optionally add clipping here if ever needed.
      gfx::RectF bounds = GetLocation();

      // For test behavior only, for bounds that are offscreen we currently do
      // not apply clipping to the bounds but we still return the offscreen
      // status.
      if (offscreen_result) {
        *offscreen_result = DetermineOffscreenResult(bounds);
      }

      return gfx::ToEnclosingRect(bounds);
    }
    case AXCoordinateSystem::kRootFrame:
    case AXCoordinateSystem::kFrame:
      BASE_UNREACHABLE();
      return gfx::Rect();
  }
}

gfx::Rect TestAXNodeHelper::GetInnerTextRangeBoundsRect(
    const int start_offset,
    const int end_offset,
    const AXCoordinateSystem coordinate_system,
    const AXClippingBehavior clipping_behavior,
    AXOffscreenResult* offscreen_result) const {
  switch (coordinate_system) {
    case AXCoordinateSystem::kScreenPhysicalPixels:
    // For unit testing purposes, assume a device scale factor of 1 and fall
    // through.
    case AXCoordinateSystem::kScreenDIPs: {
      gfx::RectF bounds = GetLocation();
      // This implementation currently only deals with text node that has role
      // kInlineTextBox and kStaticText.
      // For test purposes, assume node with kStaticText always has a single
      // child with role kInlineTextBox.
      if (GetData().role == ax::mojom::Role::kInlineTextBox) {
        bounds = GetInlineTextRect(start_offset, end_offset);
      } else if (GetData().role == ax::mojom::Role::kStaticText &&
                 InternalChildCount() > 0) {
        TestAXNodeHelper* child = InternalGetChild(0);
        if (child != nullptr &&
            child->GetData().role == ax::mojom::Role::kInlineTextBox) {
          bounds = child->GetInlineTextRect(start_offset, end_offset);
        }
      }

      // For test behavior only, for bounds that are offscreen we currently do
      // not apply clipping to the bounds but we still return the offscreen
      // status.
      if (offscreen_result) {
        *offscreen_result = DetermineOffscreenResult(bounds);
      }

      return gfx::ToEnclosingRect(bounds);
    }
    case AXCoordinateSystem::kRootFrame:
    case AXCoordinateSystem::kFrame:
      BASE_UNREACHABLE();
      return gfx::Rect();
  }
}

const AXNodeData& TestAXNodeHelper::GetData() const {
  return node_->data();
}

gfx::RectF TestAXNodeHelper::GetLocation() const {
  return GetData().relative_bounds.bounds;
}

int TestAXNodeHelper::InternalChildCount() const {
  return static_cast<int>(node_->GetUnignoredChildCount());
}

TestAXNodeHelper* TestAXNodeHelper::InternalGetChild(int index) const {
  BASE_CHECK(index >= 0);
  BASE_CHECK(index < InternalChildCount());
  return GetOrCreate(
      tree_, node_->GetUnignoredChildAtIndex(static_cast<size_t>(index)));
}

gfx::RectF TestAXNodeHelper::GetInlineTextRect(const int start_offset,
                                               const int end_offset) const {
  BASE_DCHECK(start_offset >= 0 && end_offset >= 0 &&
              start_offset <= end_offset);
  const std::vector<int32_t>& character_offsets = GetData().GetIntListAttribute(
      ax::mojom::IntListAttribute::kCharacterOffsets);
  gfx::RectF location = GetLocation();
  gfx::RectF bounds;

  switch (static_cast<ax::mojom::WritingDirection>(
      GetData().GetIntAttribute(ax::mojom::IntAttribute::kTextDirection))) {
    // Currently only kNone and kLtr are supported text direction.
    case ax::mojom::WritingDirection::kNone:
    case ax::mojom::WritingDirection::kLtr: {
      int start_pixel_offset =
          start_offset > 0 ? character_offsets[start_offset - 1] : location.x();
      int end_pixel_offset =
          end_offset > 0 ? character_offsets[end_offset - 1] : location.x();
      bounds =
          gfx::RectF(start_pixel_offset, location.y(),
                     end_pixel_offset - start_pixel_offset, location.height());
      break;
    }
    default:
      BASE_UNREACHABLE();
  }
  return bounds;
}

AXOffscreenResult TestAXNodeHelper::DetermineOffscreenResult(
    gfx::RectF bounds) const {
  if (!tree_ || !tree_->root())
    return AXOffscreenResult::kOnscreen;

  const AXNodeData& root_web_area_node_data = tree_->root()->data();
  gfx::RectF root_web_area_bounds =
      root_web_area_node_data.relative_bounds.bounds;

  // For testing, we only look at the current node's bound relative to the root
  // web area bounds to determine offscreen status. We currently do not look at
  // the bounds of the immediate parent of the node for determining offscreen
  // status.
  // We only determine offscreen result if the root web area bounds is actually
  // set in the test. We default the offscreen result of every other situation
  // to AXOffscreenResult::kOnscreen.
  if (!root_web_area_bounds.IsEmpty()) {
    bounds.Intersect(root_web_area_bounds);
    if (bounds.IsEmpty())
      return AXOffscreenResult::kOffscreen;
  }
  return AXOffscreenResult::kOnscreen;
}

}  // namespace ui
