// Copyright 2015 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "test_ax_node_wrapper.h"

#include <map>
#include <utility>

#include "ax/ax_action_data.h"
#include "ax/ax_role_properties.h"
#include "ax/ax_table_info.h"
#include "ax/ax_tree_observer.h"
#include "base/numerics/ranges.h"
#include "base/string_utils.h"
#include "gfx/geometry/rect_conversions.h"

namespace ui {

namespace {

// A global map from AXNodes to TestAXNodeWrappers.
std::map<AXNode::AXID, TestAXNodeWrapper*> g_node_id_to_wrapper_map;

// A global coordinate offset.
gfx::Vector2d g_offset;

// A global scale factor.
float g_scale_factor = 1.0;

// A global map that stores which node is focused on a determined tree.
//   - If a tree has no node being focused, there shouldn't be any entry on the
//     map associated with such tree, i.e. a pair {tree, nullptr} is invalid.
//   - For testing purposes, assume there is a single node being focused in the
//     entire tree and if such node is deleted, focus is completely lost.
std::map<AXTree*, AXNode*> g_focused_node_in_tree;

// A global indicating the last node which ShowContextMenu was called from.
AXNode* g_node_from_last_show_context_menu;

// A global indicating the last node which accessibility perform action
// default action was called from.
AXNode* g_node_from_last_default_action;

// A global indicating that AXPlatformNodeDelegate objects are web content.
bool g_is_web_content = false;

// A map of hit test results - a map from source node ID to destination node
// ID.
std::map<AXNode::AXID, AXNode::AXID> g_hit_test_result;

// A simple implementation of AXTreeObserver to catch when AXNodes are
// deleted so we can delete their wrappers.
class TestAXTreeObserver : public AXTreeObserver {
 private:
  void OnNodeDeleted(AXTree* tree, int32_t node_id) override {
    const auto& iter = g_node_id_to_wrapper_map.find(node_id);
    if (iter != g_node_id_to_wrapper_map.end()) {
      TestAXNodeWrapper* wrapper = iter->second;
      const auto& focus_iter = g_focused_node_in_tree.find(tree);
      if (focus_iter != g_focused_node_in_tree.end() &&
          focus_iter->second->id() == node_id) {
        g_focused_node_in_tree.erase(tree);
      }
      delete wrapper;
      g_node_id_to_wrapper_map.erase(node_id);
    }
  }
};

TestAXTreeObserver g_ax_tree_observer;

}  // namespace

// static
TestAXNodeWrapper* TestAXNodeWrapper::GetOrCreate(AXTree* tree, AXNode* node) {
  if (!tree || !node)
    return nullptr;

  if (!tree->HasObserver(&g_ax_tree_observer))
    tree->AddObserver(&g_ax_tree_observer);
  auto iter = g_node_id_to_wrapper_map.find(node->id());
  if (iter != g_node_id_to_wrapper_map.end())
    return iter->second;
  TestAXNodeWrapper* wrapper = new TestAXNodeWrapper(tree, node);
  g_node_id_to_wrapper_map[node->id()] = wrapper;
  return wrapper;
}

// static
void TestAXNodeWrapper::SetGlobalCoordinateOffset(const gfx::Vector2d& offset) {
  g_offset = offset;
}

// static
const AXNode* TestAXNodeWrapper::GetNodeFromLastShowContextMenu() {
  return g_node_from_last_show_context_menu;
}

// static
const AXNode* TestAXNodeWrapper::GetNodeFromLastDefaultAction() {
  return g_node_from_last_default_action;
}

// static
void TestAXNodeWrapper::SetNodeFromLastDefaultAction(AXNode* node) {
  g_node_from_last_default_action = node;
}

// static
std::unique_ptr<base::AutoReset<float>> TestAXNodeWrapper::SetScaleFactor(
    float value) {
  return std::make_unique<base::AutoReset<float>>(&g_scale_factor, value);
}

// static
void TestAXNodeWrapper::SetGlobalIsWebContent(bool is_web_content) {
  g_is_web_content = is_web_content;
}

// static
void TestAXNodeWrapper::SetHitTestResult(AXNode::AXID src_node_id,
                                         AXNode::AXID dst_node_id) {
  g_hit_test_result[src_node_id] = dst_node_id;
}

// static
void TestAXNodeWrapper::ClearHitTestResults() {
  g_hit_test_result.clear();
}

TestAXNodeWrapper::~TestAXNodeWrapper() {
  platform_node_->Destroy();
}

const AXNodeData& TestAXNodeWrapper::GetData() const {
  return node_->data();
}

const AXTreeData& TestAXNodeWrapper::GetTreeData() const {
  return tree_->data();
}

const AXTree::Selection TestAXNodeWrapper::GetUnignoredSelection() const {
  return tree_->GetUnignoredSelection();
}

AXNodePosition::AXPositionInstance TestAXNodeWrapper::CreateTextPositionAt(
    int offset) const {
  return ui::AXNodePosition::CreateTextPosition(
      GetTreeData().tree_id, node_->id(), offset,
      ax::mojom::TextAffinity::kDownstream);
}

gfx::NativeViewAccessible TestAXNodeWrapper::GetNativeViewAccessible() {
  return ax_platform_node()->GetNativeViewAccessible();
}

gfx::NativeViewAccessible TestAXNodeWrapper::GetParent() {
  TestAXNodeWrapper* parent_wrapper =
      GetOrCreate(tree_, node_->GetUnignoredParent());
  return parent_wrapper
             ? parent_wrapper->ax_platform_node()->GetNativeViewAccessible()
             : nullptr;
}

int TestAXNodeWrapper::GetChildCount() const {
  return InternalChildCount();
}

gfx::NativeViewAccessible TestAXNodeWrapper::ChildAtIndex(int index) {
  TestAXNodeWrapper* child_wrapper = InternalGetChild(index);
  return child_wrapper
             ? child_wrapper->ax_platform_node()->GetNativeViewAccessible()
             : nullptr;
}

gfx::Rect TestAXNodeWrapper::GetBoundsRect(
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
      bounds.Offset(g_offset);

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

gfx::Rect TestAXNodeWrapper::GetInnerTextRangeBoundsRect(
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
        TestAXNodeWrapper* child = InternalGetChild(0);
        if (child != nullptr &&
            child->GetData().role == ax::mojom::Role::kInlineTextBox) {
          bounds = child->GetInlineTextRect(start_offset, end_offset);
        }
      }

      bounds.Offset(g_offset);

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

gfx::Rect TestAXNodeWrapper::GetHypertextRangeBoundsRect(
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
      // Ignoring start, len, and clipped, as there's no clean way to map these
      // via unit tests.
      gfx::RectF bounds = GetLocation();
      bounds.Offset(g_offset);
      return gfx::ToEnclosingRect(bounds);
    }
    case AXCoordinateSystem::kRootFrame:
    case AXCoordinateSystem::kFrame:
      BASE_UNREACHABLE();
      return gfx::Rect();
  }
}

TestAXNodeWrapper* TestAXNodeWrapper::HitTestSyncInternal(int x, int y) {
  if (g_hit_test_result.find(node_->id()) != g_hit_test_result.end()) {
    int result_id = g_hit_test_result[node_->id()];
    AXNode* result_node = tree_->GetFromId(result_id);
    return GetOrCreate(tree_, result_node);
  }

  // Here we find the deepest child whose bounding box contains the given point.
  // The assumptions are that there are no overlapping bounding rects and that
  // all children have smaller bounding rects than their parents.
  if (!GetClippedScreenBoundsRect().Contains(gfx::Rect(x, y, 0, 0)))
    return nullptr;

  for (int i = 0; i < GetChildCount(); i++) {
    TestAXNodeWrapper* child = GetOrCreate(tree_, node_->children()[i]);
    if (!child)
      return nullptr;

    TestAXNodeWrapper* result = child->HitTestSyncInternal(x, y);
    if (result) {
      return result;
    }
  }
  return this;
}

gfx::NativeViewAccessible TestAXNodeWrapper::HitTestSync(
    int screen_physical_pixel_x,
    int screen_physical_pixel_y) const {
  const TestAXNodeWrapper* wrapper =
      const_cast<TestAXNodeWrapper*>(this)->HitTestSyncInternal(
          screen_physical_pixel_x / g_scale_factor,
          screen_physical_pixel_y / g_scale_factor);
  return wrapper ? wrapper->ax_platform_node()->GetNativeViewAccessible()
                 : nullptr;
}

gfx::NativeViewAccessible TestAXNodeWrapper::GetFocus() {
  auto focused = g_focused_node_in_tree.find(tree_);
  if (focused != g_focused_node_in_tree.end() &&
      focused->second->IsDescendantOf(node_)) {
    return GetOrCreate(tree_, focused->second)
        ->ax_platform_node()
        ->GetNativeViewAccessible();
  }
  return nullptr;
}

bool TestAXNodeWrapper::IsMinimized() const {
  return minimized_;
}

bool TestAXNodeWrapper::IsWebContent() const {
  return g_is_web_content;
}

// Walk the AXTree and ensure that all wrappers are created
void TestAXNodeWrapper::BuildAllWrappers(AXTree* tree, AXNode* node) {
  for (auto* child : node->children()) {
    TestAXNodeWrapper::GetOrCreate(tree, child);
    BuildAllWrappers(tree, child);
  }
}

void TestAXNodeWrapper::ResetNativeEventTarget() {
  native_event_target_ = gfx::kNullAcceleratedWidget;
}

AXPlatformNode* TestAXNodeWrapper::GetFromNodeID(int32_t id) {
  // Force creating all of the wrappers for this tree.
  BuildAllWrappers(tree_, node_);

  const auto iter = g_node_id_to_wrapper_map.find(id);
  if (iter != g_node_id_to_wrapper_map.end())
    return iter->second->ax_platform_node();

  return nullptr;
}

AXPlatformNode* TestAXNodeWrapper::GetFromTreeIDAndNodeID(
    const ui::AXTreeID& ax_tree_id,
    int32_t id) {
  // TestAXNodeWrapper only supports one accessibility tree.
  // Additional work would need to be done to support multiple trees.
  BASE_CHECK(GetTreeData().tree_id == ax_tree_id);
  return GetFromNodeID(id);
}

int TestAXNodeWrapper::GetIndexInParent() {
  return node_ ? static_cast<int>(node_->GetUnignoredIndexInParent()) : -1;
}

void TestAXNodeWrapper::ReplaceIntAttribute(int32_t node_id,
                                            ax::mojom::IntAttribute attribute,
                                            int32_t value) {
  if (!tree_)
    return;

  AXNode* node = tree_->GetFromId(node_id);
  if (!node)
    return;

  AXNodeData new_data = node->data();
  std::vector<std::pair<ax::mojom::IntAttribute, int32_t>>& attributes =
      new_data.int_attributes;

  auto it = std::remove_if(
      attributes.begin(), attributes.end(),
      [attribute](auto& pair) { return pair.first == attribute; });
  attributes.erase(it, attributes.end());

  new_data.AddIntAttribute(attribute, value);
  node->SetData(new_data);
}

void TestAXNodeWrapper::ReplaceFloatAttribute(
    ax::mojom::FloatAttribute attribute,
    float value) {
  AXNodeData new_data = GetData();
  std::vector<std::pair<ax::mojom::FloatAttribute, float>>& attributes =
      new_data.float_attributes;

  auto it = std::remove_if(
      attributes.begin(), attributes.end(),
      [attribute](auto& pair) { return pair.first == attribute; });
  attributes.erase(it, attributes.end());

  new_data.AddFloatAttribute(attribute, value);
  node_->SetData(new_data);
}

void TestAXNodeWrapper::ReplaceBoolAttribute(ax::mojom::BoolAttribute attribute,
                                             bool value) {
  AXNodeData new_data = GetData();
  std::vector<std::pair<ax::mojom::BoolAttribute, bool>>& attributes =
      new_data.bool_attributes;

  auto it = std::remove_if(
      attributes.begin(), attributes.end(),
      [attribute](auto& pair) { return pair.first == attribute; });
  attributes.erase(it, attributes.end());

  new_data.AddBoolAttribute(attribute, value);
  node_->SetData(new_data);
}

void TestAXNodeWrapper::ReplaceStringAttribute(
    ax::mojom::StringAttribute attribute,
    std::string value) {
  AXNodeData new_data = GetData();
  std::vector<std::pair<ax::mojom::StringAttribute, std::string>>& attributes =
      new_data.string_attributes;

  auto it = std::remove_if(
      attributes.begin(), attributes.end(),
      [attribute](auto& pair) { return pair.first == attribute; });
  attributes.erase(it, attributes.end());

  new_data.AddStringAttribute(attribute, value);
  node_->SetData(new_data);
}

void TestAXNodeWrapper::ReplaceTreeDataTextSelection(int32_t anchor_node_id,
                                                     int32_t anchor_offset,
                                                     int32_t focus_node_id,
                                                     int32_t focus_offset) {
  if (!tree_)
    return;

  AXTreeData new_tree_data = GetTreeData();
  new_tree_data.sel_anchor_object_id = anchor_node_id;
  new_tree_data.sel_anchor_offset = anchor_offset;
  new_tree_data.sel_focus_object_id = focus_node_id;
  new_tree_data.sel_focus_offset = focus_offset;

  tree_->UpdateData(new_tree_data);
}

bool TestAXNodeWrapper::IsTable() const {
  return node_->IsTable();
}

std::optional<int> TestAXNodeWrapper::GetTableRowCount() const {
  return node_->GetTableRowCount();
}

std::optional<int> TestAXNodeWrapper::GetTableColCount() const {
  return node_->GetTableColCount();
}

std::optional<int> TestAXNodeWrapper::GetTableAriaRowCount() const {
  return node_->GetTableAriaRowCount();
}

std::optional<int> TestAXNodeWrapper::GetTableAriaColCount() const {
  return node_->GetTableAriaColCount();
}

std::optional<int> TestAXNodeWrapper::GetTableCellCount() const {
  return node_->GetTableCellCount();
}

std::optional<bool> TestAXNodeWrapper::GetTableHasColumnOrRowHeaderNode()
    const {
  return node_->GetTableHasColumnOrRowHeaderNode();
}

std::vector<AXNode::AXID> TestAXNodeWrapper::GetColHeaderNodeIds() const {
  return node_->GetTableColHeaderNodeIds();
}

std::vector<AXNode::AXID> TestAXNodeWrapper::GetColHeaderNodeIds(
    int col_index) const {
  return node_->GetTableColHeaderNodeIds(col_index);
}

std::vector<AXNode::AXID> TestAXNodeWrapper::GetRowHeaderNodeIds() const {
  return node_->GetTableCellRowHeaderNodeIds();
}

std::vector<AXNode::AXID> TestAXNodeWrapper::GetRowHeaderNodeIds(
    int row_index) const {
  return node_->GetTableRowHeaderNodeIds(row_index);
}

bool TestAXNodeWrapper::IsTableRow() const {
  return node_->IsTableRow();
}

std::optional<int> TestAXNodeWrapper::GetTableRowRowIndex() const {
  return node_->GetTableRowRowIndex();
}

bool TestAXNodeWrapper::IsTableCellOrHeader() const {
  return node_->IsTableCellOrHeader();
}

std::optional<int> TestAXNodeWrapper::GetTableCellIndex() const {
  return node_->GetTableCellIndex();
}

std::optional<int> TestAXNodeWrapper::GetTableCellColIndex() const {
  return node_->GetTableCellColIndex();
}

std::optional<int> TestAXNodeWrapper::GetTableCellRowIndex() const {
  return node_->GetTableCellRowIndex();
}

std::optional<int> TestAXNodeWrapper::GetTableCellColSpan() const {
  return node_->GetTableCellColSpan();
}

std::optional<int> TestAXNodeWrapper::GetTableCellRowSpan() const {
  return node_->GetTableCellRowSpan();
}

std::optional<int> TestAXNodeWrapper::GetTableCellAriaColIndex() const {
  return node_->GetTableCellAriaColIndex();
}

std::optional<int> TestAXNodeWrapper::GetTableCellAriaRowIndex() const {
  return node_->GetTableCellAriaRowIndex();
}

std::optional<int32_t> TestAXNodeWrapper::GetCellId(int row_index,
                                                    int col_index) const {
  AXNode* cell = node_->GetTableCellFromCoords(row_index, col_index);
  if (!cell)
    return std::nullopt;
  return cell->id();
}

gfx::AcceleratedWidget
TestAXNodeWrapper::GetTargetForNativeAccessibilityEvent() {
  return native_event_target_;
}

std::optional<int32_t> TestAXNodeWrapper::CellIndexToId(int cell_index) const {
  AXNode* cell = node_->GetTableCellFromIndex(cell_index);
  if (!cell)
    return std::nullopt;
  return cell->id();
}

bool TestAXNodeWrapper::IsCellOrHeaderOfARIATable() const {
  return node_->IsCellOrHeaderOfARIATable();
}

bool TestAXNodeWrapper::IsCellOrHeaderOfARIAGrid() const {
  return node_->IsCellOrHeaderOfARIAGrid();
}

bool TestAXNodeWrapper::AccessibilityPerformAction(
    const ui::AXActionData& data) {
  switch (data.action) {
    case ax::mojom::Action::kScrollToPoint:
      g_offset = gfx::Vector2d(data.target_point.x(), data.target_point.y());
      return true;
    case ax::mojom::Action::kSetScrollOffset: {
      int scroll_x_min =
          GetData().GetIntAttribute(ax::mojom::IntAttribute::kScrollXMin);
      int scroll_x_max =
          GetData().GetIntAttribute(ax::mojom::IntAttribute::kScrollXMax);
      int scroll_y_min =
          GetData().GetIntAttribute(ax::mojom::IntAttribute::kScrollYMin);
      int scroll_y_max =
          GetData().GetIntAttribute(ax::mojom::IntAttribute::kScrollYMax);
      int scroll_x =
          base::ClampToRange(data.target_point.x(), scroll_x_min, scroll_x_max);
      int scroll_y =
          base::ClampToRange(data.target_point.y(), scroll_y_min, scroll_y_max);

      ReplaceIntAttribute(node_->id(), ax::mojom::IntAttribute::kScrollX,
                          scroll_x);
      ReplaceIntAttribute(node_->id(), ax::mojom::IntAttribute::kScrollY,
                          scroll_y);
      return true;
    }
    case ax::mojom::Action::kScrollToMakeVisible: {
      auto offset = node_->data().relative_bounds.bounds.OffsetFromOrigin();
      g_offset = gfx::Vector2d(-offset.x(), -offset.y());
      return true;
    }

    case ax::mojom::Action::kDoDefault: {
      // If a default action such as a click is performed on an element, it
      // could result in a selected state change. In which case, the element's
      // selected state no longer comes from focus action, so we should set
      // |kSelectedFromFocus| to false.
      if (GetData().HasBoolAttribute(
              ax::mojom::BoolAttribute::kSelectedFromFocus))
        ReplaceBoolAttribute(ax::mojom::BoolAttribute::kSelectedFromFocus,
                             false);

      switch (GetData().role) {
        case ax::mojom::Role::kListBoxOption:
        case ax::mojom::Role::kCell: {
          bool current_value =
              GetData().GetBoolAttribute(ax::mojom::BoolAttribute::kSelected);
          ReplaceBoolAttribute(ax::mojom::BoolAttribute::kSelected,
                               !current_value);
          break;
        }
        case ax::mojom::Role::kRadioButton:
        case ax::mojom::Role::kMenuItemRadio: {
          if (GetData().GetCheckedState() == ax::mojom::CheckedState::kTrue)
            ReplaceIntAttribute(
                node_->id(), ax::mojom::IntAttribute::kCheckedState,
                static_cast<int32_t>(ax::mojom::CheckedState::kFalse));
          else if (GetData().GetCheckedState() ==
                   ax::mojom::CheckedState::kFalse)
            ReplaceIntAttribute(
                node_->id(), ax::mojom::IntAttribute::kCheckedState,
                static_cast<int32_t>(ax::mojom::CheckedState::kTrue));
          break;
        }
        default:
          break;
      }
      SetNodeFromLastDefaultAction(node_);
      return true;
    }

    case ax::mojom::Action::kSetValue:
      if (GetData().IsRangeValueSupported()) {
        ReplaceFloatAttribute(ax::mojom::FloatAttribute::kValueForRange,
                              std::stof(data.value));
      } else if (GetData().role == ax::mojom::Role::kTextField) {
        ReplaceStringAttribute(ax::mojom::StringAttribute::kValue, data.value);
      }
      return true;

    case ax::mojom::Action::kSetSelection: {
      ReplaceIntAttribute(data.anchor_node_id,
                          ax::mojom::IntAttribute::kTextSelStart,
                          data.anchor_offset);
      ReplaceIntAttribute(data.focus_node_id,
                          ax::mojom::IntAttribute::kTextSelEnd,
                          data.focus_offset);
      ReplaceTreeDataTextSelection(data.anchor_node_id, data.anchor_offset,
                                   data.focus_node_id, data.focus_offset);
      return true;
    }

    case ax::mojom::Action::kFocus: {
      g_focused_node_in_tree[tree_] = node_;

      // The platform has select follows focus behavior:
      // https://www.w3.org/TR/wai-aria-practices-1.1/#kbd_selection_follows_focus
      // For test purpose, we support select follows focus for all elements, and
      // not just single-selection container elements.
      if (SupportsSelected(GetData().role)) {
        ReplaceBoolAttribute(ax::mojom::BoolAttribute::kSelected, true);
        ReplaceBoolAttribute(ax::mojom::BoolAttribute::kSelectedFromFocus,
                             true);
      }

      return true;
    }

    case ax::mojom::Action::kShowContextMenu:
      g_node_from_last_show_context_menu = node_;
      return true;

    default:
      return true;
  }
}

std::u16string TestAXNodeWrapper::GetLocalizedRoleDescriptionForUnlabeledImage()
    const {
  return base::ASCIIToUTF16("Unlabeled image");
}

std::u16string TestAXNodeWrapper::GetLocalizedStringForLandmarkType() const {
  const AXNodeData& data = GetData();
  switch (data.role) {
    case ax::mojom::Role::kBanner:
    case ax::mojom::Role::kHeader:
      return base::ASCIIToUTF16("banner");

    case ax::mojom::Role::kComplementary:
      return base::ASCIIToUTF16("complementary");

    case ax::mojom::Role::kContentInfo:
    case ax::mojom::Role::kFooter:
      return base::ASCIIToUTF16("content information");

    case ax::mojom::Role::kRegion:
    case ax::mojom::Role::kSection:
      if (data.HasStringAttribute(ax::mojom::StringAttribute::kName))
        return base::ASCIIToUTF16("region");

    default:
      return {};
  }
}

std::u16string TestAXNodeWrapper::GetLocalizedStringForRoleDescription() const {
  const AXNodeData& data = GetData();

  switch (data.role) {
    case ax::mojom::Role::kArticle:
      return base::ASCIIToUTF16("article");

    case ax::mojom::Role::kAudio:
      return base::ASCIIToUTF16("audio");

    case ax::mojom::Role::kCode:
      return base::ASCIIToUTF16("code");

    case ax::mojom::Role::kColorWell:
      return base::ASCIIToUTF16("color picker");

    case ax::mojom::Role::kContentInfo:
      return base::ASCIIToUTF16("content information");

    case ax::mojom::Role::kDate:
      return base::ASCIIToUTF16("date picker");

    case ax::mojom::Role::kDateTime: {
      std::string input_type;
      if (data.GetStringAttribute(ax::mojom::StringAttribute::kInputType,
                                  &input_type)) {
        if (input_type == "datetime-local") {
          return base::ASCIIToUTF16("local date and time picker");
        } else if (input_type == "week") {
          return base::ASCIIToUTF16("week picker");
        }
      }
      return {};
    }

    case ax::mojom::Role::kDetails:
      return base::ASCIIToUTF16("details");

    case ax::mojom::Role::kEmphasis:
      return base::ASCIIToUTF16("emphasis");

    case ax::mojom::Role::kFigure:
      return base::ASCIIToUTF16("figure");

    case ax::mojom::Role::kFooter:
    case ax::mojom::Role::kFooterAsNonLandmark:
      return base::ASCIIToUTF16("footer");

    case ax::mojom::Role::kHeader:
    case ax::mojom::Role::kHeaderAsNonLandmark:
      return base::ASCIIToUTF16("header");

    case ax::mojom::Role::kMark:
      return base::ASCIIToUTF16("highlight");

    case ax::mojom::Role::kMeter:
      return base::ASCIIToUTF16("meter");

    case ax::mojom::Role::kSearchBox:
      return base::ASCIIToUTF16("search box");

    case ax::mojom::Role::kSection: {
      if (data.HasStringAttribute(ax::mojom::StringAttribute::kName))
        return base::ASCIIToUTF16("section");

      return {};
    }

    case ax::mojom::Role::kStatus:
      return base::ASCIIToUTF16("output");

    case ax::mojom::Role::kStrong:
      return base::ASCIIToUTF16("strong");

    case ax::mojom::Role::kTextField: {
      std::string input_type;
      if (data.GetStringAttribute(ax::mojom::StringAttribute::kInputType,
                                  &input_type)) {
        if (input_type == "email") {
          return base::ASCIIToUTF16("email");
        } else if (input_type == "tel") {
          return base::ASCIIToUTF16("telephone");
        } else if (input_type == "url") {
          return base::ASCIIToUTF16("url");
        }
      }
      return {};
    }

    case ax::mojom::Role::kTime:
      return base::ASCIIToUTF16("time");

    default:
      return {};
  }
}

std::u16string TestAXNodeWrapper::GetLocalizedStringForImageAnnotationStatus(
    ax::mojom::ImageAnnotationStatus status) const {
  switch (status) {
    case ax::mojom::ImageAnnotationStatus::kEligibleForAnnotation:
      return base::ASCIIToUTF16(
          "To get missing image descriptions, open the context menu.");
    case ax::mojom::ImageAnnotationStatus::kAnnotationPending:
      return base::ASCIIToUTF16("Getting description...");
    case ax::mojom::ImageAnnotationStatus::kAnnotationAdult:
      return base::ASCIIToUTF16(
          "Appears to contain adult content. No description available.");
    case ax::mojom::ImageAnnotationStatus::kAnnotationEmpty:
    case ax::mojom::ImageAnnotationStatus::kAnnotationProcessFailed:
      return base::ASCIIToUTF16("No description available.");
    case ax::mojom::ImageAnnotationStatus::kNone:
    case ax::mojom::ImageAnnotationStatus::kWillNotAnnotateDueToScheme:
    case ax::mojom::ImageAnnotationStatus::kIneligibleForAnnotation:
    case ax::mojom::ImageAnnotationStatus::kSilentlyEligibleForAnnotation:
    case ax::mojom::ImageAnnotationStatus::kAnnotationSucceeded:
      return std::u16string();
  }

  BASE_UNREACHABLE();
  return std::u16string();
}

std::u16string TestAXNodeWrapper::GetStyleNameAttributeAsLocalizedString()
    const {
  AXNode* current_node = node_;
  while (current_node) {
    if (current_node->data().role == ax::mojom::Role::kMark)
      return base::ASCIIToUTF16("mark");
    current_node = current_node->parent();
  }
  return std::u16string();
}

bool TestAXNodeWrapper::ShouldIgnoreHoveredStateForTesting() {
  return true;
}

bool TestAXNodeWrapper::HasVisibleCaretOrSelection() const {
  ui::AXTree::Selection unignored_selection = GetUnignoredSelection();
  int32_t focus_id = unignored_selection.focus_object_id;
  AXNode* focus_object = tree_->GetFromId(focus_id);
  if (!focus_object)
    return false;

  // Selection or caret will be visible in a focused editable area.
  if (GetData().HasState(ax::mojom::State::kEditable)) {
    return GetData().IsPlainTextField() ? focus_object == node_
                                        : focus_object->IsDescendantOf(node_);
  }

  // The selection will be visible in non-editable content only if it is not
  // collapsed into a caret.
  return (focus_id != unignored_selection.anchor_object_id ||
          unignored_selection.focus_offset !=
              unignored_selection.anchor_offset) &&
         focus_object->IsDescendantOf(node_);
}

std::set<AXPlatformNode*> TestAXNodeWrapper::GetReverseRelations(
    ax::mojom::IntAttribute attr) {
  BASE_DCHECK(IsNodeIdIntAttribute(attr));
  return GetNodesForNodeIds(tree_->GetReverseRelations(attr, GetData().id));
}

std::set<AXPlatformNode*> TestAXNodeWrapper::GetReverseRelations(
    ax::mojom::IntListAttribute attr) {
  BASE_DCHECK(IsNodeIdIntListAttribute(attr));
  return GetNodesForNodeIds(tree_->GetReverseRelations(attr, GetData().id));
}

const ui::AXUniqueId& TestAXNodeWrapper::GetUniqueId() const {
  return unique_id_;
}

TestAXNodeWrapper::TestAXNodeWrapper(AXTree* tree, AXNode* node)
    : tree_(tree), node_(node), platform_node_(AXPlatformNode::Create(this)) {
#if defined(OS_WIN)
  native_event_target_ = gfx::kMockAcceleratedWidget;
#else
  native_event_target_ = gfx::kNullAcceleratedWidget;
#endif
}

bool TestAXNodeWrapper::IsOrderedSetItem() const {
  return node_->IsOrderedSetItem();
}

bool TestAXNodeWrapper::IsOrderedSet() const {
  return node_->IsOrderedSet();
}

std::optional<int> TestAXNodeWrapper::GetPosInSet() const {
  return node_->GetPosInSet();
}

std::optional<int> TestAXNodeWrapper::GetSetSize() const {
  return node_->GetSetSize();
}

gfx::RectF TestAXNodeWrapper::GetLocation() const {
  return GetData().relative_bounds.bounds;
}

int TestAXNodeWrapper::InternalChildCount() const {
  return static_cast<int>(node_->GetUnignoredChildCount());
}

TestAXNodeWrapper* TestAXNodeWrapper::InternalGetChild(int index) const {
  BASE_CHECK(index >= 0);
  BASE_CHECK(index < InternalChildCount());
  return GetOrCreate(
      tree_, node_->GetUnignoredChildAtIndex(static_cast<size_t>(index)));
}

// Recursive helper function for GetUIADescendants. Aggregates all of the
// descendants for a given node within the descendants vector.
void TestAXNodeWrapper::UIADescendants(
    const AXNode* node,
    std::vector<gfx::NativeViewAccessible>* descendants) const {
  if (ShouldHideChildrenForUIA(node))
    return;

  for (auto it = node->UnignoredChildrenBegin();
       it != node->UnignoredChildrenEnd(); ++it) {
    descendants->emplace_back(ax_platform_node()
                                  ->GetDelegate()
                                  ->GetFromNodeID(it->id())
                                  ->GetNativeViewAccessible());
    UIADescendants(it.get(), descendants);
  }
}

const std::vector<gfx::NativeViewAccessible>
TestAXNodeWrapper::GetUIADescendants() const {
  std::vector<gfx::NativeViewAccessible> descendants;
  UIADescendants(node_, &descendants);
  return descendants;
}

// static
// Needs to stay in sync with AXPlatformNodeWin::ShouldHideChildrenForUIA.
bool TestAXNodeWrapper::ShouldHideChildrenForUIA(const AXNode* node) {
  if (!node)
    return false;

  auto role = node->data().role;

  if (ui::HasPresentationalChildren(role))
    return true;

  switch (role) {
    case ax::mojom::Role::kLink:
    case ax::mojom::Role::kTextField:
      return true;
    default:
      return false;
  }
}

gfx::RectF TestAXNodeWrapper::GetInlineTextRect(const int start_offset,
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

AXOffscreenResult TestAXNodeWrapper::DetermineOffscreenResult(
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
