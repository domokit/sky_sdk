// Copyright 2018 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "ax_platform_node_delegate_base.h"

#include <vector>

#include "ax/ax_action_data.h"
#include "ax/ax_constants.h"
#include "ax/ax_node_data.h"
#include "ax/ax_role_properties.h"
#include "ax/ax_tree_data.h"
#include "base/no_destructor.h"

#include "ax_platform_node.h"
#include "ax_platform_node_base.h"

namespace ui {

AXPlatformNodeDelegateBase::AXPlatformNodeDelegateBase() = default;

AXPlatformNodeDelegateBase::~AXPlatformNodeDelegateBase() = default;

const AXNodeData& AXPlatformNodeDelegateBase::GetData() const {
  static base::NoDestructor<AXNodeData> empty_data;
  return *empty_data;
}

const AXTreeData& AXPlatformNodeDelegateBase::GetTreeData() const {
  static base::NoDestructor<AXTreeData> empty_data;
  return *empty_data;
}

std::u16string AXPlatformNodeDelegateBase::GetInnerText() const {
  // Unlike in web content The "kValue" attribute always takes precedence,
  // because we assume that users of this base class, such as Views controls,
  // are carefully crafted by hand, in contrast to HTML pages, where any content
  // that might be present in the shadow DOM (AKA in the internal accessibility
  // tree) is actually used by the renderer when assigning the "kValue"
  // attribute, including any redundant white space.
  std::u16string value =
      GetData().GetString16Attribute(ax::mojom::StringAttribute::kValue);
  if (!value.empty())
    return value;

  // TODO(https://crbug.com/1030703): The check for IsInvisibleOrIgnored()
  // should not be needed. ChildAtIndex() and GetChildCount() are already
  // supposed to skip over nodes that are invisible or ignored, but
  // ViewAXPlatformNodeDelegate does not currently implement this behavior.
  if (IsLeaf() && !GetData().IsInvisibleOrIgnored())
    return GetData().GetString16Attribute(ax::mojom::StringAttribute::kName);

  std::u16string inner_text;
  for (int i = 0; i < GetChildCount(); ++i) {
    // TODO(nektar): Add const to all tree traversal methods and remove
    // const_cast.
    const AXPlatformNode* child = AXPlatformNode::FromNativeViewAccessible(
        const_cast<AXPlatformNodeDelegateBase*>(this)->ChildAtIndex(i));
    if (!child || !child->GetDelegate())
      continue;
    inner_text += child->GetDelegate()->GetInnerText();
  }
  return inner_text;
}

const AXTree::Selection AXPlatformNodeDelegateBase::GetUnignoredSelection()
    const {
  return AXTree::Selection{false, -1, -1, ax::mojom::TextAffinity::kDownstream};
}

AXNodePosition::AXPositionInstance
AXPlatformNodeDelegateBase::CreateTextPositionAt(int offset) const {
  return AXNodePosition::CreateNullPosition();
}

gfx::NativeViewAccessible AXPlatformNodeDelegateBase::GetNSWindow() {
  return nullptr;
}

gfx::NativeViewAccessible
AXPlatformNodeDelegateBase::GetNativeViewAccessible() {
  return nullptr;
}

gfx::NativeViewAccessible AXPlatformNodeDelegateBase::GetParent() {
  return nullptr;
}

gfx::NativeViewAccessible
AXPlatformNodeDelegateBase::GetLowestPlatformAncestor() const {
  AXPlatformNodeDelegateBase* current_delegate =
      const_cast<AXPlatformNodeDelegateBase*>(this);
  AXPlatformNodeDelegateBase* lowest_unignored_delegate = current_delegate;

  // `highest_leaf_delegate` could be nullptr.
  AXPlatformNodeDelegateBase* highest_leaf_delegate = lowest_unignored_delegate;
  // For the purposes of this method, a leaf node does not include leaves in the
  // internal accessibility tree, only in the platform exposed tree.
  for (AXPlatformNodeDelegateBase* ancestor_delegate =
           lowest_unignored_delegate;
       ancestor_delegate;
       ancestor_delegate = static_cast<AXPlatformNodeDelegateBase*>(
           ancestor_delegate->GetParentDelegate())) {
    if (ancestor_delegate->IsLeaf())
      highest_leaf_delegate = ancestor_delegate;
  }
  if (highest_leaf_delegate)
    return highest_leaf_delegate->GetNativeViewAccessible();

  if (lowest_unignored_delegate)
    return lowest_unignored_delegate->GetNativeViewAccessible();
  return current_delegate->GetNativeViewAccessible();
}

int AXPlatformNodeDelegateBase::GetChildCount() const {
  return 0;
}

gfx::NativeViewAccessible AXPlatformNodeDelegateBase::ChildAtIndex(int index) {
  return nullptr;
}

bool AXPlatformNodeDelegateBase::HasModalDialog() const {
  return false;
}

gfx::NativeViewAccessible AXPlatformNodeDelegateBase::GetFirstChild() {
  if (GetChildCount() > 0)
    return ChildAtIndex(0);
  return nullptr;
}

gfx::NativeViewAccessible AXPlatformNodeDelegateBase::GetLastChild() {
  if (GetChildCount() > 0)
    return ChildAtIndex(GetChildCount() - 1);
  return nullptr;
}

gfx::NativeViewAccessible AXPlatformNodeDelegateBase::GetNextSibling() {
  AXPlatformNodeDelegate* parent = GetParentDelegate();
  if (parent && GetIndexInParent() >= 0) {
    int next_index = GetIndexInParent() + 1;
    if (next_index >= 0 && next_index < parent->GetChildCount())
      return parent->ChildAtIndex(next_index);
  }
  return nullptr;
}

gfx::NativeViewAccessible AXPlatformNodeDelegateBase::GetPreviousSibling() {
  AXPlatformNodeDelegate* parent = GetParentDelegate();
  if (parent && GetIndexInParent() >= 0) {
    int next_index = GetIndexInParent() - 1;
    if (next_index >= 0 && next_index < parent->GetChildCount())
      return parent->ChildAtIndex(next_index);
  }
  return nullptr;
}

bool AXPlatformNodeDelegateBase::IsChildOfLeaf() const {
  // TODO(nektar): Make all tree traversal methods const and remove const_cast.
  const AXPlatformNodeDelegate* parent =
      const_cast<AXPlatformNodeDelegateBase*>(this)->GetParentDelegate();
  if (!parent)
    return false;
  if (parent->IsLeaf())
    return true;
  return parent->IsChildOfLeaf();
}

bool AXPlatformNodeDelegateBase::IsLeaf() const {
  return !GetChildCount();
}

bool AXPlatformNodeDelegateBase::IsToplevelBrowserWindow() {
  return false;
}

bool AXPlatformNodeDelegateBase::IsChildOfPlainTextField() const {
  return false;
}

gfx::NativeViewAccessible AXPlatformNodeDelegateBase::GetClosestPlatformObject()
    const {
  return nullptr;
}

AXPlatformNodeDelegateBase::ChildIteratorBase::ChildIteratorBase(
    AXPlatformNodeDelegateBase* parent,
    int index)
    : index_(index), parent_(parent) {
  BASE_DCHECK(parent);
  BASE_DCHECK(0 <= index && index <= parent->GetChildCount());
}

AXPlatformNodeDelegateBase::ChildIteratorBase::ChildIteratorBase(
    const AXPlatformNodeDelegateBase::ChildIteratorBase& it)
    : index_(it.index_), parent_(it.parent_) {
  BASE_DCHECK(parent_);
}

bool AXPlatformNodeDelegateBase::ChildIteratorBase::operator==(
    const AXPlatformNodeDelegate::ChildIterator& rhs) const {
  return rhs.GetIndexInParent() == index_;
}

bool AXPlatformNodeDelegateBase::ChildIteratorBase::operator!=(
    const AXPlatformNodeDelegate::ChildIterator& rhs) const {
  return rhs.GetIndexInParent() != index_;
}

void AXPlatformNodeDelegateBase::ChildIteratorBase::operator++() {
  index_++;
}

void AXPlatformNodeDelegateBase::ChildIteratorBase::operator++(int) {
  index_++;
}

void AXPlatformNodeDelegateBase::ChildIteratorBase::operator--() {
  BASE_DCHECK(index_ > 0);
  index_--;
}

void AXPlatformNodeDelegateBase::ChildIteratorBase::operator--(int) {
  BASE_DCHECK(index_ > 0);
  index_--;
}

gfx::NativeViewAccessible
AXPlatformNodeDelegateBase::ChildIteratorBase::GetNativeViewAccessible() const {
  if (index_ < parent_->GetChildCount())
    return parent_->ChildAtIndex(index_);

  return nullptr;
}

int AXPlatformNodeDelegateBase::ChildIteratorBase::GetIndexInParent() const {
  return index_;
}

AXPlatformNodeDelegate&
AXPlatformNodeDelegateBase::ChildIteratorBase::operator*() const {
  AXPlatformNode* platform_node =
      AXPlatformNode::FromNativeViewAccessible(GetNativeViewAccessible());
  BASE_DCHECK(platform_node && platform_node->GetDelegate());
  return *(platform_node->GetDelegate());
}

AXPlatformNodeDelegate*
AXPlatformNodeDelegateBase::ChildIteratorBase::operator->() const {
  AXPlatformNode* platform_node =
      AXPlatformNode::FromNativeViewAccessible(GetNativeViewAccessible());
  return platform_node ? platform_node->GetDelegate() : nullptr;
}

std::unique_ptr<AXPlatformNodeDelegate::ChildIterator>
AXPlatformNodeDelegateBase::ChildrenBegin() {
  return std::make_unique<ChildIteratorBase>(this, 0);
}

std::unique_ptr<AXPlatformNodeDelegate::ChildIterator>
AXPlatformNodeDelegateBase::ChildrenEnd() {
  return std::make_unique<ChildIteratorBase>(this, GetChildCount());
}

std::string AXPlatformNodeDelegateBase::GetName() const {
  return GetData().GetStringAttribute(ax::mojom::StringAttribute::kName);
}

std::u16string AXPlatformNodeDelegateBase::GetHypertext() const {
  return std::u16string();
}

bool AXPlatformNodeDelegateBase::SetHypertextSelection(int start_offset,
                                                       int end_offset) {
  AXActionData action_data;
  action_data.action = ax::mojom::Action::kSetSelection;
  action_data.anchor_node_id = action_data.focus_node_id = GetData().id;
  action_data.anchor_offset = start_offset;
  action_data.focus_offset = end_offset;
  return AccessibilityPerformAction(action_data);
}

gfx::Rect AXPlatformNodeDelegateBase::GetBoundsRect(
    const AXCoordinateSystem coordinate_system,
    const AXClippingBehavior clipping_behavior,
    AXOffscreenResult* offscreen_result) const {
  return gfx::Rect();
}

gfx::Rect AXPlatformNodeDelegateBase::GetHypertextRangeBoundsRect(
    const int start_offset,
    const int end_offset,
    const AXCoordinateSystem coordinate_system,
    const AXClippingBehavior clipping_behavior,
    AXOffscreenResult* offscreen_result) const {
  return gfx::Rect();
}

gfx::Rect AXPlatformNodeDelegateBase::GetInnerTextRangeBoundsRect(
    const int start_offset,
    const int end_offset,
    const AXCoordinateSystem coordinate_system,
    const AXClippingBehavior clipping_behavior,
    AXOffscreenResult* offscreen_result = nullptr) const {
  return gfx::Rect();
}

gfx::Rect AXPlatformNodeDelegateBase::GetClippedScreenBoundsRect(
    AXOffscreenResult* offscreen_result) const {
  return GetBoundsRect(AXCoordinateSystem::kScreenDIPs,
                       AXClippingBehavior::kClipped, offscreen_result);
}

gfx::Rect AXPlatformNodeDelegateBase::GetUnclippedScreenBoundsRect(
    AXOffscreenResult* offscreen_result) const {
  return GetBoundsRect(AXCoordinateSystem::kScreenDIPs,
                       AXClippingBehavior::kUnclipped, offscreen_result);
}

gfx::NativeViewAccessible AXPlatformNodeDelegateBase::HitTestSync(
    int screen_physical_pixel_x,
    int screen_physical_pixel_y) const {
  return nullptr;
}

gfx::NativeViewAccessible AXPlatformNodeDelegateBase::GetFocus() {
  return nullptr;
}

AXPlatformNode* AXPlatformNodeDelegateBase::GetFromNodeID(int32_t id) {
  return nullptr;
}

AXPlatformNode* AXPlatformNodeDelegateBase::GetFromTreeIDAndNodeID(
    const ui::AXTreeID& ax_tree_id,
    int32_t id) {
  return nullptr;
}

int AXPlatformNodeDelegateBase::GetIndexInParent() {
  AXPlatformNodeDelegate* parent = GetParentDelegate();
  if (!parent)
    return -1;

  for (int i = 0; i < parent->GetChildCount(); i++) {
    AXPlatformNode* child_node =
        AXPlatformNode::FromNativeViewAccessible(parent->ChildAtIndex(i));
    if (child_node && child_node->GetDelegate() == this)
      return i;
  }
  return -1;
}

gfx::AcceleratedWidget
AXPlatformNodeDelegateBase::GetTargetForNativeAccessibilityEvent() {
  return gfx::kNullAcceleratedWidget;
}

bool AXPlatformNodeDelegateBase::IsTable() const {
  return ui::IsTableLike(GetData().role);
}

std::optional<int> AXPlatformNodeDelegateBase::GetTableRowCount() const {
  return GetData().GetIntAttribute(ax::mojom::IntAttribute::kTableRowCount);
}

std::optional<int> AXPlatformNodeDelegateBase::GetTableColCount() const {
  return GetData().GetIntAttribute(ax::mojom::IntAttribute::kTableColumnCount);
}

std::optional<int> AXPlatformNodeDelegateBase::GetTableAriaColCount() const {
  int aria_column_count;
  if (!GetData().GetIntAttribute(ax::mojom::IntAttribute::kAriaColumnCount,
                                 &aria_column_count)) {
    return std::nullopt;
  }
  return aria_column_count;
}

std::optional<int> AXPlatformNodeDelegateBase::GetTableAriaRowCount() const {
  int aria_row_count;
  if (!GetData().GetIntAttribute(ax::mojom::IntAttribute::kAriaRowCount,
                                 &aria_row_count)) {
    return std::nullopt;
  }
  return aria_row_count;
}

std::optional<int> AXPlatformNodeDelegateBase::GetTableCellCount() const {
  return std::nullopt;
}

std::optional<bool>
AXPlatformNodeDelegateBase::GetTableHasColumnOrRowHeaderNode() const {
  return std::nullopt;
}

std::vector<int32_t> AXPlatformNodeDelegateBase::GetColHeaderNodeIds() const {
  return {};
}

std::vector<int32_t> AXPlatformNodeDelegateBase::GetColHeaderNodeIds(
    int col_index) const {
  return {};
}

std::vector<int32_t> AXPlatformNodeDelegateBase::GetRowHeaderNodeIds() const {
  return {};
}

std::vector<int32_t> AXPlatformNodeDelegateBase::GetRowHeaderNodeIds(
    int row_index) const {
  return {};
}

AXPlatformNode* AXPlatformNodeDelegateBase::GetTableCaption() const {
  return nullptr;
}

bool AXPlatformNodeDelegateBase::IsTableRow() const {
  return ui::IsTableRow(GetData().role);
}

std::optional<int> AXPlatformNodeDelegateBase::GetTableRowRowIndex() const {
  return GetData().GetIntAttribute(ax::mojom::IntAttribute::kTableRowIndex);
}

bool AXPlatformNodeDelegateBase::IsTableCellOrHeader() const {
  return ui::IsCellOrTableHeader(GetData().role);
}

std::optional<int> AXPlatformNodeDelegateBase::GetTableCellColIndex() const {
  return GetData().GetIntAttribute(
      ax::mojom::IntAttribute::kTableCellColumnIndex);
}

std::optional<int> AXPlatformNodeDelegateBase::GetTableCellRowIndex() const {
  return GetData().GetIntAttribute(ax::mojom::IntAttribute::kTableCellRowIndex);
}

std::optional<int> AXPlatformNodeDelegateBase::GetTableCellColSpan() const {
  return GetData().GetIntAttribute(
      ax::mojom::IntAttribute::kTableCellColumnSpan);
}

std::optional<int> AXPlatformNodeDelegateBase::GetTableCellRowSpan() const {
  return GetData().GetIntAttribute(ax::mojom::IntAttribute::kTableCellRowSpan);
}

std::optional<int> AXPlatformNodeDelegateBase::GetTableCellAriaColIndex()
    const {
  return GetData().GetIntAttribute(
      ax::mojom::IntAttribute::kAriaCellColumnIndex);
}

std::optional<int> AXPlatformNodeDelegateBase::GetTableCellAriaRowIndex()
    const {
  return GetData().GetIntAttribute(ax::mojom::IntAttribute::kAriaCellRowIndex);
}

std::optional<int32_t> AXPlatformNodeDelegateBase::GetCellId(
    int row_index,
    int col_index) const {
  return std::nullopt;
}

std::optional<int> AXPlatformNodeDelegateBase::GetTableCellIndex() const {
  return std::nullopt;
}

std::optional<int32_t> AXPlatformNodeDelegateBase::CellIndexToId(
    int cell_index) const {
  return std::nullopt;
}

bool AXPlatformNodeDelegateBase::IsCellOrHeaderOfARIATable() const {
  return false;
}

bool AXPlatformNodeDelegateBase::IsCellOrHeaderOfARIAGrid() const {
  return false;
}

bool AXPlatformNodeDelegateBase::IsOrderedSetItem() const {
  return false;
}

bool AXPlatformNodeDelegateBase::IsOrderedSet() const {
  return false;
}

std::optional<int> AXPlatformNodeDelegateBase::GetPosInSet() const {
  return std::nullopt;
}

std::optional<int> AXPlatformNodeDelegateBase::GetSetSize() const {
  return std::nullopt;
}

bool AXPlatformNodeDelegateBase::AccessibilityPerformAction(
    const ui::AXActionData& data) {
  return false;
}

std::u16string
AXPlatformNodeDelegateBase::GetLocalizedStringForImageAnnotationStatus(
    ax::mojom::ImageAnnotationStatus status) const {
  return std::u16string();
}

std::u16string
AXPlatformNodeDelegateBase::GetLocalizedRoleDescriptionForUnlabeledImage()
    const {
  return std::u16string();
}

std::u16string AXPlatformNodeDelegateBase::GetLocalizedStringForLandmarkType()
    const {
  return std::u16string();
}

std::u16string
AXPlatformNodeDelegateBase::GetLocalizedStringForRoleDescription() const {
  return std::u16string();
}

std::u16string
AXPlatformNodeDelegateBase::GetStyleNameAttributeAsLocalizedString() const {
  return std::u16string();
}

TextAttributeMap AXPlatformNodeDelegateBase::ComputeTextAttributeMap(
    const TextAttributeList& default_attributes) const {
  ui::TextAttributeMap attributes_map;
  attributes_map[0] = default_attributes;
  return attributes_map;
}

std::string AXPlatformNodeDelegateBase::GetInheritedFontFamilyName() const {
  // We don't have access to AXNodeData here, so we cannot return
  // an inherited font family name.
  return std::string();
}

bool AXPlatformNodeDelegateBase::ShouldIgnoreHoveredStateForTesting() {
  return true;
}

bool AXPlatformNodeDelegateBase::IsOffscreen() const {
  return false;
}

bool AXPlatformNodeDelegateBase::IsMinimized() const {
  return false;
}

bool AXPlatformNodeDelegateBase::IsText() const {
  return ui::IsText(GetData().role);
}

bool AXPlatformNodeDelegateBase::IsWebContent() const {
  return false;
}

bool AXPlatformNodeDelegateBase::HasVisibleCaretOrSelection() const {
  return false;
}

AXPlatformNode* AXPlatformNodeDelegateBase::GetTargetNodeForRelation(
    ax::mojom::IntAttribute attr) {
  BASE_DCHECK(IsNodeIdIntAttribute(attr));

  int target_id;
  if (!GetData().GetIntAttribute(attr, &target_id))
    return nullptr;

  return GetFromNodeID(target_id);
}

std::set<AXPlatformNode*> AXPlatformNodeDelegateBase::GetNodesForNodeIds(
    const std::set<int32_t>& ids) {
  std::set<AXPlatformNode*> nodes;
  for (int32_t node_id : ids) {
    if (AXPlatformNode* node = GetFromNodeID(node_id)) {
      nodes.insert(node);
    }
  }
  return nodes;
}

std::vector<AXPlatformNode*>
AXPlatformNodeDelegateBase::GetTargetNodesForRelation(
    ax::mojom::IntListAttribute attr) {
  BASE_DCHECK(IsNodeIdIntListAttribute(attr));
  std::vector<int32_t> target_ids;
  if (!GetData().GetIntListAttribute(attr, &target_ids))
    return std::vector<AXPlatformNode*>();

  // If we use std::set to eliminate duplicates, the resulting set will be
  // sorted by the id and we will lose the original order which may be of
  // interest to ATs. The number of ids should be small.

  std::vector<ui::AXPlatformNode*> nodes;
  for (int32_t target_id : target_ids) {
    if (ui::AXPlatformNode* node = GetFromNodeID(target_id)) {
      if (std::find(nodes.begin(), nodes.end(), node) == nodes.end())
        nodes.push_back(node);
    }
  }

  return nodes;
}

std::set<AXPlatformNode*> AXPlatformNodeDelegateBase::GetReverseRelations(
    ax::mojom::IntAttribute attr) {
  // TODO(accessibility) Implement these if views ever use relations more
  // widely. The use so far has been for the Omnibox to the suggestion popup.
  // If this is ever implemented, then the "popup for" to "controlled by"
  // mapping in AXPlatformRelationWin can be removed, as it would be
  // redundant with setting the controls relationship.
  return std::set<AXPlatformNode*>();
}

std::set<AXPlatformNode*> AXPlatformNodeDelegateBase::GetReverseRelations(
    ax::mojom::IntListAttribute attr) {
  return std::set<AXPlatformNode*>();
}

std::u16string AXPlatformNodeDelegateBase::GetAuthorUniqueId() const {
  return std::u16string();
}

const AXUniqueId& AXPlatformNodeDelegateBase::GetUniqueId() const {
  static base::NoDestructor<AXUniqueId> dummy_unique_id;
  return *dummy_unique_id;
}

std::optional<int> AXPlatformNodeDelegateBase::FindTextBoundary(
    ax::mojom::TextBoundary boundary,
    int offset,
    ax::mojom::MoveDirection direction,
    ax::mojom::TextAffinity affinity) const {
  return std::nullopt;
}

const std::vector<gfx::NativeViewAccessible>
AXPlatformNodeDelegateBase::GetUIADescendants() const {
  return {};
}

std::string AXPlatformNodeDelegateBase::GetLanguage() const {
  return std::string();
}

AXPlatformNodeDelegate* AXPlatformNodeDelegateBase::GetParentDelegate() {
  AXPlatformNode* parent_node =
      ui::AXPlatformNode::FromNativeViewAccessible(GetParent());
  if (parent_node)
    return parent_node->GetDelegate();

  return nullptr;
}

std::string AXPlatformNodeDelegateBase::SubtreeToStringHelper(size_t level) {
  std::string result(level * 2, '+');
  result += ToString();
  result += '\n';

  // We can't use ChildrenBegin() and ChildrenEnd() here, because they both
  // return an std::unique_ptr<ChildIterator> which is an abstract class.
  //
  // TODO(accessibility): Refactor ChildIterator into a separate base
  // (non-abstract) class.
  auto iter_start = ChildIteratorBase(this, 0);
  auto iter_end = ChildIteratorBase(this, GetChildCount());
  for (auto iter = iter_start; iter != iter_end; ++iter) {
    AXPlatformNodeDelegateBase& child =
        static_cast<AXPlatformNodeDelegateBase&>(*iter);
    result += child.SubtreeToStringHelper(level + 1);
  }

  return result;
}

}  // namespace ui
