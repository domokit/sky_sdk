// Copyright 2013 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "ax_node.h"

#include <algorithm>
#include <utility>

#include "ax_enums.h"
#include "ax_role_properties.h"
#include "ax_table_info.h"
#include "ax_tree.h"
#include "ax_tree_manager.h"
#include "ax_tree_manager_map.h"
#include "base/color_utils.h"
#include "base/string_utils.h"

namespace ui {

constexpr AXNode::AXID AXNode::kInvalidAXID;

AXNode::AXNode(AXNode::OwnerTree* tree,
               AXNode* parent,
               int32_t id,
               size_t index_in_parent,
               size_t unignored_index_in_parent)
    : tree_(tree),
      index_in_parent_(index_in_parent),
      unignored_index_in_parent_(unignored_index_in_parent),
      parent_(parent) {
  data_.id = id;
}

AXNode::~AXNode() = default;

size_t AXNode::GetUnignoredChildCount() const {
  // TODO(nektar): Should BASE_DCHECK if the node is not ignored.
  BASE_DCHECK(!tree_->GetTreeUpdateInProgressState());
  return unignored_child_count_;
}

AXNodeData&& AXNode::TakeData() {
  return std::move(data_);
}

AXNode* AXNode::GetUnignoredChildAtIndex(size_t index) const {
  BASE_DCHECK(!tree_->GetTreeUpdateInProgressState());
  size_t count = 0;
  for (auto it = UnignoredChildrenBegin(); it != UnignoredChildrenEnd(); ++it) {
    if (count == index)
      return it.get();
    ++count;
  }
  return nullptr;
}

AXNode* AXNode::GetUnignoredParent() const {
  BASE_DCHECK(!tree_->GetTreeUpdateInProgressState());
  AXNode* result = parent();
  while (result && result->IsIgnored())
    result = result->parent();
  return result;
}

size_t AXNode::GetUnignoredIndexInParent() const {
  BASE_DCHECK(!tree_->GetTreeUpdateInProgressState());
  return unignored_index_in_parent_;
}

size_t AXNode::GetIndexInParent() const {
  BASE_DCHECK(!tree_->GetTreeUpdateInProgressState());
  return index_in_parent_;
}

AXNode* AXNode::GetFirstUnignoredChild() const {
  BASE_DCHECK(!tree_->GetTreeUpdateInProgressState());
  return ComputeFirstUnignoredChildRecursive();
}

AXNode* AXNode::GetLastUnignoredChild() const {
  BASE_DCHECK(!tree_->GetTreeUpdateInProgressState());
  return ComputeLastUnignoredChildRecursive();
}

AXNode* AXNode::GetDeepestFirstUnignoredChild() const {
  if (!GetUnignoredChildCount())
    return nullptr;

  AXNode* deepest_child = GetFirstUnignoredChild();
  while (deepest_child->GetUnignoredChildCount()) {
    deepest_child = deepest_child->GetFirstUnignoredChild();
  }

  return deepest_child;
}

AXNode* AXNode::GetDeepestLastUnignoredChild() const {
  if (!GetUnignoredChildCount())
    return nullptr;

  AXNode* deepest_child = GetLastUnignoredChild();
  while (deepest_child->GetUnignoredChildCount()) {
    deepest_child = deepest_child->GetLastUnignoredChild();
  }

  return deepest_child;
}

// Search for the next sibling of this node, skipping over any ignored nodes
// encountered.
//
// In our search:
//   If we find an ignored sibling, we consider its children as our siblings.
//   If we run out of siblings, we consider an ignored parent's siblings as our
//     own siblings.
//
// Note: this behaviour of 'skipping over' an ignored node makes this subtly
// different to finding the next (direct) sibling which is unignored.
//
// Consider a tree, where (i) marks a node as ignored:
//
//   1
//   ├── 2
//   ├── 3(i)
//   │   └── 5
//   └── 4
//
// The next sibling of node 2 is node 3, which is ignored.
// The next unignored sibling of node 2 could be either:
//  1) node 4 - next unignored sibling in the literal tree, or
//  2) node 5 - next unignored sibling in the logical document.
//
// There is no next sibling of node 5.
// The next unignored sibling of node 5 could be either:
//  1) null   - no next sibling in the literal tree, or
//  2) node 4 - next unignored sibling in the logical document.
//
// In both cases, this method implements approach (2).
//
// TODO(chrishall): Can we remove this non-reflexive case by forbidding
//   GetNextUnignoredSibling calls on an ignored started node?
// Note: this means that Next/Previous-UnignoredSibling are not reflexive if
// either of the nodes in question are ignored. From above we get an example:
//   NextUnignoredSibling(3)     is 4, but
//   PreviousUnignoredSibling(4) is 5.
//
// The view of unignored siblings for node 3 includes both node 2 and node 4:
//    2 <-- [3(i)] --> 4
//
// Whereas nodes 2, 5, and 4 do not consider node 3 to be an unignored sibling:
// null <-- [2] --> 5
//    2 <-- [5] --> 4
//    5 <-- [4] --> null
AXNode* AXNode::GetNextUnignoredSibling() const {
  BASE_DCHECK(!tree_->GetTreeUpdateInProgressState());
  const AXNode* current = this;

  // If there are children of the |current| node still to consider.
  bool considerChildren = false;

  while (current) {
    // A |candidate| sibling to consider.
    // If it is unignored then we have found our result.
    // Otherwise promote it to |current| and consider its children.
    AXNode* candidate;

    if (considerChildren && (candidate = current->GetFirstChild())) {
      if (!candidate->IsIgnored())
        return candidate;
      current = candidate;

    } else if ((candidate = current->GetNextSibling())) {
      if (!candidate->IsIgnored())
        return candidate;
      current = candidate;
      // Look through the ignored candidate node to consider their children as
      // though they were siblings.
      considerChildren = true;

    } else {
      // Continue our search through a parent iff they are ignored.
      //
      // If |current| has an ignored parent, then we consider the parent's
      // siblings as though they were siblings of |current|.
      //
      // Given a tree:
      //   1
      //   ├── 2(?)
      //   │   └── [4]
      //   └── 3
      //
      // Node 4's view of siblings:
      //   literal tree:   null <-- [4] --> null
      //
      // If node 2 is not ignored, then node 4's view doesn't change, and we
      // have no more nodes to consider:
      //   unignored tree: null <-- [4] --> null
      //
      // If instead node 2 is ignored, then node 4's view of siblings grows to
      // include node 3, and we have more nodes to consider:
      //   unignored tree: null <-- [4] --> 3
      current = current->parent();
      if (!current || !current->IsIgnored())
        return nullptr;

      // We have already considered all relevant descendants of |current|.
      considerChildren = false;
    }
  }

  return nullptr;
}

// Search for the previous sibling of this node, skipping over any ignored nodes
// encountered.
//
// In our search for a sibling:
//   If we find an ignored sibling, we may consider its children as siblings.
//   If we run out of siblings, we may consider an ignored parent's siblings as
//     our own.
//
// See the documentation for |GetNextUnignoredSibling| for more details.
AXNode* AXNode::GetPreviousUnignoredSibling() const {
  BASE_DCHECK(!tree_->GetTreeUpdateInProgressState());
  const AXNode* current = this;

  // If there are children of the |current| node still to consider.
  bool considerChildren = false;

  while (current) {
    // A |candidate| sibling to consider.
    // If it is unignored then we have found our result.
    // Otherwise promote it to |current| and consider its children.
    AXNode* candidate;

    if (considerChildren && (candidate = current->GetLastChild())) {
      if (!candidate->IsIgnored())
        return candidate;
      current = candidate;

    } else if ((candidate = current->GetPreviousSibling())) {
      if (!candidate->IsIgnored())
        return candidate;
      current = candidate;
      // Look through the ignored candidate node to consider their children as
      // though they were siblings.
      considerChildren = true;

    } else {
      // Continue our search through a parent iff they are ignored.
      //
      // If |current| has an ignored parent, then we consider the parent's
      // siblings as though they were siblings of |current|.
      //
      // Given a tree:
      //   1
      //   ├── 2
      //   └── 3(?)
      //       └── [4]
      //
      // Node 4's view of siblings:
      //   literal tree:   null <-- [4] --> null
      //
      // If node 3 is not ignored, then node 4's view doesn't change, and we
      // have no more nodes to consider:
      //   unignored tree: null <-- [4] --> null
      //
      // If instead node 3 is ignored, then node 4's view of siblings grows to
      // include node 2, and we have more nodes to consider:
      //   unignored tree:    2 <-- [4] --> null
      current = current->parent();
      if (!current || !current->IsIgnored())
        return nullptr;

      // We have already considered all relevant descendants of |current|.
      considerChildren = false;
    }
  }

  return nullptr;
}

AXNode* AXNode::GetNextUnignoredInTreeOrder() const {
  if (GetUnignoredChildCount())
    return GetFirstUnignoredChild();

  const AXNode* node = this;
  while (node) {
    AXNode* sibling = node->GetNextUnignoredSibling();
    if (sibling)
      return sibling;

    node = node->GetUnignoredParent();
  }

  return nullptr;
}

AXNode* AXNode::GetPreviousUnignoredInTreeOrder() const {
  AXNode* sibling = GetPreviousUnignoredSibling();
  if (!sibling)
    return GetUnignoredParent();

  if (sibling->GetUnignoredChildCount())
    return sibling->GetDeepestLastUnignoredChild();

  return sibling;
}

AXNode::UnignoredChildIterator AXNode::UnignoredChildrenBegin() const {
  BASE_DCHECK(!tree_->GetTreeUpdateInProgressState());
  return UnignoredChildIterator(this, GetFirstUnignoredChild());
}

AXNode::UnignoredChildIterator AXNode::UnignoredChildrenEnd() const {
  BASE_DCHECK(!tree_->GetTreeUpdateInProgressState());
  return UnignoredChildIterator(this, nullptr);
}

// The first (direct) child, ignored or unignored.
AXNode* AXNode::GetFirstChild() const {
  if (children().empty())
    return nullptr;
  return children()[0];
}

// The last (direct) child, ignored or unignored.
AXNode* AXNode::GetLastChild() const {
  size_t n = children().size();
  if (n == 0)
    return nullptr;
  return children()[n - 1];
}

// The previous (direct) sibling, ignored or unignored.
AXNode* AXNode::GetPreviousSibling() const {
  // Root nodes lack a parent, their index_in_parent should be 0.
  BASE_DCHECK(!parent() ? index_in_parent() == 0 : true);
  size_t index = index_in_parent();
  if (index == 0)
    return nullptr;
  return parent()->children()[index - 1];
}

// The next (direct) sibling, ignored or unignored.
AXNode* AXNode::GetNextSibling() const {
  if (!parent())
    return nullptr;
  size_t nextIndex = index_in_parent() + 1;
  if (nextIndex >= parent()->children().size())
    return nullptr;
  return parent()->children()[nextIndex];
}

bool AXNode::IsText() const {
  // In Legacy Layout, a list marker has no children and is thus represented on
  // all platforms as a leaf node that exposes the marker itself, i.e., it forms
  // part of the AX tree's text representation. In contrast, in Layout NG, a
  // list marker has a static text child.
  if (data().role == ax::mojom::Role::kListMarker)
    return !children().size();
  return ui::IsText(data().role);
}

bool AXNode::IsLineBreak() const {
  return data().role == ax::mojom::Role::kLineBreak ||
         (data().role == ax::mojom::Role::kInlineTextBox &&
          data().GetBoolAttribute(
              ax::mojom::BoolAttribute::kIsLineBreakingObject));
}

void AXNode::SetData(const AXNodeData& src) {
  data_ = src;
}

void AXNode::SetLocation(int32_t offset_container_id,
                         const gfx::RectF& location,
                         gfx::Transform* transform) {
  data_.relative_bounds.offset_container_id = offset_container_id;
  data_.relative_bounds.bounds = location;
  if (transform) {
    data_.relative_bounds.transform =
        std::make_unique<gfx::Transform>(*transform);
  } else {
    data_.relative_bounds.transform.reset();
  }
}

void AXNode::SetIndexInParent(size_t index_in_parent) {
  index_in_parent_ = index_in_parent;
}

void AXNode::UpdateUnignoredCachedValues() {
  if (!IsIgnored())
    UpdateUnignoredCachedValuesRecursive(0);
}

void AXNode::SwapChildren(std::vector<AXNode*>* children) {
  children->swap(children_);
}

void AXNode::Destroy() {
  delete this;
}

bool AXNode::IsDescendantOf(const AXNode* ancestor) const {
  if (this == ancestor)
    return true;
  if (parent())
    return parent()->IsDescendantOf(ancestor);

  return false;
}

std::vector<int> AXNode::GetOrComputeLineStartOffsets() {
  std::vector<int> line_offsets;
  if (data().GetIntListAttribute(ax::mojom::IntListAttribute::kCachedLineStarts,
                                 &line_offsets)) {
    return line_offsets;
  }

  int start_offset = 0;
  ComputeLineStartOffsets(&line_offsets, &start_offset);
  data_.AddIntListAttribute(ax::mojom::IntListAttribute::kCachedLineStarts,
                            line_offsets);
  return line_offsets;
}

void AXNode::ComputeLineStartOffsets(std::vector<int>* line_offsets,
                                     int* start_offset) const {
  BASE_DCHECK(line_offsets);
  BASE_DCHECK(start_offset);
  for (const AXNode* child : children()) {
    BASE_DCHECK(child);
    if (!child->children().empty()) {
      child->ComputeLineStartOffsets(line_offsets, start_offset);
      continue;
    }

    // Don't report if the first piece of text starts a new line or not.
    if (*start_offset && !child->data().HasIntAttribute(
                             ax::mojom::IntAttribute::kPreviousOnLineId)) {
      // If there are multiple objects with an empty accessible label at the
      // start of a line, only include a single line start offset.
      if (line_offsets->empty() || line_offsets->back() != *start_offset)
        line_offsets->push_back(*start_offset);
    }

    std::u16string text =
        child->data().GetString16Attribute(ax::mojom::StringAttribute::kName);
    *start_offset += static_cast<int>(text.length());
  }
}

const std::string& AXNode::GetInheritedStringAttribute(
    ax::mojom::StringAttribute attribute) const {
  const AXNode* current_node = this;
  do {
    if (current_node->data().HasStringAttribute(attribute))
      return current_node->data().GetStringAttribute(attribute);
    current_node = current_node->parent();
  } while (current_node);
  return base::EmptyString();
}

std::u16string AXNode::GetInheritedString16Attribute(
    ax::mojom::StringAttribute attribute) const {
  return base::UTF8ToUTF16(GetInheritedStringAttribute(attribute));
}

std::string AXNode::GetInnerText() const {
  // If a text field has no descendants, then we compute its inner text from its
  // value or its placeholder. Otherwise we prefer to look at its descendant
  // text nodes because Blink doesn't always add all trailing white space to the
  // value attribute.
  const bool is_plain_text_field_without_descendants =
      (data().IsTextField() && !GetUnignoredChildCount());
  if (is_plain_text_field_without_descendants) {
    std::string value =
        data().GetStringAttribute(ax::mojom::StringAttribute::kValue);
    // If the value is empty, then there might be some placeholder text in the
    // text field, or any other name that is derived from visible contents, even
    // if the text field has no children.
    if (!value.empty())
      return value;
  }

  // Ordinarily, plain text fields are leaves. We need to exclude them from the
  // set of leaf nodes when they expose any descendants. This is because we want
  // to compute their inner text from their descendant text nodes as we don't
  // always trust the "value" attribute provided by Blink.
  const bool is_plain_text_field_with_descendants =
      (data().IsTextField() && GetUnignoredChildCount());
  if (IsLeaf() && !is_plain_text_field_with_descendants) {
    switch (data().GetNameFrom()) {
      case ax::mojom::NameFrom::kNone:
      case ax::mojom::NameFrom::kUninitialized:
      // The accessible name is not displayed on screen, e.g. aria-label, or is
      // not displayed directly inside the node, e.g. an associated label
      // element.
      case ax::mojom::NameFrom::kAttribute:
      // The node's accessible name is explicitly empty.
      case ax::mojom::NameFrom::kAttributeExplicitlyEmpty:
      // The accessible name does not represent the entirety of the node's inner
      // text, e.g. a table's caption or a figure's figcaption.
      case ax::mojom::NameFrom::kCaption:
      case ax::mojom::NameFrom::kRelatedElement:
      // The accessible name is not displayed directly inside the node but is
      // visible via e.g. a tooltip.
      case ax::mojom::NameFrom::kTitle:
        return std::string();

      case ax::mojom::NameFrom::kContents:
      // The placeholder text is initially displayed inside the text field and
      // takes the place of its value.
      case ax::mojom::NameFrom::kPlaceholder:
      // The value attribute takes the place of the node's inner text, e.g. the
      // value of a submit button is displayed inside the button itself.
      case ax::mojom::NameFrom::kValue:
        return data().GetStringAttribute(ax::mojom::StringAttribute::kName);
    }
  }

  std::string inner_text;
  for (auto it = UnignoredChildrenBegin(); it != UnignoredChildrenEnd(); ++it) {
    inner_text += it->GetInnerText();
  }
  return inner_text;
}

std::string AXNode::GetLanguage() const {
  return std::string();
}

std::ostream& operator<<(std::ostream& stream, const AXNode& node) {
  return stream << node.data().ToString();
}

bool AXNode::IsTable() const {
  return IsTableLike(data().role);
}

std::optional<int> AXNode::GetTableColCount() const {
  const AXTableInfo* table_info = GetAncestorTableInfo();
  if (!table_info)
    return std::nullopt;
  return static_cast<int>(table_info->col_count);
}

std::optional<int> AXNode::GetTableRowCount() const {
  const AXTableInfo* table_info = GetAncestorTableInfo();
  if (!table_info)
    return std::nullopt;
  return static_cast<int>(table_info->row_count);
}

std::optional<int> AXNode::GetTableAriaColCount() const {
  const AXTableInfo* table_info = GetAncestorTableInfo();
  if (!table_info)
    return std::nullopt;
  return std::make_optional(table_info->aria_col_count);
}

std::optional<int> AXNode::GetTableAriaRowCount() const {
  const AXTableInfo* table_info = GetAncestorTableInfo();
  if (!table_info)
    return std::nullopt;
  return std::make_optional(table_info->aria_row_count);
}

std::optional<int> AXNode::GetTableCellCount() const {
  const AXTableInfo* table_info = GetAncestorTableInfo();
  if (!table_info)
    return std::nullopt;

  return static_cast<int>(table_info->unique_cell_ids.size());
}

std::optional<bool> AXNode::GetTableHasColumnOrRowHeaderNode() const {
  const AXTableInfo* table_info = GetAncestorTableInfo();
  if (!table_info)
    return std::nullopt;

  return !table_info->all_headers.empty();
}

AXNode* AXNode::GetTableCellFromIndex(int index) const {
  const AXTableInfo* table_info = GetAncestorTableInfo();
  if (!table_info)
    return nullptr;

  // There is a table but there is no cell with the given index.
  if (index < 0 ||
      static_cast<size_t>(index) >= table_info->unique_cell_ids.size()) {
    return nullptr;
  }

  return tree_->GetFromId(
      table_info->unique_cell_ids[static_cast<size_t>(index)]);
}

AXNode* AXNode::GetTableCaption() const {
  const AXTableInfo* table_info = GetAncestorTableInfo();
  if (!table_info)
    return nullptr;

  return tree_->GetFromId(table_info->caption_id);
}

AXNode* AXNode::GetTableCellFromCoords(int row_index, int col_index) const {
  const AXTableInfo* table_info = GetAncestorTableInfo();
  if (!table_info)
    return nullptr;

  // There is a table but the given coordinates are outside the table.
  if (row_index < 0 ||
      static_cast<size_t>(row_index) >= table_info->row_count ||
      col_index < 0 ||
      static_cast<size_t>(col_index) >= table_info->col_count) {
    return nullptr;
  }

  return tree_->GetFromId(table_info->cell_ids[static_cast<size_t>(row_index)]
                                              [static_cast<size_t>(col_index)]);
}

std::vector<AXNode::AXID> AXNode::GetTableColHeaderNodeIds() const {
  const AXTableInfo* table_info = GetAncestorTableInfo();
  if (!table_info)
    return std::vector<AXNode::AXID>();

  std::vector<AXNode::AXID> col_header_ids;
  // Flatten and add column header ids of each column to |col_header_ids|.
  for (std::vector<AXNode::AXID> col_headers_at_index :
       table_info->col_headers) {
    col_header_ids.insert(col_header_ids.end(), col_headers_at_index.begin(),
                          col_headers_at_index.end());
  }

  return col_header_ids;
}

std::vector<AXNode::AXID> AXNode::GetTableColHeaderNodeIds(
    int col_index) const {
  const AXTableInfo* table_info = GetAncestorTableInfo();
  if (!table_info)
    return std::vector<AXNode::AXID>();

  if (col_index < 0 || static_cast<size_t>(col_index) >= table_info->col_count)
    return std::vector<AXNode::AXID>();

  return std::vector<AXNode::AXID>(
      table_info->col_headers[static_cast<size_t>(col_index)]);
}

std::vector<AXNode::AXID> AXNode::GetTableRowHeaderNodeIds(
    int row_index) const {
  const AXTableInfo* table_info = GetAncestorTableInfo();
  if (!table_info)
    return std::vector<AXNode::AXID>();

  if (row_index < 0 || static_cast<size_t>(row_index) >= table_info->row_count)
    return std::vector<AXNode::AXID>();

  return std::vector<AXNode::AXID>(
      table_info->row_headers[static_cast<size_t>(row_index)]);
}

std::vector<AXNode::AXID> AXNode::GetTableUniqueCellIds() const {
  const AXTableInfo* table_info = GetAncestorTableInfo();
  if (!table_info)
    return std::vector<AXNode::AXID>();

  return std::vector<AXNode::AXID>(table_info->unique_cell_ids);
}

const std::vector<AXNode*>* AXNode::GetExtraMacNodes() const {
  // Should only be available on the table node itself, not any of its children.
  const AXTableInfo* table_info = tree_->GetTableInfo(this);
  if (!table_info)
    return nullptr;

  return &table_info->extra_mac_nodes;
}

//
// Table row-like nodes.
//

bool AXNode::IsTableRow() const {
  return ui::IsTableRow(data().role);
}

std::optional<int> AXNode::GetTableRowRowIndex() const {
  if (!IsTableRow())
    return std::nullopt;

  const AXTableInfo* table_info = GetAncestorTableInfo();
  if (!table_info)
    return std::nullopt;

  const auto& iter = table_info->row_id_to_index.find(id());
  if (iter == table_info->row_id_to_index.end())
    return std::nullopt;
  return static_cast<int>(iter->second);
}

std::vector<AXNode::AXID> AXNode::GetTableRowNodeIds() const {
  std::vector<AXNode::AXID> row_node_ids;
  const AXTableInfo* table_info = GetAncestorTableInfo();
  if (!table_info)
    return row_node_ids;

  for (AXNode* node : table_info->row_nodes)
    row_node_ids.push_back(node->data().id);

  return row_node_ids;
}

#if defined(OS_APPLE)

//
// Table column-like nodes. These nodes are only present on macOS.
//

bool AXNode::IsTableColumn() const {
  return ui::IsTableColumn(data().role);
}

std::optional<int> AXNode::GetTableColColIndex() const {
  if (!IsTableColumn())
    return std::nullopt;

  const AXTableInfo* table_info = GetAncestorTableInfo();
  if (!table_info)
    return std::nullopt;

  int index = 0;
  for (const AXNode* node : table_info->extra_mac_nodes) {
    if (node == this)
      break;
    index++;
  }
  return index;
}

#endif  // defined(OS_APPLE)

//
// Table cell-like nodes.
//

bool AXNode::IsTableCellOrHeader() const {
  return IsCellOrTableHeader(data().role);
}

std::optional<int> AXNode::GetTableCellIndex() const {
  if (!IsTableCellOrHeader())
    return std::nullopt;

  const AXTableInfo* table_info = GetAncestorTableInfo();
  if (!table_info)
    return std::nullopt;

  const auto& iter = table_info->cell_id_to_index.find(id());
  if (iter != table_info->cell_id_to_index.end())
    return static_cast<int>(iter->second);
  return std::nullopt;
}

std::optional<int> AXNode::GetTableCellColIndex() const {
  const AXTableInfo* table_info = GetAncestorTableInfo();
  if (!table_info)
    return std::nullopt;

  std::optional<int> index = GetTableCellIndex();
  if (!index)
    return std::nullopt;

  return static_cast<int>(table_info->cell_data_vector[*index].col_index);
}

std::optional<int> AXNode::GetTableCellRowIndex() const {
  const AXTableInfo* table_info = GetAncestorTableInfo();
  if (!table_info)
    return std::nullopt;

  std::optional<int> index = GetTableCellIndex();
  if (!index)
    return std::nullopt;

  return static_cast<int>(table_info->cell_data_vector[*index].row_index);
}

std::optional<int> AXNode::GetTableCellColSpan() const {
  // If it's not a table cell, don't return a col span.
  if (!IsTableCellOrHeader())
    return std::nullopt;

  // Otherwise, try to return a colspan, with 1 as the default if it's not
  // specified.
  int col_span;
  if (GetIntAttribute(ax::mojom::IntAttribute::kTableCellColumnSpan, &col_span))
    return col_span;
  return 1;
}

std::optional<int> AXNode::GetTableCellRowSpan() const {
  // If it's not a table cell, don't return a row span.
  if (!IsTableCellOrHeader())
    return std::nullopt;

  // Otherwise, try to return a row span, with 1 as the default if it's not
  // specified.
  int row_span;
  if (GetIntAttribute(ax::mojom::IntAttribute::kTableCellRowSpan, &row_span))
    return row_span;
  return 1;
}

std::optional<int> AXNode::GetTableCellAriaColIndex() const {
  const AXTableInfo* table_info = GetAncestorTableInfo();
  if (!table_info)
    return std::nullopt;

  std::optional<int> index = GetTableCellIndex();
  if (!index)
    return std::nullopt;

  return static_cast<int>(table_info->cell_data_vector[*index].aria_col_index);
}

std::optional<int> AXNode::GetTableCellAriaRowIndex() const {
  const AXTableInfo* table_info = GetAncestorTableInfo();
  if (!table_info)
    return std::nullopt;

  std::optional<int> index = GetTableCellIndex();
  if (!index)
    return std::nullopt;

  return static_cast<int>(table_info->cell_data_vector[*index].aria_row_index);
}

std::vector<AXNode::AXID> AXNode::GetTableCellColHeaderNodeIds() const {
  const AXTableInfo* table_info = GetAncestorTableInfo();
  if (!table_info || table_info->col_count <= 0)
    return std::vector<AXNode::AXID>();

  // If this node is not a cell, then return the headers for the first column.
  int col_index = GetTableCellColIndex().value_or(0);

  return std::vector<AXNode::AXID>(table_info->col_headers[col_index]);
}

void AXNode::GetTableCellColHeaders(std::vector<AXNode*>* col_headers) const {
  BASE_DCHECK(col_headers);

  std::vector<int32_t> col_header_ids = GetTableCellColHeaderNodeIds();
  IdVectorToNodeVector(col_header_ids, col_headers);
}

std::vector<AXNode::AXID> AXNode::GetTableCellRowHeaderNodeIds() const {
  const AXTableInfo* table_info = GetAncestorTableInfo();
  if (!table_info || table_info->row_count <= 0)
    return std::vector<AXNode::AXID>();

  // If this node is not a cell, then return the headers for the first row.
  int row_index = GetTableCellRowIndex().value_or(0);

  return std::vector<AXNode::AXID>(table_info->row_headers[row_index]);
}

void AXNode::GetTableCellRowHeaders(std::vector<AXNode*>* row_headers) const {
  BASE_DCHECK(row_headers);

  std::vector<int32_t> row_header_ids = GetTableCellRowHeaderNodeIds();
  IdVectorToNodeVector(row_header_ids, row_headers);
}

bool AXNode::IsCellOrHeaderOfARIATable() const {
  if (!IsTableCellOrHeader())
    return false;

  const AXNode* node = this;
  while (node && !node->IsTable())
    node = node->parent();
  if (!node)
    return false;

  return node->data().role == ax::mojom::Role::kTable;
}

bool AXNode::IsCellOrHeaderOfARIAGrid() const {
  if (!IsTableCellOrHeader())
    return false;

  const AXNode* node = this;
  while (node && !node->IsTable())
    node = node->parent();
  if (!node)
    return false;

  return node->data().role == ax::mojom::Role::kGrid ||
         node->data().role == ax::mojom::Role::kTreeGrid;
}

AXTableInfo* AXNode::GetAncestorTableInfo() const {
  const AXNode* node = this;
  while (node && !node->IsTable())
    node = node->parent();
  if (node)
    return tree_->GetTableInfo(node);
  return nullptr;
}

void AXNode::IdVectorToNodeVector(const std::vector<int32_t>& ids,
                                  std::vector<AXNode*>* nodes) const {
  for (int32_t id : ids) {
    AXNode* node = tree_->GetFromId(id);
    if (node)
      nodes->push_back(node);
  }
}

std::optional<int> AXNode::GetHierarchicalLevel() const {
  int hierarchical_level =
      GetIntAttribute(ax::mojom::IntAttribute::kHierarchicalLevel);

  // According to the WAI_ARIA spec, a defined hierarchical level value is
  // greater than 0.
  // https://www.w3.org/TR/wai-aria-1.1/#aria-level
  if (hierarchical_level > 0)
    return hierarchical_level;

  return std::nullopt;
}

bool AXNode::IsOrderedSetItem() const {
  return ui::IsItemLike(data().role);
}

bool AXNode::IsOrderedSet() const {
  return ui::IsSetLike(data().role);
}

// Uses AXTree's cache to calculate node's PosInSet.
std::optional<int> AXNode::GetPosInSet() {
  return tree_->GetPosInSet(*this);
}

// Uses AXTree's cache to calculate node's SetSize.
std::optional<int> AXNode::GetSetSize() {
  return tree_->GetSetSize(*this);
}

// Returns true if the role of ordered set matches the role of item.
// Returns false otherwise.
bool AXNode::SetRoleMatchesItemRole(const AXNode* ordered_set) const {
  ax::mojom::Role item_role = data().role;
  // Switch on role of ordered set
  switch (ordered_set->data().role) {
    case ax::mojom::Role::kFeed:
      return item_role == ax::mojom::Role::kArticle;
    case ax::mojom::Role::kList:
      return item_role == ax::mojom::Role::kListItem;
    case ax::mojom::Role::kGroup:
      return item_role == ax::mojom::Role::kComment ||
             item_role == ax::mojom::Role::kListItem ||
             item_role == ax::mojom::Role::kMenuItem ||
             item_role == ax::mojom::Role::kMenuItemRadio ||
             item_role == ax::mojom::Role::kListBoxOption ||
             item_role == ax::mojom::Role::kTreeItem;
    case ax::mojom::Role::kMenu:
      return item_role == ax::mojom::Role::kMenuItem ||
             item_role == ax::mojom::Role::kMenuItemRadio ||
             item_role == ax::mojom::Role::kMenuItemCheckBox;
    case ax::mojom::Role::kMenuBar:
      return item_role == ax::mojom::Role::kMenuItem ||
             item_role == ax::mojom::Role::kMenuItemRadio ||
             item_role == ax::mojom::Role::kMenuItemCheckBox;
    case ax::mojom::Role::kTabList:
      return item_role == ax::mojom::Role::kTab;
    case ax::mojom::Role::kTree:
      return item_role == ax::mojom::Role::kTreeItem;
    case ax::mojom::Role::kListBox:
      return item_role == ax::mojom::Role::kListBoxOption;
    case ax::mojom::Role::kMenuListPopup:
      return item_role == ax::mojom::Role::kMenuListOption ||
             item_role == ax::mojom::Role::kMenuItem ||
             item_role == ax::mojom::Role::kMenuItemRadio ||
             item_role == ax::mojom::Role::kMenuItemCheckBox;
    case ax::mojom::Role::kRadioGroup:
      return item_role == ax::mojom::Role::kRadioButton;
    case ax::mojom::Role::kDescriptionList:
      // Only the term for each description list entry should receive posinset
      // and setsize.
      return item_role == ax::mojom::Role::kDescriptionListTerm ||
             item_role == ax::mojom::Role::kTerm;
    case ax::mojom::Role::kPopUpButton:
      // kPopUpButtons can wrap a kMenuListPopUp.
      return item_role == ax::mojom::Role::kMenuListPopup;
    default:
      return false;
  }
}

bool AXNode::IsIgnoredContainerForOrderedSet() const {
  return IsIgnored() || IsEmbeddedGroup() ||
         data().role == ax::mojom::Role::kListItem ||
         data().role == ax::mojom::Role::kGenericContainer ||
         data().role == ax::mojom::Role::kUnknown;
}

int AXNode::UpdateUnignoredCachedValuesRecursive(int startIndex) {
  int count = 0;
  for (AXNode* child : children_) {
    if (child->IsIgnored()) {
      child->unignored_index_in_parent_ = 0;
      count += child->UpdateUnignoredCachedValuesRecursive(startIndex + count);
    } else {
      child->unignored_index_in_parent_ = startIndex + count++;
    }
  }
  unignored_child_count_ = count;
  return count;
}

// Finds ordered set that contains node.
// Is not required for set's role to match node's role.
AXNode* AXNode::GetOrderedSet() const {
  AXNode* result = parent();
  // Continue walking up while parent is invalid, ignored, a generic container,
  // unknown, or embedded group.
  while (result && result->IsIgnoredContainerForOrderedSet()) {
    result = result->parent();
  }

  return result;
}

AXNode* AXNode::ComputeLastUnignoredChildRecursive() const {
  BASE_DCHECK(!tree_->GetTreeUpdateInProgressState());
  if (children().empty())
    return nullptr;

  for (int i = static_cast<int>(children().size()) - 1; i >= 0; --i) {
    AXNode* child = children_[i];
    if (!child->IsIgnored())
      return child;

    AXNode* descendant = child->ComputeLastUnignoredChildRecursive();
    if (descendant)
      return descendant;
  }
  return nullptr;
}

AXNode* AXNode::ComputeFirstUnignoredChildRecursive() const {
  BASE_DCHECK(!tree_->GetTreeUpdateInProgressState());
  for (size_t i = 0; i < children().size(); i++) {
    AXNode* child = children_[i];
    if (!child->IsIgnored())
      return child;

    AXNode* descendant = child->ComputeFirstUnignoredChildRecursive();
    if (descendant)
      return descendant;
  }
  return nullptr;
}

bool AXNode::IsIgnored() const {
  return data().IsIgnored();
}

bool AXNode::IsChildOfLeaf() const {
  const AXNode* ancestor = GetUnignoredParent();
  while (ancestor) {
    if (ancestor->IsLeaf())
      return true;
    ancestor = ancestor->GetUnignoredParent();
  }
  return false;
}

bool AXNode::IsLeaf() const {
  // A node is also a leaf if all of it's descendants are ignored.
  if (children().empty() || !GetUnignoredChildCount())
    return true;

#if defined(OS_WIN)
  // On Windows, we want to hide the subtree of a collapsed <select> element.
  // Otherwise, ATs are always going to announce its options whether it's
  // collapsed or expanded. In the AXTree, this element corresponds to a node
  // with role ax::mojom::Role::kPopUpButton that is the parent of a node with
  // role ax::mojom::Role::kMenuListPopup.
  if (IsCollapsedMenuListPopUpButton())
    return true;
#endif  // defined(OS_WIN)

  // These types of objects may have children that we use as internal
  // implementation details, but we want to expose them as leaves to platform
  // accessibility APIs because screen readers might be confused if they find
  // any children.
  if (data().IsPlainTextField() || IsText())
    return true;

  // Roles whose children are only presentational according to the ARIA and
  // HTML5 Specs should be hidden from screen readers.
  switch (data().role) {
    // According to the ARIA and Core-AAM specs:
    // https://w3c.github.io/aria/#button,
    // https://www.w3.org/TR/core-aam-1.1/#exclude_elements
    // buttons' children are presentational only and should be hidden from
    // screen readers. However, we cannot enforce the leafiness of buttons
    // because they may contain many rich, interactive descendants such as a day
    // in a calendar, and screen readers will need to interact with these
    // contents. See https://crbug.com/689204.
    // So we decided to not enforce the leafiness of buttons and expose all
    // children.
    case ax::mojom::Role::kButton:
      return false;
    case ax::mojom::Role::kDocCover:
    case ax::mojom::Role::kGraphicsSymbol:
    case ax::mojom::Role::kImage:
    case ax::mojom::Role::kMeter:
    case ax::mojom::Role::kScrollBar:
    case ax::mojom::Role::kSlider:
    case ax::mojom::Role::kSplitter:
    case ax::mojom::Role::kProgressIndicator:
      return true;
    default:
      return false;
  }
}

bool AXNode::IsInListMarker() const {
  if (data().role == ax::mojom::Role::kListMarker)
    return true;

  // List marker node's children can only be text elements.
  if (!IsText())
    return false;

  // There is no need to iterate over all the ancestors of the current anchor
  // since a list marker node only has children on 2 levels.
  // i.e.:
  // AXLayoutObject role=kListMarker
  // ++StaticText
  // ++++InlineTextBox
  AXNode* parent_node = GetUnignoredParent();
  if (parent_node && parent_node->data().role == ax::mojom::Role::kListMarker)
    return true;

  AXNode* grandparent_node = parent_node->GetUnignoredParent();
  return grandparent_node &&
         grandparent_node->data().role == ax::mojom::Role::kListMarker;
}

bool AXNode::IsCollapsedMenuListPopUpButton() const {
  if (data().role != ax::mojom::Role::kPopUpButton ||
      !data().HasState(ax::mojom::State::kCollapsed)) {
    return false;
  }

  // When a popup button contains a menu list popup, its only child is unignored
  // and is a menu list popup.
  AXNode* node = GetFirstUnignoredChild();
  if (!node)
    return false;

  return node->data().role == ax::mojom::Role::kMenuListPopup;
}

AXNode* AXNode::GetCollapsedMenuListPopUpButtonAncestor() const {
  AXNode* node = GetOrderedSet();

  if (!node)
    return nullptr;

  // The ordered set returned is either the popup element child of the popup
  // button (e.g., the AXMenuListPopup) or the popup button itself. We need
  // |node| to point to the popup button itself.
  if (node->data().role != ax::mojom::Role::kPopUpButton) {
    node = node->parent();
    if (!node)
      return nullptr;
  }

  return node->IsCollapsedMenuListPopUpButton() ? node : nullptr;
}

bool AXNode::IsEmbeddedGroup() const {
  if (data().role != ax::mojom::Role::kGroup || !parent())
    return false;

  return ui::IsSetLike(parent()->data().role);
}

AXNode* AXNode::GetLowestPlatformAncestor() const {
  AXNode* current_node = const_cast<AXNode*>(this);
  AXNode* lowest_unignored_node = current_node;
  for (; lowest_unignored_node && lowest_unignored_node->IsIgnored();
       lowest_unignored_node = lowest_unignored_node->parent()) {
  }

  // `highest_leaf_node` could be nullptr.
  AXNode* highest_leaf_node = lowest_unignored_node;
  // For the purposes of this method, a leaf node does not include leaves in the
  // internal accessibility tree, only in the platform exposed tree.
  for (AXNode* ancestor_node = lowest_unignored_node; ancestor_node;
       ancestor_node = ancestor_node->GetUnignoredParent()) {
    if (ancestor_node->IsLeaf())
      highest_leaf_node = ancestor_node;
  }
  if (highest_leaf_node)
    return highest_leaf_node;

  if (lowest_unignored_node)
    return lowest_unignored_node;
  return current_node;
}

AXNode* AXNode::GetTextFieldAncestor() const {
  AXNode* parent = GetUnignoredParent();

  while (parent && parent->data().HasState(ax::mojom::State::kEditable)) {
    if (parent->data().IsPlainTextField() || parent->data().IsRichTextField())
      return parent;

    parent = parent->GetUnignoredParent();
  }

  return nullptr;
}

bool AXNode::IsDescendantOfCrossingTreeBoundary(const AXNode* ancestor) const {
  if (!ancestor)
    return false;
  if (this == ancestor)
    return true;
  if (const AXNode* parent = GetParentCrossingTreeBoundary())
    return parent->IsDescendantOfCrossingTreeBoundary(ancestor);
  return false;
}

AXNode* AXNode::GetParentCrossingTreeBoundary() const {
  BASE_DCHECK(!tree_->GetTreeUpdateInProgressState());
  if (parent_)
    return parent_;
  const AXTreeManager* manager =
      AXTreeManagerMap::GetInstance().GetManager(tree_->GetAXTreeID());
  if (manager)
    return manager->GetParentNodeFromParentTreeAsAXNode();
  return nullptr;
}

AXTree::Selection AXNode::GetUnignoredSelection() const {
  BASE_DCHECK(tree())
      << "Cannot retrieve the current selection if the node is not "
         "attached to an accessibility tree.\n"
      << *this;
  AXTree::Selection selection = tree()->GetUnignoredSelection();

  // "selection.anchor_offset" and "selection.focus_ofset" might need to be
  // adjusted if the anchor or the focus nodes include ignored children.
  //
  // TODO(nektar): Move this logic into its own "AXSelection" class and cache
  // the result for faster reuse.
  const AXNode* anchor = tree()->GetFromId(selection.anchor_object_id);
  if (anchor && !anchor->IsLeaf()) {
    BASE_DCHECK(selection.anchor_offset >= 0);
    if (static_cast<size_t>(selection.anchor_offset) <
        anchor->children().size()) {
      const AXNode* anchor_child = anchor->children()[selection.anchor_offset];
      BASE_DCHECK(anchor_child);
      selection.anchor_offset =
          static_cast<int>(anchor_child->GetUnignoredIndexInParent());
    } else {
      selection.anchor_offset =
          static_cast<int>(anchor->GetUnignoredChildCount());
    }
  }

  const AXNode* focus = tree()->GetFromId(selection.focus_object_id);
  if (focus && !focus->IsLeaf()) {
    BASE_DCHECK(selection.focus_offset >= 0);
    if (static_cast<size_t>(selection.focus_offset) <
        focus->children().size()) {
      const AXNode* focus_child = focus->children()[selection.focus_offset];
      BASE_DCHECK(focus_child);
      selection.focus_offset =
          static_cast<int>(focus_child->GetUnignoredIndexInParent());
    } else {
      selection.focus_offset =
          static_cast<int>(focus->GetUnignoredChildCount());
    }
  }
  return selection;
}

}  // namespace ui
