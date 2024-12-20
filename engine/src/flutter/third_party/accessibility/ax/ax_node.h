// Copyright 2013 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef UI_ACCESSIBILITY_AX_NODE_H_
#define UI_ACCESSIBILITY_AX_NODE_H_

#include <cstdint>
#include <memory>
#include <optional>
#include <ostream>
#include <string>
#include <vector>

#include "ax_build/build_config.h"
#include "ax_export.h"
#include "ax_node_data.h"
#include "ax_tree_id.h"
#include "base/logging.h"
#include "gfx/geometry/rect.h"
#include "gfx/transform.h"

#ifdef _WIN32
// windowx.h defines GetNextSibling as a macro.
#undef GetNextSibling
#endif

namespace ui {

class AXTableInfo;

// One node in an AXTree.
class AX_EXPORT AXNode final {
 public:
  // Defines the type used for AXNode IDs.
  using AXID = int32_t;

  // TODO(chunhtai): I modified this to be -1 so it can work with flutter.
  // If a node is not yet or no longer valid, its ID should have a value of
  // kInvalidAXID.
  static constexpr AXID kInvalidAXID = -1;

  // Interface to the tree class that owns an AXNode. We use this instead
  // of letting AXNode have a pointer to its AXTree directly so that we're
  // forced to think twice before calling an AXTree interface that might not
  // be necessary.
  class OwnerTree {
   public:
    struct Selection {
      bool is_backward;
      AXID anchor_object_id;
      int anchor_offset;
      ax::mojom::TextAffinity anchor_affinity;
      AXID focus_object_id;
      int focus_offset;
      ax::mojom::TextAffinity focus_affinity;
    };

    // See AXTree::GetAXTreeID.
    virtual AXTreeID GetAXTreeID() const = 0;
    // See AXTree::GetTableInfo.
    virtual AXTableInfo* GetTableInfo(const AXNode* table_node) const = 0;
    // See AXTree::GetFromId.
    virtual AXNode* GetFromId(int32_t id) const = 0;

    virtual std::optional<int> GetPosInSet(const AXNode& node) = 0;
    virtual std::optional<int> GetSetSize(const AXNode& node) = 0;

    virtual Selection GetUnignoredSelection() const = 0;
    virtual bool GetTreeUpdateInProgressState() const = 0;
    virtual bool HasPaginationSupport() const = 0;
  };

  template <typename NodeType,
            NodeType* (NodeType::*NextSibling)() const,
            NodeType* (NodeType::*PreviousSibling)() const,
            NodeType* (NodeType::*FirstChild)() const,
            NodeType* (NodeType::*LastChild)() const>
  class ChildIteratorBase {
   public:
    ChildIteratorBase(const NodeType* parent, NodeType* child);
    ChildIteratorBase(const ChildIteratorBase& it);
    ~ChildIteratorBase() {}
    bool operator==(const ChildIteratorBase& rhs) const;
    bool operator!=(const ChildIteratorBase& rhs) const;
    ChildIteratorBase& operator++();
    ChildIteratorBase& operator--();
    NodeType* get() const;
    NodeType& operator*() const;
    NodeType* operator->() const;

   protected:
    const NodeType* parent_;
    NodeType* child_;
  };

  // The constructor requires a parent, id, and index in parent, but
  // the data is not required. After initialization, only index_in_parent
  // and unignored_index_in_parent is allowed to change, the others are
  // guaranteed to never change.
  AXNode(OwnerTree* tree,
         AXNode* parent,
         int32_t id,
         size_t index_in_parent,
         size_t unignored_index_in_parent = 0);
  virtual ~AXNode();

  // Accessors.
  OwnerTree* tree() const { return tree_; }
  AXID id() const { return data_.id; }
  AXNode* parent() const { return parent_; }
  const AXNodeData& data() const { return data_; }
  const std::vector<AXNode*>& children() const { return children_; }
  size_t index_in_parent() const { return index_in_parent_; }

  // Returns ownership of |data_| to the caller; effectively clearing |data_|.
  AXNodeData&& TakeData();

  // Walking the tree skipping ignored nodes.
  size_t GetUnignoredChildCount() const;
  AXNode* GetUnignoredChildAtIndex(size_t index) const;
  AXNode* GetUnignoredParent() const;
  // Gets the unignored selection from the accessibility tree, meaning the
  // selection whose endpoints are on unignored nodes. (An "ignored" node is a
  // node that is not exposed to platform APIs: See `IsIgnored`.)
  OwnerTree::Selection GetUnignoredSelection() const;
  size_t GetUnignoredIndexInParent() const;
  size_t GetIndexInParent() const;
  AXNode* GetFirstUnignoredChild() const;
  AXNode* GetLastUnignoredChild() const;
  AXNode* GetDeepestFirstUnignoredChild() const;
  AXNode* GetDeepestLastUnignoredChild() const;
  AXNode* GetNextUnignoredSibling() const;
  AXNode* GetPreviousUnignoredSibling() const;
  AXNode* GetNextUnignoredInTreeOrder() const;
  AXNode* GetPreviousUnignoredInTreeOrder() const;

  using UnignoredChildIterator =
      ChildIteratorBase<AXNode,
                        &AXNode::GetNextUnignoredSibling,
                        &AXNode::GetPreviousUnignoredSibling,
                        &AXNode::GetFirstUnignoredChild,
                        &AXNode::GetLastUnignoredChild>;
  UnignoredChildIterator UnignoredChildrenBegin() const;
  UnignoredChildIterator UnignoredChildrenEnd() const;

  // Walking the tree including both ignored and unignored nodes.
  // These methods consider only the direct children or siblings of a node.
  AXNode* GetFirstChild() const;
  AXNode* GetLastChild() const;
  AXNode* GetPreviousSibling() const;
  AXNode* GetNextSibling() const;

  // Returns true if the node has any of the text related roles, including
  // kStaticText, kInlineTextBox and kListMarker (for Legacy Layout). Does not
  // include any text field roles.
  bool IsText() const;

  // Returns true if the node has any line break related roles or is the child
  // of a node with line break related roles.
  bool IsLineBreak() const;

  // Set the node's accessibility data. This may be done during initialization
  // or later when the node data changes.
  void SetData(const AXNodeData& src);

  // Update this node's location. This is separate from |SetData| just because
  // changing only the location is common and should be more efficient than
  // re-copying all of the data.
  //
  // The node's location is stored as a relative bounding box, the ID of
  // the element it's relative to, and an optional transformation matrix.
  // See ax_node_data.h for details.
  void SetLocation(int32_t offset_container_id,
                   const gfx::RectF& location,
                   gfx::Transform* transform);

  // Set the index in parent, for example if siblings were inserted or deleted.
  void SetIndexInParent(size_t index_in_parent);

  // Update the unignored index in parent for unignored children.
  void UpdateUnignoredCachedValues();

  // Swap the internal children vector with |children|. This instance
  // now owns all of the passed children.
  void SwapChildren(std::vector<AXNode*>* children);

  // This is called when the AXTree no longer includes this node in the
  // tree. Reference counting is used on some platforms because the
  // operating system may hold onto a reference to an AXNode
  // object even after we're through with it, so this may decrement the
  // reference count and clear out the object's data.
  void Destroy();

  // Return true if this object is equal to or a descendant of |ancestor|.
  bool IsDescendantOf(const AXNode* ancestor) const;

  bool IsDescendantOfCrossingTreeBoundary(const AXNode* ancestor) const;
  AXNode* GetParentCrossingTreeBoundary() const;

  // Gets the text offsets where new lines start either from the node's data or
  // by computing them and caching the result.
  std::vector<int> GetOrComputeLineStartOffsets();

  // Accessing accessibility attributes.
  // See |AXNodeData| for more information.

  bool HasBoolAttribute(ax::mojom::BoolAttribute attribute) const {
    return data().HasBoolAttribute(attribute);
  }
  bool GetBoolAttribute(ax::mojom::BoolAttribute attribute) const {
    return data().GetBoolAttribute(attribute);
  }
  bool GetBoolAttribute(ax::mojom::BoolAttribute attribute, bool* value) const {
    return data().GetBoolAttribute(attribute, value);
  }

  bool HasFloatAttribute(ax::mojom::FloatAttribute attribute) const {
    return data().HasFloatAttribute(attribute);
  }
  float GetFloatAttribute(ax::mojom::FloatAttribute attribute) const {
    return data().GetFloatAttribute(attribute);
  }
  bool GetFloatAttribute(ax::mojom::FloatAttribute attribute,
                         float* value) const {
    return data().GetFloatAttribute(attribute, value);
  }

  bool HasIntAttribute(ax::mojom::IntAttribute attribute) const {
    return data().HasIntAttribute(attribute);
  }
  int GetIntAttribute(ax::mojom::IntAttribute attribute) const {
    return data().GetIntAttribute(attribute);
  }
  bool GetIntAttribute(ax::mojom::IntAttribute attribute, int* value) const {
    return data().GetIntAttribute(attribute, value);
  }

  bool HasStringAttribute(ax::mojom::StringAttribute attribute) const {
    return data().HasStringAttribute(attribute);
  }
  const std::string& GetStringAttribute(
      ax::mojom::StringAttribute attribute) const {
    return data().GetStringAttribute(attribute);
  }
  bool GetStringAttribute(ax::mojom::StringAttribute attribute,
                          std::string* value) const {
    return data().GetStringAttribute(attribute, value);
  }

  bool GetString16Attribute(ax::mojom::StringAttribute attribute,
                            std::u16string* value) const {
    return data().GetString16Attribute(attribute, value);
  }
  std::u16string GetString16Attribute(
      ax::mojom::StringAttribute attribute) const {
    return data().GetString16Attribute(attribute);
  }

  bool HasIntListAttribute(ax::mojom::IntListAttribute attribute) const {
    return data().HasIntListAttribute(attribute);
  }
  const std::vector<int32_t>& GetIntListAttribute(
      ax::mojom::IntListAttribute attribute) const {
    return data().GetIntListAttribute(attribute);
  }
  bool GetIntListAttribute(ax::mojom::IntListAttribute attribute,
                           std::vector<int32_t>* value) const {
    return data().GetIntListAttribute(attribute, value);
  }

  bool HasStringListAttribute(ax::mojom::StringListAttribute attribute) const {
    return data().HasStringListAttribute(attribute);
  }
  const std::vector<std::string>& GetStringListAttribute(
      ax::mojom::StringListAttribute attribute) const {
    return data().GetStringListAttribute(attribute);
  }
  bool GetStringListAttribute(ax::mojom::StringListAttribute attribute,
                              std::vector<std::string>* value) const {
    return data().GetStringListAttribute(attribute, value);
  }

  bool GetHtmlAttribute(const char* attribute, std::u16string* value) const {
    return data().GetHtmlAttribute(attribute, value);
  }
  bool GetHtmlAttribute(const char* attribute, std::string* value) const {
    return data().GetHtmlAttribute(attribute, value);
  }

  // Return the hierarchical level if supported.
  std::optional<int> GetHierarchicalLevel() const;

  // PosInSet and SetSize public methods.
  bool IsOrderedSetItem() const;
  bool IsOrderedSet() const;
  std::optional<int> GetPosInSet();
  std::optional<int> GetSetSize();

  // Helpers for GetPosInSet and GetSetSize.
  // Returns true if the role of ordered set matches the role of item.
  // Returns false otherwise.
  bool SetRoleMatchesItemRole(const AXNode* ordered_set) const;

  // Container objects that should be ignored for computing PosInSet and SetSize
  // for ordered sets.
  bool IsIgnoredContainerForOrderedSet() const;

  const std::string& GetInheritedStringAttribute(
      ax::mojom::StringAttribute attribute) const;
  std::u16string GetInheritedString16Attribute(
      ax::mojom::StringAttribute attribute) const;
  // Returns the text of this node and all descendant nodes; including text
  // found in embedded objects.
  //
  // Only text displayed on screen is included. Text from ARIA and HTML
  // attributes that is either not displayed on screen, or outside this node, is
  // not returned.
  std::string GetInnerText() const;

  // Return a string representing the language code.
  //
  // This will consider the language declared in the DOM, and may eventually
  // attempt to automatically detect the language from the text.
  //
  // This language code will be BCP 47.
  //
  // Returns empty string if no appropriate language was found.
  std::string GetLanguage() const;

  // Helper functions for tables, table rows, and table cells.
  // Most of these functions construct and cache an AXTableInfo behind
  // the scenes to infer many properties of tables.
  //
  // These interfaces use attributes provided by the source of the
  // AX tree where possible, but fills in missing details and ignores
  // specified attributes when they're bad or inconsistent. That way
  // you're guaranteed to get a valid, consistent table when using these
  // interfaces.
  //

  // Table-like nodes (including grids). All indices are 0-based except
  // ARIA indices are all 1-based. In other words, the top-left corner
  // of the table is row 0, column 0, cell index 0 - but that same cell
  // has a minimum ARIA row index of 1 and column index of 1.
  //
  // The below methods return std::nullopt if the AXNode they are called on is
  // not inside a table.
  bool IsTable() const;
  std::optional<int> GetTableColCount() const;
  std::optional<int> GetTableRowCount() const;
  std::optional<int> GetTableAriaColCount() const;
  std::optional<int> GetTableAriaRowCount() const;
  std::optional<int> GetTableCellCount() const;
  std::optional<bool> GetTableHasColumnOrRowHeaderNode() const;
  AXNode* GetTableCaption() const;
  AXNode* GetTableCellFromIndex(int index) const;
  AXNode* GetTableCellFromCoords(int row_index, int col_index) const;
  // Get all the column header node ids of the table.
  std::vector<AXNode::AXID> GetTableColHeaderNodeIds() const;
  // Get the column header node ids associated with |col_index|.
  std::vector<AXNode::AXID> GetTableColHeaderNodeIds(int col_index) const;
  // Get the row header node ids associated with |row_index|.
  std::vector<AXNode::AXID> GetTableRowHeaderNodeIds(int row_index) const;
  std::vector<AXNode::AXID> GetTableUniqueCellIds() const;
  // Extra computed nodes for the accessibility tree for macOS:
  // one column node for each table column, followed by one
  // table header container node, or nullptr if not applicable.
  const std::vector<AXNode*>* GetExtraMacNodes() const;

  // Table row-like nodes.
  bool IsTableRow() const;
  std::optional<int> GetTableRowRowIndex() const;
  // Get the node ids that represent rows in a table.
  std::vector<AXNode::AXID> GetTableRowNodeIds() const;

#if defined(OS_APPLE)
  // Table column-like nodes. These nodes are only present on macOS.
  bool IsTableColumn() const;
  std::optional<int> GetTableColColIndex() const;
#endif  // defined(OS_APPLE)

  // Table cell-like nodes.
  bool IsTableCellOrHeader() const;
  std::optional<int> GetTableCellIndex() const;
  std::optional<int> GetTableCellColIndex() const;
  std::optional<int> GetTableCellRowIndex() const;
  std::optional<int> GetTableCellColSpan() const;
  std::optional<int> GetTableCellRowSpan() const;
  std::optional<int> GetTableCellAriaColIndex() const;
  std::optional<int> GetTableCellAriaRowIndex() const;
  std::vector<AXNode::AXID> GetTableCellColHeaderNodeIds() const;
  std::vector<AXNode::AXID> GetTableCellRowHeaderNodeIds() const;
  void GetTableCellColHeaders(std::vector<AXNode*>* col_headers) const;
  void GetTableCellRowHeaders(std::vector<AXNode*>* row_headers) const;

  // Helper methods to check if a cell is an ARIA-1.1+ 'cell' or 'gridcell'
  bool IsCellOrHeaderOfARIATable() const;
  bool IsCellOrHeaderOfARIAGrid() const;

  // Returns true if node is a group and is a direct descendant of a set-like
  // element.
  bool IsEmbeddedGroup() const;

  // Returns true if node has ignored state or ignored role.
  bool IsIgnored() const;

  // Returns true if an ancestor of this node (not including itself) is a
  // leaf node, meaning that this node is not actually exposed to any
  // platform's accessibility layer.
  bool IsChildOfLeaf() const;

  // Returns true if this is a leaf node, meaning all its
  // children should not be exposed to any platform's native accessibility
  // layer.
  //
  // The definition of a leaf includes nodes with children that are exclusively
  // an internal renderer implementation, such as the children of an HTML native
  // text field, as well as nodes with presentational children according to the
  // ARIA and HTML5 Specs. Also returns true if all of the node's descendants
  // are ignored.
  //
  // A leaf node should never have children that are focusable or
  // that might send notifications.
  bool IsLeaf() const;

  // Returns true if this node is a list marker or if it's a descendant
  // of a list marker node. Returns false otherwise.
  bool IsInListMarker() const;

  // Returns true if this node is a collapsed popup button that is parent to a
  // menu list popup.
  bool IsCollapsedMenuListPopUpButton() const;

  // Returns the popup button ancestor of this current node if any. The popup
  // button needs to be the parent of a menu list popup and needs to be
  // collapsed.
  AXNode* GetCollapsedMenuListPopUpButtonAncestor() const;

  // Returns the text field ancestor of this current node if any.
  AXNode* GetTextFieldAncestor() const;

  // Finds and returns a pointer to ordered set containing node.
  AXNode* GetOrderedSet() const;

  // If this node is exposed to the platform's accessibility layer, returns this
  // node. Otherwise, returns the lowest ancestor that is exposed to the
  // platform. (See `IsLeaf` and `IsIgnored` for information on what is
  // exposed to platform APIs.)
  AXNode* GetLowestPlatformAncestor() const;

 private:
  // Computes the text offset where each line starts by traversing all child
  // leaf nodes.
  void ComputeLineStartOffsets(std::vector<int>* line_offsets,
                               int* start_offset) const;
  AXTableInfo* GetAncestorTableInfo() const;
  void IdVectorToNodeVector(const std::vector<int32_t>& ids,
                            std::vector<AXNode*>* nodes) const;

  int UpdateUnignoredCachedValuesRecursive(int startIndex);
  AXNode* ComputeLastUnignoredChildRecursive() const;
  AXNode* ComputeFirstUnignoredChildRecursive() const;

  OwnerTree* const tree_;  // Owns this.
  size_t index_in_parent_;
  size_t unignored_index_in_parent_;
  size_t unignored_child_count_ = 0;
  AXNode* const parent_;
  std::vector<AXNode*> children_;
  AXNodeData data_;
};

AX_EXPORT std::ostream& operator<<(std::ostream& stream, const AXNode& node);

template <typename NodeType,
          NodeType* (NodeType::*NextSibling)() const,
          NodeType* (NodeType::*PreviousSibling)() const,
          NodeType* (NodeType::*FirstChild)() const,
          NodeType* (NodeType::*LastChild)() const>
AXNode::ChildIteratorBase<NodeType,
                          NextSibling,
                          PreviousSibling,
                          FirstChild,
                          LastChild>::ChildIteratorBase(const NodeType* parent,
                                                        NodeType* child)
    : parent_(parent), child_(child) {}

template <typename NodeType,
          NodeType* (NodeType::*NextSibling)() const,
          NodeType* (NodeType::*PreviousSibling)() const,
          NodeType* (NodeType::*FirstChild)() const,
          NodeType* (NodeType::*LastChild)() const>
AXNode::ChildIteratorBase<NodeType,
                          NextSibling,
                          PreviousSibling,
                          FirstChild,
                          LastChild>::ChildIteratorBase(const ChildIteratorBase&
                                                            it)
    : parent_(it.parent_), child_(it.child_) {}

template <typename NodeType,
          NodeType* (NodeType::*NextSibling)() const,
          NodeType* (NodeType::*PreviousSibling)() const,
          NodeType* (NodeType::*FirstChild)() const,
          NodeType* (NodeType::*LastChild)() const>
bool AXNode::ChildIteratorBase<NodeType,
                               NextSibling,
                               PreviousSibling,
                               FirstChild,
                               LastChild>::operator==(const ChildIteratorBase&
                                                          rhs) const {
  return parent_ == rhs.parent_ && child_ == rhs.child_;
}

template <typename NodeType,
          NodeType* (NodeType::*NextSibling)() const,
          NodeType* (NodeType::*PreviousSibling)() const,
          NodeType* (NodeType::*FirstChild)() const,
          NodeType* (NodeType::*LastChild)() const>
bool AXNode::ChildIteratorBase<NodeType,
                               NextSibling,
                               PreviousSibling,
                               FirstChild,
                               LastChild>::operator!=(const ChildIteratorBase&
                                                          rhs) const {
  return parent_ != rhs.parent_ || child_ != rhs.child_;
}

template <typename NodeType,
          NodeType* (NodeType::*NextSibling)() const,
          NodeType* (NodeType::*PreviousSibling)() const,
          NodeType* (NodeType::*FirstChild)() const,
          NodeType* (NodeType::*LastChild)() const>
AXNode::ChildIteratorBase<NodeType,
                          NextSibling,
                          PreviousSibling,
                          FirstChild,
                          LastChild>&
AXNode::ChildIteratorBase<NodeType,
                          NextSibling,
                          PreviousSibling,
                          FirstChild,
                          LastChild>::operator++() {
  // |child_ = nullptr| denotes the iterator's past-the-end condition. When we
  // increment the iterator past the end, we remain at the past-the-end iterator
  // condition.
  if (child_ && parent_) {
    if (child_ == (parent_->*LastChild)())
      child_ = nullptr;
    else
      child_ = (child_->*NextSibling)();
  }

  return *this;
}

template <typename NodeType,
          NodeType* (NodeType::*NextSibling)() const,
          NodeType* (NodeType::*PreviousSibling)() const,
          NodeType* (NodeType::*FirstChild)() const,
          NodeType* (NodeType::*LastChild)() const>
AXNode::ChildIteratorBase<NodeType,
                          NextSibling,
                          PreviousSibling,
                          FirstChild,
                          LastChild>&
AXNode::ChildIteratorBase<NodeType,
                          NextSibling,
                          PreviousSibling,
                          FirstChild,
                          LastChild>::operator--() {
  if (parent_) {
    // If the iterator is past the end, |child_=nullptr|, decrement the iterator
    // gives us the last iterator element.
    if (!child_)
      child_ = (parent_->*LastChild)();
    // Decrement the iterator gives us the previous element, except when the
    // iterator is at the beginning; in which case, decrementing the iterator
    // remains at the beginning.
    else if (child_ != (parent_->*FirstChild)())
      child_ = (child_->*PreviousSibling)();
  }

  return *this;
}

template <typename NodeType,
          NodeType* (NodeType::*NextSibling)() const,
          NodeType* (NodeType::*PreviousSibling)() const,
          NodeType* (NodeType::*FirstChild)() const,
          NodeType* (NodeType::*LastChild)() const>
NodeType* AXNode::ChildIteratorBase<NodeType,
                                    NextSibling,
                                    PreviousSibling,
                                    FirstChild,
                                    LastChild>::get() const {
  BASE_DCHECK(child_);
  return child_;
}

template <typename NodeType,
          NodeType* (NodeType::*NextSibling)() const,
          NodeType* (NodeType::*PreviousSibling)() const,
          NodeType* (NodeType::*FirstChild)() const,
          NodeType* (NodeType::*LastChild)() const>
NodeType& AXNode::ChildIteratorBase<NodeType,
                                    NextSibling,
                                    PreviousSibling,
                                    FirstChild,
                                    LastChild>::operator*() const {
  BASE_DCHECK(child_);
  return *child_;
}

template <typename NodeType,
          NodeType* (NodeType::*NextSibling)() const,
          NodeType* (NodeType::*PreviousSibling)() const,
          NodeType* (NodeType::*FirstChild)() const,
          NodeType* (NodeType::*LastChild)() const>
NodeType* AXNode::ChildIteratorBase<NodeType,
                                    NextSibling,
                                    PreviousSibling,
                                    FirstChild,
                                    LastChild>::operator->() const {
  BASE_DCHECK(child_);
  return child_;
}

}  // namespace ui

#endif  // UI_ACCESSIBILITY_AX_NODE_H_
