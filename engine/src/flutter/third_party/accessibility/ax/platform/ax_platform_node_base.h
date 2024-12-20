// Copyright 2014 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef UI_ACCESSIBILITY_PLATFORM_AX_PLATFORM_NODE_BASE_H_
#define UI_ACCESSIBILITY_PLATFORM_AX_PLATFORM_NODE_BASE_H_

#include <map>
#include <string>
#include <vector>

#include "ax/ax_enums.h"
#include "ax/ax_node.h"
#include "ax_build/build_config.h"
#include "ax_platform_node.h"
#include "ax_platform_node_delegate.h"
#include "base/macros.h"
#include "gfx/geometry/point.h"
#include "gfx/native_widget_types.h"

namespace ui {

struct AXNodeData;

struct AX_EXPORT AXHypertext {
  AXHypertext();
  ~AXHypertext();
  AXHypertext(const AXHypertext& other);
  AXHypertext& operator=(const AXHypertext& other);

  // A flag that should be set if the hypertext information in this struct is
  // out-of-date and needs to be updated. This flag should always be set upon
  // construction because constructing this struct doesn't compute the
  // hypertext.
  bool needs_update = true;

  // Maps an embedded character offset in |hypertext| to an index in
  // |hyperlinks|.
  std::map<int32_t, int32_t> hyperlink_offset_to_index;

  // The unique id of a AXPlatformNodes for each hyperlink.
  // TODO(nektar): Replace object IDs with child indices if we decide that
  // we are not implementing IA2 hyperlinks for anything other than IA2
  // Hypertext.
  std::vector<int32_t> hyperlinks;

  std::u16string hypertext;
};

class AX_EXPORT AXPlatformNodeBase : public AXPlatformNode {
 public:
  AXPlatformNodeBase();
  ~AXPlatformNodeBase() override;

  virtual void Init(AXPlatformNodeDelegate* delegate);

  // These are simple wrappers to our delegate.
  const AXNodeData& GetData() const;
  gfx::NativeViewAccessible GetFocus();
  gfx::NativeViewAccessible GetParent() const;
  int GetChildCount() const;
  gfx::NativeViewAccessible ChildAtIndex(int index) const;

  std::string GetName() const;
  std::u16string GetNameAsString16() const;

  // This returns nullopt if there's no parent, it's unable to find the child in
  // the list of its parent's children, or its parent doesn't have children.
  virtual std::optional<int> GetIndexInParent();

  // Returns a stack of ancestors of this node. The node at the top of the stack
  // is the top most ancestor.
  std::stack<gfx::NativeViewAccessible> GetAncestors();

  // Returns an optional integer indicating the logical order of this node
  // compared to another node or returns an empty optional if the nodes
  // are not comparable.
  //    0: if this position is logically equivalent to the other node
  //   <0: if this position is logically less than (before) the other node
  //   >0: if this position is logically greater than (after) the other node
  std::optional<int> CompareTo(AXPlatformNodeBase& other);

  // AXPlatformNode.
  void Destroy() override;
  gfx::NativeViewAccessible GetNativeViewAccessible() override;
  void NotifyAccessibilityEvent(ax::mojom::Event event_type) override;

#if defined(OS_APPLE)
  void AnnounceText(const std::u16string& text) override;
#endif

  AXPlatformNodeDelegate* GetDelegate() const override;
  bool IsDescendantOf(AXPlatformNode* ancestor) const override;

  // Helpers.
  AXPlatformNodeBase* GetPreviousSibling() const;
  AXPlatformNodeBase* GetNextSibling() const;
  AXPlatformNodeBase* GetFirstChild() const;
  AXPlatformNodeBase* GetLastChild() const;
  bool IsDescendant(AXPlatformNodeBase* descendant);

  using AXPlatformNodeChildIterator =
      ui::AXNode::ChildIteratorBase<AXPlatformNodeBase,
                                    &AXPlatformNodeBase::GetNextSibling,
                                    &AXPlatformNodeBase::GetPreviousSibling,
                                    &AXPlatformNodeBase::GetFirstChild,
                                    &AXPlatformNodeBase::GetLastChild>;
  AXPlatformNodeChildIterator AXPlatformNodeChildrenBegin() const;
  AXPlatformNodeChildIterator AXPlatformNodeChildrenEnd() const;

  bool HasBoolAttribute(ax::mojom::BoolAttribute attr) const;
  bool GetBoolAttribute(ax::mojom::BoolAttribute attr) const;
  bool GetBoolAttribute(ax::mojom::BoolAttribute attr, bool* value) const;

  bool HasFloatAttribute(ax::mojom::FloatAttribute attr) const;
  float GetFloatAttribute(ax::mojom::FloatAttribute attr) const;
  bool GetFloatAttribute(ax::mojom::FloatAttribute attr, float* value) const;

  bool HasIntAttribute(ax::mojom::IntAttribute attribute) const;
  int GetIntAttribute(ax::mojom::IntAttribute attribute) const;
  bool GetIntAttribute(ax::mojom::IntAttribute attribute, int* value) const;

  bool HasStringAttribute(ax::mojom::StringAttribute attribute) const;
  const std::string& GetStringAttribute(
      ax::mojom::StringAttribute attribute) const;
  bool GetStringAttribute(ax::mojom::StringAttribute attribute,
                          std::string* value) const;
  bool GetString16Attribute(ax::mojom::StringAttribute attribute,
                            std::u16string* value) const;
  std::u16string GetString16Attribute(
      ax::mojom::StringAttribute attribute) const;
  bool HasInheritedStringAttribute(ax::mojom::StringAttribute attribute) const;
  const std::string& GetInheritedStringAttribute(
      ax::mojom::StringAttribute attribute) const;
  std::u16string GetInheritedString16Attribute(
      ax::mojom::StringAttribute attribute) const;
  bool GetInheritedStringAttribute(ax::mojom::StringAttribute attribute,
                                   std::string* value) const;
  bool GetInheritedString16Attribute(ax::mojom::StringAttribute attribute,
                                     std::u16string* value) const;

  bool HasIntListAttribute(ax::mojom::IntListAttribute attribute) const;
  const std::vector<int32_t>& GetIntListAttribute(
      ax::mojom::IntListAttribute attribute) const;

  bool GetIntListAttribute(ax::mojom::IntListAttribute attribute,
                           std::vector<int32_t>* value) const;

  // Returns the selection container if inside one.
  AXPlatformNodeBase* GetSelectionContainer() const;

  // Returns the table or ARIA grid if inside one.
  AXPlatformNodeBase* GetTable() const;

  // If inside an HTML or ARIA table, returns the object containing the caption.
  // Returns nullptr if not inside a table, or if there is no
  // caption.
  AXPlatformNodeBase* GetTableCaption() const;

  // If inside a table or ARIA grid, returns the cell found at the given index.
  // Indices are in row major order and each cell is counted once regardless of
  // its span. Returns nullptr if the cell is not found or if not inside a
  // table.
  AXPlatformNodeBase* GetTableCell(int index) const;

  // If inside a table or ARIA grid, returns the cell at the given row and
  // column (0-based). Works correctly with cells that span multiple rows or
  // columns. Returns nullptr if the cell is not found or if not inside a
  // table.
  AXPlatformNodeBase* GetTableCell(int row, int column) const;

  // If inside a table or ARIA grid, returns the zero-based index of the cell.
  // Indices are in row major order and each cell is counted once regardless of
  // its span. Returns std::nullopt if not a cell or if not inside a table.
  std::optional<int> GetTableCellIndex() const;

  // If inside a table or ARIA grid, returns the physical column number for the
  // current cell. In contrast to logical columns, physical columns always start
  // from 0 and have no gaps in their numbering. Logical columns can be set
  // using aria-colindex. Returns std::nullopt if not a cell or if not inside a
  // table.
  std::optional<int> GetTableColumn() const;

  // If inside a table or ARIA grid, returns the number of physical columns.
  // Returns std::nullopt if not inside a table.
  std::optional<int> GetTableColumnCount() const;

  // If inside a table or ARIA grid, returns the number of ARIA columns.
  // Returns std::nullopt if not inside a table.
  std::optional<int> GetTableAriaColumnCount() const;

  // If inside a table or ARIA grid, returns the number of physical columns that
  // this cell spans. Returns std::nullopt if not a cell or if not inside a
  // table.
  std::optional<int> GetTableColumnSpan() const;

  // If inside a table or ARIA grid, returns the physical row number for the
  // current cell. In contrast to logical rows, physical rows always start from
  // 0 and have no gaps in their numbering. Logical rows can be set using
  // aria-rowindex. Returns std::nullopt if not a cell or if not inside a
  // table.
  std::optional<int> GetTableRow() const;

  // If inside a table or ARIA grid, returns the number of physical rows.
  // Returns std::nullopt if not inside a table.
  std::optional<int> GetTableRowCount() const;

  // If inside a table or ARIA grid, returns the number of ARIA rows.
  // Returns std::nullopt if not inside a table.
  std::optional<int> GetTableAriaRowCount() const;

  // If inside a table or ARIA grid, returns the number of physical rows that
  // this cell spans. Returns std::nullopt if not a cell or if not inside a
  // table.
  std::optional<int> GetTableRowSpan() const;

  // Returns the font size converted to points, if available.
  std::optional<float> GetFontSizeInPoints() const;

  // Returns true if either a descendant has selection (sel_focus_object_id) or
  // if this node is a simple text element and has text selection attributes.
  // Optionally accepts an unignored selection to avoid redundant computation.
  bool HasCaret(const AXTree::Selection* unignored_selection = nullptr);

  // See AXPlatformNodeDelegate::IsChildOfLeaf().
  bool IsChildOfLeaf() const;

  // See AXPlatformNodeDelegate::IsLeaf().
  bool IsLeaf() const;

  // See AXPlatformNodeDelegate::IsInvisibleOrIgnored().
  bool IsInvisibleOrIgnored() const;

  // Returns true if this node can be scrolled either in the horizontal or the
  // vertical direction.
  bool IsScrollable() const;

  // Returns true if this node can be scrolled in the horizontal direction.
  bool IsHorizontallyScrollable() const;

  // Returns true if this node can be scrolled in the vertical direction.
  bool IsVerticallyScrollable() const;

  // See AXNodeData::IsTextField().
  bool IsTextField() const;

  // See AXNodeData::IsPlainTextField().
  bool IsPlainTextField() const;

  // See AXNodeData::IsRichTextField().
  bool IsRichTextField() const;

  // See AXNode::IsText().
  bool IsText() const;

  // Determines whether an element should be exposed with checkable state, and
  // possibly the checked state. Examples are check box and radio button.
  // Objects that are exposed as toggle buttons use the platform pressed state
  // in some platform APIs, and should not be exposed as checkable. They don't
  // expose the platform equivalent of the internal checked state.
  virtual bool IsPlatformCheckable() const;

  bool HasFocus();

  // If this node is a leaf, returns the visible accessible name of this node.
  // Otherwise represents every non-leaf child node with a special "embedded
  // object character", and every leaf child node with its visible accessible
  // name. This is how displayed text and embedded objects are represented in
  // ATK and IA2 APIs.
  std::u16string GetHypertext() const;

  // Returns the text of this node and all descendant nodes; including text
  // found in embedded objects.
  //
  // Only text displayed on screen is included. Text from ARIA and HTML
  // attributes that is either not displayed on screen, or outside this node,
  // e.g. aria-label and HTML title, is not returned.
  std::u16string GetInnerText() const;

  virtual std::u16string GetValue() const;

  // Represents a non-static text node in IAccessibleHypertext (and ATK in the
  // future). This character is embedded in the response to
  // IAccessibleText::get_text, indicating the position where a non-static text
  // child object appears.
  static const char16_t kEmbeddedCharacter;

  // Get a node given its unique id or null in the case that the id is unknown.
  static AXPlatformNode* GetFromUniqueId(int32_t unique_id);

  // Return the number of instances of AXPlatformNodeBase, for leak testing.
  static size_t GetInstanceCountForTesting();

  static void SetOnNotifyEventCallbackForTesting(
      ax::mojom::Event event_type,
      std::function<void()> callback);

  enum ScrollType {
    TopLeft,
    BottomRight,
    TopEdge,
    BottomEdge,
    LeftEdge,
    RightEdge,
    Anywhere,
  };
  bool ScrollToNode(ScrollType scroll_type);

  // This will return the nearest leaf node to the point, the leaf node will not
  // necessarily be directly under the point. This utilizes
  // AXPlatformNodeDelegate::HitTestSync, which in the case of
  // BrowserAccessibility, may not be accurate after a single call. See
  // BrowserAccessibilityManager::CachingAsyncHitTest
  AXPlatformNodeBase* NearestLeafToPoint(gfx::Point point) const;

  // Return the nearest text index to a point in screen coordinates for an
  // accessibility node. If the node is not a text only node, the implicit
  // nearest index is zero. Note this will only find the index of text on the
  // input node. Due to perf concerns, this should only be called on leaf nodes.
  int NearestTextIndexToPoint(gfx::Point point);

  ui::TextAttributeList ComputeTextAttributes() const;

  // Get the number of items selected. It checks kMultiselectable and
  // kFocusable. and uses GetSelectedItems to get the selected number.
  int GetSelectionCount() const;

  // If this object is a container that supports selectable children, returns
  // the selected item at the provided index.
  AXPlatformNodeBase* GetSelectedItem(int selected_index) const;

  // If this object is a container that supports selectable children,
  // returns the number of selected items in this container.
  // |out_selected_items| could be set to nullptr if the caller just
  // needs to know the number of items selected.
  // |max_items| represents the number that the caller expects as a
  // maximum. For a single selection list box, it will be 1.
  int GetSelectedItems(
      int max_items,
      std::vector<AXPlatformNodeBase*>* out_selected_items = nullptr) const;

  //
  // Delegate.  This is a weak reference which owns |this|.
  //
  AXPlatformNodeDelegate* delegate_ = nullptr;

 protected:
  bool IsDocument() const;

  bool IsSelectionItemSupported() const;

  // Get the range value text, which might come from aria-valuetext or
  // a floating-point value. This is different from the value string
  // attribute used in input controls such as text boxes and combo boxes.
  std::u16string GetRangeValueText() const;

  // Get the role description from the node data or from the image annotation
  // status.
  std::u16string GetRoleDescription() const;
  std::u16string GetRoleDescriptionFromImageAnnotationStatusOrFromAttribute()
      const;

  // Cast a gfx::NativeViewAccessible to an AXPlatformNodeBase if it is one,
  // or return NULL if it's not an instance of this class.
  static AXPlatformNodeBase* FromNativeViewAccessible(
      gfx::NativeViewAccessible accessible);

  virtual void Dispose();

  // Sets the hypertext selection in this object if possible.
  bool SetHypertextSelection(int start_offset, int end_offset);

  using PlatformAttributeList = std::vector<std::u16string>;

  // Compute the attributes exposed via platform accessibility objects and put
  // them into an attribute list, |attributes|. Currently only used by
  // IAccessible2 on Windows and ATK on Aura Linux.
  void ComputeAttributes(PlatformAttributeList* attributes);

  // If the string attribute |attribute| is present, add its value as an
  // IAccessible2 attribute with the name |name|.
  void AddAttributeToList(const ax::mojom::StringAttribute attribute,
                          const char* name,
                          PlatformAttributeList* attributes);

  // If the bool attribute |attribute| is present, add its value as an
  // IAccessible2 attribute with the name |name|.
  void AddAttributeToList(const ax::mojom::BoolAttribute attribute,
                          const char* name,
                          PlatformAttributeList* attributes);

  // If the int attribute |attribute| is present, add its value as an
  // IAccessible2 attribute with the name |name|.
  void AddAttributeToList(const ax::mojom::IntAttribute attribute,
                          const char* name,
                          PlatformAttributeList* attributes);

  // A helper to add the given string value to |attributes|.
  virtual void AddAttributeToList(const char* name,
                                  const std::string& value,
                                  PlatformAttributeList* attributes);

  // A virtual method that subclasses use to actually add the attribute to
  // |attributes|.
  virtual void AddAttributeToList(const char* name,
                                  const char* value,
                                  PlatformAttributeList* attributes);

  // Escapes characters in string attributes as required by the IA2 Spec
  // and AT-SPI2. It's okay for input to be the same as output.
  static void SanitizeStringAttribute(const std::string& input,
                                      std::string* output);

  // Escapes characters in text attribute values as required by the platform.
  // It's okay for input to be the same as output. The default implementation
  // does nothing to the input value.
  virtual void SanitizeTextAttributeValue(const std::string& input,
                                          std::string* output) const;

  // Compute the hypertext for this node to be exposed via IA2 and ATK This
  // method is responsible for properly embedding children using the special
  // embedded element character.
  void UpdateComputedHypertext() const;

  // Selection helper functions.
  // The following functions retrieve the endpoints of the current selection.
  // First they check for a local selection found on the current control, e.g.
  // when querying the selection on a textarea.
  // If not found they retrieve the global selection found on the current frame.
  int GetSelectionAnchor(const AXTree::Selection* selection);
  int GetSelectionFocus(const AXTree::Selection* selection);

  // Retrieves the selection offsets in the way required by the IA2 APIs.
  // selection_start and selection_end are -1 when there is no selection active
  // on this object.
  // The greatest of the two offsets is one past the last character of the
  // selection.)
  void GetSelectionOffsets(int* selection_start, int* selection_end);
  void GetSelectionOffsets(const AXTree::Selection* selection,
                           int* selection_start,
                           int* selection_end);
  void GetSelectionOffsetsFromTree(const AXTree::Selection* selection,
                                   int* selection_start,
                                   int* selection_end);

  // Returns the hyperlink at the given text position, or nullptr if no
  // hyperlink can be found.
  AXPlatformNodeBase* GetHyperlinkFromHypertextOffset(int offset);

  // Functions for retrieving offsets for hyperlinks and hypertext.
  // Return -1 in case of failure.
  int32_t GetHyperlinkIndexFromChild(AXPlatformNodeBase* child);
  int32_t GetHypertextOffsetFromHyperlinkIndex(int32_t hyperlink_index);
  int32_t GetHypertextOffsetFromChild(AXPlatformNodeBase* child);
  int32_t GetHypertextOffsetFromDescendant(AXPlatformNodeBase* descendant);

  // If the selection endpoint is either equal to or an ancestor of this object,
  // returns endpoint_offset.
  // If the selection endpoint is a descendant of this object, returns its
  // offset. Otherwise, returns either 0 or the length of the hypertext
  // depending on the direction of the selection.
  // Returns -1 in case of unexpected failure, e.g. the selection endpoint
  // cannot be found in the accessibility tree.
  int GetHypertextOffsetFromEndpoint(AXPlatformNodeBase* endpoint_object,
                                     int endpoint_offset);

  bool IsSameHypertextCharacter(const AXHypertext& old_hypertext,
                                size_t old_char_index,
                                size_t new_char_index);

  std::optional<int> GetPosInSet() const;
  std::optional<int> GetSetSize() const;

  std::string GetInvalidValue() const;

  // Based on the characteristics of this object, such as its role and the
  // presence of a multiselectable attribute, returns the maximum number of
  // selectable children that this object could potentially contain.
  int GetMaxSelectableItems() const;

  mutable AXHypertext hypertext_;

 private:
  // Returns true if the index represents a text character.
  bool IsText(const std::u16string& text,
              size_t index,
              bool is_indexed_from_end = false);

  // Compute value for object attribute details-roles on aria-details nodes.
  std::string ComputeDetailsRoles() const;

  BASE_DISALLOW_COPY_AND_ASSIGN(AXPlatformNodeBase);
};

}  // namespace ui

#endif  // UI_ACCESSIBILITY_PLATFORM_AX_PLATFORM_NODE_BASE_H_
