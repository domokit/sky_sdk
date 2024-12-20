// Copyright 2013 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef UI_ACCESSIBILITY_AX_NODE_DATA_H_
#define UI_ACCESSIBILITY_AX_NODE_DATA_H_

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "ax_base_export.h"
#include "ax_enums.h"
#include "ax_node_text_styles.h"
#include "ax_relative_bounds.h"

namespace ui {

// Return true if |attr| should be interpreted as the id of another node
// in the same tree.
AX_BASE_EXPORT bool IsNodeIdIntAttribute(ax::mojom::IntAttribute attr);

// Return true if |attr| should be interpreted as a list of ids of
// nodes in the same tree.
AX_BASE_EXPORT bool IsNodeIdIntListAttribute(ax::mojom::IntListAttribute attr);

// A compact representation of the accessibility information for a
// single accessible object, in a form that can be serialized and sent from
// one process to another.
struct AX_BASE_EXPORT AXNodeData {
  // Defines the type used for AXNode IDs.
  using AXID = int32_t;

  AXNodeData();
  virtual ~AXNodeData();

  AXNodeData(const AXNodeData& other);
  AXNodeData(AXNodeData&& other);
  AXNodeData& operator=(AXNodeData other);

  // Accessing accessibility attributes:
  //
  // There are dozens of possible attributes for an accessibility node,
  // but only a few tend to apply to any one object, so we store them
  // in sparse arrays of <attribute id, attribute value> pairs, organized
  // by type (bool, int, float, string, int list).
  //
  // There are three accessors for each type of attribute: one that returns
  // true if the attribute is present and false if not, one that takes a
  // pointer argument and returns true if the attribute is present (if you
  // need to distinguish between the default value and a missing attribute),
  // and another that returns the default value for that type if the
  // attribute is not present. In addition, strings can be returned as
  // either std::string or std::u16string, for convenience.

  bool HasBoolAttribute(ax::mojom::BoolAttribute attribute) const;
  bool GetBoolAttribute(ax::mojom::BoolAttribute attribute) const;
  bool GetBoolAttribute(ax::mojom::BoolAttribute attribute, bool* value) const;

  bool HasFloatAttribute(ax::mojom::FloatAttribute attribute) const;
  float GetFloatAttribute(ax::mojom::FloatAttribute attribute) const;
  bool GetFloatAttribute(ax::mojom::FloatAttribute attribute,
                         float* value) const;

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

  bool HasIntListAttribute(ax::mojom::IntListAttribute attribute) const;
  const std::vector<int32_t>& GetIntListAttribute(
      ax::mojom::IntListAttribute attribute) const;
  bool GetIntListAttribute(ax::mojom::IntListAttribute attribute,
                           std::vector<int32_t>* value) const;

  bool HasStringListAttribute(ax::mojom::StringListAttribute attribute) const;
  const std::vector<std::string>& GetStringListAttribute(
      ax::mojom::StringListAttribute attribute) const;
  bool GetStringListAttribute(ax::mojom::StringListAttribute attribute,
                              std::vector<std::string>* value) const;
  bool GetHtmlAttribute(const char* attribute, std::u16string* value) const;
  bool GetHtmlAttribute(const char* attribute, std::string* value) const;

  //
  // Setting accessibility attributes.
  //
  // Replaces an attribute if present. This is safer than crashing via a
  // BASE_DCHECK or doing nothing, because most likely replacing is what the
  // caller would have wanted or what existing code already assumes.
  //

  void AddStringAttribute(ax::mojom::StringAttribute attribute,
                          const std::string& value);
  void AddIntAttribute(ax::mojom::IntAttribute attribute, int32_t value);
  void AddFloatAttribute(ax::mojom::FloatAttribute attribute, float value);
  void AddBoolAttribute(ax::mojom::BoolAttribute attribute, bool value);
  void AddIntListAttribute(ax::mojom::IntListAttribute attribute,
                           const std::vector<int32_t>& value);
  void AddStringListAttribute(ax::mojom::StringListAttribute attribute,
                              const std::vector<std::string>& value);

  //
  // Removing accessibility attributes.
  //

  void RemoveStringAttribute(ax::mojom::StringAttribute attribute);
  void RemoveIntAttribute(ax::mojom::IntAttribute attribute);
  void RemoveFloatAttribute(ax::mojom::FloatAttribute attribute);
  void RemoveBoolAttribute(ax::mojom::BoolAttribute attribute);
  void RemoveIntListAttribute(ax::mojom::IntListAttribute attribute);
  void RemoveStringListAttribute(ax::mojom::StringListAttribute attribute);

  //
  // Text styles.
  //
  AXNodeTextStyles GetTextStyles() const;

  //
  // Convenience functions.
  //

  // Adds the name attribute or replaces it if already present. Also sets the
  // NameFrom attribute if not already set.
  void SetName(const std::string& name);
  void SetName(const std::u16string& name);

  // Allows nameless objects to pass accessibility checks.
  void SetNameExplicitlyEmpty();

  // Adds the description attribute or replaces it if already present.
  void SetDescription(const std::string& description);
  void SetDescription(const std::u16string& description);

  // Adds the value attribute or replaces it if already present.
  void SetValue(const std::string& value);
  void SetValue(const std::u16string& value);

  // Adds the tooltip attribute or replaces it if already present.
  void SetTooltip(const std::string& value);
  void SetTooltip(const std::u16string& value);

  // Returns true if the given enum bit is 1.
  bool HasState(ax::mojom::State state) const;
  bool HasAction(ax::mojom::Action action) const;
  bool HasTextStyle(ax::mojom::TextStyle text_style) const;
  // aria-dropeffect is deprecated in WAI-ARIA 1.1.
  bool HasDropeffect(ax::mojom::Dropeffect dropeffect) const;

  // Set or remove bits in the given enum's corresponding bitfield.
  void AddState(ax::mojom::State state);
  void RemoveState(ax::mojom::State state);
  void AddAction(ax::mojom::Action action);
  void AddTextStyle(ax::mojom::TextStyle text_style);
  // aria-dropeffect is deprecated in WAI-ARIA 1.1.
  void AddDropeffect(ax::mojom::Dropeffect dropeffect);

  // Helper functions to get or set some common int attributes with some
  // specific enum types. To remove an attribute, set it to None.
  //
  // Please keep in alphabetic order.
  ax::mojom::CheckedState GetCheckedState() const;
  void SetCheckedState(ax::mojom::CheckedState checked_state);
  bool HasCheckedState() const;
  ax::mojom::DefaultActionVerb GetDefaultActionVerb() const;
  void SetDefaultActionVerb(ax::mojom::DefaultActionVerb default_action_verb);
  ax::mojom::HasPopup GetHasPopup() const;
  void SetHasPopup(ax::mojom::HasPopup has_popup);
  ax::mojom::InvalidState GetInvalidState() const;
  void SetInvalidState(ax::mojom::InvalidState invalid_state);
  ax::mojom::NameFrom GetNameFrom() const;
  void SetNameFrom(ax::mojom::NameFrom name_from);
  ax::mojom::DescriptionFrom GetDescriptionFrom() const;
  void SetDescriptionFrom(ax::mojom::DescriptionFrom description_from);
  ax::mojom::TextPosition GetTextPosition() const;
  void SetTextPosition(ax::mojom::TextPosition text_position);
  ax::mojom::Restriction GetRestriction() const;
  void SetRestriction(ax::mojom::Restriction restriction);
  ax::mojom::ListStyle GetListStyle() const;
  void SetListStyle(ax::mojom::ListStyle list_style);
  ax::mojom::TextAlign GetTextAlign() const;
  void SetTextAlign(ax::mojom::TextAlign text_align);
  ax::mojom::WritingDirection GetTextDirection() const;
  void SetTextDirection(ax::mojom::WritingDirection text_direction);
  ax::mojom::ImageAnnotationStatus GetImageAnnotationStatus() const;
  void SetImageAnnotationStatus(ax::mojom::ImageAnnotationStatus status);

  // Helper to determine if the data belongs to a node that gains focus when
  // clicked, such as a text field or a native HTML list box.
  bool IsActivatable() const;

  // Helper to determine if the data belongs to a node that is a native button
  // or ARIA role="button" in a pressed state.
  bool IsButtonPressed() const;

  // Helper to determine if the data belongs to a node that can respond to
  // clicks.
  bool IsClickable() const;

  // Helper to determine if the object is selectable.
  bool IsSelectable() const;

  // Helper to determine if the data has the ignored state or ignored role.
  bool IsIgnored() const;

  // Helper to determine if the data has the invisible state.
  bool IsInvisible() const;

  // Helper to determine if the data has the ignored state, the invisible state
  // or the ignored role.
  bool IsInvisibleOrIgnored() const;

  // Helper to determine if the data belongs to a node that is invocable.
  bool IsInvocable() const;

  // Helper to determine if the data belongs to a node that is a menu button.
  bool IsMenuButton() const;

  // This data belongs to a text field. This is any widget in which the user
  // should be able to enter and edit text.
  //
  // Examples include <input type="text">, <input type="password">, <textarea>,
  // <div contenteditable="true">, <div role="textbox">, <div role="searchbox">
  // and <div role="combobox">. Note that when an ARIA role that indicates that
  // the widget is editable is used, such as "role=textbox", the element doesn't
  // need to be contenteditable for this method to return true, as in theory
  // JavaScript could be used to implement editing functionality. In practice,
  // this situation should be rare.
  bool IsTextField() const;

  // This data belongs to a text field that is used for entering passwords.
  bool IsPasswordField() const;

  // This data belongs to a text field that doesn't accept rich text content,
  // such as text with special formatting or styling.
  bool IsPlainTextField() const;

  // This data belongs to a text field that accepts rich text content, such as
  // text with special formatting or styling.
  bool IsRichTextField() const;

  // Helper to determine if |GetRestriction| is either ReadOnly or Disabled.
  // By default, all nodes that can't be edited are readonly.
  bool IsReadOnlyOrDisabled() const;

  // Helper to determine if the data belongs to a node that supports
  // range-based value.
  bool IsRangeValueSupported() const;

  // Helper to determine if the data belongs to a node that supports
  // expand/collapse.
  bool SupportsExpandCollapse() const;

  // Return a string representation of this data, for debugging.
  virtual std::string ToString() const;

  // Return a string representation of |aria-dropeffect| values, for testing
  // and debugging.
  // aria-dropeffect is deprecated in WAI-ARIA 1.1.
  std::string DropeffectBitfieldToString() const;

  // As much as possible this should behave as a simple, serializable,
  // copyable struct.
  int32_t id = -1;
  ax::mojom::Role role;
  uint32_t state;
  uint64_t actions;
  std::vector<std::pair<ax::mojom::StringAttribute, std::string>>
      string_attributes;
  std::vector<std::pair<ax::mojom::IntAttribute, int32_t>> int_attributes;
  std::vector<std::pair<ax::mojom::FloatAttribute, float>> float_attributes;
  std::vector<std::pair<ax::mojom::BoolAttribute, bool>> bool_attributes;
  std::vector<std::pair<ax::mojom::IntListAttribute, std::vector<int32_t>>>
      intlist_attributes;
  std::vector<
      std::pair<ax::mojom::StringListAttribute, std::vector<std::string>>>
      stringlist_attributes;
  std::vector<std::pair<std::string, std::string>> html_attributes;
  std::vector<int32_t> child_ids;

  AXRelativeBounds relative_bounds;
};

}  // namespace ui

#endif  // UI_ACCESSIBILITY_AX_NODE_DATA_H_
