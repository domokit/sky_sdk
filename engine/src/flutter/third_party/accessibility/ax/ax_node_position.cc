// Copyright 2016 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "ax_node_position.h"

#include "ax_build/build_config.h"
#include "ax_enums.h"
#include "ax_node_data.h"
#include "ax_tree_manager.h"
#include "ax_tree_manager_map.h"
#include "base/string_utils.h"

namespace ui {

AXEmbeddedObjectBehavior g_ax_embedded_object_behavior =
#if defined(OS_WIN)
    AXEmbeddedObjectBehavior::kExposeCharacter;
#else
    AXEmbeddedObjectBehavior::kSuppressCharacter;
#endif  // defined(OS_WIN)

ScopedAXEmbeddedObjectBehaviorSetter::ScopedAXEmbeddedObjectBehaviorSetter(
    AXEmbeddedObjectBehavior behavior) {
  prev_behavior_ = g_ax_embedded_object_behavior;
  g_ax_embedded_object_behavior = behavior;
}

ScopedAXEmbeddedObjectBehaviorSetter::~ScopedAXEmbeddedObjectBehaviorSetter() {
  g_ax_embedded_object_behavior = prev_behavior_;
}

// static
AXNodePosition::AXPositionInstance AXNodePosition::CreatePosition(
    const AXNode& node,
    int child_index_or_text_offset,
    ax::mojom::TextAffinity affinity) {
  if (!node.tree())
    return CreateNullPosition();

  AXTreeID tree_id = node.tree()->GetAXTreeID();
  if (node.IsText()) {
    return CreateTextPosition(tree_id, node.id(), child_index_or_text_offset,
                              affinity);
  }

  return CreateTreePosition(tree_id, node.id(), child_index_or_text_offset);
}

AXNodePosition::AXNodePosition() = default;

AXNodePosition::~AXNodePosition() = default;

AXNodePosition::AXNodePosition(const AXNodePosition& other)
    : AXPosition<AXNodePosition, AXNode>(other) {}

AXNodePosition::AXPositionInstance AXNodePosition::Clone() const {
  return AXPositionInstance(new AXNodePosition(*this));
}

void AXNodePosition::AnchorChild(int child_index,
                                 AXTreeID* tree_id,
                                 AXNode::AXID* child_id) const {
  BASE_DCHECK(tree_id);
  BASE_DCHECK(child_id);

  if (!GetAnchor() || child_index < 0 || child_index >= AnchorChildCount()) {
    *tree_id = AXTreeIDUnknown();
    *child_id = AXNode::kInvalidAXID;
    return;
  }

  AXNode* child = nullptr;
  const AXTreeManager* child_tree_manager =
      AXTreeManagerMap::GetInstance().GetManagerForChildTree(*GetAnchor());
  if (child_tree_manager) {
    // The child node exists in a separate tree from its parent.
    child = child_tree_manager->GetRootAsAXNode();
    *tree_id = child_tree_manager->GetTreeID();
  } else {
    child = GetAnchor()->children()[static_cast<size_t>(child_index)];
    *tree_id = this->tree_id();
  }

  BASE_DCHECK(child);
  *child_id = child->id();
}

int AXNodePosition::AnchorChildCount() const {
  if (!GetAnchor())
    return 0;

  const AXTreeManager* child_tree_manager =
      AXTreeManagerMap::GetInstance().GetManagerForChildTree(*GetAnchor());
  if (child_tree_manager)
    return 1;

  return static_cast<int>(GetAnchor()->children().size());
}

int AXNodePosition::AnchorUnignoredChildCount() const {
  if (!GetAnchor())
    return 0;

  return static_cast<int>(GetAnchor()->GetUnignoredChildCount());
}

int AXNodePosition::AnchorIndexInParent() const {
  return GetAnchor() ? static_cast<int>(GetAnchor()->index_in_parent())
                     : INVALID_INDEX;
}

int AXNodePosition::AnchorSiblingCount() const {
  AXNode* parent = GetAnchor()->GetUnignoredParent();
  if (parent)
    return static_cast<int>(parent->GetUnignoredChildCount());

  return 0;
}

std::stack<AXNode*> AXNodePosition::GetAncestorAnchors() const {
  std::stack<AXNode*> anchors;
  AXNode* current_anchor = GetAnchor();

  AXNode::AXID current_anchor_id = GetAnchor()->id();
  AXTreeID current_tree_id = tree_id();

  AXNode::AXID parent_anchor_id = AXNode::kInvalidAXID;
  AXTreeID parent_tree_id = AXTreeIDUnknown();

  while (current_anchor) {
    anchors.push(current_anchor);
    current_anchor = GetParent(
        current_anchor /*child*/, current_tree_id /*child_tree_id*/,
        &parent_tree_id /*parent_tree_id*/, &parent_anchor_id /*parent_id*/);

    current_anchor_id = parent_anchor_id;
    current_tree_id = parent_tree_id;
  }
  return anchors;
}

AXNode* AXNodePosition::GetLowestUnignoredAncestor() const {
  if (!GetAnchor())
    return nullptr;

  return GetAnchor()->GetUnignoredParent();
}

void AXNodePosition::AnchorParent(AXTreeID* tree_id,
                                  AXNode::AXID* parent_id) const {
  BASE_DCHECK(tree_id);
  BASE_DCHECK(parent_id);

  *tree_id = AXTreeIDUnknown();
  *parent_id = AXNode::kInvalidAXID;

  if (!GetAnchor())
    return;

  AXNode* parent =
      GetParent(GetAnchor() /*child*/, this->tree_id() /*child_tree_id*/,
                tree_id /*parent_tree_id*/, parent_id /*parent_id*/);

  if (!parent) {
    *tree_id = AXTreeIDUnknown();
    *parent_id = AXNode::kInvalidAXID;
  }
}

AXNode* AXNodePosition::GetNodeInTree(AXTreeID tree_id,
                                      AXNode::AXID node_id) const {
  if (node_id == AXNode::kInvalidAXID)
    return nullptr;

  AXTreeManager* manager = AXTreeManagerMap::GetInstance().GetManager(tree_id);
  if (manager)
    return manager->GetNodeFromTree(tree_id, node_id);
  return nullptr;
}

AXNode::AXID AXNodePosition::GetAnchorID(AXNode* node) const {
  return node->id();
}

AXTreeID AXNodePosition::GetTreeID(AXNode* node) const {
  return node->tree()->GetAXTreeID();
}

std::u16string AXNodePosition::GetText() const {
  if (IsNullPosition())
    return {};

  std::u16string text;
  if (IsEmptyObjectReplacedByCharacter()) {
    text += kEmbeddedCharacter;
    return text;
  }

  const AXNode* anchor = GetAnchor();
  BASE_DCHECK(anchor);
  // TODO(nektar): Replace with PlatformChildCount when AXNodePosition and
  // BrowserAccessibilityPosition are merged into one class.
  if (!AnchorChildCount()) {
    // Special case: Allows us to get text even in non-web content, e.g. in the
    // browser's UI.
    text =
        anchor->data().GetString16Attribute(ax::mojom::StringAttribute::kValue);
    if (!text.empty())
      return text;
  }

  if (anchor->IsText()) {
    return anchor->data().GetString16Attribute(
        ax::mojom::StringAttribute::kName);
  }

  for (int i = 0; i < AnchorChildCount(); ++i) {
    text += CreateChildPositionAt(i)->GetText();
  }

  return text;
}

bool AXNodePosition::IsInLineBreak() const {
  if (IsNullPosition())
    return false;
  BASE_DCHECK(GetAnchor());
  return GetAnchor()->IsLineBreak();
}

bool AXNodePosition::IsInTextObject() const {
  if (IsNullPosition())
    return false;
  BASE_DCHECK(GetAnchor());
  return GetAnchor()->IsText();
}

bool AXNodePosition::IsInWhiteSpace() const {
  if (IsNullPosition())
    return false;
  BASE_DCHECK(GetAnchor());
  return GetAnchor()->IsLineBreak() ||
         base::ContainsOnlyChars(GetText(), base::kWhitespaceUTF16);
}

// This override is an optimized version AXPosition::MaxTextOffset. Instead of
// concatenating the strings in GetText() to then get their text length, we sum
// the lengths of the individual strings. This is faster than concatenating the
// strings first and then taking their length, especially when the process
// is recursive.
int AXNodePosition::MaxTextOffset() const {
  if (IsNullPosition())
    return INVALID_OFFSET;

  if (IsEmptyObjectReplacedByCharacter())
    return 1;

  const AXNode* anchor = GetAnchor();
  BASE_DCHECK(anchor);
  // TODO(nektar): Replace with PlatformChildCount when AXNodePosition and
  // BrowserAccessibilityPosition will make one.
  if (!AnchorChildCount()) {
    std::u16string value =
        anchor->data().GetString16Attribute(ax::mojom::StringAttribute::kValue);
    if (!value.empty())
      return value.length();
  }

  if (anchor->IsText()) {
    return anchor->data()
        .GetString16Attribute(ax::mojom::StringAttribute::kName)
        .length();
  }

  int text_length = 0;
  for (int i = 0; i < AnchorChildCount(); ++i) {
    text_length += CreateChildPositionAt(i)->MaxTextOffset();
  }

  return text_length;
}

bool AXNodePosition::IsEmbeddedObjectInParent() const {
  switch (g_ax_embedded_object_behavior) {
    case AXEmbeddedObjectBehavior::kSuppressCharacter:
      return false;
    case AXEmbeddedObjectBehavior::kExposeCharacter:
      // We don't need to expose an "embedded object character" for textual
      // nodes and nodes that are invisible to platform APIs. Textual nodes are
      // represented by their actual text.
      return !IsNullPosition() && !GetAnchor()->IsText() &&
             GetAnchor()->IsChildOfLeaf();
  }
}

bool AXNodePosition::IsInLineBreakingObject() const {
  if (IsNullPosition())
    return false;
  BASE_DCHECK(GetAnchor());
  return (GetAnchor()->data().GetBoolAttribute(
              ax::mojom::BoolAttribute::kIsLineBreakingObject) &&
          !GetAnchor()->IsInListMarker()) ||
         GetAnchor()->data().role == ax::mojom::Role::kLineBreak;
}

ax::mojom::Role AXNodePosition::GetAnchorRole() const {
  if (IsNullPosition())
    return ax::mojom::Role::kNone;
  BASE_DCHECK(GetAnchor());
  return GetRole(GetAnchor());
}

ax::mojom::Role AXNodePosition::GetRole(AXNode* node) const {
  return node->data().role;
}

AXNodeTextStyles AXNodePosition::GetTextStyles() const {
  // Check either the current anchor or its parent for text styles.
  AXNodeTextStyles current_anchor_text_styles =
      !IsNullPosition() ? GetAnchor()->data().GetTextStyles()
                        : AXNodeTextStyles();
  if (current_anchor_text_styles.IsUnset()) {
    AXPositionInstance parent = CreateParentPosition();
    if (!parent->IsNullPosition())
      return parent->GetAnchor()->data().GetTextStyles();
  }
  return current_anchor_text_styles;
}

std::vector<int32_t> AXNodePosition::GetWordStartOffsets() const {
  if (IsNullPosition())
    return std::vector<int32_t>();
  BASE_DCHECK(GetAnchor());

  // Embedded object replacement characters are not represented in |kWordStarts|
  // attribute.
  if (IsEmptyObjectReplacedByCharacter())
    return {0};

  return GetAnchor()->data().GetIntListAttribute(
      ax::mojom::IntListAttribute::kWordStarts);
}

std::vector<int32_t> AXNodePosition::GetWordEndOffsets() const {
  if (IsNullPosition())
    return std::vector<int32_t>();
  BASE_DCHECK(GetAnchor());

  // Embedded object replacement characters are not represented in |kWordEnds|
  // attribute. Since the whole text exposed inside of an embedded object is of
  // length 1 (the embedded object replacement character), the word end offset
  // is positioned at 1. Because we want to treat the embedded object
  // replacement characters as ordinary characters, it wouldn't be consistent to
  // assume they have no length and return 0 instead of 1.
  if (IsEmptyObjectReplacedByCharacter())
    return {1};

  return GetAnchor()->data().GetIntListAttribute(
      ax::mojom::IntListAttribute::kWordEnds);
}

AXNode::AXID AXNodePosition::GetNextOnLineID(AXNode::AXID node_id) const {
  if (IsNullPosition())
    return AXNode::kInvalidAXID;
  AXNode* node = GetNodeInTree(tree_id(), node_id);
  int next_on_line_id;
  if (!node || !node->data().GetIntAttribute(
                   ax::mojom::IntAttribute::kNextOnLineId, &next_on_line_id)) {
    return AXNode::kInvalidAXID;
  }
  return static_cast<AXNode::AXID>(next_on_line_id);
}

AXNode::AXID AXNodePosition::GetPreviousOnLineID(AXNode::AXID node_id) const {
  if (IsNullPosition())
    return AXNode::kInvalidAXID;
  AXNode* node = GetNodeInTree(tree_id(), node_id);
  int previous_on_line_id;
  if (!node ||
      !node->data().GetIntAttribute(ax::mojom::IntAttribute::kPreviousOnLineId,
                                    &previous_on_line_id)) {
    return AXNode::kInvalidAXID;
  }
  return static_cast<AXNode::AXID>(previous_on_line_id);
}

AXNode* AXNodePosition::GetParent(AXNode* child,
                                  AXTreeID child_tree_id,
                                  AXTreeID* parent_tree_id,
                                  AXNode::AXID* parent_id) {
  BASE_DCHECK(parent_tree_id);
  BASE_DCHECK(parent_id);

  *parent_tree_id = AXTreeIDUnknown();
  *parent_id = AXNode::kInvalidAXID;

  if (!child)
    return nullptr;

  AXNode* parent = child->parent();
  *parent_tree_id = child_tree_id;

  if (!parent) {
    AXTreeManager* manager =
        AXTreeManagerMap::GetInstance().GetManager(child_tree_id);
    if (manager) {
      parent = manager->GetParentNodeFromParentTreeAsAXNode();
      *parent_tree_id = manager->GetParentTreeID();
    }
  }

  if (!parent) {
    *parent_tree_id = AXTreeIDUnknown();
    return parent;
  }

  *parent_id = parent->id();
  return parent;
}

}  // namespace ui
