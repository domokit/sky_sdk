// Copyright 2015 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef UI_ACCESSIBILITY_AX_TREE_DATA_H_
#define UI_ACCESSIBILITY_AX_TREE_DATA_H_

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "ax_enums.h"
#include "ax_export.h"
#include "ax_node.h"
#include "ax_tree_id_registry.h"

namespace ui {

// The data associated with an accessibility tree that's global to the
// tree and not associated with any particular node in the tree.
struct AX_EXPORT AXTreeData {
  AXTreeData();
  AXTreeData(const AXTreeData& other);
  virtual ~AXTreeData();

  // Return a string representation of this data, for debugging.
  virtual std::string ToString() const;

  // This is a simple serializable struct. All member variables should be
  // public and copyable.

  // The globally unique ID of this accessibility tree.
  AXTreeID tree_id;

  // The ID of the accessibility tree that this tree is contained in, if any.
  AXTreeID parent_tree_id;

  // The ID of the accessibility tree that has focus. This is typically set
  // on the root frame in a frame tree.
  AXTreeID focused_tree_id;

  // Attributes specific to trees that are web frames.
  std::string doctype;
  bool loaded = false;
  float loading_progress = 0.0f;
  std::string mimetype;
  std::string title;
  std::string url;

  // The node with keyboard focus within this tree, if any, or
  // AXNode::kInvalidAXID if no node in this tree has focus.
  AXNode::AXID focus_id = AXNode::kInvalidAXID;

  // The current text selection within this tree, if any, expressed as the
  // node ID and character offset of the anchor (selection start) and focus
  // (selection end). If the offset could correspond to a position on two
  // different lines, sel_upstream_affinity means the cursor is on the first
  // line, otherwise it's on the second line.
  // Most use cases will want to use ui::OwnerTree::GetUnignoredSelection.
  bool sel_is_backward = false;
  AXNode::AXID sel_anchor_object_id = AXNode::kInvalidAXID;
  int32_t sel_anchor_offset = -1;
  ax::mojom::TextAffinity sel_anchor_affinity;
  AXNode::AXID sel_focus_object_id = AXNode::kInvalidAXID;
  int32_t sel_focus_offset = -1;
  ax::mojom::TextAffinity sel_focus_affinity;

  // The node that's used as the root scroller. On some platforms
  // like Android we need to ignore accessibility scroll offsets for
  // that node and get them from the viewport instead.
  AXNode::AXID root_scroller_id = AXNode::kInvalidAXID;
};

AX_EXPORT bool operator==(const AXTreeData& lhs, const AXTreeData& rhs);
AX_EXPORT bool operator!=(const AXTreeData& lhs, const AXTreeData& rhs);

}  // namespace ui

#endif  // UI_ACCESSIBILITY_AX_TREE_DATA_H_
