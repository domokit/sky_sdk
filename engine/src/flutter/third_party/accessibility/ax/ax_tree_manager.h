// Copyright 2019 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef UI_ACCESSIBILITY_AX_TREE_MANAGER_H_
#define UI_ACCESSIBILITY_AX_TREE_MANAGER_H_

#include "ax_export.h"
#include "ax_node.h"
#include "ax_tree.h"
#include "ax_tree_id.h"

namespace ui {

// Abstract interface for a class that owns an AXTree and manages its
// connections to other AXTrees in the same page or desktop (parent and child
// trees).
class AX_EXPORT AXTreeManager {
 public:
  virtual ~AXTreeManager() = default;

  // Returns the AXNode with the given |node_id| from the tree that has the
  // given |tree_id|. This allows for callers to access nodes outside of their
  // own tree. Returns nullptr if |tree_id| or |node_id| is not found.
  virtual AXNode* GetNodeFromTree(const AXTreeID tree_id,
                                  const AXNode::AXID node_id) const = 0;

  // Returns the AXNode in the current tree that has the given |node_id|.
  // Returns nullptr if |node_id| is not found.
  virtual AXNode* GetNodeFromTree(const AXNode::AXID node_id) const = 0;

  // Returns the tree id of the tree managed by this AXTreeManager.
  virtual AXTreeID GetTreeID() const = 0;

  // Returns the tree id of the parent tree.
  // Returns AXTreeIDUnknown if this tree doesn't have a parent tree.
  virtual AXTreeID GetParentTreeID() const = 0;

  // Returns the AXNode that is at the root of the current tree.
  virtual AXNode* GetRootAsAXNode() const = 0;

  // If this tree has a parent tree, returns the node in the parent tree that
  // hosts the current tree. Returns nullptr if this tree doesn't have a parent
  // tree.
  virtual AXNode* GetParentNodeFromParentTreeAsAXNode() const = 0;

  virtual AXTree* GetTree() const = 0;
};

}  // namespace ui

#endif  // UI_ACCESSIBILITY_AX_TREE_MANAGER_H_
