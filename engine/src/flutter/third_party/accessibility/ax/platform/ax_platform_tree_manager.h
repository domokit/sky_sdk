// Copyright 2019 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef UI_ACCESSIBILITY_PLATFORM_AX_PLATFORM_TREE_MANAGER_H_
#define UI_ACCESSIBILITY_PLATFORM_AX_PLATFORM_TREE_MANAGER_H_

#include "ax/ax_export.h"
#include "ax/ax_node.h"
#include "ax/ax_tree_id.h"
#include "ax/ax_tree_manager.h"

namespace ui {

class AXPlatformNode;
class AXPlatformNodeDelegate;

// Abstract interface for a class that owns an AXTree and manages its
// connections to other AXTrees in the same page or desktop (parent and child
// trees).
class AX_EXPORT AXPlatformTreeManager : public AXTreeManager {
 public:
  virtual ~AXPlatformTreeManager() = default;

  // Returns an AXPlatformNode with the specified and |node_id|.
  virtual AXPlatformNode* GetPlatformNodeFromTree(
      const AXNode::AXID node_id) const = 0;

  // Returns an AXPlatformNode that corresponds to the given |node|.
  virtual AXPlatformNode* GetPlatformNodeFromTree(const AXNode& node) const = 0;

  // Returns an AXPlatformNodeDelegate that corresponds to a root node
  // of the accessibility tree.
  virtual AXPlatformNodeDelegate* RootDelegate() const = 0;
};

}  // namespace ui

#endif  // UI_ACCESSIBILITY_PLATFORM_AX_PLATFORM_TREE_MANAGER_H_
