// Copyright 2020 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef ACCESSIBILITY_TEST_AX_AX_TREE_MANAGER_H_
#define ACCESSIBILITY_TEST_AX_AX_TREE_MANAGER_H_

#include <memory>

#include "ax_tree.h"
#include "ax_tree_id.h"
#include "ax_tree_manager.h"

namespace ui {

class AXNode;

// A basic implementation of AXTreeManager that can be used in tests.
//
// For simplicity, this class supports only a single tree and doesn't perform
// any walking across multiple trees.
class TestAXTreeManager : public AXTreeManager {
 public:
  // This constructor does not create an empty AXTree. Call "SetTree" if you
  // need to manage a specific tree. Useful when you need to test for the
  // situation when no AXTree has been loaded yet.
  TestAXTreeManager();

  // Takes ownership of |tree|.
  explicit TestAXTreeManager(std::unique_ptr<AXTree> tree);

  virtual ~TestAXTreeManager();

  TestAXTreeManager(const TestAXTreeManager& manager) = delete;
  TestAXTreeManager& operator=(const TestAXTreeManager& manager) = delete;

  void DestroyTree();
  AXTree* GetTree() const;
  // Takes ownership of |tree|.
  void SetTree(std::unique_ptr<AXTree> tree);

  // AXTreeManager implementation.
  AXNode* GetNodeFromTree(const AXTreeID tree_id,
                          const AXNode::AXID node_id) const override;
  AXNode* GetNodeFromTree(const AXNode::AXID node_id) const override;
  AXTreeID GetTreeID() const override;
  AXTreeID GetParentTreeID() const override;
  AXNode* GetRootAsAXNode() const override;
  AXNode* GetParentNodeFromParentTreeAsAXNode() const override;

 private:
  std::unique_ptr<AXTree> tree_;
};

}  // namespace ui

#endif  // ACCESSIBILITY_TEST_AX_AX_TREE_MANAGER_H_
