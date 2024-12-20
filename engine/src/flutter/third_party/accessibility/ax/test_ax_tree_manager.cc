// Copyright 2020 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "test_ax_tree_manager.h"

#include "ax_node.h"
#include "ax_tree_data.h"
#include "ax_tree_manager_map.h"

namespace ui {

TestAXTreeManager::TestAXTreeManager() = default;

TestAXTreeManager::TestAXTreeManager(std::unique_ptr<AXTree> tree)
    : tree_(std::move(tree)) {
  AXTreeManagerMap::GetInstance().AddTreeManager(GetTreeID(), this);
}

TestAXTreeManager::~TestAXTreeManager() {
  if (tree_)
    AXTreeManagerMap::GetInstance().RemoveTreeManager(GetTreeID());
}

void TestAXTreeManager::DestroyTree() {
  if (!tree_)
    return;

  AXTreeManagerMap::GetInstance().RemoveTreeManager(GetTreeID());
  tree_.reset();
}

AXTree* TestAXTreeManager::GetTree() const {
  if (!tree_) {
    BASE_LOG() << "Did you forget to call SetTree?";
    BASE_UNREACHABLE();
  }
  return tree_.get();
}

void TestAXTreeManager::SetTree(std::unique_ptr<AXTree> tree) {
  if (tree_)
    AXTreeManagerMap::GetInstance().RemoveTreeManager(GetTreeID());

  tree_ = std::move(tree);
  AXTreeManagerMap::GetInstance().AddTreeManager(GetTreeID(), this);
}

AXNode* TestAXTreeManager::GetNodeFromTree(const AXTreeID tree_id,
                                           const AXNode::AXID node_id) const {
  return (tree_ && GetTreeID() == tree_id) ? tree_->GetFromId(node_id)
                                           : nullptr;
}

AXNode* TestAXTreeManager::GetNodeFromTree(const AXNode::AXID node_id) const {
  return tree_ ? tree_->GetFromId(node_id) : nullptr;
}

AXTreeID TestAXTreeManager::GetTreeID() const {
  return tree_ ? tree_->data().tree_id : AXTreeIDUnknown();
}

AXTreeID TestAXTreeManager::GetParentTreeID() const {
  return AXTreeIDUnknown();
}

AXNode* TestAXTreeManager::GetRootAsAXNode() const {
  return tree_ ? tree_->root() : nullptr;
}

AXNode* TestAXTreeManager::GetParentNodeFromParentTreeAsAXNode() const {
  return nullptr;
}

}  // namespace ui
