// Copyright 2013 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "ax_tree.h"

#include <cstddef>
#include <cstdint>
#include <memory>

#include "ax_enum_util.h"
#include "ax_node.h"
#include "ax_node_position.h"
#include "ax_tree_data.h"
#include "ax_tree_id.h"
#include "ax_tree_observer.h"
#include "base/string_utils.h"
#include "gtest/gtest.h"
#include "test_ax_tree_manager.h"

// Helper macro for testing selection values and maintain
// correct stack tracing and failure causality.
#define TEST_SELECTION(tree_update, tree, input, expected)         \
  {                                                                \
    tree_update.has_tree_data = true;                              \
    tree_update.tree_data.sel_anchor_object_id = input.anchor_id;  \
    tree_update.tree_data.sel_anchor_offset = input.anchor_offset; \
    tree_update.tree_data.sel_focus_object_id = input.focus_id;    \
    tree_update.tree_data.sel_focus_offset = input.focus_offset;   \
    EXPECT_TRUE(tree->Unserialize(tree_update));                   \
    AXTree::Selection actual = tree->GetUnignoredSelection();      \
    EXPECT_EQ(expected.anchor_id, actual.anchor_object_id);        \
    EXPECT_EQ(expected.anchor_offset, actual.anchor_offset);       \
    EXPECT_EQ(expected.focus_id, actual.focus_object_id);          \
    EXPECT_EQ(expected.focus_offset, actual.focus_offset);         \
  }

namespace ui {

namespace {

std::string IntVectorToString(const std::vector<int>& items) {
  std::string str;
  for (size_t i = 0; i < items.size(); ++i) {
    if (i > 0)
      str += ",";
    str += base::NumberToString(items[i]);
  }
  return str;
}

std::string GetBoundsAsString(const AXTree& tree, int32_t id) {
  AXNode* node = tree.GetFromId(id);
  gfx::RectF bounds = tree.GetTreeBounds(node);
  return base::StringPrintf("(%.0f, %.0f) size (%.0f x %.0f)", bounds.x(),
                            bounds.y(), bounds.width(), bounds.height());
}

std::string GetUnclippedBoundsAsString(const AXTree& tree, int32_t id) {
  AXNode* node = tree.GetFromId(id);
  gfx::RectF bounds = tree.GetTreeBounds(node, nullptr, false);
  return base::StringPrintf("(%.0f, %.0f) size (%.0f x %.0f)", bounds.x(),
                            bounds.y(), bounds.width(), bounds.height());
}

bool IsNodeOffscreen(const AXTree& tree, int32_t id) {
  AXNode* node = tree.GetFromId(id);
  bool result = false;
  tree.GetTreeBounds(node, &result);
  return result;
}

class TestAXTreeObserver : public AXTreeObserver {
 public:
  TestAXTreeObserver(AXTree* tree)
      : tree_(tree), tree_data_changed_(false), root_changed_(false) {
    tree_->AddObserver(this);
  }
  ~TestAXTreeObserver() { tree_->RemoveObserver(this); }

  void OnNodeDataWillChange(AXTree* tree,
                            const AXNodeData& old_node_data,
                            const AXNodeData& new_node_data) override {}
  void OnNodeDataChanged(AXTree* tree,
                         const AXNodeData& old_node_data,
                         const AXNodeData& new_node_data) override {}
  void OnTreeDataChanged(AXTree* tree,
                         const ui::AXTreeData& old_data,
                         const ui::AXTreeData& new_data) override {
    tree_data_changed_ = true;
  }

  std::optional<AXNode::AXID> unignored_parent_id_before_node_deleted;
  void OnNodeWillBeDeleted(AXTree* tree, AXNode* node) override {
    // When this observer function is called in an update, the actual node
    // deletion has not happened yet. Verify that node still exists in the tree.
    ASSERT_NE(nullptr, tree->GetFromId(node->id()));
    node_will_be_deleted_ids_.push_back(node->id());

    if (unignored_parent_id_before_node_deleted) {
      ASSERT_NE(nullptr, node->GetUnignoredParent());
      ASSERT_EQ(*unignored_parent_id_before_node_deleted,
                node->GetUnignoredParent()->id());
    }
  }

  void OnSubtreeWillBeDeleted(AXTree* tree, AXNode* node) override {
    subtree_deleted_ids_.push_back(node->id());
  }

  void OnNodeWillBeReparented(AXTree* tree, AXNode* node) override {
    node_will_be_reparented_ids_.push_back(node->id());
  }

  void OnSubtreeWillBeReparented(AXTree* tree, AXNode* node) override {
    subtree_will_be_reparented_ids_.push_back(node->id());
  }

  void OnNodeCreated(AXTree* tree, AXNode* node) override {
    created_ids_.push_back(node->id());
  }

  void OnNodeDeleted(AXTree* tree, int32_t node_id) override {
    // When this observer function is called in an update, node has already been
    // deleted from the tree. Verify that the node is absent from the tree.
    ASSERT_EQ(nullptr, tree->GetFromId(node_id));
    deleted_ids_.push_back(node_id);
  }

  void OnNodeReparented(AXTree* tree, AXNode* node) override {
    node_reparented_ids_.push_back(node->id());
  }

  void OnNodeChanged(AXTree* tree, AXNode* node) override {
    changed_ids_.push_back(node->id());
  }

  void OnAtomicUpdateFinished(AXTree* tree,
                              bool root_changed,
                              const std::vector<Change>& changes) override {
    root_changed_ = root_changed;

    for (size_t i = 0; i < changes.size(); ++i) {
      int id = changes[i].node->id();
      switch (changes[i].type) {
        case NODE_CREATED:
          node_creation_finished_ids_.push_back(id);
          break;
        case SUBTREE_CREATED:
          subtree_creation_finished_ids_.push_back(id);
          break;
        case NODE_REPARENTED:
          node_reparented_finished_ids_.push_back(id);
          break;
        case SUBTREE_REPARENTED:
          subtree_reparented_finished_ids_.push_back(id);
          break;
        case NODE_CHANGED:
          change_finished_ids_.push_back(id);
          break;
      }
    }
  }

  void OnRoleChanged(AXTree* tree,
                     AXNode* node,
                     ax::mojom::Role old_role,
                     ax::mojom::Role new_role) override {
    attribute_change_log_.push_back(base::StringPrintf(
        "Role changed from %s to %s", ToString(old_role), ToString(new_role)));
  }

  void OnStateChanged(AXTree* tree,
                      AXNode* node,
                      ax::mojom::State state,
                      bool new_value) override {
    attribute_change_log_.push_back(base::StringPrintf(
        "%s changed to %s", ToString(state), new_value ? "true" : "false"));
  }

  void OnStringAttributeChanged(AXTree* tree,
                                AXNode* node,
                                ax::mojom::StringAttribute attr,
                                const std::string& old_value,
                                const std::string& new_value) override {
    attribute_change_log_.push_back(
        base::StringPrintf("%s changed from %s to %s", ToString(attr),
                           old_value.c_str(), new_value.c_str()));
  }

  void OnIntAttributeChanged(AXTree* tree,
                             AXNode* node,
                             ax::mojom::IntAttribute attr,
                             int32_t old_value,
                             int32_t new_value) override {
    attribute_change_log_.push_back(base::StringPrintf(
        "%s changed from %d to %d", ToString(attr), old_value, new_value));
  }

  void OnFloatAttributeChanged(AXTree* tree,
                               AXNode* node,
                               ax::mojom::FloatAttribute attr,
                               float old_value,
                               float new_value) override {
    attribute_change_log_.push_back(base::StringPrintf(
        "%s changed from %.1f to %.1f", ToString(attr), old_value, new_value));
  }

  void OnBoolAttributeChanged(AXTree* tree,
                              AXNode* node,
                              ax::mojom::BoolAttribute attr,
                              bool new_value) override {
    attribute_change_log_.push_back(base::StringPrintf(
        "%s changed to %s", ToString(attr), new_value ? "true" : "false"));
  }

  void OnIntListAttributeChanged(
      AXTree* tree,
      AXNode* node,
      ax::mojom::IntListAttribute attr,
      const std::vector<int32_t>& old_value,
      const std::vector<int32_t>& new_value) override {
    attribute_change_log_.push_back(
        base::StringPrintf("%s changed from %s to %s", ToString(attr),
                           IntVectorToString(old_value).c_str(),
                           IntVectorToString(new_value).c_str()));
  }

  bool tree_data_changed() const { return tree_data_changed_; }
  bool root_changed() const { return root_changed_; }
  const std::vector<int32_t>& deleted_ids() { return deleted_ids_; }
  const std::vector<int32_t>& subtree_deleted_ids() {
    return subtree_deleted_ids_;
  }
  const std::vector<int32_t>& created_ids() { return created_ids_; }
  const std::vector<int32_t>& node_creation_finished_ids() {
    return node_creation_finished_ids_;
  }
  const std::vector<int32_t>& subtree_creation_finished_ids() {
    return subtree_creation_finished_ids_;
  }
  const std::vector<int32_t>& node_reparented_finished_ids() {
    return node_reparented_finished_ids_;
  }
  const std::vector<int32_t>& subtree_will_be_reparented_ids() {
    return subtree_will_be_reparented_ids_;
  }
  const std::vector<int32_t>& node_will_be_reparented_ids() {
    return node_will_be_reparented_ids_;
  }
  const std::vector<int32_t>& node_will_be_deleted_ids() {
    return node_will_be_deleted_ids_;
  }
  const std::vector<int32_t>& node_reparented_ids() {
    return node_reparented_ids_;
  }
  const std::vector<int32_t>& subtree_reparented_finished_ids() {
    return subtree_reparented_finished_ids_;
  }
  const std::vector<int32_t>& change_finished_ids() {
    return change_finished_ids_;
  }
  const std::vector<std::string>& attribute_change_log() {
    return attribute_change_log_;
  }

 private:
  AXTree* tree_;
  bool tree_data_changed_;
  bool root_changed_;
  std::vector<int32_t> deleted_ids_;
  std::vector<int32_t> subtree_deleted_ids_;
  std::vector<int32_t> created_ids_;
  std::vector<int32_t> changed_ids_;
  std::vector<int32_t> subtree_will_be_reparented_ids_;
  std::vector<int32_t> node_will_be_reparented_ids_;
  std::vector<int32_t> node_will_be_deleted_ids_;
  std::vector<int32_t> node_creation_finished_ids_;
  std::vector<int32_t> subtree_creation_finished_ids_;
  std::vector<int32_t> node_reparented_ids_;
  std::vector<int32_t> node_reparented_finished_ids_;
  std::vector<int32_t> subtree_reparented_finished_ids_;
  std::vector<int32_t> change_finished_ids_;
  std::vector<std::string> attribute_change_log_;
};

}  // namespace

// A macro for testing that a std::optional has both a value and that its value
// is set to a particular expectation.
#define EXPECT_OPTIONAL_EQ(expected, actual) \
  EXPECT_TRUE(actual.has_value());           \
  if (actual) {                              \
    EXPECT_EQ(expected, actual.value());     \
  }

TEST(AXTreeTest, SerializeAXTreeUpdate) {
  AXNodeData list;
  list.id = 3;
  list.role = ax::mojom::Role::kList;
  list.child_ids.push_back(4);
  list.child_ids.push_back(5);
  list.child_ids.push_back(6);

  AXNodeData list_item_2;
  list_item_2.id = 5;
  list_item_2.role = ax::mojom::Role::kListItem;

  AXNodeData list_item_3;
  list_item_3.id = 6;
  list_item_3.role = ax::mojom::Role::kListItem;

  AXNodeData button;
  button.id = 7;
  button.role = ax::mojom::Role::kButton;

  AXTreeUpdate update;
  update.root_id = 3;
  update.nodes.push_back(list);
  update.nodes.push_back(list_item_2);
  update.nodes.push_back(list_item_3);
  update.nodes.push_back(button);

  EXPECT_EQ(
      "AXTreeUpdate: root id 3\n"
      "id=3 list (0, 0)-(0, 0) child_ids=4,5,6\n"
      "  id=5 listItem (0, 0)-(0, 0)\n"
      "  id=6 listItem (0, 0)-(0, 0)\n"
      "id=7 button (0, 0)-(0, 0)\n",
      update.ToString());
}

TEST(AXTreeTest, LeaveOrphanedDeletedSubtreeFails) {
  AXTreeUpdate initial_state;
  initial_state.root_id = 1;
  initial_state.nodes.resize(3);
  initial_state.nodes[0].id = 1;
  initial_state.nodes[0].child_ids.push_back(2);
  initial_state.nodes[0].child_ids.push_back(3);
  initial_state.nodes[1].id = 2;
  initial_state.nodes[2].id = 3;
  AXTree tree(initial_state);

  // This should fail because we delete a subtree rooted at id=2
  // but never update it.
  AXTreeUpdate update;
  update.node_id_to_clear = 2;
  update.nodes.resize(1);
  update.nodes[0].id = 3;
  EXPECT_FALSE(tree.Unserialize(update));
  ASSERT_EQ("Nodes left pending by the update: 2", tree.error());
}

TEST(AXTreeTest, LeaveOrphanedNewChildFails) {
  AXTreeUpdate initial_state;
  initial_state.root_id = 1;
  initial_state.nodes.resize(1);
  initial_state.nodes[0].id = 1;
  AXTree tree(initial_state);

  // This should fail because we add a new child to the root node
  // but never update it.
  AXTreeUpdate update;
  update.nodes.resize(1);
  update.nodes[0].id = 1;
  update.nodes[0].child_ids.push_back(2);
  EXPECT_FALSE(tree.Unserialize(update));
  ASSERT_EQ("Nodes left pending by the update: 2", tree.error());
}

TEST(AXTreeTest, DuplicateChildIdFails) {
  AXTreeUpdate initial_state;
  initial_state.root_id = 1;
  initial_state.nodes.resize(1);
  initial_state.nodes[0].id = 1;
  AXTree tree(initial_state);

  // This should fail because a child id appears twice.
  AXTreeUpdate update;
  update.nodes.resize(2);
  update.nodes[0].id = 1;
  update.nodes[0].child_ids.push_back(2);
  update.nodes[0].child_ids.push_back(2);
  update.nodes[1].id = 2;
  EXPECT_FALSE(tree.Unserialize(update));
  ASSERT_EQ("Node 1 has duplicate child id 2", tree.error());
}

TEST(AXTreeTest, InvalidReparentingFails) {
  AXTreeUpdate initial_state;
  initial_state.root_id = 1;
  initial_state.nodes.resize(3);
  initial_state.nodes[0].id = 1;
  initial_state.nodes[0].child_ids.push_back(2);
  initial_state.nodes[1].id = 2;
  initial_state.nodes[1].child_ids.push_back(3);
  initial_state.nodes[2].id = 3;

  AXTree tree(initial_state);

  // This should fail because node 3 is reparented from node 2 to node 1
  // without deleting node 1's subtree first.
  AXTreeUpdate update;
  update.nodes.resize(3);
  update.nodes[0].id = 1;
  update.nodes[0].child_ids.push_back(3);
  update.nodes[0].child_ids.push_back(2);
  update.nodes[1].id = 2;
  update.nodes[2].id = 3;
  EXPECT_FALSE(tree.Unserialize(update));
  ASSERT_EQ("Node 3 is not marked for destruction, would be reparented to 1",
            tree.error());
}

TEST(AXTreeTest, NoReparentingOfRootIfNoNewRoot) {
  AXNodeData root;
  root.id = 1;
  AXNodeData child1;
  child1.id = 2;
  AXNodeData child2;
  child2.id = 3;

  root.child_ids = {child1.id};
  child1.child_ids = {child2.id};

  AXTreeUpdate initial_state;
  initial_state.root_id = root.id;
  initial_state.nodes = {root, child1, child2};

  AXTree tree(initial_state);

  // Update the root but don't change it by reparenting |child2| to be a child
  // of the root.
  root.child_ids = {child1.id, child2.id};
  child1.child_ids = {};

  AXTreeUpdate update;
  update.root_id = root.id;
  update.node_id_to_clear = root.id;
  update.nodes = {root, child1, child2};

  TestAXTreeObserver test_observer(&tree);
  ASSERT_TRUE(tree.Unserialize(update));

  EXPECT_EQ(0U, test_observer.deleted_ids().size());
  EXPECT_EQ(0U, test_observer.subtree_deleted_ids().size());
  EXPECT_EQ(0U, test_observer.created_ids().size());

  EXPECT_EQ(0U, test_observer.node_creation_finished_ids().size());
  EXPECT_EQ(0U, test_observer.subtree_creation_finished_ids().size());
  EXPECT_EQ(0U, test_observer.node_reparented_finished_ids().size());

  ASSERT_EQ(2U, test_observer.subtree_reparented_finished_ids().size());
  EXPECT_EQ(child1.id, test_observer.subtree_reparented_finished_ids()[0]);
  EXPECT_EQ(child2.id, test_observer.subtree_reparented_finished_ids()[1]);

  ASSERT_EQ(1U, test_observer.change_finished_ids().size());
  EXPECT_EQ(root.id, test_observer.change_finished_ids()[0]);

  EXPECT_FALSE(test_observer.root_changed());
  EXPECT_FALSE(test_observer.tree_data_changed());
}

TEST(AXTreeTest, NoReparentingIfOnlyRemovedAndChangedNotReAdded) {
  AXNodeData root;
  root.id = 1;
  AXNodeData child1;
  child1.id = 2;
  AXNodeData child2;
  child2.id = 3;

  root.child_ids = {child1.id};
  child1.child_ids = {child2.id};

  AXTreeUpdate initial_state;
  initial_state.root_id = root.id;
  initial_state.nodes = {root, child1, child2};

  AXTree tree(initial_state);

  // Change existing attributes.
  AXTreeUpdate update;
  update.nodes.resize(2);
  update.nodes[0].id = 2;
  update.nodes[0].AddIntAttribute(ax::mojom::IntAttribute::kActivedescendantId,
                                  3);
  update.nodes[1].id = 1;

  TestAXTreeObserver test_observer(&tree);
  EXPECT_TRUE(tree.Unserialize(update)) << tree.error();

  EXPECT_EQ(2U, test_observer.deleted_ids().size());
  EXPECT_EQ(2U, test_observer.subtree_deleted_ids().size());
  EXPECT_EQ(0U, test_observer.created_ids().size());

  EXPECT_EQ(0U, test_observer.node_creation_finished_ids().size());
  EXPECT_EQ(0U, test_observer.subtree_creation_finished_ids().size());
  EXPECT_EQ(0U, test_observer.node_will_be_reparented_ids().size());
  EXPECT_EQ(2U, test_observer.node_will_be_deleted_ids().size());
  EXPECT_EQ(0U, test_observer.subtree_will_be_reparented_ids().size());
  EXPECT_EQ(0U, test_observer.node_reparented_ids().size());
  EXPECT_EQ(0U, test_observer.node_reparented_finished_ids().size());
  EXPECT_EQ(0U, test_observer.subtree_reparented_finished_ids().size());

  EXPECT_FALSE(test_observer.root_changed());
  EXPECT_FALSE(test_observer.tree_data_changed());
}

// Tests a fringe scenario that may happen if multiple AXTreeUpdates are merged.
// Make sure that when a node is reparented then removed from the tree
// that it notifies OnNodeDeleted rather than OnNodeReparented.
TEST(AXTreeTest, NoReparentingIfRemovedMultipleTimesAndNotInFinalTree) {
  AXTreeUpdate initial_state;
  initial_state.root_id = 1;
  initial_state.nodes.resize(4);
  initial_state.nodes[0].id = 1;
  initial_state.nodes[0].child_ids = {2, 4};
  initial_state.nodes[1].id = 2;
  initial_state.nodes[1].child_ids = {3};
  initial_state.nodes[2].id = 3;
  initial_state.nodes[3].id = 4;

  AXTree tree(initial_state);

  AXTreeUpdate update;
  update.nodes.resize(4);
  // Delete AXID 3
  update.nodes[0].id = 2;
  // Reparent AXID 3 onto AXID 4
  update.nodes[1].id = 4;
  update.nodes[1].child_ids = {3};
  update.nodes[2].id = 3;
  // Delete AXID 3
  update.nodes[3].id = 4;

  TestAXTreeObserver test_observer(&tree);
  ASSERT_TRUE(tree.Unserialize(update)) << tree.error();

  EXPECT_EQ(1U, test_observer.deleted_ids().size());
  EXPECT_EQ(1U, test_observer.subtree_deleted_ids().size());
  EXPECT_EQ(0U, test_observer.created_ids().size());

  EXPECT_EQ(0U, test_observer.node_creation_finished_ids().size());
  EXPECT_EQ(0U, test_observer.subtree_creation_finished_ids().size());
  EXPECT_EQ(0U, test_observer.node_will_be_reparented_ids().size());
  EXPECT_EQ(1U, test_observer.node_will_be_deleted_ids().size());
  EXPECT_EQ(0U, test_observer.subtree_will_be_reparented_ids().size());
  EXPECT_EQ(0U, test_observer.node_reparented_ids().size());
  EXPECT_EQ(0U, test_observer.node_reparented_finished_ids().size());
  EXPECT_EQ(0U, test_observer.subtree_reparented_finished_ids().size());

  EXPECT_FALSE(test_observer.root_changed());
  EXPECT_FALSE(test_observer.tree_data_changed());
}

// Tests a fringe scenario that may happen if multiple AXTreeUpdates are merged.
// Make sure that when a node is reparented multiple times and exists in the
// final tree that it notifies OnNodeReparented rather than OnNodeDeleted.
TEST(AXTreeTest, ReparentIfRemovedMultipleTimesButExistsInFinalTree) {
  AXTreeUpdate initial_state;
  initial_state.root_id = 1;
  initial_state.nodes.resize(4);
  initial_state.nodes[0].id = 1;
  initial_state.nodes[0].child_ids = {2, 4};
  initial_state.nodes[1].id = 2;
  initial_state.nodes[1].child_ids = {3};
  initial_state.nodes[2].id = 3;
  initial_state.nodes[3].id = 4;

  AXTree tree(initial_state);

  AXTreeUpdate update;
  update.nodes.resize(6);
  // Delete AXID 3
  update.nodes[0].id = 2;
  // Reparent AXID 3 onto AXID 4
  update.nodes[1].id = 4;
  update.nodes[1].child_ids = {3};
  update.nodes[2].id = 3;
  // Delete AXID 3
  update.nodes[3].id = 4;
  // Reparent AXID 3 onto AXID 2
  update.nodes[4].id = 2;
  update.nodes[4].child_ids = {3};
  update.nodes[5].id = 3;

  TestAXTreeObserver test_observer(&tree);
  ASSERT_TRUE(tree.Unserialize(update)) << tree.error();

  EXPECT_EQ(0U, test_observer.deleted_ids().size());
  EXPECT_EQ(0U, test_observer.subtree_deleted_ids().size());
  EXPECT_EQ(0U, test_observer.created_ids().size());

  EXPECT_EQ(0U, test_observer.node_creation_finished_ids().size());
  EXPECT_EQ(0U, test_observer.subtree_creation_finished_ids().size());
  EXPECT_EQ(1U, test_observer.node_will_be_reparented_ids().size());
  EXPECT_EQ(0U, test_observer.node_will_be_deleted_ids().size());
  EXPECT_EQ(1U, test_observer.subtree_will_be_reparented_ids().size());
  EXPECT_EQ(1U, test_observer.node_reparented_ids().size());
  EXPECT_EQ(0U, test_observer.node_reparented_finished_ids().size());
  EXPECT_EQ(1U, test_observer.subtree_reparented_finished_ids().size());

  EXPECT_FALSE(test_observer.root_changed());
  EXPECT_FALSE(test_observer.tree_data_changed());
}

TEST(AXTreeTest, ReparentRootIfRootChanged) {
  AXNodeData root;
  root.id = 1;
  AXNodeData child1;
  child1.id = 2;
  AXNodeData child2;
  child2.id = 3;

  root.child_ids = {child1.id};
  child1.child_ids = {child2.id};

  AXTreeUpdate initial_state;
  initial_state.root_id = root.id;
  initial_state.nodes = {root, child1, child2};

  AXTree tree(initial_state);

  // Create a new root and reparent |child2| to be a child of the new root.
  AXNodeData root2;
  root2.id = 4;
  root2.child_ids = {child1.id, child2.id};
  child1.child_ids = {};

  AXTreeUpdate update;
  update.root_id = root2.id;
  update.node_id_to_clear = root.id;
  update.nodes = {root2, child1, child2};

  TestAXTreeObserver test_observer(&tree);
  ASSERT_TRUE(tree.Unserialize(update));

  ASSERT_EQ(1U, test_observer.deleted_ids().size());
  EXPECT_EQ(root.id, test_observer.deleted_ids()[0]);

  ASSERT_EQ(1U, test_observer.subtree_deleted_ids().size());
  EXPECT_EQ(root.id, test_observer.subtree_deleted_ids()[0]);

  ASSERT_EQ(1U, test_observer.created_ids().size());
  EXPECT_EQ(root2.id, test_observer.created_ids()[0]);

  EXPECT_EQ(0U, test_observer.node_creation_finished_ids().size());

  ASSERT_EQ(1U, test_observer.subtree_creation_finished_ids().size());
  EXPECT_EQ(root2.id, test_observer.subtree_creation_finished_ids()[0]);

  ASSERT_EQ(2U, test_observer.node_reparented_finished_ids().size());
  EXPECT_EQ(child1.id, test_observer.node_reparented_finished_ids()[0]);
  EXPECT_EQ(child2.id, test_observer.node_reparented_finished_ids()[1]);

  EXPECT_EQ(0U, test_observer.subtree_reparented_finished_ids().size());

  EXPECT_EQ(0U, test_observer.change_finished_ids().size());

  EXPECT_TRUE(test_observer.root_changed());
  EXPECT_FALSE(test_observer.tree_data_changed());
}

TEST(AXTreeTest, ImplicitChildrenDelete) {
  // This test covers the case where an AXTreeUpdate includes a node without
  // mentioning that node's children, this should cause a delete of those child
  // nodes.

  // Setup initial tree state
  // Tree:
  //      1
  //    2   3
  AXTreeUpdate initial_state;
  initial_state.root_id = 1;
  initial_state.nodes.resize(3);
  initial_state.nodes[0].id = 1;
  initial_state.nodes[0].child_ids.resize(2);
  initial_state.nodes[0].child_ids[0] = 2;
  initial_state.nodes[0].child_ids[1] = 3;
  initial_state.nodes[1].id = 2;
  initial_state.nodes[2].id = 3;
  AXTree tree(initial_state);

  EXPECT_NE(tree.GetFromId(1), nullptr);
  EXPECT_NE(tree.GetFromId(2), nullptr);
  EXPECT_NE(tree.GetFromId(3), nullptr);

  // Perform a no-op update of node 1 but omit any mention of its children. This
  // should delete all of the node's children.
  AXTreeUpdate update;
  update.nodes.resize(1);
  update.nodes[0].id = 1;

  ASSERT_TRUE(tree.Unserialize(update));

  // Check that nodes 2 and 3 have been deleted.
  EXPECT_NE(tree.GetFromId(1), nullptr);
  EXPECT_EQ(tree.GetFromId(2), nullptr);
  EXPECT_EQ(tree.GetFromId(3), nullptr);
}

TEST(AXTreeTest, IndexInParentAfterReorder) {
  // This test covers the case where an AXTreeUpdate includes
  // reordered children.  The unignored index in parent
  // values should be updated.

  // Setup initial tree state.
  // Tree:
  //        1
  //    2   3  4
  AXTreeUpdate initial_state;
  initial_state.root_id = 1;
  initial_state.nodes.resize(4);
  initial_state.nodes[0].id = 1;
  initial_state.nodes[0].child_ids.resize(3);
  initial_state.nodes[0].child_ids[0] = 2;
  initial_state.nodes[0].child_ids[1] = 3;
  initial_state.nodes[0].child_ids[2] = 4;
  initial_state.nodes[1].id = 2;
  initial_state.nodes[2].id = 3;
  initial_state.nodes[3].id = 4;
  AXTree tree(initial_state);

  // Index in parent correct.
  EXPECT_EQ(0U, tree.GetFromId(2)->GetUnignoredIndexInParent());
  EXPECT_EQ(1U, tree.GetFromId(3)->GetUnignoredIndexInParent());
  EXPECT_EQ(2U, tree.GetFromId(4)->GetUnignoredIndexInParent());

  // Perform an update where we reorder children to [ 4 3 2 ]
  AXTreeUpdate update;
  update.nodes.resize(4);
  update.root_id = 1;
  update.nodes[0].id = 1;
  update.nodes[0].child_ids.resize(3);
  update.nodes[0].child_ids[0] = 4;
  update.nodes[0].child_ids[1] = 3;
  update.nodes[0].child_ids[2] = 2;
  update.nodes[1].id = 2;
  update.nodes[2].id = 3;
  update.nodes[3].id = 4;

  ASSERT_TRUE(tree.Unserialize(update));

  // Index in parent should have changed as well.
  EXPECT_EQ(0U, tree.GetFromId(4)->GetUnignoredIndexInParent());
  EXPECT_EQ(1U, tree.GetFromId(3)->GetUnignoredIndexInParent());
  EXPECT_EQ(2U, tree.GetFromId(2)->GetUnignoredIndexInParent());
}

TEST(AXTreeTest, IndexInParentAfterReorderIgnoredNode) {
  // This test covers another case where an AXTreeUpdate includes
  // reordered children.  If one of the reordered nodes is ignored, its
  // children's unignored index in parent should also be updated.

  // Setup initial tree state.
  // Tree:
  //        1
  //    2   3i  4
  //       5  6
  AXTreeUpdate initial_state;
  initial_state.root_id = 1;
  initial_state.nodes.resize(6);
  initial_state.nodes[0].id = 1;
  initial_state.nodes[0].child_ids.resize(3);
  initial_state.nodes[0].child_ids[0] = 2;
  initial_state.nodes[0].child_ids[1] = 3;
  initial_state.nodes[0].child_ids[2] = 4;
  initial_state.nodes[1].id = 2;
  initial_state.nodes[2].id = 3;
  initial_state.nodes[2].AddState(ax::mojom::State::kIgnored);
  initial_state.nodes[2].child_ids.resize(2);
  initial_state.nodes[2].child_ids[0] = 5;
  initial_state.nodes[2].child_ids[1] = 6;
  initial_state.nodes[3].id = 4;
  initial_state.nodes[4].id = 5;
  initial_state.nodes[5].id = 6;
  AXTree tree(initial_state);

  // Index in parent correct.
  EXPECT_EQ(0U, tree.GetFromId(2)->GetUnignoredIndexInParent());
  EXPECT_EQ(1U, tree.GetFromId(5)->GetUnignoredIndexInParent());
  EXPECT_EQ(2U, tree.GetFromId(6)->GetUnignoredIndexInParent());
  EXPECT_EQ(3U, tree.GetFromId(4)->GetUnignoredIndexInParent());

  // Perform an update where we reorder children to [ 3i 2 4 ]. The
  // unignored index in parent for the children of the ignored node (3) should
  // be updated.
  AXTreeUpdate update;
  update.root_id = 1;
  update.nodes.resize(6);
  update.nodes[0].id = 1;
  update.nodes[0].child_ids.resize(3);
  update.nodes[0].child_ids[0] = 3;
  update.nodes[0].child_ids[1] = 2;
  update.nodes[0].child_ids[2] = 4;
  update.nodes[1].id = 2;
  update.nodes[2].id = 3;
  update.nodes[2].AddState(ax::mojom::State::kIgnored);
  update.nodes[2].child_ids.resize(2);
  update.nodes[2].child_ids[0] = 5;
  update.nodes[2].child_ids[1] = 6;
  update.nodes[3].id = 4;
  update.nodes[4].id = 5;
  update.nodes[5].id = 6;

  ASSERT_TRUE(tree.Unserialize(update));

  EXPECT_EQ(2U, tree.GetFromId(2)->GetUnignoredIndexInParent());
  EXPECT_EQ(0U, tree.GetFromId(5)->GetUnignoredIndexInParent());
  EXPECT_EQ(1U, tree.GetFromId(6)->GetUnignoredIndexInParent());
  EXPECT_EQ(3U, tree.GetFromId(4)->GetUnignoredIndexInParent());
}

TEST(AXTreeTest, ImplicitAttributeDelete) {
  // This test covers the case where an AXTreeUpdate includes a node without
  // mentioning one of that node's attributes, this should cause a delete of any
  // unmentioned attribute that was previously set on the node.

  AXTreeUpdate initial_state;
  initial_state.root_id = 1;
  initial_state.nodes.resize(1);
  initial_state.nodes[0].id = 1;
  initial_state.nodes[0].SetName("Node 1 name");
  AXTree tree(initial_state);

  EXPECT_NE(tree.GetFromId(1), nullptr);
  EXPECT_EQ(
      tree.GetFromId(1)->GetStringAttribute(ax::mojom::StringAttribute::kName),
      "Node 1 name");

  // Perform a no-op update of node 1 but omit any mention of the name
  // attribute. This should delete the name attribute.
  AXTreeUpdate update;
  update.nodes.resize(1);
  update.nodes[0].id = 1;
  ASSERT_TRUE(tree.Unserialize(update));

  // Check that the name attribute is no longer present.
  EXPECT_NE(tree.GetFromId(1), nullptr);
  EXPECT_FALSE(
      tree.GetFromId(1)->HasStringAttribute(ax::mojom::StringAttribute::kName));
}

TEST(AXTreeTest, TreeObserverIsCalled) {
  AXTreeUpdate initial_state;
  initial_state.root_id = 1;
  initial_state.nodes.resize(2);
  initial_state.nodes[0].id = 1;
  initial_state.nodes[0].child_ids.push_back(2);
  initial_state.nodes[1].id = 2;

  AXTree tree(initial_state);
  AXTreeUpdate update;
  update.root_id = 3;
  update.node_id_to_clear = 1;
  update.nodes.resize(2);
  update.nodes[0].id = 3;
  update.nodes[0].child_ids.push_back(4);
  update.nodes[1].id = 4;

  TestAXTreeObserver test_observer(&tree);
  ASSERT_TRUE(tree.Unserialize(update));

  ASSERT_EQ(2U, test_observer.deleted_ids().size());
  EXPECT_EQ(1, test_observer.deleted_ids()[0]);
  EXPECT_EQ(2, test_observer.deleted_ids()[1]);

  ASSERT_EQ(1U, test_observer.subtree_deleted_ids().size());
  EXPECT_EQ(1, test_observer.subtree_deleted_ids()[0]);

  ASSERT_EQ(2U, test_observer.created_ids().size());
  EXPECT_EQ(3, test_observer.created_ids()[0]);
  EXPECT_EQ(4, test_observer.created_ids()[1]);

  ASSERT_EQ(1U, test_observer.subtree_creation_finished_ids().size());
  EXPECT_EQ(3, test_observer.subtree_creation_finished_ids()[0]);

  ASSERT_EQ(1U, test_observer.node_creation_finished_ids().size());
  EXPECT_EQ(4, test_observer.node_creation_finished_ids()[0]);

  ASSERT_TRUE(test_observer.root_changed());
}

TEST(AXTreeTest, TreeObserverIsCalledForTreeDataChanges) {
  AXTreeUpdate initial_state;
  initial_state.root_id = 1;
  initial_state.nodes.resize(1);
  initial_state.nodes[0].id = 1;
  initial_state.has_tree_data = true;
  initial_state.tree_data.title = "Initial";
  AXTree tree(initial_state);

  TestAXTreeObserver test_observer(&tree);

  // An empty update shouldn't change tree data.
  AXTreeUpdate empty_update;
  EXPECT_TRUE(tree.Unserialize(empty_update));
  EXPECT_FALSE(test_observer.tree_data_changed());
  EXPECT_EQ("Initial", tree.data().title);

  // An update with tree data shouldn't change tree data if
  // |has_tree_data| isn't set.
  AXTreeUpdate ignored_tree_data_update;
  ignored_tree_data_update.tree_data.title = "Ignore Me";
  EXPECT_TRUE(tree.Unserialize(ignored_tree_data_update));
  EXPECT_FALSE(test_observer.tree_data_changed());
  EXPECT_EQ("Initial", tree.data().title);

  // An update with |has_tree_data| set should update the tree data.
  AXTreeUpdate tree_data_update;
  tree_data_update.has_tree_data = true;
  tree_data_update.tree_data.title = "New Title";
  EXPECT_TRUE(tree.Unserialize(tree_data_update));
  EXPECT_TRUE(test_observer.tree_data_changed());
  EXPECT_EQ("New Title", tree.data().title);
}

TEST(AXTreeTest, ReparentingDoesNotTriggerNodeCreated) {
  AXTreeUpdate initial_state;
  initial_state.root_id = 1;
  initial_state.nodes.resize(3);
  initial_state.nodes[0].id = 1;
  initial_state.nodes[0].child_ids.push_back(2);
  initial_state.nodes[1].id = 2;
  initial_state.nodes[1].child_ids.push_back(3);
  initial_state.nodes[2].id = 3;

  AXTree tree(initial_state);
  TestAXTreeObserver test_observer(&tree);

  AXTreeUpdate update;
  update.nodes.resize(2);
  update.node_id_to_clear = 2;
  update.root_id = 1;
  update.nodes[0].id = 1;
  update.nodes[0].child_ids.push_back(3);
  update.nodes[1].id = 3;
  EXPECT_TRUE(tree.Unserialize(update)) << tree.error();
  std::vector<int> created = test_observer.node_creation_finished_ids();
  std::vector<int> subtree_reparented =
      test_observer.subtree_reparented_finished_ids();
  std::vector<int> node_reparented =
      test_observer.node_reparented_finished_ids();
  ASSERT_FALSE(base::Contains(created, 3));
  ASSERT_TRUE(base::Contains(subtree_reparented, 3));
  ASSERT_FALSE(base::Contains(node_reparented, 3));
}

TEST(AXTreeTest, MultipleIgnoredChangesDoesNotBreakCache) {
  AXTreeUpdate initial_state;
  initial_state.root_id = 1;
  initial_state.nodes.resize(3);
  initial_state.nodes[0].id = 1;
  initial_state.nodes[0].child_ids.push_back(2);

  initial_state.nodes[1].id = 2;
  initial_state.nodes[1].AddState(ax::mojom::State::kIgnored);
  initial_state.nodes[1].child_ids.push_back(3);

  initial_state.nodes[2].id = 3;

  AXTree tree(initial_state);
  TestAXTreeObserver test_observer(&tree);
  EXPECT_EQ(1u, tree.GetFromId(2)->GetUnignoredChildCount());

  AXTreeUpdate update;
  update.nodes.resize(2);
  update.nodes[0].id = 3;
  update.nodes[0].AddState(ax::mojom::State::kIgnored);

  update.nodes[1].id = 2;
  update.nodes[1].child_ids.push_back(3);

  EXPECT_TRUE(tree.Unserialize(update)) << tree.error();
  EXPECT_EQ(0u, tree.GetFromId(2)->GetUnignoredChildCount());
  EXPECT_FALSE(tree.GetFromId(2)->data().HasState(ax::mojom::State::kIgnored));
  EXPECT_TRUE(tree.GetFromId(3)->data().HasState(ax::mojom::State::kIgnored));
}

TEST(AXTreeTest, NodeToClearUpdatesParentUnignoredCount) {
  AXTreeUpdate initial_state;
  initial_state.root_id = 1;
  initial_state.nodes.resize(4);
  initial_state.nodes[0].id = 1;
  initial_state.nodes[0].child_ids.push_back(2);
  initial_state.nodes[1].id = 2;
  initial_state.nodes[1].AddState(ax::mojom::State::kIgnored);
  initial_state.nodes[1].child_ids.push_back(3);
  initial_state.nodes[1].child_ids.push_back(4);
  initial_state.nodes[2].id = 3;
  initial_state.nodes[3].id = 4;

  AXTree tree(initial_state);
  EXPECT_EQ(2u, tree.GetFromId(1)->GetUnignoredChildCount());
  EXPECT_EQ(2u, tree.GetFromId(2)->GetUnignoredChildCount());

  AXTreeUpdate update;
  update.nodes.resize(1);
  update.node_id_to_clear = 2;
  update.root_id = 1;
  update.nodes[0] = initial_state.nodes[1];
  update.nodes[0].state = 0;
  update.nodes[0].child_ids.resize(0);
  EXPECT_TRUE(tree.Unserialize(update)) << tree.error();

  EXPECT_EQ(1u, tree.GetFromId(1)->GetUnignoredChildCount());
}

TEST(AXTreeTest, TreeObserverIsNotCalledForReparenting) {
  AXTreeUpdate initial_state;
  initial_state.root_id = 1;
  initial_state.nodes.resize(2);
  initial_state.nodes[0].id = 1;
  initial_state.nodes[0].child_ids.push_back(2);
  initial_state.nodes[1].id = 2;

  AXTree tree(initial_state);
  AXTreeUpdate update;
  update.node_id_to_clear = 1;
  update.root_id = 2;
  update.nodes.resize(2);
  update.nodes[0].id = 2;
  update.nodes[0].child_ids.push_back(4);
  update.nodes[1].id = 4;

  TestAXTreeObserver test_observer(&tree);

  EXPECT_TRUE(tree.Unserialize(update));

  ASSERT_EQ(1U, test_observer.deleted_ids().size());
  EXPECT_EQ(1, test_observer.deleted_ids()[0]);

  ASSERT_EQ(1U, test_observer.subtree_deleted_ids().size());
  EXPECT_EQ(1, test_observer.subtree_deleted_ids()[0]);

  ASSERT_EQ(1U, test_observer.created_ids().size());
  EXPECT_EQ(4, test_observer.created_ids()[0]);

  ASSERT_EQ(1U, test_observer.subtree_creation_finished_ids().size());
  EXPECT_EQ(4, test_observer.subtree_creation_finished_ids()[0]);

  ASSERT_EQ(1U, test_observer.subtree_reparented_finished_ids().size());
  EXPECT_EQ(2, test_observer.subtree_reparented_finished_ids()[0]);

  EXPECT_EQ(0U, test_observer.node_creation_finished_ids().size());
  EXPECT_EQ(0U, test_observer.node_reparented_finished_ids().size());

  ASSERT_TRUE(test_observer.root_changed());
}

// UAF caught by ax_tree_fuzzer
TEST(AXTreeTest, BogusAXTree) {
  AXTreeUpdate initial_state;
  AXNodeData node;
  node.id = 0;
  initial_state.nodes.push_back(node);
  initial_state.nodes.push_back(node);
  ui::AXTree tree;
  tree.Unserialize(initial_state);
}

// UAF caught by ax_tree_fuzzer
TEST(AXTreeTest, BogusAXTree2) {
  AXTreeUpdate initial_state;
  AXNodeData node;
  node.id = 0;
  initial_state.nodes.push_back(node);
  AXNodeData node2;
  node2.id = 0;
  node2.child_ids.push_back(0);
  node2.child_ids.push_back(0);
  initial_state.nodes.push_back(node2);
  ui::AXTree tree;
  tree.Unserialize(initial_state);
}

// UAF caught by ax_tree_fuzzer
TEST(AXTreeTest, BogusAXTree3) {
  AXTreeUpdate initial_state;
  AXNodeData node;
  node.id = 0;
  node.child_ids.push_back(1);
  initial_state.nodes.push_back(node);

  AXNodeData node2;
  node2.id = 1;
  node2.child_ids.push_back(1);
  node2.child_ids.push_back(1);
  initial_state.nodes.push_back(node2);

  ui::AXTree tree;
  tree.Unserialize(initial_state);
}

TEST(AXTreeTest, RoleAndStateChangeCallbacks) {
  AXTreeUpdate initial_state;
  initial_state.root_id = 1;
  initial_state.nodes.resize(1);
  initial_state.nodes[0].id = 1;
  initial_state.nodes[0].role = ax::mojom::Role::kButton;
  initial_state.nodes[0].SetCheckedState(ax::mojom::CheckedState::kTrue);
  initial_state.nodes[0].AddState(ax::mojom::State::kFocusable);
  AXTree tree(initial_state);

  TestAXTreeObserver test_observer(&tree);

  // Change the role and state.
  AXTreeUpdate update;
  update.root_id = 1;
  update.nodes.resize(1);
  update.nodes[0].id = 1;
  update.nodes[0].role = ax::mojom::Role::kCheckBox;
  update.nodes[0].SetCheckedState(ax::mojom::CheckedState::kFalse);
  update.nodes[0].AddState(ax::mojom::State::kFocusable);
  update.nodes[0].AddState(ax::mojom::State::kVisited);
  EXPECT_TRUE(tree.Unserialize(update));

  const std::vector<std::string>& change_log =
      test_observer.attribute_change_log();
  ASSERT_EQ(3U, change_log.size());
  EXPECT_EQ("Role changed from button to checkBox", change_log[0]);
  EXPECT_EQ("visited changed to true", change_log[1]);
  EXPECT_EQ("checkedState changed from 2 to 1", change_log[2]);
}

TEST(AXTreeTest, AttributeChangeCallbacks) {
  AXTreeUpdate initial_state;
  initial_state.root_id = 1;
  initial_state.nodes.resize(1);
  initial_state.nodes[0].id = 1;
  initial_state.nodes[0].AddStringAttribute(ax::mojom::StringAttribute::kName,
                                            "N1");
  initial_state.nodes[0].AddStringAttribute(
      ax::mojom::StringAttribute::kDescription, "D1");
  initial_state.nodes[0].AddBoolAttribute(ax::mojom::BoolAttribute::kLiveAtomic,
                                          true);
  initial_state.nodes[0].AddBoolAttribute(ax::mojom::BoolAttribute::kBusy,
                                          false);
  initial_state.nodes[0].AddFloatAttribute(
      ax::mojom::FloatAttribute::kMinValueForRange, 1.0);
  initial_state.nodes[0].AddFloatAttribute(
      ax::mojom::FloatAttribute::kMaxValueForRange, 10.0);
  initial_state.nodes[0].AddFloatAttribute(
      ax::mojom::FloatAttribute::kStepValueForRange, 3.0);
  initial_state.nodes[0].AddIntAttribute(ax::mojom::IntAttribute::kScrollX, 5);
  initial_state.nodes[0].AddIntAttribute(ax::mojom::IntAttribute::kScrollXMin,
                                         1);
  AXTree tree(initial_state);

  TestAXTreeObserver test_observer(&tree);

  // Change existing attributes.
  AXTreeUpdate update0;
  update0.root_id = 1;
  update0.nodes.resize(1);
  update0.nodes[0].id = 1;
  update0.nodes[0].AddStringAttribute(ax::mojom::StringAttribute::kName, "N2");
  update0.nodes[0].AddStringAttribute(ax::mojom::StringAttribute::kDescription,
                                      "D2");
  update0.nodes[0].AddBoolAttribute(ax::mojom::BoolAttribute::kLiveAtomic,
                                    false);
  update0.nodes[0].AddBoolAttribute(ax::mojom::BoolAttribute::kBusy, true);
  update0.nodes[0].AddFloatAttribute(
      ax::mojom::FloatAttribute::kMinValueForRange, 2.0);
  update0.nodes[0].AddFloatAttribute(
      ax::mojom::FloatAttribute::kMaxValueForRange, 9.0);
  update0.nodes[0].AddFloatAttribute(
      ax::mojom::FloatAttribute::kStepValueForRange, 0.5);
  update0.nodes[0].AddIntAttribute(ax::mojom::IntAttribute::kScrollX, 6);
  update0.nodes[0].AddIntAttribute(ax::mojom::IntAttribute::kScrollXMin, 2);
  EXPECT_TRUE(tree.Unserialize(update0));

  const std::vector<std::string>& change_log =
      test_observer.attribute_change_log();
  ASSERT_EQ(9U, change_log.size());
  EXPECT_EQ("name changed from N1 to N2", change_log[0]);
  EXPECT_EQ("description changed from D1 to D2", change_log[1]);
  EXPECT_EQ("liveAtomic changed to false", change_log[2]);
  EXPECT_EQ("busy changed to true", change_log[3]);
  EXPECT_EQ("minValueForRange changed from 1.0 to 2.0", change_log[4]);
  EXPECT_EQ("maxValueForRange changed from 10.0 to 9.0", change_log[5]);
  EXPECT_EQ("stepValueForRange changed from 3.0 to 0.5", change_log[6]);
  EXPECT_EQ("scrollX changed from 5 to 6", change_log[7]);
  EXPECT_EQ("scrollXMin changed from 1 to 2", change_log[8]);

  TestAXTreeObserver test_observer2(&tree);

  // Add and remove attributes.
  AXTreeUpdate update1;
  update1.root_id = 1;
  update1.nodes.resize(1);
  update1.nodes[0].id = 1;
  update1.nodes[0].AddStringAttribute(ax::mojom::StringAttribute::kDescription,
                                      "D3");
  update1.nodes[0].AddStringAttribute(ax::mojom::StringAttribute::kValue, "V3");
  update1.nodes[0].AddBoolAttribute(ax::mojom::BoolAttribute::kModal, true);
  update1.nodes[0].AddFloatAttribute(ax::mojom::FloatAttribute::kValueForRange,
                                     5.0);
  update1.nodes[0].AddFloatAttribute(
      ax::mojom::FloatAttribute::kMaxValueForRange, 9.0);
  update1.nodes[0].AddIntAttribute(ax::mojom::IntAttribute::kScrollX, 7);
  update1.nodes[0].AddIntAttribute(ax::mojom::IntAttribute::kScrollXMax, 10);
  EXPECT_TRUE(tree.Unserialize(update1));

  const std::vector<std::string>& change_log2 =
      test_observer2.attribute_change_log();
  ASSERT_EQ(11U, change_log2.size());
  EXPECT_EQ("name changed from N2 to ", change_log2[0]);
  EXPECT_EQ("description changed from D2 to D3", change_log2[1]);
  EXPECT_EQ("value changed from  to V3", change_log2[2]);
  EXPECT_EQ("busy changed to false", change_log2[3]);
  EXPECT_EQ("modal changed to true", change_log2[4]);
  EXPECT_EQ("minValueForRange changed from 2.0 to 0.0", change_log2[5]);
  EXPECT_EQ("stepValueForRange changed from 3.0 to 0.5", change_log[6]);
  EXPECT_EQ("valueForRange changed from 0.0 to 5.0", change_log2[7]);
  EXPECT_EQ("scrollXMin changed from 2 to 0", change_log2[8]);
  EXPECT_EQ("scrollX changed from 6 to 7", change_log2[9]);
  EXPECT_EQ("scrollXMax changed from 0 to 10", change_log2[10]);
}

TEST(AXTreeTest, IntListChangeCallbacks) {
  std::vector<int32_t> one;
  one.push_back(1);

  std::vector<int32_t> two;
  two.push_back(2);
  two.push_back(2);

  std::vector<int32_t> three;
  three.push_back(3);

  AXTreeUpdate initial_state;
  initial_state.root_id = 1;
  initial_state.nodes.resize(1);
  initial_state.nodes[0].id = 1;
  initial_state.nodes[0].AddIntListAttribute(
      ax::mojom::IntListAttribute::kControlsIds, one);
  initial_state.nodes[0].AddIntListAttribute(
      ax::mojom::IntListAttribute::kRadioGroupIds, two);
  AXTree tree(initial_state);

  TestAXTreeObserver test_observer(&tree);

  // Change existing attributes.
  AXTreeUpdate update0;
  update0.root_id = 1;
  update0.nodes.resize(1);
  update0.nodes[0].id = 1;
  update0.nodes[0].AddIntListAttribute(
      ax::mojom::IntListAttribute::kControlsIds, two);
  update0.nodes[0].AddIntListAttribute(
      ax::mojom::IntListAttribute::kRadioGroupIds, three);
  EXPECT_TRUE(tree.Unserialize(update0));

  const std::vector<std::string>& change_log =
      test_observer.attribute_change_log();
  ASSERT_EQ(2U, change_log.size());
  EXPECT_EQ("controlsIds changed from 1 to 2,2", change_log[0]);
  EXPECT_EQ("radioGroupIds changed from 2,2 to 3", change_log[1]);

  TestAXTreeObserver test_observer2(&tree);

  // Add and remove attributes.
  AXTreeUpdate update1;
  update1.root_id = 1;
  update1.nodes.resize(1);
  update1.nodes[0].id = 1;
  update1.nodes[0].AddIntListAttribute(
      ax::mojom::IntListAttribute::kRadioGroupIds, two);
  update1.nodes[0].AddIntListAttribute(ax::mojom::IntListAttribute::kFlowtoIds,
                                       three);
  EXPECT_TRUE(tree.Unserialize(update1));

  const std::vector<std::string>& change_log2 =
      test_observer2.attribute_change_log();
  ASSERT_EQ(3U, change_log2.size());
  EXPECT_EQ("controlsIds changed from 2,2 to ", change_log2[0]);
  EXPECT_EQ("radioGroupIds changed from 3 to 2,2", change_log2[1]);
  EXPECT_EQ("flowtoIds changed from  to 3", change_log2[2]);
}

// Create a very simple tree and make sure that we can get the bounds of
// any node.
TEST(AXTreeTest, GetBoundsBasic) {
  AXTreeUpdate tree_update;
  tree_update.root_id = 1;
  tree_update.nodes.resize(2);
  tree_update.nodes[0].id = 1;
  tree_update.nodes[0].relative_bounds.bounds = gfx::RectF(0, 0, 800, 600);
  tree_update.nodes[0].child_ids.push_back(2);
  tree_update.nodes[1].id = 2;
  tree_update.nodes[1].relative_bounds.bounds = gfx::RectF(100, 10, 400, 300);
  AXTree tree(tree_update);

  EXPECT_EQ("(0, 0) size (800 x 600)", GetBoundsAsString(tree, 1));
  EXPECT_EQ("(100, 10) size (400 x 300)", GetBoundsAsString(tree, 2));
}

// If a node doesn't specify its location but at least one child does have
// a location, its computed bounds should be the union of all child bounds.
TEST(AXTreeTest, EmptyNodeBoundsIsUnionOfChildren) {
  AXTreeUpdate tree_update;
  tree_update.root_id = 1;
  tree_update.nodes.resize(4);
  tree_update.nodes[0].id = 1;
  tree_update.nodes[0].relative_bounds.bounds = gfx::RectF(0, 0, 800, 600);
  tree_update.nodes[0].child_ids.push_back(2);
  tree_update.nodes[1].id = 2;
  tree_update.nodes[1].relative_bounds.bounds =
      gfx::RectF();  // Deliberately empty.
  tree_update.nodes[1].child_ids.push_back(3);
  tree_update.nodes[1].child_ids.push_back(4);
  tree_update.nodes[2].id = 3;
  tree_update.nodes[2].relative_bounds.bounds = gfx::RectF(100, 10, 400, 20);
  tree_update.nodes[3].id = 4;
  tree_update.nodes[3].relative_bounds.bounds = gfx::RectF(200, 30, 400, 20);

  AXTree tree(tree_update);
  EXPECT_EQ("(100, 10) size (500 x 40)", GetBoundsAsString(tree, 2));
}

// If a node doesn't specify its location but at least one child does have
// a location, it will be offscreen if all of its children are offscreen.
TEST(AXTreeTest, EmptyNodeNotOffscreenEvenIfAllChildrenOffscreen) {
  AXTreeUpdate tree_update;
  tree_update.root_id = 1;
  tree_update.nodes.resize(4);
  tree_update.nodes[0].id = 1;
  tree_update.nodes[0].relative_bounds.bounds = gfx::RectF(0, 0, 800, 600);
  tree_update.nodes[0].role = ax::mojom::Role::kRootWebArea;
  tree_update.nodes[0].AddBoolAttribute(
      ax::mojom::BoolAttribute::kClipsChildren, true);
  tree_update.nodes[0].child_ids.push_back(2);
  tree_update.nodes[1].id = 2;
  tree_update.nodes[1].relative_bounds.bounds =
      gfx::RectF();  // Deliberately empty.
  tree_update.nodes[1].child_ids.push_back(3);
  tree_update.nodes[1].child_ids.push_back(4);
  // Both children are offscreen
  tree_update.nodes[2].id = 3;
  tree_update.nodes[2].relative_bounds.bounds = gfx::RectF(900, 10, 400, 20);
  tree_update.nodes[3].id = 4;
  tree_update.nodes[3].relative_bounds.bounds = gfx::RectF(1000, 30, 400, 20);

  AXTree tree(tree_update);
  EXPECT_FALSE(IsNodeOffscreen(tree, 2));
  EXPECT_TRUE(IsNodeOffscreen(tree, 3));
  EXPECT_TRUE(IsNodeOffscreen(tree, 4));
}

// Test that getting the bounds of a node works when there's a transform.
TEST(AXTreeTest, GetBoundsWithTransform) {
  AXTreeUpdate tree_update;
  tree_update.root_id = 1;
  tree_update.nodes.resize(3);
  tree_update.nodes[0].id = 1;
  tree_update.nodes[0].relative_bounds.bounds = gfx::RectF(0, 0, 400, 300);
  tree_update.nodes[0].relative_bounds.transform =
      std::make_unique<gfx::Transform>();
  tree_update.nodes[0].relative_bounds.transform->Scale(2.0, 2.0);
  tree_update.nodes[0].child_ids.push_back(2);
  tree_update.nodes[0].child_ids.push_back(3);
  tree_update.nodes[1].id = 2;
  tree_update.nodes[1].relative_bounds.bounds = gfx::RectF(20, 10, 50, 5);
  tree_update.nodes[2].id = 3;
  tree_update.nodes[2].relative_bounds.bounds = gfx::RectF(20, 30, 50, 5);
  tree_update.nodes[2].relative_bounds.transform =
      std::make_unique<gfx::Transform>();
  tree_update.nodes[2].relative_bounds.transform->Scale(2.0, 2.0);

  AXTree tree(tree_update);
  EXPECT_EQ("(0, 0) size (800 x 600)", GetBoundsAsString(tree, 1));
  EXPECT_EQ("(40, 20) size (100 x 10)", GetBoundsAsString(tree, 2));
  EXPECT_EQ("(80, 120) size (200 x 20)", GetBoundsAsString(tree, 3));
}

// Test that getting the bounds of a node that's inside a container
// works correctly.
TEST(AXTreeTest, GetBoundsWithContainerId) {
  AXTreeUpdate tree_update;
  tree_update.root_id = 1;
  tree_update.nodes.resize(4);
  tree_update.nodes[0].id = 1;
  tree_update.nodes[0].relative_bounds.bounds = gfx::RectF(0, 0, 800, 600);
  tree_update.nodes[0].child_ids.push_back(2);
  tree_update.nodes[1].id = 2;
  tree_update.nodes[1].relative_bounds.bounds = gfx::RectF(100, 50, 600, 500);
  tree_update.nodes[1].child_ids.push_back(3);
  tree_update.nodes[1].child_ids.push_back(4);
  tree_update.nodes[2].id = 3;
  tree_update.nodes[2].relative_bounds.offset_container_id = 2;
  tree_update.nodes[2].relative_bounds.bounds = gfx::RectF(20, 30, 50, 5);
  tree_update.nodes[3].id = 4;
  tree_update.nodes[3].relative_bounds.bounds = gfx::RectF(20, 30, 50, 5);

  AXTree tree(tree_update);
  EXPECT_EQ("(120, 80) size (50 x 5)", GetBoundsAsString(tree, 3));
  EXPECT_EQ("(20, 30) size (50 x 5)", GetBoundsAsString(tree, 4));
}

// Test that getting the bounds of a node that's inside a scrolling container
// works correctly.
TEST(AXTreeTest, GetBoundsWithScrolling) {
  AXTreeUpdate tree_update;
  tree_update.root_id = 1;
  tree_update.nodes.resize(3);
  tree_update.nodes[0].id = 1;
  tree_update.nodes[0].relative_bounds.bounds = gfx::RectF(0, 0, 800, 600);
  tree_update.nodes[0].child_ids.push_back(2);
  tree_update.nodes[1].id = 2;
  tree_update.nodes[1].relative_bounds.bounds = gfx::RectF(100, 50, 600, 500);
  tree_update.nodes[1].AddIntAttribute(ax::mojom::IntAttribute::kScrollX, 5);
  tree_update.nodes[1].AddIntAttribute(ax::mojom::IntAttribute::kScrollY, 10);
  tree_update.nodes[1].child_ids.push_back(3);
  tree_update.nodes[2].id = 3;
  tree_update.nodes[2].relative_bounds.offset_container_id = 2;
  tree_update.nodes[2].relative_bounds.bounds = gfx::RectF(20, 30, 50, 5);

  AXTree tree(tree_update);
  EXPECT_EQ("(115, 70) size (50 x 5)", GetBoundsAsString(tree, 3));
}

// When a node has zero size, we try to get the bounds from an ancestor.
TEST(AXTreeTest, GetBoundsOfNodeWithZeroSize) {
  AXTreeUpdate tree_update;
  tree_update.root_id = 1;
  tree_update.nodes.resize(5);
  tree_update.nodes[0].id = 1;
  tree_update.nodes[0].relative_bounds.bounds = gfx::RectF(0, 0, 800, 600);
  tree_update.nodes[0].child_ids = {2};
  tree_update.nodes[1].id = 2;
  tree_update.nodes[1].relative_bounds.bounds = gfx::RectF(100, 100, 300, 200);
  tree_update.nodes[1].child_ids = {3, 4, 5};

  // This child has relative coordinates and no offset and no size.
  tree_update.nodes[2].id = 3;
  tree_update.nodes[2].relative_bounds.offset_container_id = 2;
  tree_update.nodes[2].relative_bounds.bounds = gfx::RectF(0, 0, 0, 0);

  // This child has relative coordinates and an offset, but no size.
  tree_update.nodes[3].id = 4;
  tree_update.nodes[3].relative_bounds.offset_container_id = 2;
  tree_update.nodes[3].relative_bounds.bounds = gfx::RectF(20, 20, 0, 0);

  // This child has absolute coordinates, an offset, and no size.
  tree_update.nodes[4].id = 5;
  tree_update.nodes[4].relative_bounds.bounds = gfx::RectF(120, 120, 0, 0);

  AXTree tree(tree_update);
  EXPECT_EQ("(100, 100) size (300 x 200)", GetBoundsAsString(tree, 3));
  EXPECT_EQ("(120, 120) size (280 x 180)", GetBoundsAsString(tree, 4));
  EXPECT_EQ("(120, 120) size (280 x 180)", GetBoundsAsString(tree, 5));
}

TEST(AXTreeTest, GetBoundsEmptyBoundsInheritsFromParent) {
  AXTreeUpdate tree_update;
  tree_update.root_id = 1;
  tree_update.nodes.resize(3);
  tree_update.nodes[0].id = 1;
  tree_update.nodes[0].relative_bounds.bounds = gfx::RectF(0, 0, 800, 600);
  tree_update.nodes[1].AddBoolAttribute(
      ax::mojom::BoolAttribute::kClipsChildren, true);
  tree_update.nodes[0].child_ids.push_back(2);
  tree_update.nodes[1].id = 2;
  tree_update.nodes[1].relative_bounds.bounds = gfx::RectF(300, 200, 100, 100);
  tree_update.nodes[1].child_ids.push_back(3);
  tree_update.nodes[2].id = 3;
  tree_update.nodes[2].relative_bounds.bounds = gfx::RectF();

  AXTree tree(tree_update);
  EXPECT_EQ("(0, 0) size (800 x 600)", GetBoundsAsString(tree, 1));
  EXPECT_EQ("(300, 200) size (100 x 100)", GetBoundsAsString(tree, 2));
  EXPECT_EQ("(300, 200) size (100 x 100)", GetBoundsAsString(tree, 3));
  EXPECT_EQ("(0, 0) size (800 x 600)", GetUnclippedBoundsAsString(tree, 1));
  EXPECT_EQ("(300, 200) size (100 x 100)", GetUnclippedBoundsAsString(tree, 2));
  EXPECT_EQ("(300, 200) size (100 x 100)", GetUnclippedBoundsAsString(tree, 3));
  EXPECT_FALSE(IsNodeOffscreen(tree, 1));
  EXPECT_FALSE(IsNodeOffscreen(tree, 2));
  EXPECT_TRUE(IsNodeOffscreen(tree, 3));
}

TEST(AXTreeTest, GetBoundsCropsChildToRoot) {
  AXTreeUpdate tree_update;
  tree_update.root_id = 1;
  tree_update.nodes.resize(5);
  tree_update.nodes[0].id = 1;
  tree_update.nodes[0].relative_bounds.bounds = gfx::RectF(0, 0, 800, 600);
  tree_update.nodes[0].AddBoolAttribute(
      ax::mojom::BoolAttribute::kClipsChildren, true);
  tree_update.nodes[0].child_ids.push_back(2);
  tree_update.nodes[0].child_ids.push_back(3);
  tree_update.nodes[0].child_ids.push_back(4);
  tree_update.nodes[0].child_ids.push_back(5);
  // Cropped in the top left
  tree_update.nodes[1].id = 2;
  tree_update.nodes[1].relative_bounds.bounds =
      gfx::RectF(-100, -100, 150, 150);
  // Cropped in the bottom right
  tree_update.nodes[2].id = 3;
  tree_update.nodes[2].relative_bounds.bounds = gfx::RectF(700, 500, 150, 150);
  // Offscreen on the top
  tree_update.nodes[3].id = 4;
  tree_update.nodes[3].relative_bounds.bounds = gfx::RectF(50, -200, 150, 150);
  // Offscreen on the bottom
  tree_update.nodes[4].id = 5;
  tree_update.nodes[4].relative_bounds.bounds = gfx::RectF(50, 700, 150, 150);

  AXTree tree(tree_update);
  EXPECT_EQ("(0, 0) size (50 x 50)", GetBoundsAsString(tree, 2));
  EXPECT_EQ("(700, 500) size (100 x 100)", GetBoundsAsString(tree, 3));
  EXPECT_EQ("(50, 0) size (150 x 1)", GetBoundsAsString(tree, 4));
  EXPECT_EQ("(50, 599) size (150 x 1)", GetBoundsAsString(tree, 5));

  // Check the unclipped bounds are as expected.
  EXPECT_EQ("(-100, -100) size (150 x 150)",
            GetUnclippedBoundsAsString(tree, 2));
  EXPECT_EQ("(700, 500) size (150 x 150)", GetUnclippedBoundsAsString(tree, 3));
  EXPECT_EQ("(50, -200) size (150 x 150)", GetUnclippedBoundsAsString(tree, 4));
  EXPECT_EQ("(50, 700) size (150 x 150)", GetUnclippedBoundsAsString(tree, 5));
}

TEST(AXTreeTest, GetBoundsSetsOffscreenIfClipsChildren) {
  AXTreeUpdate tree_update;
  tree_update.root_id = 1;
  tree_update.nodes.resize(5);
  tree_update.nodes[0].id = 1;
  tree_update.nodes[0].relative_bounds.bounds = gfx::RectF(0, 0, 800, 600);
  tree_update.nodes[0].AddBoolAttribute(
      ax::mojom::BoolAttribute::kClipsChildren, true);
  tree_update.nodes[0].child_ids.push_back(2);
  tree_update.nodes[0].child_ids.push_back(3);

  tree_update.nodes[1].id = 2;
  tree_update.nodes[1].relative_bounds.bounds = gfx::RectF(0, 0, 200, 200);
  tree_update.nodes[1].AddBoolAttribute(
      ax::mojom::BoolAttribute::kClipsChildren, true);
  tree_update.nodes[1].child_ids.push_back(4);

  tree_update.nodes[2].id = 3;
  tree_update.nodes[2].relative_bounds.bounds = gfx::RectF(0, 0, 200, 200);
  tree_update.nodes[2].child_ids.push_back(5);

  // Clipped by its parent
  tree_update.nodes[3].id = 4;
  tree_update.nodes[3].relative_bounds.bounds = gfx::RectF(250, 250, 100, 100);
  tree_update.nodes[3].relative_bounds.offset_container_id = 2;

  // Outside of its parent, but its parent does not clip children,
  // so it should not be offscreen.
  tree_update.nodes[4].id = 5;
  tree_update.nodes[4].relative_bounds.bounds = gfx::RectF(250, 250, 100, 100);
  tree_update.nodes[4].relative_bounds.offset_container_id = 3;

  AXTree tree(tree_update);
  EXPECT_TRUE(IsNodeOffscreen(tree, 4));
  EXPECT_FALSE(IsNodeOffscreen(tree, 5));
}

TEST(AXTreeTest, GetBoundsUpdatesOffscreen) {
  AXTreeUpdate tree_update;
  tree_update.root_id = 1;
  tree_update.nodes.resize(5);
  tree_update.nodes[0].id = 1;
  tree_update.nodes[0].relative_bounds.bounds = gfx::RectF(0, 0, 800, 600);
  tree_update.nodes[0].role = ax::mojom::Role::kRootWebArea;
  tree_update.nodes[0].AddBoolAttribute(
      ax::mojom::BoolAttribute::kClipsChildren, true);
  tree_update.nodes[0].child_ids.push_back(2);
  tree_update.nodes[0].child_ids.push_back(3);
  tree_update.nodes[0].child_ids.push_back(4);
  tree_update.nodes[0].child_ids.push_back(5);
  // Fully onscreen
  tree_update.nodes[1].id = 2;
  tree_update.nodes[1].relative_bounds.bounds = gfx::RectF(10, 10, 150, 150);
  // Cropped in the bottom right
  tree_update.nodes[2].id = 3;
  tree_update.nodes[2].relative_bounds.bounds = gfx::RectF(700, 500, 150, 150);
  // Offscreen on the top
  tree_update.nodes[3].id = 4;
  tree_update.nodes[3].relative_bounds.bounds = gfx::RectF(50, -200, 150, 150);
  // Offscreen on the bottom
  tree_update.nodes[4].id = 5;
  tree_update.nodes[4].relative_bounds.bounds = gfx::RectF(50, 700, 150, 150);

  AXTree tree(tree_update);
  EXPECT_FALSE(IsNodeOffscreen(tree, 2));
  EXPECT_FALSE(IsNodeOffscreen(tree, 3));
  EXPECT_TRUE(IsNodeOffscreen(tree, 4));
  EXPECT_TRUE(IsNodeOffscreen(tree, 5));
}

TEST(AXTreeTest, IntReverseRelations) {
  AXTreeUpdate initial_state;
  initial_state.root_id = 1;
  initial_state.nodes.resize(4);
  initial_state.nodes[0].id = 1;
  initial_state.nodes[0].AddIntAttribute(
      ax::mojom::IntAttribute::kActivedescendantId, 2);
  initial_state.nodes[0].child_ids.push_back(2);
  initial_state.nodes[0].child_ids.push_back(3);
  initial_state.nodes[0].child_ids.push_back(4);
  initial_state.nodes[1].id = 2;
  initial_state.nodes[2].id = 3;
  initial_state.nodes[2].AddIntAttribute(ax::mojom::IntAttribute::kMemberOfId,
                                         1);
  initial_state.nodes[3].id = 4;
  initial_state.nodes[3].AddIntAttribute(ax::mojom::IntAttribute::kMemberOfId,
                                         1);
  AXTree tree(initial_state);

  auto reverse_active_descendant =
      tree.GetReverseRelations(ax::mojom::IntAttribute::kActivedescendantId, 2);
  ASSERT_EQ(1U, reverse_active_descendant.size());
  EXPECT_TRUE(base::Contains(reverse_active_descendant, 1));

  reverse_active_descendant =
      tree.GetReverseRelations(ax::mojom::IntAttribute::kActivedescendantId, 1);
  ASSERT_EQ(0U, reverse_active_descendant.size());

  auto reverse_errormessage =
      tree.GetReverseRelations(ax::mojom::IntAttribute::kErrormessageId, 1);
  ASSERT_EQ(0U, reverse_errormessage.size());

  auto reverse_member_of =
      tree.GetReverseRelations(ax::mojom::IntAttribute::kMemberOfId, 1);
  ASSERT_EQ(2U, reverse_member_of.size());
  EXPECT_TRUE(base::Contains(reverse_member_of, 3));
  EXPECT_TRUE(base::Contains(reverse_member_of, 4));

  AXTreeUpdate update = initial_state;
  update.nodes.resize(5);
  update.nodes[0].int_attributes.clear();
  update.nodes[0].AddIntAttribute(ax::mojom::IntAttribute::kActivedescendantId,
                                  5);
  update.nodes[0].child_ids.push_back(5);
  update.nodes[2].int_attributes.clear();
  update.nodes[4].id = 5;
  update.nodes[4].AddIntAttribute(ax::mojom::IntAttribute::kMemberOfId, 1);

  EXPECT_TRUE(tree.Unserialize(update));

  reverse_active_descendant =
      tree.GetReverseRelations(ax::mojom::IntAttribute::kActivedescendantId, 2);
  ASSERT_EQ(0U, reverse_active_descendant.size());

  reverse_active_descendant =
      tree.GetReverseRelations(ax::mojom::IntAttribute::kActivedescendantId, 5);
  ASSERT_EQ(1U, reverse_active_descendant.size());
  EXPECT_TRUE(base::Contains(reverse_active_descendant, 1));

  reverse_member_of =
      tree.GetReverseRelations(ax::mojom::IntAttribute::kMemberOfId, 1);
  ASSERT_EQ(2U, reverse_member_of.size());
  EXPECT_TRUE(base::Contains(reverse_member_of, 4));
  EXPECT_TRUE(base::Contains(reverse_member_of, 5));
}

TEST(AXTreeTest, IntListReverseRelations) {
  std::vector<int32_t> node_two;
  node_two.push_back(2);

  std::vector<int32_t> nodes_two_three;
  nodes_two_three.push_back(2);
  nodes_two_three.push_back(3);

  AXTreeUpdate initial_state;
  initial_state.root_id = 1;
  initial_state.nodes.resize(3);
  initial_state.nodes[0].id = 1;
  initial_state.nodes[0].AddIntListAttribute(
      ax::mojom::IntListAttribute::kLabelledbyIds, node_two);
  initial_state.nodes[0].child_ids.push_back(2);
  initial_state.nodes[0].child_ids.push_back(3);
  initial_state.nodes[1].id = 2;
  initial_state.nodes[2].id = 3;

  AXTree tree(initial_state);

  auto reverse_labelled_by =
      tree.GetReverseRelations(ax::mojom::IntListAttribute::kLabelledbyIds, 2);
  ASSERT_EQ(1U, reverse_labelled_by.size());
  EXPECT_TRUE(base::Contains(reverse_labelled_by, 1));

  reverse_labelled_by =
      tree.GetReverseRelations(ax::mojom::IntListAttribute::kLabelledbyIds, 3);
  ASSERT_EQ(0U, reverse_labelled_by.size());

  // Change existing attributes.
  AXTreeUpdate update = initial_state;
  update.nodes[0].intlist_attributes.clear();
  update.nodes[0].AddIntListAttribute(
      ax::mojom::IntListAttribute::kLabelledbyIds, nodes_two_three);
  EXPECT_TRUE(tree.Unserialize(update));

  reverse_labelled_by =
      tree.GetReverseRelations(ax::mojom::IntListAttribute::kLabelledbyIds, 3);
  ASSERT_EQ(1U, reverse_labelled_by.size());
  EXPECT_TRUE(base::Contains(reverse_labelled_by, 1));
}

TEST(AXTreeTest, DeletingNodeUpdatesReverseRelations) {
  AXTreeUpdate initial_state;
  initial_state.root_id = 1;
  initial_state.nodes.resize(3);
  initial_state.nodes[0].id = 1;
  initial_state.nodes[0].child_ids = {2, 3};
  initial_state.nodes[1].id = 2;
  initial_state.nodes[2].id = 3;
  initial_state.nodes[2].AddIntAttribute(
      ax::mojom::IntAttribute::kActivedescendantId, 2);
  AXTree tree(initial_state);

  auto reverse_active_descendant =
      tree.GetReverseRelations(ax::mojom::IntAttribute::kActivedescendantId, 2);
  ASSERT_EQ(1U, reverse_active_descendant.size());
  EXPECT_TRUE(base::Contains(reverse_active_descendant, 3));

  AXTreeUpdate update;
  update.root_id = 1;
  update.nodes.resize(1);
  update.nodes[0].id = 1;
  update.nodes[0].child_ids = {2};
  EXPECT_TRUE(tree.Unserialize(update));

  reverse_active_descendant =
      tree.GetReverseRelations(ax::mojom::IntAttribute::kActivedescendantId, 2);
  ASSERT_EQ(0U, reverse_active_descendant.size());
}

TEST(AXTreeTest, ReverseRelationsDoNotKeepGrowing) {
  // The number of total entries in int_reverse_relations and
  // intlist_reverse_relations should not keep growing as the tree
  // changes.

  AXTreeUpdate initial_state;
  initial_state.root_id = 1;
  initial_state.nodes.resize(2);
  initial_state.nodes[0].id = 1;
  initial_state.nodes[0].AddIntAttribute(
      ax::mojom::IntAttribute::kActivedescendantId, 2);
  initial_state.nodes[0].AddIntListAttribute(
      ax::mojom::IntListAttribute::kLabelledbyIds, {2});
  initial_state.nodes[0].child_ids.push_back(2);
  initial_state.nodes[1].id = 2;
  AXTree tree(initial_state);

  for (int i = 0; i < 1000; ++i) {
    AXTreeUpdate update;
    update.root_id = 1;
    update.nodes.resize(2);
    update.nodes[0].id = 1;
    update.nodes[1].id = i + 3;
    update.nodes[0].AddIntAttribute(
        ax::mojom::IntAttribute::kActivedescendantId, update.nodes[1].id);
    update.nodes[0].AddIntListAttribute(
        ax::mojom::IntListAttribute::kLabelledbyIds, {update.nodes[1].id});
    update.nodes[1].AddIntAttribute(ax::mojom::IntAttribute::kMemberOfId, 1);
    update.nodes[0].child_ids.push_back(update.nodes[1].id);
    EXPECT_TRUE(tree.Unserialize(update));
  }

  size_t map_key_count = 0;
  size_t set_entry_count = 0;
  for (auto& iter : tree.int_reverse_relations()) {
    map_key_count += iter.second.size() + 1;
    for (auto it2 = iter.second.begin(); it2 != iter.second.end(); ++it2) {
      set_entry_count += it2->second.size();
    }
  }

  // Note: 10 is arbitrary, the idea here is just that we mutated the tree
  // 1000 times, so if we have fewer than 10 entries in the maps / sets then
  // the map isn't growing / leaking. Same below.
  EXPECT_LT(map_key_count, 10U);
  EXPECT_LT(set_entry_count, 10U);

  map_key_count = 0;
  set_entry_count = 0;
  for (auto& iter : tree.intlist_reverse_relations()) {
    map_key_count += iter.second.size() + 1;
    for (auto it2 = iter.second.begin(); it2 != iter.second.end(); ++it2) {
      set_entry_count += it2->second.size();
    }
  }
  EXPECT_LT(map_key_count, 10U);
  EXPECT_LT(set_entry_count, 10U);
}

TEST(AXTreeTest, SkipIgnoredNodes) {
  AXTreeUpdate tree_update;
  tree_update.root_id = 1;
  tree_update.nodes.resize(5);
  tree_update.nodes[0].id = 1;
  tree_update.nodes[0].child_ids = {2, 3};
  tree_update.nodes[1].id = 2;
  tree_update.nodes[1].AddState(ax::mojom::State::kIgnored);
  tree_update.nodes[1].child_ids = {4, 5};
  tree_update.nodes[2].id = 3;
  tree_update.nodes[3].id = 4;
  tree_update.nodes[4].id = 5;

  AXTree tree(tree_update);
  AXNode* root = tree.root();
  ASSERT_EQ(2u, root->children().size());
  ASSERT_EQ(2, root->children()[0]->id());
  ASSERT_EQ(3, root->children()[1]->id());

  EXPECT_EQ(3u, root->GetUnignoredChildCount());
  EXPECT_EQ(4, root->GetUnignoredChildAtIndex(0)->id());
  EXPECT_EQ(5, root->GetUnignoredChildAtIndex(1)->id());
  EXPECT_EQ(3, root->GetUnignoredChildAtIndex(2)->id());
  EXPECT_EQ(0u, root->GetUnignoredChildAtIndex(0)->GetUnignoredIndexInParent());
  EXPECT_EQ(1u, root->GetUnignoredChildAtIndex(1)->GetUnignoredIndexInParent());
  EXPECT_EQ(2u, root->GetUnignoredChildAtIndex(2)->GetUnignoredIndexInParent());

  EXPECT_EQ(1, root->GetUnignoredChildAtIndex(0)->GetUnignoredParent()->id());
}

TEST(AXTreeTest, CachedUnignoredValues) {
  AXTreeUpdate initial_state;
  initial_state.root_id = 1;
  initial_state.nodes.resize(5);
  initial_state.nodes[0].id = 1;
  initial_state.nodes[0].child_ids = {2, 3};
  initial_state.nodes[1].id = 2;
  initial_state.nodes[1].AddState(ax::mojom::State::kIgnored);
  initial_state.nodes[1].child_ids = {4, 5};
  initial_state.nodes[2].id = 3;
  initial_state.nodes[3].id = 4;
  initial_state.nodes[4].id = 5;

  AXTree tree(initial_state);
  AXNode* root = tree.root();
  ASSERT_EQ(2u, root->children().size());
  ASSERT_EQ(2, root->children()[0]->id());
  ASSERT_EQ(3, root->children()[1]->id());

  EXPECT_EQ(3u, root->GetUnignoredChildCount());
  EXPECT_EQ(4, root->GetUnignoredChildAtIndex(0)->id());
  EXPECT_EQ(5, root->GetUnignoredChildAtIndex(1)->id());
  EXPECT_EQ(3, root->GetUnignoredChildAtIndex(2)->id());
  EXPECT_EQ(0u, root->GetUnignoredChildAtIndex(0)->GetUnignoredIndexInParent());
  EXPECT_EQ(1u, root->GetUnignoredChildAtIndex(1)->GetUnignoredIndexInParent());
  EXPECT_EQ(2u, root->GetUnignoredChildAtIndex(2)->GetUnignoredIndexInParent());

  EXPECT_EQ(1, root->GetUnignoredChildAtIndex(0)->GetUnignoredParent()->id());

  // Ensure when a node goes from ignored to unignored, its children have their
  // unignored_index_in_parent updated.
  AXTreeUpdate update = initial_state;
  update.nodes[1].RemoveState(ax::mojom::State::kIgnored);

  EXPECT_TRUE(tree.Unserialize(update));

  root = tree.root();
  EXPECT_EQ(2u, root->GetUnignoredChildCount());
  EXPECT_EQ(2, root->GetUnignoredChildAtIndex(0)->id());
  EXPECT_EQ(2u, tree.GetFromId(2)->GetUnignoredChildCount());
  EXPECT_EQ(0u, tree.GetFromId(4)->GetUnignoredIndexInParent());
  EXPECT_EQ(1u, tree.GetFromId(5)->GetUnignoredIndexInParent());

  // Ensure when a node goes from unignored to unignored, siblings are correctly
  // updated.
  AXTreeUpdate update2 = update;
  update2.nodes[3].AddState(ax::mojom::State::kIgnored);

  EXPECT_TRUE(tree.Unserialize(update2));

  EXPECT_EQ(1u, tree.GetFromId(2)->GetUnignoredChildCount());
  EXPECT_EQ(0u, tree.GetFromId(5)->GetUnignoredIndexInParent());

  // Ensure siblings of a deleted node are updated.
  AXTreeUpdate update3 = update2;
  update3.nodes.resize(1);
  update3.nodes[0].id = 1;
  update3.nodes[0].child_ids = {3};

  EXPECT_TRUE(tree.Unserialize(update3));

  EXPECT_EQ(1u, tree.GetFromId(1)->GetUnignoredChildCount());
  EXPECT_EQ(0u, tree.GetFromId(3)->GetUnignoredIndexInParent());

  // Ensure new nodes are correctly updated.
  AXTreeUpdate update4 = update3;
  update4.nodes.resize(3);
  update4.nodes[0].id = 1;
  update4.nodes[0].child_ids = {3, 6};
  update4.nodes[1].id = 6;
  update4.nodes[1].child_ids = {7};
  update4.nodes[2].id = 7;

  EXPECT_TRUE(tree.Unserialize(update4));

  EXPECT_EQ(2u, tree.GetFromId(1)->GetUnignoredChildCount());
  EXPECT_EQ(0u, tree.GetFromId(3)->GetUnignoredIndexInParent());
  EXPECT_EQ(1u, tree.GetFromId(6)->GetUnignoredIndexInParent());
  EXPECT_EQ(0u, tree.GetFromId(7)->GetUnignoredIndexInParent());

  // Ensure reparented nodes are correctly updated.
  AXTreeUpdate update5 = update4;
  update5.nodes.resize(2);
  update5.node_id_to_clear = 6;
  update5.nodes[0].id = 1;
  update5.nodes[0].child_ids = {3, 7};
  update5.nodes[1].id = 7;
  update5.nodes[1].child_ids = {};

  EXPECT_TRUE(tree.Unserialize(update5));

  EXPECT_EQ(2u, tree.GetFromId(1)->GetUnignoredChildCount());
  EXPECT_EQ(0u, tree.GetFromId(3)->GetUnignoredIndexInParent());
  EXPECT_EQ(1u, tree.GetFromId(7)->GetUnignoredIndexInParent());

  AXTreeUpdate update6;
  update6.nodes.resize(1);
  update6.nodes[0].id = 7;
  update6.nodes[0].AddState(ax::mojom::State::kIgnored);

  EXPECT_TRUE(tree.Unserialize(update6));

  EXPECT_EQ(1u, tree.GetFromId(1)->GetUnignoredChildCount());
  EXPECT_EQ(0u, tree.GetFromId(3)->GetUnignoredIndexInParent());

  AXTreeUpdate update7 = update6;
  update7.nodes.resize(2);
  update7.nodes[0].id = 7;
  update7.nodes[0].child_ids = {8};
  update7.nodes[1].id = 8;

  EXPECT_TRUE(tree.Unserialize(update7));

  EXPECT_EQ(2u, tree.GetFromId(1)->GetUnignoredChildCount());
  EXPECT_EQ(0u, tree.GetFromId(3)->GetUnignoredIndexInParent());
}

TEST(AXTreeTest, TestRecursionUnignoredChildCount) {
  AXTreeUpdate tree_update;
  tree_update.root_id = 1;
  tree_update.nodes.resize(5);
  tree_update.nodes[0].id = 1;
  tree_update.nodes[0].child_ids = {2, 3};
  tree_update.nodes[1].id = 2;
  tree_update.nodes[1].AddState(ax::mojom::State::kIgnored);
  tree_update.nodes[1].child_ids = {4};
  tree_update.nodes[2].id = 3;
  tree_update.nodes[2].AddState(ax::mojom::State::kIgnored);
  tree_update.nodes[3].id = 4;
  tree_update.nodes[3].child_ids = {5};
  tree_update.nodes[3].AddState(ax::mojom::State::kIgnored);
  tree_update.nodes[4].id = 5;
  AXTree tree(tree_update);

  AXNode* root = tree.root();
  EXPECT_EQ(2u, root->children().size());
  EXPECT_EQ(1u, root->GetUnignoredChildCount());
  EXPECT_EQ(5, root->GetUnignoredChildAtIndex(0)->id());
  AXNode* unignored = tree.GetFromId(5);
  EXPECT_EQ(0u, unignored->GetUnignoredChildCount());
}

TEST(AXTreeTest, NullUnignoredChildren) {
  AXTreeUpdate tree_update;
  tree_update.root_id = 1;
  tree_update.nodes.resize(3);
  tree_update.nodes[0].id = 1;
  tree_update.nodes[0].child_ids = {2, 3};
  tree_update.nodes[1].id = 2;
  tree_update.nodes[1].AddState(ax::mojom::State::kIgnored);
  tree_update.nodes[2].id = 3;
  tree_update.nodes[2].AddState(ax::mojom::State::kIgnored);
  AXTree tree(tree_update);

  AXNode* root = tree.root();
  EXPECT_EQ(2u, root->children().size());
  EXPECT_EQ(0u, root->GetUnignoredChildCount());
  EXPECT_EQ(nullptr, root->GetUnignoredChildAtIndex(0));
  EXPECT_EQ(nullptr, root->GetUnignoredChildAtIndex(1));
}

TEST(AXTreeTest, UnignoredChildIteratorIncrementDecrementPastEnd) {
  AXTreeUpdate tree_update;

  // RootWebArea #1
  // ++StaticText "text1" #2

  tree_update.root_id = 1;
  tree_update.nodes.resize(2);

  tree_update.nodes[0].id = 1;
  tree_update.nodes[0].role = ax::mojom::Role::kWebArea;
  tree_update.nodes[0].child_ids = {2};

  tree_update.nodes[1].id = 2;
  tree_update.nodes[1].role = ax::mojom::Role::kStaticText;
  tree_update.nodes[1].SetName("text1");

  AXTree tree(tree_update);
  AXNode* root = tree.root();

  {
    {
      AXNode::UnignoredChildIterator root_unignored_iter =
          root->UnignoredChildrenBegin();
      EXPECT_EQ(2, root_unignored_iter->id());
      EXPECT_EQ("text1", root_unignored_iter->GetStringAttribute(
                             ax::mojom::StringAttribute::kName));

      // Call unignored child iterator on root and increment, we should reach
      // the end since there is only one iterator element.
      EXPECT_EQ(root->UnignoredChildrenEnd(), ++root_unignored_iter);

      // We increment past the end, and we should still stay at the end.
      EXPECT_EQ(root->UnignoredChildrenEnd(), ++root_unignored_iter);

      // When we decrement from the end, we should get the last iterator element
      // "text1".
      --root_unignored_iter;
      EXPECT_EQ(2, root_unignored_iter->id());
      EXPECT_EQ("text1", root_unignored_iter->GetStringAttribute(
                             ax::mojom::StringAttribute::kName));
    }

    {
      AXNode::UnignoredChildIterator root_unignored_iter =
          root->UnignoredChildrenBegin();
      EXPECT_EQ(2, root_unignored_iter->id());
      EXPECT_EQ("text1", root_unignored_iter->GetStringAttribute(
                             ax::mojom::StringAttribute::kName));

      // Call unignored child iterator on root and decrement from the beginning,
      // we should stay at the beginning.
      --root_unignored_iter;
      EXPECT_EQ(2, root_unignored_iter->id());
      EXPECT_EQ("text1", root_unignored_iter->GetStringAttribute(
                             ax::mojom::StringAttribute::kName));

      // When we decrement past the beginning, we should still stay at the
      // beginning.
      --root_unignored_iter;
      EXPECT_EQ(2, root_unignored_iter->id());
      EXPECT_EQ("text1", root_unignored_iter->GetStringAttribute(
                             ax::mojom::StringAttribute::kName));

      // We increment past the end, and we should still reach the end.
      EXPECT_EQ(root->UnignoredChildrenEnd(), ++root_unignored_iter);
    }
  }
}

TEST(AXTreeTest, UnignoredChildIteratorIgnoredContainerSiblings) {
  AXTreeUpdate tree_update;

  // RootWebArea #1
  // ++genericContainer IGNORED #2
  // ++++StaticText "text1" #3
  // ++genericContainer IGNORED #4
  // ++++StaticText "text2" #5
  // ++genericContainer IGNORED #6
  // ++++StaticText "text3" #7

  tree_update.root_id = 1;
  tree_update.nodes.resize(7);

  tree_update.nodes[0].id = 1;
  tree_update.nodes[0].role = ax::mojom::Role::kWebArea;
  tree_update.nodes[0].child_ids = {2, 4, 6};

  tree_update.nodes[1].id = 2;
  tree_update.nodes[1].child_ids = {3};
  tree_update.nodes[1].role = ax::mojom::Role::kGenericContainer;
  tree_update.nodes[1].AddState(ax::mojom::State::kIgnored);

  tree_update.nodes[2].id = 3;
  tree_update.nodes[2].role = ax::mojom::Role::kStaticText;
  tree_update.nodes[2].SetName("text1");

  tree_update.nodes[3].id = 4;
  tree_update.nodes[3].child_ids = {5};
  tree_update.nodes[3].role = ax::mojom::Role::kGenericContainer;
  tree_update.nodes[3].AddState(ax::mojom::State::kIgnored);

  tree_update.nodes[4].id = 5;
  tree_update.nodes[4].role = ax::mojom::Role::kStaticText;
  tree_update.nodes[4].SetName("text2");

  tree_update.nodes[5].id = 6;
  tree_update.nodes[5].child_ids = {7};
  tree_update.nodes[5].role = ax::mojom::Role::kGenericContainer;
  tree_update.nodes[5].AddState(ax::mojom::State::kIgnored);

  tree_update.nodes[6].id = 7;
  tree_update.nodes[6].role = ax::mojom::Role::kStaticText;
  tree_update.nodes[6].SetName("text3");

  AXTree tree(tree_update);

  {
    // Call unignored child iterator on root and iterate till the end, we should
    // get "text1", "text2", "text3" respectively because the sibling text nodes
    // share the same parent (i.e. root) as |unignored_iter|.
    AXNode* root = tree.root();
    AXNode::UnignoredChildIterator root_unignored_iter =
        root->UnignoredChildrenBegin();
    EXPECT_EQ(3, root_unignored_iter->id());
    EXPECT_EQ("text1", root_unignored_iter->GetStringAttribute(
                           ax::mojom::StringAttribute::kName));

    EXPECT_EQ(5, (++root_unignored_iter)->id());
    EXPECT_EQ("text2",
              (*root_unignored_iter)
                  .GetStringAttribute(ax::mojom::StringAttribute::kName));

    EXPECT_EQ(7, (++root_unignored_iter)->id());
    EXPECT_EQ("text3", root_unignored_iter->GetStringAttribute(
                           ax::mojom::StringAttribute::kName));
    EXPECT_EQ(root->UnignoredChildrenEnd(), ++root_unignored_iter);
  }

  {
    // Call unignored child iterator on the ignored generic container of "text1"
    // (id=2), When we iterate to the next of "text1", we should
    // reach the end because the sibling text node "text2" does not share the
    // same parent as |unignored_iter| of "text1".
    AXNode* text1_ignored_container = tree.GetFromId(2);
    AXNode::UnignoredChildIterator unignored_iter =
        text1_ignored_container->UnignoredChildrenBegin();
    EXPECT_EQ(3, unignored_iter->id());
    EXPECT_EQ("text1", unignored_iter->GetStringAttribute(
                           ax::mojom::StringAttribute::kName));
    // The next child of "text1" should be the end.
    EXPECT_EQ(text1_ignored_container->UnignoredChildrenEnd(),
              ++unignored_iter);

    // Call unignored child iterator on the ignored generic container of "text2"
    // (id=4), When we iterate to the previous of "text2", we should
    // reach the end because the sibling text node "text1" does not share the
    // same parent as |unignored_iter| of "text2".
    AXNode* text2_ignored_container = tree.GetFromId(4);
    unignored_iter = text2_ignored_container->UnignoredChildrenBegin();
    EXPECT_EQ(5, unignored_iter->id());
    EXPECT_EQ("text2", unignored_iter->GetStringAttribute(
                           ax::mojom::StringAttribute::kName));
    // Decrement the iterator of "text2" should still remain on "text2" since
    // the beginning of iterator is "text2."
    --unignored_iter;
    EXPECT_EQ(5, unignored_iter->id());
    EXPECT_EQ("text2", unignored_iter->GetStringAttribute(
                           ax::mojom::StringAttribute::kName));
  }
}

TEST(AXTreeTest, UnignoredChildIterator) {
  AXTreeUpdate tree_update;
  // (i) => node is ignored
  // 1
  // |__________
  // |     |   |
  // 2(i)  3   4
  // |_______________________
  // |   |      |           |
  // 5   6      7(i)        8(i)
  // |   |      |________
  // |   |      |       |
  // 9   10(i)  11(i)   12
  //     |      |____
  //     |      |   |
  //     13(i)  14  15
  tree_update.root_id = 1;
  tree_update.nodes.resize(15);
  tree_update.nodes[0].id = 1;
  tree_update.nodes[0].child_ids = {2, 3, 4};

  tree_update.nodes[1].id = 2;
  tree_update.nodes[1].child_ids = {5, 6, 7, 8};
  tree_update.nodes[1].AddState(ax::mojom::State::kIgnored);

  tree_update.nodes[2].id = 3;
  tree_update.nodes[3].id = 4;

  tree_update.nodes[4].id = 5;
  tree_update.nodes[4].child_ids = {9};

  tree_update.nodes[5].id = 6;
  tree_update.nodes[5].child_ids = {10};

  tree_update.nodes[6].id = 7;
  tree_update.nodes[6].child_ids = {11, 12};
  tree_update.nodes[6].AddState(ax::mojom::State::kIgnored);

  tree_update.nodes[7].id = 8;
  tree_update.nodes[7].AddState(ax::mojom::State::kIgnored);

  tree_update.nodes[8].id = 9;

  tree_update.nodes[9].id = 10;
  tree_update.nodes[9].child_ids = {13};
  tree_update.nodes[9].AddState(ax::mojom::State::kIgnored);

  tree_update.nodes[10].id = 11;
  tree_update.nodes[10].child_ids = {14, 15};
  tree_update.nodes[10].AddState(ax::mojom::State::kIgnored);

  tree_update.nodes[11].id = 12;

  tree_update.nodes[12].id = 13;
  tree_update.nodes[12].AddState(ax::mojom::State::kIgnored);

  tree_update.nodes[13].id = 14;

  tree_update.nodes[14].id = 15;

  AXTree tree(tree_update);
  AXNode* root = tree.root();

  // Test traversal
  // UnignoredChildren(root) = {5, 6, 14, 15, 12, 3, 4}
  AXNode::UnignoredChildIterator unignored_iterator =
      root->UnignoredChildrenBegin();
  EXPECT_EQ(5, unignored_iterator->id());

  EXPECT_EQ(6, (++unignored_iterator)->id());

  EXPECT_EQ(14, (++unignored_iterator)->id());

  EXPECT_EQ(15, (++unignored_iterator)->id());

  EXPECT_EQ(14, (--unignored_iterator)->id());

  EXPECT_EQ(6, (--unignored_iterator)->id());

  EXPECT_EQ(14, (++unignored_iterator)->id());

  EXPECT_EQ(15, (++unignored_iterator)->id());

  EXPECT_EQ(12, (++unignored_iterator)->id());

  EXPECT_EQ(3, (++unignored_iterator)->id());

  EXPECT_EQ(4, (++unignored_iterator)->id());

  EXPECT_EQ(root->UnignoredChildrenEnd(), ++unignored_iterator);

  // test empty list
  // UnignoredChildren(3) = {}
  AXNode* node3 = tree.GetFromId(3);
  unignored_iterator = node3->UnignoredChildrenBegin();
  EXPECT_EQ(node3->UnignoredChildrenEnd(), unignored_iterator);

  // empty list from ignored node with no children
  // UnignoredChildren(8) = {}
  AXNode* node8 = tree.GetFromId(8);
  unignored_iterator = node8->UnignoredChildrenBegin();
  EXPECT_EQ(node8->UnignoredChildrenEnd(), unignored_iterator);

  // empty list from ignored node with unignored children
  // UnignoredChildren(11) = {}
  AXNode* node11 = tree.GetFromId(11);
  unignored_iterator = node11->UnignoredChildrenBegin();
  EXPECT_EQ(14, unignored_iterator->id());

  // Two UnignoredChildIterators from the same parent at the same position
  // should be equivalent, even in end position.
  unignored_iterator = root->UnignoredChildrenBegin();
  AXNode::UnignoredChildIterator unignored_iterator2 =
      root->UnignoredChildrenBegin();
  auto end = root->UnignoredChildrenEnd();
  while (unignored_iterator != end) {
    ASSERT_EQ(unignored_iterator, unignored_iterator2);
    ++unignored_iterator;
    ++unignored_iterator2;
  }
  ASSERT_EQ(unignored_iterator, unignored_iterator2);
}

TEST(AXTreeTest, UnignoredAccessors) {
  AXTreeUpdate tree_update;
  // (i) => node is ignored
  // 1
  // |__________
  // |     |   |
  // 2(i)  3   4
  // |_______________________
  // |   |      |           |
  // 5   6      7(i)        8(i)
  // |   |      |________
  // |   |      |       |
  // 9   10(i)  11(i)   12
  //     |      |____
  //     |      |   |
  //     13(i)  14  15
  //     |      |
  //     16     17(i)
  tree_update.root_id = 1;
  tree_update.nodes.resize(17);
  tree_update.nodes[0].id = 1;
  tree_update.nodes[0].child_ids = {2, 3, 4};

  tree_update.nodes[1].id = 2;
  tree_update.nodes[1].child_ids = {5, 6, 7, 8};
  tree_update.nodes[1].AddState(ax::mojom::State::kIgnored);

  tree_update.nodes[2].id = 3;
  tree_update.nodes[3].id = 4;

  tree_update.nodes[4].id = 5;
  tree_update.nodes[4].child_ids = {9};

  tree_update.nodes[5].id = 6;
  tree_update.nodes[5].child_ids = {10};

  tree_update.nodes[6].id = 7;
  tree_update.nodes[6].child_ids = {11, 12};
  tree_update.nodes[6].AddState(ax::mojom::State::kIgnored);

  tree_update.nodes[7].id = 8;
  tree_update.nodes[7].AddState(ax::mojom::State::kIgnored);

  tree_update.nodes[8].id = 9;

  tree_update.nodes[9].id = 10;
  tree_update.nodes[9].child_ids = {13};
  tree_update.nodes[9].AddState(ax::mojom::State::kIgnored);

  tree_update.nodes[10].id = 11;
  tree_update.nodes[10].child_ids = {14, 15};
  tree_update.nodes[10].AddState(ax::mojom::State::kIgnored);

  tree_update.nodes[11].id = 12;

  tree_update.nodes[12].id = 13;
  tree_update.nodes[12].child_ids = {16};
  tree_update.nodes[12].AddState(ax::mojom::State::kIgnored);

  tree_update.nodes[13].id = 14;
  tree_update.nodes[13].child_ids = {17};

  tree_update.nodes[14].id = 15;

  tree_update.nodes[15].id = 16;

  tree_update.nodes[16].id = 17;
  tree_update.nodes[16].AddState(ax::mojom::State::kIgnored);

  AXTree tree(tree_update);

  EXPECT_EQ(4, tree.GetFromId(1)->GetLastUnignoredChild()->id());
  EXPECT_EQ(12, tree.GetFromId(2)->GetLastUnignoredChild()->id());
  EXPECT_EQ(nullptr, tree.GetFromId(3)->GetLastUnignoredChild());
  EXPECT_EQ(nullptr, tree.GetFromId(4)->GetLastUnignoredChild());
  EXPECT_EQ(9, tree.GetFromId(5)->GetLastUnignoredChild()->id());
  EXPECT_EQ(16, tree.GetFromId(6)->GetLastUnignoredChild()->id());
  EXPECT_EQ(12, tree.GetFromId(7)->GetLastUnignoredChild()->id());
  EXPECT_EQ(nullptr, tree.GetFromId(8)->GetLastUnignoredChild());
  EXPECT_EQ(nullptr, tree.GetFromId(9)->GetLastUnignoredChild());
  EXPECT_EQ(16, tree.GetFromId(10)->GetLastUnignoredChild()->id());
  EXPECT_EQ(15, tree.GetFromId(11)->GetLastUnignoredChild()->id());
  EXPECT_EQ(nullptr, tree.GetFromId(12)->GetLastUnignoredChild());
  EXPECT_EQ(16, tree.GetFromId(13)->GetLastUnignoredChild()->id());
  EXPECT_EQ(nullptr, tree.GetFromId(14)->GetLastUnignoredChild());
  EXPECT_EQ(nullptr, tree.GetFromId(15)->GetLastUnignoredChild());
  EXPECT_EQ(nullptr, tree.GetFromId(16)->GetLastUnignoredChild());
  EXPECT_EQ(nullptr, tree.GetFromId(17)->GetLastUnignoredChild());
}

TEST(AXTreeTest, UnignoredNextPreviousChild) {
  AXTreeUpdate tree_update;
  // (i) => node is ignored
  // 1
  // |__________
  // |     |   |
  // 2(i)  3   4
  // |_______________________
  // |   |      |           |
  // 5   6      7(i)        8(i)
  // |   |      |________
  // |   |      |       |
  // 9   10(i)  11(i)   12
  //     |      |____
  //     |      |   |
  //     13(i)  14  15
  //     |
  //     16
  tree_update.root_id = 1;
  tree_update.nodes.resize(16);
  tree_update.nodes[0].id = 1;
  tree_update.nodes[0].child_ids = {2, 3, 4};

  tree_update.nodes[1].id = 2;
  tree_update.nodes[1].child_ids = {5, 6, 7, 8};
  tree_update.nodes[1].AddState(ax::mojom::State::kIgnored);

  tree_update.nodes[2].id = 3;
  tree_update.nodes[3].id = 4;

  tree_update.nodes[4].id = 5;
  tree_update.nodes[4].child_ids = {9};

  tree_update.nodes[5].id = 6;
  tree_update.nodes[5].child_ids = {10};

  tree_update.nodes[6].id = 7;
  tree_update.nodes[6].child_ids = {11, 12};
  tree_update.nodes[6].AddState(ax::mojom::State::kIgnored);

  tree_update.nodes[7].id = 8;
  tree_update.nodes[7].AddState(ax::mojom::State::kIgnored);

  tree_update.nodes[8].id = 9;

  tree_update.nodes[9].id = 10;
  tree_update.nodes[9].child_ids = {13};
  tree_update.nodes[9].AddState(ax::mojom::State::kIgnored);

  tree_update.nodes[10].id = 11;
  tree_update.nodes[10].child_ids = {14, 15};
  tree_update.nodes[10].AddState(ax::mojom::State::kIgnored);

  tree_update.nodes[11].id = 12;

  tree_update.nodes[12].id = 13;
  tree_update.nodes[12].child_ids = {16};
  tree_update.nodes[12].AddState(ax::mojom::State::kIgnored);

  tree_update.nodes[13].id = 14;

  tree_update.nodes[14].id = 15;

  tree_update.nodes[15].id = 16;

  AXTree tree(tree_update);

  EXPECT_EQ(nullptr, tree.GetFromId(1)->GetNextUnignoredSibling());
  EXPECT_EQ(nullptr, tree.GetFromId(1)->GetPreviousUnignoredSibling());

  EXPECT_EQ(tree.GetFromId(3), tree.GetFromId(2)->GetNextUnignoredSibling());
  EXPECT_EQ(nullptr, tree.GetFromId(2)->GetPreviousUnignoredSibling());

  EXPECT_EQ(tree.GetFromId(4), tree.GetFromId(3)->GetNextUnignoredSibling());
  EXPECT_EQ(tree.GetFromId(12),
            tree.GetFromId(3)->GetPreviousUnignoredSibling());

  EXPECT_EQ(nullptr, tree.GetFromId(4)->GetNextUnignoredSibling());
  EXPECT_EQ(tree.GetFromId(3),
            tree.GetFromId(4)->GetPreviousUnignoredSibling());

  EXPECT_EQ(tree.GetFromId(6), tree.GetFromId(5)->GetNextUnignoredSibling());
  EXPECT_EQ(nullptr, tree.GetFromId(5)->GetPreviousUnignoredSibling());

  EXPECT_EQ(tree.GetFromId(14), tree.GetFromId(6)->GetNextUnignoredSibling());
  EXPECT_EQ(tree.GetFromId(5),
            tree.GetFromId(6)->GetPreviousUnignoredSibling());

  EXPECT_EQ(tree.GetFromId(3), tree.GetFromId(7)->GetNextUnignoredSibling());
  EXPECT_EQ(tree.GetFromId(6),
            tree.GetFromId(7)->GetPreviousUnignoredSibling());

  EXPECT_EQ(tree.GetFromId(3), tree.GetFromId(8)->GetNextUnignoredSibling());
  EXPECT_EQ(tree.GetFromId(12),
            tree.GetFromId(8)->GetPreviousUnignoredSibling());

  EXPECT_EQ(nullptr, tree.GetFromId(9)->GetNextUnignoredSibling());
  EXPECT_EQ(nullptr, tree.GetFromId(9)->GetPreviousUnignoredSibling());

  EXPECT_EQ(nullptr, tree.GetFromId(10)->GetNextUnignoredSibling());
  EXPECT_EQ(nullptr, tree.GetFromId(10)->GetPreviousUnignoredSibling());

  EXPECT_EQ(tree.GetFromId(12), tree.GetFromId(11)->GetNextUnignoredSibling());
  EXPECT_EQ(tree.GetFromId(6),
            tree.GetFromId(11)->GetPreviousUnignoredSibling());

  EXPECT_EQ(tree.GetFromId(3), tree.GetFromId(12)->GetNextUnignoredSibling());
  EXPECT_EQ(tree.GetFromId(15),
            tree.GetFromId(12)->GetPreviousUnignoredSibling());

  EXPECT_EQ(nullptr, tree.GetFromId(13)->GetNextUnignoredSibling());
  EXPECT_EQ(nullptr, tree.GetFromId(13)->GetPreviousUnignoredSibling());

  EXPECT_EQ(tree.GetFromId(15), tree.GetFromId(14)->GetNextUnignoredSibling());
  EXPECT_EQ(tree.GetFromId(6),
            tree.GetFromId(14)->GetPreviousUnignoredSibling());

  EXPECT_EQ(tree.GetFromId(12), tree.GetFromId(15)->GetNextUnignoredSibling());
  EXPECT_EQ(tree.GetFromId(14),
            tree.GetFromId(15)->GetPreviousUnignoredSibling());

  EXPECT_EQ(nullptr, tree.GetFromId(16)->GetNextUnignoredSibling());
  EXPECT_EQ(nullptr, tree.GetFromId(16)->GetPreviousUnignoredSibling());
}

TEST(AXTreeTest, GetSiblingsNoIgnored) {
  // Since this tree base::contains no ignored nodes, PreviousSibling and
  // NextSibling are equivalent to their unignored counterparts.
  //
  // 1
  // ├── 2
  // │   └── 4
  // └── 3
  AXTreeUpdate tree_update;
  tree_update.root_id = 1;
  tree_update.nodes.resize(4);
  tree_update.nodes[0].id = 1;
  tree_update.nodes[0].child_ids = {2, 3};
  tree_update.nodes[1].id = 2;
  tree_update.nodes[1].child_ids = {4};
  tree_update.nodes[2].id = 3;
  tree_update.nodes[3].id = 4;

  AXTree tree(tree_update);

  EXPECT_EQ(nullptr, tree.GetFromId(1)->GetPreviousSibling());
  EXPECT_EQ(nullptr, tree.GetFromId(1)->GetPreviousUnignoredSibling());
  EXPECT_EQ(nullptr, tree.GetFromId(1)->GetNextSibling());
  EXPECT_EQ(nullptr, tree.GetFromId(1)->GetNextUnignoredSibling());

  EXPECT_EQ(nullptr, tree.GetFromId(2)->GetPreviousSibling());
  EXPECT_EQ(nullptr, tree.GetFromId(2)->GetPreviousUnignoredSibling());
  EXPECT_EQ(tree.GetFromId(3), tree.GetFromId(2)->GetNextSibling());
  EXPECT_EQ(tree.GetFromId(3), tree.GetFromId(2)->GetNextUnignoredSibling());

  EXPECT_EQ(tree.GetFromId(2), tree.GetFromId(3)->GetPreviousSibling());
  EXPECT_EQ(tree.GetFromId(2),
            tree.GetFromId(3)->GetPreviousUnignoredSibling());
  EXPECT_EQ(nullptr, tree.GetFromId(3)->GetNextSibling());
  EXPECT_EQ(nullptr, tree.GetFromId(3)->GetNextUnignoredSibling());

  EXPECT_EQ(nullptr, tree.GetFromId(4)->GetPreviousSibling());
  EXPECT_EQ(nullptr, tree.GetFromId(4)->GetPreviousUnignoredSibling());
  EXPECT_EQ(nullptr, tree.GetFromId(4)->GetNextSibling());
  EXPECT_EQ(nullptr, tree.GetFromId(4)->GetNextUnignoredSibling());
}

TEST(AXTreeTest, GetUnignoredSiblingsChildrenPromoted) {
  // An ignored node has its' children considered as though they were promoted
  // to their parents place.
  //
  // (i) => node is ignored.
  //
  // 1
  // ├── 2(i)
  // │   ├── 4
  // │   └── 5
  // └── 3
  AXTreeUpdate tree_update;
  tree_update.root_id = 1;
  tree_update.nodes.resize(5);
  tree_update.nodes[0].id = 1;
  tree_update.nodes[0].child_ids = {2, 3};
  tree_update.nodes[1].id = 2;
  tree_update.nodes[1].AddState(ax::mojom::State::kIgnored);
  tree_update.nodes[1].child_ids = {4, 5};
  tree_update.nodes[2].id = 3;
  tree_update.nodes[3].id = 4;
  tree_update.nodes[4].id = 5;

  AXTree tree(tree_update);

  // Root node has no siblings.
  EXPECT_EQ(nullptr, tree.GetFromId(1)->GetPreviousUnignoredSibling());
  EXPECT_EQ(nullptr, tree.GetFromId(2)->GetPreviousUnignoredSibling());

  // Node 2's view of siblings:
  // literal tree:   null <-- [2(i)] --> 3
  // unignored tree: null <-- [2(i)] --> 3
  EXPECT_EQ(nullptr, tree.GetFromId(2)->GetPreviousSibling());
  EXPECT_EQ(nullptr, tree.GetFromId(2)->GetPreviousUnignoredSibling());
  EXPECT_EQ(tree.GetFromId(3), tree.GetFromId(2)->GetNextSibling());
  EXPECT_EQ(tree.GetFromId(3), tree.GetFromId(2)->GetNextUnignoredSibling());

  // Node 3's view of siblings:
  // literal tree:   2(i) <-- [3] --> null
  // unignored tree:    5 <-- [4] --> null
  EXPECT_EQ(tree.GetFromId(2), tree.GetFromId(3)->GetPreviousSibling());
  EXPECT_EQ(tree.GetFromId(5),
            tree.GetFromId(3)->GetPreviousUnignoredSibling());
  EXPECT_EQ(nullptr, tree.GetFromId(3)->GetNextSibling());
  EXPECT_EQ(nullptr, tree.GetFromId(3)->GetNextUnignoredSibling());

  // Node 4's view of siblings:
  // literal tree:   null <-- [4] --> 5
  // unignored tree: null <-- [4] --> 5
  EXPECT_EQ(nullptr, tree.GetFromId(4)->GetPreviousSibling());
  EXPECT_EQ(nullptr, tree.GetFromId(4)->GetPreviousUnignoredSibling());
  EXPECT_EQ(tree.GetFromId(5), tree.GetFromId(4)->GetNextSibling());
  EXPECT_EQ(tree.GetFromId(5), tree.GetFromId(4)->GetNextUnignoredSibling());

  // Node 5's view of siblings:
  // literal tree:   4 <-- [5] --> null
  // unignored tree: 4 <-- [5] --> 3
  EXPECT_EQ(tree.GetFromId(4), tree.GetFromId(5)->GetPreviousSibling());
  EXPECT_EQ(tree.GetFromId(4),
            tree.GetFromId(5)->GetPreviousUnignoredSibling());
  EXPECT_EQ(nullptr, tree.GetFromId(5)->GetNextSibling());
  EXPECT_EQ(tree.GetFromId(3), tree.GetFromId(5)->GetNextUnignoredSibling());
}

TEST(AXTreeTest, GetUnignoredSiblingsIgnoredChildSkipped) {
  // Ignored children of ignored parents are skipped over.
  //
  // (i) => node is ignored.
  //
  // 1
  // ├── 2(i)
  // │   ├── 4
  // │   └── 5(i)
  // └── 3
  AXTreeUpdate tree_update;
  tree_update.root_id = 1;
  tree_update.nodes.resize(5);
  tree_update.nodes[0].id = 1;
  tree_update.nodes[0].child_ids = {2, 3};
  tree_update.nodes[1].id = 2;
  tree_update.nodes[1].AddState(ax::mojom::State::kIgnored);
  tree_update.nodes[1].child_ids = {4, 5};
  tree_update.nodes[2].id = 3;
  tree_update.nodes[3].id = 4;
  tree_update.nodes[4].id = 5;
  tree_update.nodes[4].AddState(ax::mojom::State::kIgnored);

  AXTree tree(tree_update);

  // Root node has no siblings.
  EXPECT_EQ(nullptr, tree.GetFromId(1)->GetPreviousUnignoredSibling());
  EXPECT_EQ(nullptr, tree.GetFromId(1)->GetNextUnignoredSibling());

  // Node 2's view of siblings:
  // literal tree:   null <-- [2(i)] --> 3
  // unignored tree: null <-- [2(i)] --> 3
  EXPECT_EQ(nullptr, tree.GetFromId(2)->GetPreviousSibling());
  EXPECT_EQ(nullptr, tree.GetFromId(2)->GetPreviousUnignoredSibling());
  EXPECT_EQ(tree.GetFromId(3), tree.GetFromId(2)->GetNextSibling());
  EXPECT_EQ(tree.GetFromId(3), tree.GetFromId(2)->GetNextUnignoredSibling());

  // Node 3's view of siblings:
  // literal tree:   2(i) <-- [3] --> null
  // unignored tree:    4 <-- [3] --> null
  EXPECT_EQ(tree.GetFromId(2), tree.GetFromId(3)->GetPreviousSibling());
  EXPECT_EQ(tree.GetFromId(4),
            tree.GetFromId(3)->GetPreviousUnignoredSibling());
  EXPECT_EQ(nullptr, tree.GetFromId(3)->GetNextSibling());
  EXPECT_EQ(nullptr, tree.GetFromId(3)->GetNextUnignoredSibling());

  // Node 4's view of siblings:
  // literal tree:   null <-- [4] --> 5(i)
  // unignored tree: null <-- [4] --> 3
  EXPECT_EQ(nullptr, tree.GetFromId(4)->GetPreviousSibling());
  EXPECT_EQ(nullptr, tree.GetFromId(4)->GetPreviousUnignoredSibling());
  EXPECT_EQ(tree.GetFromId(5), tree.GetFromId(4)->GetNextSibling());
  EXPECT_EQ(tree.GetFromId(3), tree.GetFromId(4)->GetNextUnignoredSibling());

  // Node 5's view of siblings:
  // literal tree:   4 <-- [5(i)] --> null
  // unignored tree: 4 <-- [5(i)] --> 3
  EXPECT_EQ(tree.GetFromId(4), tree.GetFromId(5)->GetPreviousSibling());
  EXPECT_EQ(tree.GetFromId(4),
            tree.GetFromId(5)->GetPreviousUnignoredSibling());
  EXPECT_EQ(nullptr, tree.GetFromId(5)->GetNextSibling());
  EXPECT_EQ(tree.GetFromId(3), tree.GetFromId(5)->GetNextUnignoredSibling());
}

TEST(AXTreeTest, GetUnignoredSiblingIgnoredParentIrrelevant) {
  // An ignored parent is not relevant unless the search would need to continue
  // up through it.
  //
  // (i) => node is ignored.
  //
  // 1(i)
  // ├── 2
  // └── 3
  AXTreeUpdate tree_update;
  tree_update.root_id = 1;
  tree_update.nodes.resize(3);
  tree_update.nodes[0].id = 1;
  tree_update.nodes[0].AddState(ax::mojom::State::kIgnored);
  tree_update.nodes[0].child_ids = {2, 3};
  tree_update.nodes[1].id = 2;
  tree_update.nodes[2].id = 3;

  AXTree tree(tree_update);

  // Node 2 and 3 are each other's unignored siblings, the parent's ignored
  // status is not relevant for this search.
  EXPECT_EQ(tree.GetFromId(3), tree.GetFromId(2)->GetNextUnignoredSibling());
  EXPECT_EQ(tree.GetFromId(2),
            tree.GetFromId(3)->GetPreviousUnignoredSibling());
}

TEST(AXTreeTest, GetUnignoredSiblingsAllIgnored) {
  // Test termination when all nodes, including the root node, are ignored.
  //
  // (i) => node is ignored.
  //
  // 1(i)
  // └── 2(i)
  AXTreeUpdate tree_update;
  tree_update.root_id = 1;
  tree_update.nodes.resize(2);
  tree_update.nodes[0].id = 1;
  tree_update.nodes[0].AddState(ax::mojom::State::kIgnored);
  tree_update.nodes[0].child_ids = {2};
  tree_update.nodes[1].id = 2;
  tree_update.nodes[1].AddState(ax::mojom::State::kIgnored);

  AXTree tree(tree_update);

  EXPECT_EQ(nullptr, tree.GetFromId(1)->GetPreviousUnignoredSibling());
  EXPECT_EQ(nullptr, tree.GetFromId(1)->GetNextUnignoredSibling());
  EXPECT_EQ(nullptr, tree.GetFromId(2)->GetPreviousUnignoredSibling());
  EXPECT_EQ(nullptr, tree.GetFromId(2)->GetNextUnignoredSibling());
}

TEST(AXTreeTest, GetUnignoredSiblingsNestedIgnored) {
  // Test promotion of children through multiple layers of ignored parents.
  // (i) => node is ignored.
  //
  // 1
  // ├── 2
  // ├── 3(i)
  // │   └── 5(i)
  // │       └── 6
  // └── 4
  AXTreeUpdate tree_update;
  tree_update.root_id = 1;
  tree_update.nodes.resize(6);
  tree_update.nodes[0].id = 1;
  tree_update.nodes[0].child_ids = {2, 3, 4};
  tree_update.nodes[1].id = 2;
  tree_update.nodes[2].id = 3;
  tree_update.nodes[2].AddState(ax::mojom::State::kIgnored);
  tree_update.nodes[2].child_ids = {5};
  tree_update.nodes[3].id = 4;
  tree_update.nodes[4].id = 5;
  tree_update.nodes[4].AddState(ax::mojom::State::kIgnored);
  tree_update.nodes[4].child_ids = {6};
  tree_update.nodes[5].id = 6;

  AXTree tree(tree_update);

  EXPECT_EQ(nullptr, tree.GetFromId(1)->GetPreviousUnignoredSibling());

  const AXNode* node2 = tree.GetFromId(2);
  const AXNode* node3 = tree.GetFromId(3);
  const AXNode* node4 = tree.GetFromId(4);
  const AXNode* node5 = tree.GetFromId(5);
  const AXNode* node6 = tree.GetFromId(6);

  ASSERT_NE(nullptr, node2);
  ASSERT_NE(nullptr, node3);
  ASSERT_NE(nullptr, node4);
  ASSERT_NE(nullptr, node5);
  ASSERT_NE(nullptr, node6);

  // Node 2's view of siblings:
  // literal tree:   null <-- [2] --> 3
  // unignored tree: null <-- [2] --> 6
  EXPECT_EQ(nullptr, node2->GetPreviousSibling());
  EXPECT_EQ(nullptr, node2->GetPreviousUnignoredSibling());
  EXPECT_EQ(node3, node2->GetNextSibling());
  EXPECT_EQ(node6, node2->GetNextUnignoredSibling());

  // Node 3's view of siblings:
  // literal tree:   2 <-- [3(i)] --> 4
  // unignored tree: 2 <-- [3(i)] --> 4
  EXPECT_EQ(node2, node3->GetPreviousSibling());
  EXPECT_EQ(node2, node3->GetPreviousUnignoredSibling());
  EXPECT_EQ(node4, node3->GetNextSibling());
  EXPECT_EQ(node4, node3->GetNextUnignoredSibling());

  // Node 4's view of siblings:
  // literal tree:   3 <-- [4] --> null
  // unignored tree: 6 <-- [4] --> null
  EXPECT_EQ(node3, node4->GetPreviousSibling());
  EXPECT_EQ(node6, node4->GetPreviousUnignoredSibling());
  EXPECT_EQ(nullptr, node4->GetNextSibling());
  EXPECT_EQ(nullptr, node4->GetNextUnignoredSibling());

  // Node 5's view of siblings:
  // literal tree:   null <-- [5(i)] --> null
  // unignored tree:    2 <-- [5(i)] --> 4
  EXPECT_EQ(nullptr, node5->GetPreviousSibling());
  EXPECT_EQ(node2, node5->GetPreviousUnignoredSibling());
  EXPECT_EQ(nullptr, node5->GetNextSibling());
  EXPECT_EQ(node4, node5->GetNextUnignoredSibling());

  // Node 6's view of siblings:
  // literal tree:   null <-- [6] --> null
  // unignored tree:    2 <-- [6] --> 4
  EXPECT_EQ(nullptr, node6->GetPreviousSibling());
  EXPECT_EQ(node2, node6->GetPreviousUnignoredSibling());
  EXPECT_EQ(nullptr, node6->GetNextSibling());
  EXPECT_EQ(node4, node6->GetNextUnignoredSibling());
}

TEST(AXTreeTest, UnignoredSelection) {
  AXTreeUpdate tree_update;
  // (i) => node is ignored
  // 1
  // |__________
  // |     |   |
  // 2(i)  3   4
  // |_______________________
  // |   |      |           |
  // 5   6      7(i)        8(i)
  // |   |      |________
  // |   |      |       |
  // 9   10(i)  11(i)  12
  //     |      |____
  //     |      |   |
  //     13(i)  14  15
  //     |
  //     16
  // Unignored Tree (conceptual)
  // 1
  // |______________________
  // |  |    |   |   |  |  |
  // 5  6   14  15  12  3  4
  // |  |
  // 9  16
  tree_update.has_tree_data = true;
  tree_update.tree_data.tree_id = ui::AXTreeID::CreateNewAXTreeID();
  tree_update.root_id = 1;
  tree_update.nodes.resize(16);
  tree_update.nodes[0].id = 1;
  tree_update.nodes[0].role = ax::mojom::Role::kGenericContainer;
  tree_update.nodes[0].child_ids = {2, 3, 4};

  tree_update.nodes[1].id = 2;
  tree_update.nodes[1].child_ids = {5, 6, 7, 8};
  tree_update.nodes[1].role = ax::mojom::Role::kGenericContainer;
  tree_update.nodes[1].AddState(ax::mojom::State::kIgnored);

  tree_update.nodes[2].id = 3;
  tree_update.nodes[2].role = ax::mojom::Role::kStaticText;
  tree_update.nodes[2].SetName("text");

  tree_update.nodes[3].id = 4;
  tree_update.nodes[3].role = ax::mojom::Role::kStaticText;
  tree_update.nodes[3].SetName("text");

  tree_update.nodes[4].id = 5;
  tree_update.nodes[4].role = ax::mojom::Role::kGenericContainer;
  tree_update.nodes[4].child_ids = {9};

  tree_update.nodes[5].id = 6;
  tree_update.nodes[5].role = ax::mojom::Role::kGenericContainer;
  tree_update.nodes[5].child_ids = {10};

  tree_update.nodes[6].id = 7;
  tree_update.nodes[6].child_ids = {11, 12};
  tree_update.nodes[6].role = ax::mojom::Role::kGenericContainer;
  tree_update.nodes[6].AddState(ax::mojom::State::kIgnored);

  tree_update.nodes[7].id = 8;
  tree_update.nodes[7].role = ax::mojom::Role::kGenericContainer;
  tree_update.nodes[7].AddState(ax::mojom::State::kIgnored);

  tree_update.nodes[8].id = 9;
  tree_update.nodes[8].role = ax::mojom::Role::kStaticText;
  tree_update.nodes[8].SetName("text");

  tree_update.nodes[9].id = 10;
  tree_update.nodes[9].child_ids = {13};
  tree_update.nodes[9].role = ax::mojom::Role::kGenericContainer;
  tree_update.nodes[9].AddState(ax::mojom::State::kIgnored);

  tree_update.nodes[10].id = 11;
  tree_update.nodes[10].child_ids = {14, 15};
  tree_update.nodes[10].role = ax::mojom::Role::kGenericContainer;
  tree_update.nodes[10].AddState(ax::mojom::State::kIgnored);

  tree_update.nodes[11].id = 12;
  tree_update.nodes[11].role = ax::mojom::Role::kStaticText;
  tree_update.nodes[11].SetName("text");

  tree_update.nodes[12].id = 13;
  tree_update.nodes[12].child_ids = {16};
  tree_update.nodes[12].role = ax::mojom::Role::kGenericContainer;
  tree_update.nodes[12].AddState(ax::mojom::State::kIgnored);

  tree_update.nodes[13].id = 14;
  tree_update.nodes[13].role = ax::mojom::Role::kStaticText;
  tree_update.nodes[13].SetName("text");

  tree_update.nodes[14].id = 15;
  tree_update.nodes[14].role = ax::mojom::Role::kStaticText;
  tree_update.nodes[14].SetName("text");

  tree_update.nodes[15].id = 16;
  tree_update.nodes[15].role = ax::mojom::Role::kStaticText;
  tree_update.nodes[15].SetName("text");

  TestAXTreeManager test_ax_tree_manager(std::make_unique<AXTree>(tree_update));
  AXTree::Selection unignored_selection =
      test_ax_tree_manager.GetTree()->GetUnignoredSelection();

  EXPECT_EQ(AXNode::kInvalidAXID, unignored_selection.anchor_object_id);
  EXPECT_EQ(-1, unignored_selection.anchor_offset);
  EXPECT_EQ(AXNode::kInvalidAXID, unignored_selection.focus_object_id);
  EXPECT_EQ(-1, unignored_selection.focus_offset);
  struct SelectionData {
    int32_t anchor_id;
    int32_t anchor_offset;
    int32_t focus_id;
    int32_t focus_offset;
  };

  SelectionData input = {1, 0, 1, 0};
  SelectionData expected = {9, 0, 9, 0};
  TEST_SELECTION(tree_update, test_ax_tree_manager.GetTree(), input, expected);

  input = {1, 0, 2, 2};
  expected = {9, 0, 14, 0};
  TEST_SELECTION(tree_update, test_ax_tree_manager.GetTree(), input, expected);

  input = {2, 1, 5, 0};
  expected = {16, 0, 5, 0};
  TEST_SELECTION(tree_update, test_ax_tree_manager.GetTree(), input, expected);

  input = {5, 0, 9, 0};
  expected = {5, 0, 9, 0};
  TEST_SELECTION(tree_update, test_ax_tree_manager.GetTree(), input, expected);

  input = {9, 0, 6, 0};
  expected = {9, 0, 16, 0};
  TEST_SELECTION(tree_update, test_ax_tree_manager.GetTree(), input, expected);

  input = {6, 0, 10, 0};
  expected = {16, 0, 16, 0};
  TEST_SELECTION(tree_update, test_ax_tree_manager.GetTree(), input, expected);

  input = {10, 0, 13, 0};
  expected = {16, 0, 16, 0};
  TEST_SELECTION(tree_update, test_ax_tree_manager.GetTree(), input, expected);

  input = {13, 0, 16, 0};
  expected = {16, 0, 16, 0};
  TEST_SELECTION(tree_update, test_ax_tree_manager.GetTree(), input, expected);

  input = {16, 0, 7, 0};
  expected = {16, 0, 14, 0};
  TEST_SELECTION(tree_update, test_ax_tree_manager.GetTree(), input, expected);

  input = {7, 0, 11, 0};
  expected = {14, 0, 14, 0};
  TEST_SELECTION(tree_update, test_ax_tree_manager.GetTree(), input, expected);

  input = {11, 1, 14, 2};
  expected = {15, 0, 14, 2};
  TEST_SELECTION(tree_update, test_ax_tree_manager.GetTree(), input, expected);

  input = {14, 2, 15, 3};
  expected = {14, 2, 15, 3};
  TEST_SELECTION(tree_update, test_ax_tree_manager.GetTree(), input, expected);

  input = {15, 0, 12, 0};
  expected = {15, 0, 12, 0};
  TEST_SELECTION(tree_update, test_ax_tree_manager.GetTree(), input, expected);

  input = {12, 0, 8, 0};
  expected = {12, 0, 3, 0};
  TEST_SELECTION(tree_update, test_ax_tree_manager.GetTree(), input, expected);

  input = {8, 0, 3, 0};
  expected = {12, 4, 3, 0};
  TEST_SELECTION(tree_update, test_ax_tree_manager.GetTree(), input, expected);

  input = {3, 0, 4, 0};
  expected = {3, 0, 4, 0};
  TEST_SELECTION(tree_update, test_ax_tree_manager.GetTree(), input, expected);

  input = {4, 0, 4, 0};
  expected = {4, 0, 4, 0};
  TEST_SELECTION(tree_update, test_ax_tree_manager.GetTree(), input, expected);
}

TEST(AXTreeTest, GetChildrenOrSiblings) {
  // 1
  // ├── 2
  // │   └── 5
  // ├── 3
  // └── 4
  AXTreeUpdate tree_update;
  tree_update.root_id = 1;
  tree_update.nodes.resize(5);
  tree_update.nodes[0].id = 1;
  tree_update.nodes[0].child_ids = {2, 3, 4};
  tree_update.nodes[1].id = 2;
  tree_update.nodes[1].child_ids = {5};
  tree_update.nodes[2].id = 3;
  tree_update.nodes[3].id = 4;
  tree_update.nodes[4].id = 5;

  AXTree tree(tree_update);

  EXPECT_EQ(tree.GetFromId(2), tree.GetFromId(1)->GetFirstChild());
  EXPECT_EQ(tree.GetFromId(5), tree.GetFromId(2)->GetFirstChild());
  EXPECT_EQ(nullptr, tree.GetFromId(3)->GetFirstChild());
  EXPECT_EQ(nullptr, tree.GetFromId(4)->GetFirstChild());
  EXPECT_EQ(nullptr, tree.GetFromId(5)->GetFirstChild());

  EXPECT_EQ(tree.GetFromId(4), tree.GetFromId(1)->GetLastChild());
  EXPECT_EQ(tree.GetFromId(5), tree.GetFromId(2)->GetLastChild());
  EXPECT_EQ(nullptr, tree.GetFromId(3)->GetLastChild());
  EXPECT_EQ(nullptr, tree.GetFromId(4)->GetLastChild());
  EXPECT_EQ(nullptr, tree.GetFromId(5)->GetLastChild());

  EXPECT_EQ(nullptr, tree.GetFromId(1)->GetPreviousSibling());
  EXPECT_EQ(nullptr, tree.GetFromId(2)->GetPreviousSibling());
  EXPECT_EQ(tree.GetFromId(2), tree.GetFromId(3)->GetPreviousSibling());
  EXPECT_EQ(tree.GetFromId(3), tree.GetFromId(4)->GetPreviousSibling());
  EXPECT_EQ(nullptr, tree.GetFromId(5)->GetPreviousSibling());

  EXPECT_EQ(nullptr, tree.GetFromId(1)->GetNextSibling());
  EXPECT_EQ(tree.GetFromId(3), tree.GetFromId(2)->GetNextSibling());
  EXPECT_EQ(tree.GetFromId(4), tree.GetFromId(3)->GetNextSibling());
  EXPECT_EQ(nullptr, tree.GetFromId(4)->GetNextSibling());
  EXPECT_EQ(nullptr, tree.GetFromId(5)->GetNextSibling());
}

// Tests GetPosInSet and GetSetSize return the assigned int attribute values.
TEST(AXTreeTest, SetSizePosInSetAssigned) {
  AXTreeUpdate tree_update;
  tree_update.root_id = 1;
  tree_update.nodes.resize(4);
  tree_update.nodes[0].id = 1;
  tree_update.nodes[0].role = ax::mojom::Role::kList;
  tree_update.nodes[0].child_ids = {2, 3, 4};
  tree_update.nodes[1].id = 2;
  tree_update.nodes[1].role = ax::mojom::Role::kListItem;
  tree_update.nodes[1].AddIntAttribute(ax::mojom::IntAttribute::kPosInSet, 2);
  tree_update.nodes[1].AddIntAttribute(ax::mojom::IntAttribute::kSetSize, 12);
  tree_update.nodes[2].id = 3;
  tree_update.nodes[2].role = ax::mojom::Role::kListItem;
  tree_update.nodes[2].AddIntAttribute(ax::mojom::IntAttribute::kPosInSet, 5);
  tree_update.nodes[2].AddIntAttribute(ax::mojom::IntAttribute::kSetSize, 12);
  tree_update.nodes[3].id = 4;
  tree_update.nodes[3].role = ax::mojom::Role::kListItem;
  tree_update.nodes[3].AddIntAttribute(ax::mojom::IntAttribute::kPosInSet, 9);
  tree_update.nodes[3].AddIntAttribute(ax::mojom::IntAttribute::kSetSize, 12);
  AXTree tree(tree_update);

  AXNode* item1 = tree.GetFromId(2);
  EXPECT_OPTIONAL_EQ(2, item1->GetPosInSet());
  EXPECT_OPTIONAL_EQ(12, item1->GetSetSize());
  AXNode* item2 = tree.GetFromId(3);
  EXPECT_OPTIONAL_EQ(5, item2->GetPosInSet());
  EXPECT_OPTIONAL_EQ(12, item2->GetSetSize());
  AXNode* item3 = tree.GetFromId(4);
  EXPECT_OPTIONAL_EQ(9, item3->GetPosInSet());
  EXPECT_OPTIONAL_EQ(12, item3->GetSetSize());
}

// Tests that PosInSet and SetSize can be calculated if not assigned.
TEST(AXTreeTest, SetSizePosInSetUnassigned) {
  AXTreeUpdate tree_update;
  tree_update.root_id = 1;
  tree_update.nodes.resize(4);
  tree_update.nodes[0].id = 1;
  tree_update.nodes[0].role = ax::mojom::Role::kList;
  tree_update.nodes[0].child_ids = {2, 3, 4};
  tree_update.nodes[1].id = 2;
  tree_update.nodes[1].role = ax::mojom::Role::kListItem;
  tree_update.nodes[2].id = 3;
  tree_update.nodes[2].role = ax::mojom::Role::kListItem;
  tree_update.nodes[3].id = 4;
  tree_update.nodes[3].role = ax::mojom::Role::kListItem;
  AXTree tree(tree_update);

  AXNode* item1 = tree.GetFromId(2);
  EXPECT_OPTIONAL_EQ(1, item1->GetPosInSet());
  EXPECT_OPTIONAL_EQ(3, item1->GetSetSize());
  AXNode* item2 = tree.GetFromId(3);
  EXPECT_OPTIONAL_EQ(2, item2->GetPosInSet());
  EXPECT_OPTIONAL_EQ(3, item2->GetSetSize());
  AXNode* item3 = tree.GetFromId(4);
  EXPECT_OPTIONAL_EQ(3, item3->GetPosInSet());
  EXPECT_OPTIONAL_EQ(3, item3->GetSetSize());
}

// Tests PosInSet can be calculated if unassigned, and SetSize can be
// assigned on the outerlying ordered set.
TEST(AXTreeTest, SetSizeAssignedOnContainer) {
  AXTreeUpdate tree_update;
  tree_update.root_id = 1;
  tree_update.nodes.resize(4);
  tree_update.nodes[0].id = 1;
  tree_update.nodes[0].role = ax::mojom::Role::kList;
  tree_update.nodes[0].child_ids = {2, 3, 4};
  tree_update.nodes[0].AddIntAttribute(ax::mojom::IntAttribute::kSetSize, 7);
  tree_update.nodes[1].id = 2;
  tree_update.nodes[1].role = ax::mojom::Role::kListItem;
  tree_update.nodes[2].id = 3;
  tree_update.nodes[2].role = ax::mojom::Role::kListItem;
  tree_update.nodes[3].id = 4;
  tree_update.nodes[3].role = ax::mojom::Role::kListItem;
  AXTree tree(tree_update);

  // Items should inherit SetSize from ordered set if not specified.
  AXNode* item1 = tree.GetFromId(2);
  EXPECT_OPTIONAL_EQ(7, item1->GetSetSize());
  EXPECT_OPTIONAL_EQ(1, item1->GetPosInSet());
  AXNode* item2 = tree.GetFromId(3);
  EXPECT_OPTIONAL_EQ(7, item2->GetSetSize());
  EXPECT_OPTIONAL_EQ(2, item2->GetPosInSet());
  AXNode* item3 = tree.GetFromId(4);
  EXPECT_OPTIONAL_EQ(7, item3->GetSetSize());
  EXPECT_OPTIONAL_EQ(3, item3->GetPosInSet());
}

// Tests GetPosInSet and GetSetSize on a list containing various roles.
// Roles for items and associated ordered set should match up.
TEST(AXTreeTest, SetSizePosInSetDiverseList) {
  AXTreeUpdate tree_update;
  tree_update.root_id = 1;
  tree_update.nodes.resize(6);
  tree_update.nodes[0].id = 1;
  tree_update.nodes[0].role = ax::mojom::Role::kMenu;
  tree_update.nodes[0].child_ids = {2, 3, 4, 5, 6};
  tree_update.nodes[1].id = 2;
  tree_update.nodes[1].role = ax::mojom::Role::kMenuItem;  // 1 of 4
  tree_update.nodes[2].id = 3;
  tree_update.nodes[2].role = ax::mojom::Role::kMenuItemCheckBox;  // 2 of 4
  tree_update.nodes[3].id = 4;
  tree_update.nodes[3].role = ax::mojom::Role::kMenuItemRadio;  // 3 of 4
  tree_update.nodes[4].id = 5;
  tree_update.nodes[4].role = ax::mojom::Role::kMenuItem;  // 4 of 4
  tree_update.nodes[5].id = 6;
  tree_update.nodes[5].role = ax::mojom::Role::kTab;  // 0 of 0
  AXTree tree(tree_update);

  // kMenu is allowed to contain: kMenuItem, kMenuItemCheckbox,
  // and kMenuItemRadio. For PosInSet and SetSize purposes, these items
  // are treated as the same role.
  AXNode* item1 = tree.GetFromId(2);
  EXPECT_OPTIONAL_EQ(1, item1->GetPosInSet());
  EXPECT_OPTIONAL_EQ(4, item1->GetSetSize());
  AXNode* checkbox = tree.GetFromId(3);
  EXPECT_OPTIONAL_EQ(2, checkbox->GetPosInSet());
  EXPECT_OPTIONAL_EQ(4, checkbox->GetSetSize());
  AXNode* radio = tree.GetFromId(4);
  EXPECT_OPTIONAL_EQ(3, radio->GetPosInSet());
  EXPECT_OPTIONAL_EQ(4, radio->GetSetSize());
  AXNode* item3 = tree.GetFromId(5);
  EXPECT_OPTIONAL_EQ(4, item3->GetPosInSet());
  EXPECT_OPTIONAL_EQ(4, item3->GetSetSize());
  AXNode* tab = tree.GetFromId(6);
  EXPECT_FALSE(tab->GetPosInSet());
  EXPECT_FALSE(tab->GetSetSize());
}

// Tests GetPosInSet and GetSetSize on a nested list.
TEST(AXTreeTest, SetSizePosInSetNestedList) {
  AXTreeUpdate tree_update;
  tree_update.root_id = 1;
  tree_update.nodes.resize(7);
  tree_update.nodes[0].id = 1;
  tree_update.nodes[0].role = ax::mojom::Role::kList;
  tree_update.nodes[0].child_ids = {2, 3, 4, 7};
  tree_update.nodes[1].id = 2;
  tree_update.nodes[1].role = ax::mojom::Role::kListItem;
  tree_update.nodes[2].id = 3;
  tree_update.nodes[2].role = ax::mojom::Role::kListItem;
  tree_update.nodes[3].id = 4;
  tree_update.nodes[3].role = ax::mojom::Role::kList;
  tree_update.nodes[3].child_ids = {5, 6};
  tree_update.nodes[4].id = 5;
  tree_update.nodes[4].role = ax::mojom::Role::kListItem;
  tree_update.nodes[5].id = 6;
  tree_update.nodes[5].role = ax::mojom::Role::kListItem;
  tree_update.nodes[6].id = 7;
  tree_update.nodes[6].role = ax::mojom::Role::kListItem;
  AXTree tree(tree_update);

  AXNode* outer_item1 = tree.GetFromId(2);
  EXPECT_OPTIONAL_EQ(1, outer_item1->GetPosInSet());
  EXPECT_OPTIONAL_EQ(3, outer_item1->GetSetSize());
  AXNode* outer_item2 = tree.GetFromId(3);
  EXPECT_OPTIONAL_EQ(2, outer_item2->GetPosInSet());
  EXPECT_OPTIONAL_EQ(3, outer_item2->GetSetSize());

  AXNode* inner_item1 = tree.GetFromId(5);
  EXPECT_OPTIONAL_EQ(1, inner_item1->GetPosInSet());
  EXPECT_OPTIONAL_EQ(2, inner_item1->GetSetSize());
  AXNode* inner_item2 = tree.GetFromId(6);
  EXPECT_OPTIONAL_EQ(2, inner_item2->GetPosInSet());
  EXPECT_OPTIONAL_EQ(2, inner_item2->GetSetSize());

  AXNode* outer_item3 = tree.GetFromId(7);
  EXPECT_OPTIONAL_EQ(3, outer_item3->GetPosInSet());
  EXPECT_OPTIONAL_EQ(3, outer_item3->GetSetSize());
}

// Tests PosInSet can be calculated if one item specifies PosInSet, but
// other assignments are missing.
TEST(AXTreeTest, PosInSetMissing) {
  AXTreeUpdate tree_update;
  tree_update.root_id = 1;
  tree_update.nodes.resize(4);
  tree_update.nodes[0].id = 1;
  tree_update.nodes[0].role = ax::mojom::Role::kList;
  tree_update.nodes[0].child_ids = {2, 3, 4};
  tree_update.nodes[0].AddIntAttribute(ax::mojom::IntAttribute::kSetSize, 20);
  tree_update.nodes[1].id = 2;
  tree_update.nodes[1].role = ax::mojom::Role::kListItem;
  tree_update.nodes[2].id = 3;
  tree_update.nodes[2].role = ax::mojom::Role::kListItem;
  tree_update.nodes[2].AddIntAttribute(ax::mojom::IntAttribute::kPosInSet, 13);
  tree_update.nodes[3].id = 4;
  tree_update.nodes[3].role = ax::mojom::Role::kListItem;
  AXTree tree(tree_update);

  // Item1 should have pos of 12, since item2 is assigned a pos of 13.
  AXNode* item1 = tree.GetFromId(2);
  EXPECT_OPTIONAL_EQ(1, item1->GetPosInSet());
  EXPECT_OPTIONAL_EQ(20, item1->GetSetSize());
  AXNode* item2 = tree.GetFromId(3);
  EXPECT_OPTIONAL_EQ(13, item2->GetPosInSet());
  EXPECT_OPTIONAL_EQ(20, item2->GetSetSize());
  // Item2 should have pos of 14, since item2 is assigned a pos of 13.
  AXNode* item3 = tree.GetFromId(4);
  EXPECT_OPTIONAL_EQ(14, item3->GetPosInSet());
  EXPECT_OPTIONAL_EQ(20, item3->GetSetSize());
}

// A more difficult test that involves missing PosInSet and SetSize values.
TEST(AXTreeTest, SetSizePosInSetMissingDifficult) {
  AXTreeUpdate tree_update;
  tree_update.root_id = 1;
  tree_update.nodes.resize(6);
  tree_update.nodes[0].id = 1;
  tree_update.nodes[0].role = ax::mojom::Role::kList;
  tree_update.nodes[0].child_ids = {2, 3, 4, 5, 6};
  tree_update.nodes[1].id = 2;
  tree_update.nodes[1].role = ax::mojom::Role::kListItem;  // 1 of 11
  tree_update.nodes[2].id = 3;
  tree_update.nodes[2].role = ax::mojom::Role::kListItem;
  tree_update.nodes[2].AddIntAttribute(ax::mojom::IntAttribute::kPosInSet,
                                       5);  // 5 of 11
  tree_update.nodes[3].id = 4;
  tree_update.nodes[3].role = ax::mojom::Role::kListItem;  // 6 of 11
  tree_update.nodes[4].id = 5;
  tree_update.nodes[4].role = ax::mojom::Role::kListItem;
  tree_update.nodes[4].AddIntAttribute(ax::mojom::IntAttribute::kPosInSet,
                                       10);  // 10 of 11
  tree_update.nodes[5].id = 6;
  tree_update.nodes[5].role = ax::mojom::Role::kListItem;  // 11 of 11
  AXTree tree(tree_update);

  AXNode* item1 = tree.GetFromId(2);
  EXPECT_OPTIONAL_EQ(1, item1->GetPosInSet());
  EXPECT_OPTIONAL_EQ(11, item1->GetSetSize());
  AXNode* item2 = tree.GetFromId(3);
  EXPECT_OPTIONAL_EQ(5, item2->GetPosInSet());
  EXPECT_OPTIONAL_EQ(11, item2->GetSetSize());
  AXNode* item3 = tree.GetFromId(4);
  EXPECT_OPTIONAL_EQ(6, item3->GetPosInSet());
  EXPECT_OPTIONAL_EQ(11, item3->GetSetSize());
  AXNode* item4 = tree.GetFromId(5);
  EXPECT_OPTIONAL_EQ(10, item4->GetPosInSet());
  EXPECT_OPTIONAL_EQ(11, item4->GetSetSize());
  AXNode* item5 = tree.GetFromId(6);
  EXPECT_OPTIONAL_EQ(11, item5->GetPosInSet());
  EXPECT_OPTIONAL_EQ(11, item5->GetSetSize());
}

// Tests that code overwrites decreasing SetSize assignments to largest of
// assigned values.
TEST(AXTreeTest, SetSizeDecreasing) {
  AXTreeUpdate tree_update;
  tree_update.root_id = 1;
  tree_update.nodes.resize(4);
  tree_update.nodes[0].id = 1;
  tree_update.nodes[0].role = ax::mojom::Role::kList;
  tree_update.nodes[0].child_ids = {2, 3, 4};
  tree_update.nodes[1].id = 2;
  tree_update.nodes[1].role = ax::mojom::Role::kListItem;  // 1 of 5
  tree_update.nodes[2].id = 3;
  tree_update.nodes[2].role = ax::mojom::Role::kListItem;  // 2 of 5
  tree_update.nodes[2].AddIntAttribute(ax::mojom::IntAttribute::kSetSize, 5);
  tree_update.nodes[3].id = 4;
  tree_update.nodes[3].role = ax::mojom::Role::kListItem;  // 3 of 5
  tree_update.nodes[3].AddIntAttribute(ax::mojom::IntAttribute::kSetSize, 4);
  AXTree tree(tree_update);

  AXNode* item1 = tree.GetFromId(2);
  EXPECT_OPTIONAL_EQ(1, item1->GetPosInSet());
  EXPECT_OPTIONAL_EQ(5, item1->GetSetSize());
  AXNode* item2 = tree.GetFromId(3);
  EXPECT_OPTIONAL_EQ(2, item2->GetPosInSet());
  EXPECT_OPTIONAL_EQ(5, item2->GetSetSize());
  AXNode* item3 = tree.GetFromId(4);
  EXPECT_OPTIONAL_EQ(3, item3->GetPosInSet());
  EXPECT_OPTIONAL_EQ(5, item3->GetSetSize());
}

// Tests that code overwrites decreasing PosInSet values.
TEST(AXTreeTest, PosInSetDecreasing) {
  AXTreeUpdate tree_update;
  tree_update.root_id = 1;
  tree_update.nodes.resize(4);
  tree_update.nodes[0].id = 1;
  tree_update.nodes[0].role = ax::mojom::Role::kList;
  tree_update.nodes[0].child_ids = {2, 3, 4};
  tree_update.nodes[1].id = 2;
  tree_update.nodes[1].role = ax::mojom::Role::kListItem;  // 1 of 8
  tree_update.nodes[2].id = 3;
  tree_update.nodes[2].role = ax::mojom::Role::kListItem;  // 7 of 8
  tree_update.nodes[2].AddIntAttribute(ax::mojom::IntAttribute::kPosInSet, 7);
  tree_update.nodes[3].id = 4;
  tree_update.nodes[3].role = ax::mojom::Role::kListItem;  // 8 of 8
  tree_update.nodes[3].AddIntAttribute(ax::mojom::IntAttribute::kPosInSet, 3);
  AXTree tree(tree_update);

  AXNode* item1 = tree.GetFromId(2);
  EXPECT_OPTIONAL_EQ(1, item1->GetPosInSet());
  EXPECT_OPTIONAL_EQ(8, item1->GetSetSize());
  AXNode* item2 = tree.GetFromId(3);
  EXPECT_OPTIONAL_EQ(7, item2->GetPosInSet());
  EXPECT_OPTIONAL_EQ(8, item2->GetSetSize());
  AXNode* item3 = tree.GetFromId(4);
  EXPECT_OPTIONAL_EQ(8, item3->GetPosInSet());
  EXPECT_OPTIONAL_EQ(8, item3->GetSetSize());
}

// Tests that code overwrites duplicate PosInSet values. Note this case is
// tricky; an update to the second element causes an update to the third
// element.
TEST(AXTreeTest, PosInSetDuplicates) {
  AXTreeUpdate tree_update;
  tree_update.root_id = 1;
  tree_update.nodes.resize(4);
  tree_update.nodes[0].id = 1;
  tree_update.nodes[0].role = ax::mojom::Role::kList;
  tree_update.nodes[0].child_ids = {2, 3, 4};
  tree_update.nodes[1].id = 2;
  tree_update.nodes[1].role = ax::mojom::Role::kListItem;  // 6 of 8
  tree_update.nodes[1].AddIntAttribute(ax::mojom::IntAttribute::kPosInSet, 6);
  tree_update.nodes[2].id = 3;
  tree_update.nodes[2].role = ax::mojom::Role::kListItem;  // 7 of 8
  tree_update.nodes[2].AddIntAttribute(ax::mojom::IntAttribute::kPosInSet, 6);
  tree_update.nodes[3].id = 4;
  tree_update.nodes[3].role = ax::mojom::Role::kListItem;  // 8 of 8
  tree_update.nodes[3].AddIntAttribute(ax::mojom::IntAttribute::kPosInSet, 7);
  AXTree tree(tree_update);

  AXNode* item1 = tree.GetFromId(2);
  EXPECT_OPTIONAL_EQ(6, item1->GetPosInSet());
  EXPECT_OPTIONAL_EQ(8, item1->GetSetSize());
  AXNode* item2 = tree.GetFromId(3);
  EXPECT_OPTIONAL_EQ(7, item2->GetPosInSet());
  EXPECT_OPTIONAL_EQ(8, item2->GetSetSize());
  AXNode* item3 = tree.GetFromId(4);
  EXPECT_OPTIONAL_EQ(8, item3->GetPosInSet());
  EXPECT_OPTIONAL_EQ(8, item3->GetSetSize());
}

// Tests GetPosInSet and GetSetSize when some list items are nested in a generic
// container.
TEST(AXTreeTest, SetSizePosInSetNestedContainer) {
  AXTreeUpdate tree_update;
  tree_update.root_id = 1;
  tree_update.nodes.resize(7);
  tree_update.nodes[0].id = 1;
  tree_update.nodes[0].role = ax::mojom::Role::kList;
  tree_update.nodes[0].child_ids = {2, 3, 7};
  tree_update.nodes[1].id = 2;
  tree_update.nodes[1].role = ax::mojom::Role::kListItem;  // 1 of 4
  tree_update.nodes[2].id = 3;
  tree_update.nodes[2].role = ax::mojom::Role::kGenericContainer;
  tree_update.nodes[2].child_ids = {4, 5};
  tree_update.nodes[3].id = 4;
  tree_update.nodes[3].role = ax::mojom::Role::kListItem;  // 2 of 4
  tree_update.nodes[4].id = 5;
  tree_update.nodes[4].role = ax::mojom::Role::kIgnored;
  tree_update.nodes[4].child_ids = {6};
  tree_update.nodes[5].id = 6;
  tree_update.nodes[5].role = ax::mojom::Role::kListItem;  // 3 of 4
  tree_update.nodes[6].id = 7;
  tree_update.nodes[6].role = ax::mojom::Role::kListItem;  // 4 of 4
  AXTree tree(tree_update);

  AXNode* item1 = tree.GetFromId(2);
  EXPECT_OPTIONAL_EQ(1, item1->GetPosInSet());
  EXPECT_OPTIONAL_EQ(4, item1->GetSetSize());
  AXNode* g_container = tree.GetFromId(3);
  EXPECT_FALSE(g_container->GetPosInSet());
  EXPECT_FALSE(g_container->GetSetSize());
  AXNode* item2 = tree.GetFromId(4);
  EXPECT_OPTIONAL_EQ(2, item2->GetPosInSet());
  EXPECT_OPTIONAL_EQ(4, item2->GetSetSize());
  AXNode* ignored = tree.GetFromId(5);
  EXPECT_FALSE(ignored->GetPosInSet());
  EXPECT_FALSE(ignored->GetSetSize());
  AXNode* item3 = tree.GetFromId(6);
  EXPECT_OPTIONAL_EQ(3, item3->GetPosInSet());
  EXPECT_OPTIONAL_EQ(4, item3->GetSetSize());
  AXNode* item4 = tree.GetFromId(7);
  EXPECT_OPTIONAL_EQ(4, item4->GetPosInSet());
  EXPECT_OPTIONAL_EQ(4, item4->GetSetSize());
}

// Tests GetSetSize and GetPosInSet are correct, even when list items change.
// Tests that previously calculated values are not used after tree is updated.
TEST(AXTreeTest, SetSizePosInSetDeleteItem) {
  AXTreeUpdate initial_state;
  initial_state.root_id = 1;
  initial_state.nodes.resize(4);
  initial_state.nodes[0].id = 1;
  initial_state.nodes[0].role = ax::mojom::Role::kList;
  initial_state.nodes[0].child_ids = {2, 3, 4};
  initial_state.nodes[1].id = 2;
  initial_state.nodes[1].role = ax::mojom::Role::kListItem;  // 1 of 3
  initial_state.nodes[2].id = 3;
  initial_state.nodes[2].role = ax::mojom::Role::kListItem;  // 2 of 3
  initial_state.nodes[3].id = 4;
  initial_state.nodes[3].role = ax::mojom::Role::kListItem;  // 3 of 3
  AXTree tree(initial_state);

  AXNode* item1 = tree.GetFromId(2);
  EXPECT_OPTIONAL_EQ(1, item1->GetPosInSet());
  EXPECT_OPTIONAL_EQ(3, item1->GetSetSize());
  AXNode* item2 = tree.GetFromId(3);
  EXPECT_OPTIONAL_EQ(2, item2->GetPosInSet());
  EXPECT_OPTIONAL_EQ(3, item2->GetSetSize());
  AXNode* item3 = tree.GetFromId(4);
  EXPECT_OPTIONAL_EQ(3, item3->GetPosInSet());
  EXPECT_OPTIONAL_EQ(3, item3->GetSetSize());

  // TreeUpdates only need to describe what changed in tree.
  AXTreeUpdate update = initial_state;
  update.nodes.resize(1);
  update.nodes[0].child_ids = {2, 4};  // Delete item 2 of 3 from list.
  ASSERT_TRUE(tree.Unserialize(update));

  AXNode* new_item1 = tree.GetFromId(2);
  EXPECT_OPTIONAL_EQ(1, new_item1->GetPosInSet());
  EXPECT_OPTIONAL_EQ(2, new_item1->GetSetSize());
  AXNode* new_item2 = tree.GetFromId(4);
  EXPECT_OPTIONAL_EQ(2, new_item2->GetPosInSet());
  EXPECT_OPTIONAL_EQ(2, new_item2->GetSetSize());
}

// Tests GetSetSize and GetPosInSet are correct, even when list items change.
// This test adds an item to the front of a list, which invalidates previously
// calculated PosInSet and SetSize values. Tests that old values are not
// used after tree is updated.
TEST(AXTreeTest, SetSizePosInSetAddItem) {
  AXTreeUpdate initial_state;
  initial_state.root_id = 1;
  initial_state.nodes.resize(4);
  initial_state.nodes[0].id = 1;
  initial_state.nodes[0].role = ax::mojom::Role::kList;
  initial_state.nodes[0].child_ids = {2, 3, 4};
  initial_state.nodes[1].id = 2;
  initial_state.nodes[1].role = ax::mojom::Role::kListItem;  // 1 of 3
  initial_state.nodes[2].id = 3;
  initial_state.nodes[2].role = ax::mojom::Role::kListItem;  // 2 of 3
  initial_state.nodes[3].id = 4;
  initial_state.nodes[3].role = ax::mojom::Role::kListItem;  // 3 of 3
  AXTree tree(initial_state);

  AXNode* item1 = tree.GetFromId(2);
  EXPECT_OPTIONAL_EQ(1, item1->GetPosInSet());
  EXPECT_OPTIONAL_EQ(3, item1->GetSetSize());
  AXNode* item2 = tree.GetFromId(3);
  EXPECT_OPTIONAL_EQ(2, item2->GetPosInSet());
  EXPECT_OPTIONAL_EQ(3, item2->GetSetSize());
  AXNode* item3 = tree.GetFromId(4);
  EXPECT_OPTIONAL_EQ(3, item3->GetPosInSet());
  EXPECT_OPTIONAL_EQ(3, item3->GetSetSize());

  // Insert an item at the beginning of the list.
  AXTreeUpdate update = initial_state;
  update.nodes.resize(2);
  update.nodes[0].id = 1;
  update.nodes[0].child_ids = {5, 2, 3, 4};
  update.nodes[1].id = 5;
  update.nodes[1].role = ax::mojom::Role::kListItem;
  ASSERT_TRUE(tree.Unserialize(update));

  AXNode* new_item1 = tree.GetFromId(5);
  EXPECT_OPTIONAL_EQ(1, new_item1->GetPosInSet());
  EXPECT_OPTIONAL_EQ(4, new_item1->GetSetSize());
  AXNode* new_item2 = tree.GetFromId(2);
  EXPECT_OPTIONAL_EQ(2, new_item2->GetPosInSet());
  EXPECT_OPTIONAL_EQ(4, new_item2->GetSetSize());
  AXNode* new_item3 = tree.GetFromId(3);
  EXPECT_OPTIONAL_EQ(3, new_item3->GetPosInSet());
  EXPECT_OPTIONAL_EQ(4, new_item3->GetSetSize());
  AXNode* new_item4 = tree.GetFromId(4);
  EXPECT_OPTIONAL_EQ(4, new_item4->GetPosInSet());
  EXPECT_OPTIONAL_EQ(4, new_item4->GetSetSize());
}

// Tests that the outerlying ordered set reports a SetSize. Ordered sets
// should not report a PosInSet value other than 0, since they are not
// considered to be items within a set (even when nested).
TEST(AXTreeTest, OrderedSetReportsSetSize) {
  AXTreeUpdate tree_update;
  tree_update.root_id = 1;
  tree_update.nodes.resize(12);
  tree_update.nodes[0].id = 1;
  tree_update.nodes[0].role = ax::mojom::Role::kList;  // SetSize = 3
  tree_update.nodes[0].child_ids = {2, 3, 4, 7, 8, 9, 12};
  tree_update.nodes[1].id = 2;
  tree_update.nodes[1].role = ax::mojom::Role::kListItem;  // 1 of 3
  tree_update.nodes[2].id = 3;
  tree_update.nodes[2].role = ax::mojom::Role::kListItem;  // 2 of 3
  tree_update.nodes[3].id = 4;
  tree_update.nodes[3].role = ax::mojom::Role::kList;  // SetSize = 2
  tree_update.nodes[3].child_ids = {5, 6};
  tree_update.nodes[4].id = 5;
  tree_update.nodes[4].role = ax::mojom::Role::kListItem;  // 1 of 2
  tree_update.nodes[5].id = 6;
  tree_update.nodes[5].role = ax::mojom::Role::kListItem;  // 2 of 2
  tree_update.nodes[6].id = 7;
  tree_update.nodes[6].role = ax::mojom::Role::kListItem;  // 3 of 3
  tree_update.nodes[7].id = 8;
  tree_update.nodes[7].role = ax::mojom::Role::kList;  // SetSize = 0
  tree_update.nodes[8].id = 9;
  tree_update.nodes[8].role =
      ax::mojom::Role::kList;  // SetSize = 1 because only 1
                               // item whose role matches
  tree_update.nodes[8].child_ids = {10, 11};
  tree_update.nodes[9].id = 10;
  tree_update.nodes[9].role = ax::mojom::Role::kArticle;
  tree_update.nodes[10].id = 11;
  tree_update.nodes[10].role = ax::mojom::Role::kListItem;
  tree_update.nodes[11].id = 12;
  tree_update.nodes[11].role = ax::mojom::Role::kList;
  tree_update.nodes[11].AddIntAttribute(ax::mojom::IntAttribute::kSetSize, 5);
  AXTree tree(tree_update);

  AXNode* outer_list = tree.GetFromId(1);
  EXPECT_FALSE(outer_list->GetPosInSet());
  EXPECT_OPTIONAL_EQ(3, outer_list->GetSetSize());
  AXNode* outer_list_item1 = tree.GetFromId(2);
  EXPECT_OPTIONAL_EQ(1, outer_list_item1->GetPosInSet());
  EXPECT_OPTIONAL_EQ(3, outer_list_item1->GetSetSize());
  AXNode* outer_list_item2 = tree.GetFromId(3);
  EXPECT_OPTIONAL_EQ(2, outer_list_item2->GetPosInSet());
  EXPECT_OPTIONAL_EQ(3, outer_list_item2->GetSetSize());
  AXNode* outer_list_item3 = tree.GetFromId(7);
  EXPECT_OPTIONAL_EQ(3, outer_list_item3->GetPosInSet());
  EXPECT_OPTIONAL_EQ(3, outer_list_item3->GetSetSize());

  AXNode* inner_list1 = tree.GetFromId(4);
  EXPECT_FALSE(inner_list1->GetPosInSet());
  EXPECT_OPTIONAL_EQ(2, inner_list1->GetSetSize());
  AXNode* inner_list1_item1 = tree.GetFromId(5);
  EXPECT_OPTIONAL_EQ(1, inner_list1_item1->GetPosInSet());
  EXPECT_OPTIONAL_EQ(2, inner_list1_item1->GetSetSize());
  AXNode* inner_list1_item2 = tree.GetFromId(6);
  EXPECT_OPTIONAL_EQ(2, inner_list1_item2->GetPosInSet());
  EXPECT_OPTIONAL_EQ(2, inner_list1_item2->GetSetSize());

  AXNode* inner_list2 = tree.GetFromId(8);  // Empty list
  EXPECT_FALSE(inner_list2->GetPosInSet());
  EXPECT_OPTIONAL_EQ(0, inner_list2->GetSetSize());

  AXNode* inner_list3 = tree.GetFromId(9);
  EXPECT_FALSE(inner_list3->GetPosInSet());
  // Only 1 item whose role matches.
  EXPECT_OPTIONAL_EQ(1, inner_list3->GetSetSize());
  AXNode* inner_list3_article1 = tree.GetFromId(10);
  EXPECT_FALSE(inner_list3_article1->GetPosInSet());
  EXPECT_FALSE(inner_list3_article1->GetSetSize());
  AXNode* inner_list3_item1 = tree.GetFromId(11);
  EXPECT_OPTIONAL_EQ(1, inner_list3_item1->GetPosInSet());
  EXPECT_OPTIONAL_EQ(1, inner_list3_item1->GetSetSize());

  AXNode* inner_list4 = tree.GetFromId(12);
  EXPECT_FALSE(inner_list4->GetPosInSet());
  // Even though list is empty, kSetSize attribute was set, so it takes
  // precedence
  EXPECT_OPTIONAL_EQ(5, inner_list4->GetSetSize());
}

// Tests GetPosInSet and GetSetSize code on invalid input.
TEST(AXTreeTest, SetSizePosInSetInvalid) {
  AXTreeUpdate tree_update;
  tree_update.root_id = 1;
  tree_update.nodes.resize(3);
  tree_update.nodes[0].id = 1;
  tree_update.nodes[0].role = ax::mojom::Role::kListItem;  // 0 of 0
  tree_update.nodes[0].child_ids = {2, 3};
  tree_update.nodes[1].id = 2;
  tree_update.nodes[1].role = ax::mojom::Role::kListItem;
  tree_update.nodes[1].AddIntAttribute(ax::mojom::IntAttribute::kPosInSet,
                                       4);  // 0 of 0
  tree_update.nodes[2].id = 3;
  tree_update.nodes[2].role = ax::mojom::Role::kListItem;
  AXTree tree(tree_update);

  AXNode* item1 = tree.GetFromId(1);
  EXPECT_FALSE(item1->GetPosInSet());
  EXPECT_FALSE(item1->GetSetSize());
  AXNode* item2 = tree.GetFromId(2);
  EXPECT_FALSE(item2->GetPosInSet());
  EXPECT_FALSE(item2->GetSetSize());
  AXNode* item3 = tree.GetFromId(3);
  EXPECT_FALSE(item3->GetPosInSet());
  EXPECT_FALSE(item3->GetSetSize());
}

// Tests GetPosInSet and GetSetSize code on kRadioButtons. Radio buttons
// behave differently than other item-like elements; most notably, they do not
// need to be contained within an ordered set to report a PosInSet or SetSize.
TEST(AXTreeTest, SetSizePosInSetRadioButtons) {
  AXTreeUpdate tree_update;
  tree_update.root_id = 1;
  tree_update.nodes.resize(13);
  tree_update.nodes[0].id = 1;
  tree_update.nodes[0].child_ids = {2, 3, 4, 10, 13};
  // This test passes because the root node is a kRadioGroup.
  tree_update.nodes[0].role = ax::mojom::Role::kRadioGroup;  // Setsize = 5;

  // Radio buttons are not required to be contained within an ordered set.
  tree_update.nodes[1].id = 2;
  tree_update.nodes[1].role = ax::mojom::Role::kRadioButton;  // 1 of 5
  tree_update.nodes[1].AddStringAttribute(ax::mojom::StringAttribute::kName,
                                          "sports");
  tree_update.nodes[1].AddIntAttribute(ax::mojom::IntAttribute::kPosInSet, 1);
  tree_update.nodes[2].id = 3;
  tree_update.nodes[2].role = ax::mojom::Role::kRadioButton;  // 2 of 5
  tree_update.nodes[2].AddStringAttribute(ax::mojom::StringAttribute::kName,
                                          "books");
  tree_update.nodes[2].AddIntAttribute(ax::mojom::IntAttribute::kPosInSet, 2);
  tree_update.nodes[2].AddIntAttribute(ax::mojom::IntAttribute::kSetSize, 5);

  // Radio group with nested generic container.
  tree_update.nodes[3].id = 4;
  tree_update.nodes[3].role = ax::mojom::Role::kRadioGroup;  // setsize = 4
  tree_update.nodes[3].child_ids = {5, 6, 7};
  tree_update.nodes[4].id = 5;
  tree_update.nodes[4].role = ax::mojom::Role::kRadioButton;
  tree_update.nodes[4].AddStringAttribute(ax::mojom::StringAttribute::kName,
                                          "recipes");  // 1 of 4
  tree_update.nodes[5].id = 6;
  tree_update.nodes[5].role = ax::mojom::Role::kRadioButton;
  tree_update.nodes[5].AddStringAttribute(ax::mojom::StringAttribute::kName,
                                          "recipes");  // 2 of 4
  tree_update.nodes[6].id = 7;
  tree_update.nodes[6].role = ax::mojom::Role::kGenericContainer;
  tree_update.nodes[6].child_ids = {8, 9};
  tree_update.nodes[7].id = 8;
  tree_update.nodes[7].role = ax::mojom::Role::kRadioButton;
  tree_update.nodes[7].AddStringAttribute(ax::mojom::StringAttribute::kName,
                                          "recipes");  // 3 of 4
  tree_update.nodes[8].id = 9;
  tree_update.nodes[8].role = ax::mojom::Role::kRadioButton;
  tree_update.nodes[8].AddStringAttribute(ax::mojom::StringAttribute::kName,
                                          "recipes");  // 4 of 4

  // Radio buttons are allowed to be contained within forms.
  tree_update.nodes[9].id = 10;
  tree_update.nodes[9].role = ax::mojom::Role::kForm;
  tree_update.nodes[9].child_ids = {11, 12};
  tree_update.nodes[10].id = 11;
  tree_update.nodes[10].role = ax::mojom::Role::kRadioButton;
  tree_update.nodes[10].AddStringAttribute(ax::mojom::StringAttribute::kName,
                                           "cities");  // 1 of 2
  tree_update.nodes[11].id = 12;
  tree_update.nodes[11].role = ax::mojom::Role::kRadioButton;
  tree_update.nodes[11].AddStringAttribute(ax::mojom::StringAttribute::kName,
                                           "cities");  // 2 of 2
  tree_update.nodes[12].id = 13;
  tree_update.nodes[12].role = ax::mojom::Role::kRadioButton;  // 4 of 5
  tree_update.nodes[12].AddStringAttribute(ax::mojom::StringAttribute::kName,
                                           "sports");
  tree_update.nodes[12].AddIntAttribute(ax::mojom::IntAttribute::kPosInSet, 4);

  AXTree tree(tree_update);

  AXNode* sports_button1 = tree.GetFromId(2);
  EXPECT_OPTIONAL_EQ(1, sports_button1->GetPosInSet());
  EXPECT_OPTIONAL_EQ(5, sports_button1->GetSetSize());
  AXNode* books_button = tree.GetFromId(3);
  EXPECT_OPTIONAL_EQ(2, books_button->GetPosInSet());
  EXPECT_OPTIONAL_EQ(5, books_button->GetSetSize());

  AXNode* radiogroup1 = tree.GetFromId(4);
  EXPECT_FALSE(radiogroup1->GetPosInSet());
  EXPECT_OPTIONAL_EQ(4, radiogroup1->GetSetSize());
  AXNode* recipes_button1 = tree.GetFromId(5);
  EXPECT_OPTIONAL_EQ(1, recipes_button1->GetPosInSet());
  EXPECT_OPTIONAL_EQ(4, recipes_button1->GetSetSize());
  AXNode* recipes_button2 = tree.GetFromId(6);
  EXPECT_OPTIONAL_EQ(2, recipes_button2->GetPosInSet());
  EXPECT_OPTIONAL_EQ(4, recipes_button2->GetSetSize());

  AXNode* generic_container = tree.GetFromId(7);
  EXPECT_FALSE(generic_container->GetPosInSet());
  EXPECT_FALSE(generic_container->GetSetSize());
  AXNode* recipes_button3 = tree.GetFromId(8);
  EXPECT_OPTIONAL_EQ(3, recipes_button3->GetPosInSet());
  EXPECT_OPTIONAL_EQ(4, recipes_button3->GetSetSize());
  AXNode* recipes_button4 = tree.GetFromId(9);
  EXPECT_OPTIONAL_EQ(4, recipes_button4->GetPosInSet());
  EXPECT_OPTIONAL_EQ(4, recipes_button4->GetSetSize());

  // Elements with role kForm shouldn't report posinset or setsize
  AXNode* form = tree.GetFromId(10);
  EXPECT_FALSE(form->GetPosInSet());
  EXPECT_FALSE(form->GetSetSize());
  AXNode* cities_button1 = tree.GetFromId(11);
  EXPECT_OPTIONAL_EQ(1, cities_button1->GetPosInSet());
  EXPECT_OPTIONAL_EQ(2, cities_button1->GetSetSize());
  AXNode* cities_button2 = tree.GetFromId(12);
  EXPECT_OPTIONAL_EQ(2, cities_button2->GetPosInSet());
  EXPECT_OPTIONAL_EQ(2, cities_button2->GetSetSize());

  AXNode* sports_button2 = tree.GetFromId(13);
  EXPECT_OPTIONAL_EQ(4, sports_button2->GetPosInSet());
  EXPECT_OPTIONAL_EQ(5, sports_button2->GetSetSize());
}

// Tests GetPosInSet and GetSetSize on a list that includes radio buttons.
// Note that radio buttons do not contribute to the SetSize of the outerlying
// list.
TEST(AXTreeTest, SetSizePosInSetRadioButtonsInList) {
  AXTreeUpdate tree_update;
  tree_update.root_id = 1;
  tree_update.nodes.resize(6);
  tree_update.nodes[0].id = 1;
  tree_update.nodes[0].role =
      ax::mojom::Role::kList;  // SetSize = 2, since only base::contains 2
                               // ListItems
  tree_update.nodes[0].child_ids = {2, 3, 4, 5, 6};

  tree_update.nodes[1].id = 2;
  tree_update.nodes[1].role = ax::mojom::Role::kRadioButton;  // 1 of 3
  tree_update.nodes[2].id = 3;
  tree_update.nodes[2].role = ax::mojom::Role::kListItem;  // 1 of 2
  tree_update.nodes[3].id = 4;
  tree_update.nodes[3].role = ax::mojom::Role::kRadioButton;  // 2 of 3
  tree_update.nodes[4].id = 5;
  tree_update.nodes[4].role = ax::mojom::Role::kListItem;  // 2 of 2
  tree_update.nodes[5].id = 6;
  tree_update.nodes[5].role = ax::mojom::Role::kRadioButton;  // 3 of 3
  AXTree tree(tree_update);

  AXNode* list = tree.GetFromId(1);
  EXPECT_FALSE(list->GetPosInSet());
  EXPECT_OPTIONAL_EQ(2, list->GetSetSize());

  AXNode* radiobutton1 = tree.GetFromId(2);
  EXPECT_OPTIONAL_EQ(1, radiobutton1->GetPosInSet());
  EXPECT_OPTIONAL_EQ(3, radiobutton1->GetSetSize());
  AXNode* item1 = tree.GetFromId(3);
  EXPECT_OPTIONAL_EQ(1, item1->GetPosInSet());
  EXPECT_OPTIONAL_EQ(2, item1->GetSetSize());
  AXNode* radiobutton2 = tree.GetFromId(4);
  EXPECT_OPTIONAL_EQ(2, radiobutton2->GetPosInSet());
  EXPECT_OPTIONAL_EQ(3, radiobutton2->GetSetSize());
  AXNode* item2 = tree.GetFromId(5);
  EXPECT_OPTIONAL_EQ(2, item2->GetPosInSet());
  EXPECT_OPTIONAL_EQ(2, item2->GetSetSize());
  AXNode* radiobutton3 = tree.GetFromId(6);
  EXPECT_OPTIONAL_EQ(3, radiobutton3->GetPosInSet());
  EXPECT_OPTIONAL_EQ(3, radiobutton3->GetSetSize());

  // Ensure that the setsize of list was not modified after calling GetPosInSet
  // and GetSetSize on kRadioButtons.
  EXPECT_FALSE(list->GetPosInSet());
  EXPECT_OPTIONAL_EQ(2, list->GetSetSize());
}

// Tests GetPosInSet and GetSetSize on a flat tree representation. According
// to the tree representation, the three elements are siblings. However,
// due to the presence of the kHierarchicalLevel attribute, they all belong
// to different sets.
TEST(AXTreeTest, SetSizePosInSetFlatTree) {
  AXTreeUpdate tree_update;
  tree_update.root_id = 1;
  tree_update.nodes.resize(4);
  tree_update.nodes[0].id = 1;
  tree_update.nodes[0].role = ax::mojom::Role::kTree;
  tree_update.nodes[0].child_ids = {2, 3, 4};
  tree_update.nodes[1].id = 2;
  tree_update.nodes[1].role = ax::mojom::Role::kTreeItem;  // 1 of 1
  tree_update.nodes[1].AddIntAttribute(
      ax::mojom::IntAttribute::kHierarchicalLevel, 1);
  tree_update.nodes[2].id = 3;
  tree_update.nodes[2].role = ax::mojom::Role::kTreeItem;  // 1 of 1
  tree_update.nodes[2].AddIntAttribute(
      ax::mojom::IntAttribute::kHierarchicalLevel, 2);
  tree_update.nodes[3].id = 4;
  tree_update.nodes[3].role = ax::mojom::Role::kTreeItem;  // 1 of 1
  tree_update.nodes[3].AddIntAttribute(
      ax::mojom::IntAttribute::kHierarchicalLevel, 3);
  AXTree tree(tree_update);

  AXNode* item1_level1 = tree.GetFromId(2);
  EXPECT_OPTIONAL_EQ(1, item1_level1->GetPosInSet());
  EXPECT_OPTIONAL_EQ(1, item1_level1->GetSetSize());
  AXNode* item1_level2 = tree.GetFromId(3);
  EXPECT_OPTIONAL_EQ(1, item1_level2->GetPosInSet());
  EXPECT_OPTIONAL_EQ(1, item1_level2->GetSetSize());
  AXNode* item1_level3 = tree.GetFromId(4);
  EXPECT_OPTIONAL_EQ(1, item1_level3->GetPosInSet());
  EXPECT_OPTIONAL_EQ(1, item1_level3->GetSetSize());
}

// Tests GetPosInSet and GetSetSize on a flat tree representation, where only
// the level is specified.
TEST(AXTreeTest, SetSizePosInSetFlatTreeLevelsOnly) {
  AXTreeUpdate tree_update;
  tree_update.root_id = 1;
  tree_update.nodes.resize(9);
  tree_update.nodes[0].id = 1;
  tree_update.nodes[0].role = ax::mojom::Role::kTree;
  tree_update.nodes[0].child_ids = {2, 3, 4, 5, 6, 7, 8, 9};
  tree_update.nodes[1].id = 2;
  tree_update.nodes[1].role = ax::mojom::Role::kTreeItem;  // 1 of 3
  tree_update.nodes[1].AddIntAttribute(
      ax::mojom::IntAttribute::kHierarchicalLevel, 1);
  tree_update.nodes[2].id = 3;
  tree_update.nodes[2].role = ax::mojom::Role::kTreeItem;  // 1 of 2
  tree_update.nodes[2].AddIntAttribute(
      ax::mojom::IntAttribute::kHierarchicalLevel, 2);
  tree_update.nodes[3].id = 4;
  tree_update.nodes[3].role = ax::mojom::Role::kTreeItem;  // 2 of 2
  tree_update.nodes[3].AddIntAttribute(
      ax::mojom::IntAttribute::kHierarchicalLevel, 2);
  tree_update.nodes[4].id = 5;
  tree_update.nodes[4].role = ax::mojom::Role::kTreeItem;  // 2 of 3
  tree_update.nodes[4].AddIntAttribute(
      ax::mojom::IntAttribute::kHierarchicalLevel, 1);
  tree_update.nodes[5].id = 6;
  tree_update.nodes[5].role = ax::mojom::Role::kTreeItem;  // 1 of 3
  tree_update.nodes[5].AddIntAttribute(
      ax::mojom::IntAttribute::kHierarchicalLevel, 2);
  tree_update.nodes[6].id = 7;
  tree_update.nodes[6].role = ax::mojom::Role::kTreeItem;  // 2 of 3
  tree_update.nodes[6].AddIntAttribute(
      ax::mojom::IntAttribute::kHierarchicalLevel, 2);
  tree_update.nodes[7].id = 8;
  tree_update.nodes[7].role = ax::mojom::Role::kTreeItem;  // 3 of 3
  tree_update.nodes[7].AddIntAttribute(
      ax::mojom::IntAttribute::kHierarchicalLevel, 2);
  tree_update.nodes[8].id = 9;
  tree_update.nodes[8].role = ax::mojom::Role::kTreeItem;  // 3 of 3
  tree_update.nodes[8].AddIntAttribute(
      ax::mojom::IntAttribute::kHierarchicalLevel, 1);
  AXTree tree(tree_update);

  // The order in which we query the nodes should not matter.
  AXNode* item3_level1 = tree.GetFromId(9);
  EXPECT_OPTIONAL_EQ(3, item3_level1->GetPosInSet());
  EXPECT_OPTIONAL_EQ(3, item3_level1->GetSetSize());
  AXNode* item3_level2a = tree.GetFromId(8);
  EXPECT_OPTIONAL_EQ(3, item3_level2a->GetPosInSet());
  EXPECT_OPTIONAL_EQ(3, item3_level2a->GetSetSize());
  AXNode* item2_level2a = tree.GetFromId(7);
  EXPECT_OPTIONAL_EQ(2, item2_level2a->GetPosInSet());
  EXPECT_OPTIONAL_EQ(3, item2_level2a->GetSetSize());
  AXNode* item1_level2a = tree.GetFromId(6);
  EXPECT_OPTIONAL_EQ(1, item1_level2a->GetPosInSet());
  EXPECT_OPTIONAL_EQ(3, item1_level2a->GetSetSize());
  AXNode* item2_level1 = tree.GetFromId(5);
  EXPECT_OPTIONAL_EQ(2, item2_level1->GetPosInSet());
  EXPECT_OPTIONAL_EQ(3, item2_level1->GetSetSize());
  AXNode* item2_level2 = tree.GetFromId(4);
  EXPECT_OPTIONAL_EQ(2, item2_level2->GetPosInSet());
  EXPECT_OPTIONAL_EQ(2, item2_level2->GetSetSize());
  AXNode* item1_level2 = tree.GetFromId(3);
  EXPECT_OPTIONAL_EQ(1, item1_level2->GetPosInSet());
  EXPECT_OPTIONAL_EQ(2, item1_level2->GetSetSize());
  AXNode* item1_level1 = tree.GetFromId(2);
  EXPECT_OPTIONAL_EQ(1, item1_level1->GetPosInSet());
  EXPECT_OPTIONAL_EQ(3, item1_level1->GetSetSize());
  AXNode* ordered_set = tree.GetFromId(1);
  EXPECT_OPTIONAL_EQ(3, ordered_set->GetSetSize());
}

// Tests that GetPosInSet and GetSetSize work while a tree is being
// unserialized.
TEST(AXTreeTest, SetSizePosInSetSubtreeDeleted) {
  AXTreeUpdate initial_state;
  initial_state.root_id = 1;
  initial_state.nodes.resize(3);
  initial_state.nodes[0].id = 1;
  initial_state.nodes[0].role = ax::mojom::Role::kTree;
  initial_state.nodes[0].child_ids = {2, 3};
  initial_state.nodes[1].id = 2;
  initial_state.nodes[1].role = ax::mojom::Role::kTreeItem;
  initial_state.nodes[2].id = 3;
  initial_state.nodes[2].role = ax::mojom::Role::kTreeItem;
  AXTree tree(initial_state);

  AXNode* tree_node = tree.GetFromId(1);
  AXNode* item = tree.GetFromId(3);

  // This should work normally.
  EXPECT_OPTIONAL_EQ(2, item->GetPosInSet());
  EXPECT_OPTIONAL_EQ(2, item->GetSetSize());

  // Remove item from tree.
  AXTreeUpdate tree_update = initial_state;
  tree_update.nodes.resize(1);
  tree_update.nodes[0].child_ids = {2};

  ASSERT_TRUE(tree.Unserialize(tree_update));

  // These values are lazily created, so to test that they fail when
  // called in the middle of a tree update, fake the update state.
  tree.SetTreeUpdateInProgressState(true);
  ASSERT_FALSE(tree_node->GetPosInSet());
  ASSERT_FALSE(tree_node->GetSetSize());

  // Then reset the state to make sure we have the expected values
  // after |Unserialize|.
  tree.SetTreeUpdateInProgressState(false);
  ASSERT_FALSE(tree_node->GetPosInSet());
  EXPECT_OPTIONAL_EQ(1, tree_node->GetSetSize());
}

// Tests that GetPosInSet and GetSetSize work when there are ignored nodes.
TEST(AXTreeTest, SetSizePosInSetIgnoredItem) {
  AXTreeUpdate initial_state;
  initial_state.root_id = 1;
  initial_state.nodes.resize(3);
  initial_state.nodes[0].id = 1;
  initial_state.nodes[0].role = ax::mojom::Role::kTree;
  initial_state.nodes[0].child_ids = {2, 3};
  initial_state.nodes[1].id = 2;
  initial_state.nodes[1].role = ax::mojom::Role::kTreeItem;
  initial_state.nodes[2].id = 3;
  initial_state.nodes[2].role = ax::mojom::Role::kTreeItem;
  AXTree tree(initial_state);

  AXNode* tree_node = tree.GetFromId(1);
  AXNode* item1 = tree.GetFromId(2);
  AXNode* item2 = tree.GetFromId(3);

  // This should work normally.
  ASSERT_FALSE(tree_node->GetPosInSet());
  EXPECT_OPTIONAL_EQ(2, tree_node->GetSetSize());

  EXPECT_OPTIONAL_EQ(1, item1->GetPosInSet());
  EXPECT_OPTIONAL_EQ(2, item1->GetSetSize());

  EXPECT_OPTIONAL_EQ(2, item2->GetPosInSet());
  EXPECT_OPTIONAL_EQ(2, item2->GetSetSize());

  // Remove item from tree.
  AXTreeUpdate tree_update;
  tree_update.nodes.resize(1);
  tree_update.nodes[0] = initial_state.nodes[1];
  tree_update.nodes[0].AddState(ax::mojom::State::kIgnored);

  ASSERT_TRUE(tree.Unserialize(tree_update));

  ASSERT_FALSE(tree_node->GetPosInSet());
  EXPECT_OPTIONAL_EQ(1, tree_node->GetSetSize());

  // Ignored nodes are not part of ordered sets.
  EXPECT_FALSE(item1->GetPosInSet());
  EXPECT_FALSE(item1->GetSetSize());

  EXPECT_OPTIONAL_EQ(1, item2->GetPosInSet());
  EXPECT_OPTIONAL_EQ(1, item2->GetSetSize());
}

// Tests that kPopUpButtons are assigned the SetSize of the wrapped
// kMenuListPopup, if one is present.
TEST(AXTreeTest, SetSizePosInSetPopUpButton) {
  AXTreeUpdate initial_state;
  initial_state.root_id = 1;
  initial_state.nodes.resize(6);
  initial_state.nodes[0].id = 1;
  initial_state.nodes[0].child_ids = {2, 3};
  initial_state.nodes[1].id = 2;
  initial_state.nodes[1].role = ax::mojom::Role::kPopUpButton;
  initial_state.nodes[2].id = 3;
  initial_state.nodes[2].role = ax::mojom::Role::kPopUpButton;
  initial_state.nodes[2].child_ids = {4};
  initial_state.nodes[3].id = 4;
  initial_state.nodes[3].role = ax::mojom::Role::kMenuListPopup;
  initial_state.nodes[3].child_ids = {5, 6};
  initial_state.nodes[4].id = 5;
  initial_state.nodes[4].role = ax::mojom::Role::kMenuListOption;
  initial_state.nodes[5].id = 6;
  initial_state.nodes[5].role = ax::mojom::Role::kMenuListOption;
  AXTree tree(initial_state);

  // The first popupbutton should have SetSize of 0.
  AXNode* popup_button_1 = tree.GetFromId(2);
  EXPECT_OPTIONAL_EQ(0, popup_button_1->GetSetSize());
  // The second popupbutton should have SetSize of 2, since the menulistpopup
  // that it wraps has a SetSize of 2.
  AXNode* popup_button_2 = tree.GetFromId(3);
  EXPECT_OPTIONAL_EQ(2, popup_button_2->GetSetSize());
}

// Tests that PosInSet and SetSize are still correctly calculated when there
// are nodes with role of kUnknown layered between items and ordered set.
TEST(AXTreeTest, SetSizePosInSetUnkown) {
  AXTreeUpdate initial_state;
  initial_state.root_id = 1;
  initial_state.nodes.resize(5);
  initial_state.nodes[0].id = 1;
  initial_state.nodes[0].child_ids = {2};
  initial_state.nodes[0].role = ax::mojom::Role::kMenu;
  initial_state.nodes[1].id = 2;
  initial_state.nodes[1].role = ax::mojom::Role::kUnknown;
  initial_state.nodes[1].child_ids = {3};
  initial_state.nodes[2].id = 3;
  initial_state.nodes[2].role = ax::mojom::Role::kUnknown;
  initial_state.nodes[2].child_ids = {4, 5};
  initial_state.nodes[3].id = 4;
  initial_state.nodes[3].role = ax::mojom::Role::kMenuItem;
  initial_state.nodes[4].id = 5;
  initial_state.nodes[4].role = ax::mojom::Role::kMenuItem;
  AXTree tree(initial_state);

  AXNode* menu = tree.GetFromId(1);
  EXPECT_OPTIONAL_EQ(2, menu->GetSetSize());
  AXNode* item1 = tree.GetFromId(4);
  EXPECT_OPTIONAL_EQ(1, item1->GetPosInSet());
  EXPECT_OPTIONAL_EQ(2, item1->GetSetSize());
  AXNode* item2 = tree.GetFromId(5);
  EXPECT_OPTIONAL_EQ(2, item2->GetPosInSet());
  EXPECT_OPTIONAL_EQ(2, item2->GetSetSize());
}

TEST(AXTreeTest, SetSizePosInSetMenuItemValidChildOfMenuListPopup) {
  AXTreeUpdate initial_state;
  initial_state.root_id = 1;
  initial_state.nodes.resize(3);
  initial_state.nodes[0].id = 1;
  initial_state.nodes[0].child_ids = {2, 3};
  initial_state.nodes[0].role = ax::mojom::Role::kMenuListPopup;
  initial_state.nodes[1].id = 2;
  initial_state.nodes[1].role = ax::mojom::Role::kMenuItem;
  initial_state.nodes[2].id = 3;
  initial_state.nodes[2].role = ax::mojom::Role::kMenuListOption;
  AXTree tree(initial_state);

  AXNode* menu = tree.GetFromId(1);
  EXPECT_OPTIONAL_EQ(2, menu->GetSetSize());
  AXNode* item1 = tree.GetFromId(2);
  EXPECT_OPTIONAL_EQ(1, item1->GetPosInSet());
  EXPECT_OPTIONAL_EQ(2, item1->GetSetSize());
  AXNode* item2 = tree.GetFromId(3);
  EXPECT_OPTIONAL_EQ(2, item2->GetPosInSet());
  EXPECT_OPTIONAL_EQ(2, item2->GetSetSize());
}

TEST(AXTreeTest, SetSizePostInSetListBoxOptionWithGroup) {
  AXTreeUpdate initial_state;
  initial_state.root_id = 1;
  initial_state.nodes.resize(7);
  initial_state.nodes[0].id = 1;
  initial_state.nodes[0].child_ids = {2, 3};
  initial_state.nodes[0].role = ax::mojom::Role::kListBox;
  initial_state.nodes[1].id = 2;
  initial_state.nodes[1].child_ids = {4, 5};
  initial_state.nodes[1].role = ax::mojom::Role::kGroup;
  initial_state.nodes[2].id = 3;
  initial_state.nodes[2].child_ids = {6, 7};
  initial_state.nodes[2].role = ax::mojom::Role::kGroup;
  initial_state.nodes[3].id = 4;
  initial_state.nodes[3].role = ax::mojom::Role::kListBoxOption;
  initial_state.nodes[4].id = 5;
  initial_state.nodes[4].role = ax::mojom::Role::kListBoxOption;
  initial_state.nodes[5].id = 6;
  initial_state.nodes[5].role = ax::mojom::Role::kListBoxOption;
  initial_state.nodes[6].id = 7;
  initial_state.nodes[6].role = ax::mojom::Role::kListBoxOption;
  AXTree tree(initial_state);

  AXNode* listbox_option1 = tree.GetFromId(4);
  EXPECT_OPTIONAL_EQ(1, listbox_option1->GetPosInSet());
  EXPECT_OPTIONAL_EQ(4, listbox_option1->GetSetSize());
  AXNode* listbox_option2 = tree.GetFromId(5);
  EXPECT_OPTIONAL_EQ(2, listbox_option2->GetPosInSet());
  EXPECT_OPTIONAL_EQ(4, listbox_option2->GetSetSize());
  AXNode* listbox_option3 = tree.GetFromId(6);
  EXPECT_OPTIONAL_EQ(3, listbox_option3->GetPosInSet());
  EXPECT_OPTIONAL_EQ(4, listbox_option3->GetSetSize());
  AXNode* listbox_option4 = tree.GetFromId(7);
  EXPECT_OPTIONAL_EQ(4, listbox_option4->GetPosInSet());
  EXPECT_OPTIONAL_EQ(4, listbox_option4->GetSetSize());
}

TEST(AXTreeTest, SetSizePosInSetGroup) {
  // The behavior of a group changes depending on the context it appears in
  // i.e. if it appears alone vs. if it is contained within another set-like
  // element. The below example shows a group standing alone:
  //
  // <ul role="group"> <!-- SetSize = 3 -->
  //   <li role="menuitemradio" aria-checked="true">Small</li>
  //   <li role="menuitemradio" aria-checked="false">Medium</li>
  //   <li role="menuitemradio" aria-checked="false">Large</li>
  // </ul>
  //
  // However, when it is contained within another set-like element, like a
  // listbox, it should simply act like a generic container:
  //
  // <div role="listbox"> <!-- SetSize = 3 -->
  //   <div role="option">Red</div> <!-- 1 of 3 -->
  //   <div role="option">Yellow</div> <!-- 2 of 3 -->
  //   <div role="group"> <!-- SetSize = 0 -->
  //       <div role="option">Blue</div> <!-- 3 of 3 -->
  //   </div>
  // </div>
  //
  // Please note: the GetPosInSet and GetSetSize functions take slightly
  // different code paths when initially run on items vs. the container.
  // Exercise both code paths in this test.

  AXTreeUpdate tree_update;
  tree_update.root_id = 1;
  tree_update.nodes.resize(6);
  tree_update.nodes[0].id = 1;
  tree_update.nodes[0].role = ax::mojom::Role::kMenu;  // SetSize = 4
  tree_update.nodes[0].child_ids = {2, 6};
  tree_update.nodes[1].id = 2;
  tree_update.nodes[1].role = ax::mojom::Role::kGroup;  // SetSize = 0
  tree_update.nodes[1].child_ids = {3, 4, 5};
  tree_update.nodes[2].id = 3;
  tree_update.nodes[2].role = ax::mojom::Role::kMenuItemRadio;  // 1 of 4
  tree_update.nodes[3].id = 4;
  tree_update.nodes[3].role = ax::mojom::Role::kMenuItemRadio;  // 2 of 4
  tree_update.nodes[4].id = 5;
  tree_update.nodes[4].role = ax::mojom::Role::kMenuItemRadio;  // 3 of 4
  tree_update.nodes[5].id = 6;
  tree_update.nodes[5].role = ax::mojom::Role::kMenuItemRadio;  // 4 of 4
  AXTree tree(tree_update);

  // Get data on kMenu first.
  AXNode* menu = tree.GetFromId(1);
  EXPECT_OPTIONAL_EQ(4, menu->GetSetSize());
  AXNode* group = tree.GetFromId(2);
  EXPECT_FALSE(group->GetSetSize());
  // The below values should have already been computed and cached.
  AXNode* item1 = tree.GetFromId(3);
  EXPECT_OPTIONAL_EQ(1, item1->GetPosInSet());
  EXPECT_OPTIONAL_EQ(4, item1->GetSetSize());
  AXNode* item4 = tree.GetFromId(6);
  EXPECT_OPTIONAL_EQ(4, item4->GetPosInSet());
  EXPECT_OPTIONAL_EQ(4, item4->GetSetSize());

  AXTreeUpdate next_tree_update;
  next_tree_update.root_id = 1;
  next_tree_update.nodes.resize(6);
  next_tree_update.nodes[0].id = 1;
  next_tree_update.nodes[0].role = ax::mojom::Role::kListBox;  // SetSize = 4
  next_tree_update.nodes[0].child_ids = {2, 6};
  next_tree_update.nodes[1].id = 2;
  next_tree_update.nodes[1].role = ax::mojom::Role::kGroup;  // SetSize = 0
  next_tree_update.nodes[1].child_ids = {3, 4, 5};
  next_tree_update.nodes[2].id = 3;
  next_tree_update.nodes[2].role = ax::mojom::Role::kListBoxOption;  // 1 of 4
  next_tree_update.nodes[3].id = 4;
  next_tree_update.nodes[3].role = ax::mojom::Role::kListBoxOption;  // 2 of 4
  next_tree_update.nodes[4].id = 5;
  next_tree_update.nodes[4].role = ax::mojom::Role::kListBoxOption;  // 3 of 4
  next_tree_update.nodes[5].id = 6;
  next_tree_update.nodes[5].role = ax::mojom::Role::kListBoxOption;  // 4 of 4
  AXTree next_tree(next_tree_update);

  // Get data on kListBoxOption first.
  AXNode* option1 = next_tree.GetFromId(3);
  EXPECT_OPTIONAL_EQ(1, option1->GetPosInSet());
  EXPECT_OPTIONAL_EQ(4, option1->GetSetSize());
  AXNode* option2 = next_tree.GetFromId(4);
  EXPECT_OPTIONAL_EQ(2, option2->GetPosInSet());
  EXPECT_OPTIONAL_EQ(4, option2->GetSetSize());
  AXNode* option3 = next_tree.GetFromId(5);
  EXPECT_OPTIONAL_EQ(3, option3->GetPosInSet());
  EXPECT_OPTIONAL_EQ(4, option3->GetSetSize());
  AXNode* option4 = next_tree.GetFromId(6);
  EXPECT_OPTIONAL_EQ(4, option4->GetPosInSet());
  EXPECT_OPTIONAL_EQ(4, option4->GetSetSize());
  AXNode* next_group = next_tree.GetFromId(2);
  EXPECT_FALSE(next_group->GetSetSize());
  // The below value should have already been computed and cached.
  AXNode* listbox = next_tree.GetFromId(1);
  EXPECT_OPTIONAL_EQ(4, listbox->GetSetSize());

  // Standalone groups are allowed.
  AXTreeUpdate third_tree_update;
  third_tree_update.root_id = 1;
  third_tree_update.nodes.resize(3);
  third_tree_update.nodes[0].id = 1;
  third_tree_update.nodes[0].role = ax::mojom::Role::kGroup;
  third_tree_update.nodes[0].child_ids = {2, 3};
  third_tree_update.nodes[1].id = 2;
  third_tree_update.nodes[1].role = ax::mojom::Role::kListItem;
  third_tree_update.nodes[2].id = 3;
  third_tree_update.nodes[2].role = ax::mojom::Role::kListItem;
  AXTree third_tree(third_tree_update);

  // Ensure that groups can't also stand alone.
  AXNode* last_group = third_tree.GetFromId(1);
  EXPECT_OPTIONAL_EQ(2, last_group->GetSetSize());
  AXNode* list_item1 = third_tree.GetFromId(2);
  EXPECT_OPTIONAL_EQ(1, list_item1->GetPosInSet());
  EXPECT_OPTIONAL_EQ(2, list_item1->GetSetSize());
  AXNode* list_item2 = third_tree.GetFromId(3);
  EXPECT_OPTIONAL_EQ(2, list_item2->GetPosInSet());
  EXPECT_OPTIONAL_EQ(2, list_item2->GetSetSize());

  // Test nested groups.
  AXTreeUpdate last_tree_update;
  last_tree_update.root_id = 1;
  last_tree_update.nodes.resize(6);
  last_tree_update.nodes[0].id = 1;
  last_tree_update.nodes[0].role = ax::mojom::Role::kMenuBar;
  last_tree_update.nodes[0].child_ids = {2};
  last_tree_update.nodes[1].id = 2;
  last_tree_update.nodes[1].role = ax::mojom::Role::kGroup;
  last_tree_update.nodes[1].child_ids = {3, 4};
  last_tree_update.nodes[2].id = 3;
  last_tree_update.nodes[2].role = ax::mojom::Role::kMenuItemCheckBox;
  last_tree_update.nodes[3].id = 4;
  last_tree_update.nodes[3].role = ax::mojom::Role::kGroup;
  last_tree_update.nodes[3].child_ids = {5, 6};
  last_tree_update.nodes[4].id = 5;
  last_tree_update.nodes[4].role = ax::mojom::Role::kMenuItemCheckBox;
  last_tree_update.nodes[5].id = 6;
  last_tree_update.nodes[5].role = ax::mojom::Role::kMenuItemCheckBox;
  AXTree last_tree(last_tree_update);

  AXNode* checkbox1 = last_tree.GetFromId(3);
  EXPECT_OPTIONAL_EQ(1, checkbox1->GetPosInSet());
  EXPECT_OPTIONAL_EQ(3, checkbox1->GetSetSize());
  AXNode* checkbox2 = last_tree.GetFromId(5);
  EXPECT_OPTIONAL_EQ(2, checkbox2->GetPosInSet());
  EXPECT_OPTIONAL_EQ(3, checkbox2->GetSetSize());
  AXNode* checkbox3 = last_tree.GetFromId(6);
  EXPECT_OPTIONAL_EQ(3, checkbox3->GetPosInSet());
  EXPECT_OPTIONAL_EQ(3, checkbox3->GetSetSize());
  AXNode* menu_bar = last_tree.GetFromId(1);
  EXPECT_OPTIONAL_EQ(3, menu_bar->GetSetSize());
  AXNode* outer_group = last_tree.GetFromId(2);
  EXPECT_FALSE(outer_group->GetSetSize());
  AXNode* inner_group = last_tree.GetFromId(4);
  EXPECT_FALSE(inner_group->GetSetSize());
}

TEST(AXTreeTest, SetSizePosInSetHidden) {
  AXTreeUpdate tree_update;
  tree_update.root_id = 1;
  tree_update.nodes.resize(6);
  tree_update.nodes[0].id = 1;
  tree_update.nodes[0].role = ax::mojom::Role::kListBox;  // SetSize = 4
  tree_update.nodes[0].child_ids = {2, 3, 4, 5, 6};
  tree_update.nodes[1].id = 2;
  tree_update.nodes[1].role = ax::mojom::Role::kListBoxOption;  // 1 of 4
  tree_update.nodes[2].id = 3;
  tree_update.nodes[2].role = ax::mojom::Role::kListBoxOption;  // 2 of 4
  tree_update.nodes[3].id = 4;
  tree_update.nodes[3].role = ax::mojom::Role::kListBoxOption;  // Hidden
  tree_update.nodes[3].AddState(ax::mojom::State::kInvisible);
  tree_update.nodes[4].id = 5;
  tree_update.nodes[4].role = ax::mojom::Role::kListBoxOption;  // 3 of 4
  tree_update.nodes[5].id = 6;
  tree_update.nodes[5].role = ax::mojom::Role::kListBoxOption;  // 4 of 4
  AXTree tree(tree_update);

  AXNode* list_box = tree.GetFromId(1);
  EXPECT_OPTIONAL_EQ(4, list_box->GetSetSize());
  AXNode* option1 = tree.GetFromId(2);
  EXPECT_OPTIONAL_EQ(1, option1->GetPosInSet());
  EXPECT_OPTIONAL_EQ(4, option1->GetSetSize());
  AXNode* option2 = tree.GetFromId(3);
  EXPECT_OPTIONAL_EQ(2, option2->GetPosInSet());
  EXPECT_OPTIONAL_EQ(4, option2->GetSetSize());
  AXNode* option_hidden = tree.GetFromId(4);
  EXPECT_FALSE(option_hidden->GetPosInSet());
  EXPECT_FALSE(option_hidden->GetSetSize());
  AXNode* option3 = tree.GetFromId(5);
  EXPECT_OPTIONAL_EQ(3, option3->GetPosInSet());
  EXPECT_OPTIONAL_EQ(4, option3->GetSetSize());
  AXNode* option4 = tree.GetFromId(6);
  EXPECT_OPTIONAL_EQ(4, option4->GetPosInSet());
  EXPECT_OPTIONAL_EQ(4, option4->GetSetSize());
}

// Tests that we get the correct PosInSet and SetSize values when using an
// aria-controls relationship.
TEST(AXTreeTest, SetSizePosInSetControls) {
  std::vector<int32_t> three;
  three.push_back(3);
  std::vector<int32_t> hundred;
  hundred.push_back(100);
  std::vector<int32_t> eight;
  eight.push_back(8);
  AXTreeUpdate tree_update;
  tree_update.root_id = 1;
  tree_update.nodes.resize(8);
  tree_update.nodes[0].id = 1;
  tree_update.nodes[0].role = ax::mojom::Role::kGenericContainer;
  tree_update.nodes[0].child_ids = {2, 3, 7, 8};
  tree_update.nodes[1].id = 2;
  tree_update.nodes[1].role = ax::mojom::Role::kPopUpButton;  // SetSize = 3
  tree_update.nodes[1].AddIntListAttribute(
      ax::mojom::IntListAttribute::kControlsIds, three);
  tree_update.nodes[1].SetHasPopup(ax::mojom::HasPopup::kMenu);
  tree_update.nodes[2].id = 3;
  tree_update.nodes[2].role = ax::mojom::Role::kMenu;  // SetSize = 3
  tree_update.nodes[2].child_ids = {4, 5, 6};
  tree_update.nodes[3].id = 4;
  tree_update.nodes[3].role = ax::mojom::Role::kMenuItem;  // 1 of 3
  tree_update.nodes[4].id = 5;
  tree_update.nodes[4].role = ax::mojom::Role::kMenuItem;  // 2 of 3
  tree_update.nodes[5].id = 6;
  tree_update.nodes[5].role = ax::mojom::Role::kMenuItem;  // 3 of 3
  tree_update.nodes[6].id = 7;
  tree_update.nodes[6].role =
      ax::mojom::Role::kPopUpButton;  // Test an invalid controls id.
  tree_update.nodes[6].AddIntListAttribute(
      ax::mojom::IntListAttribute::kControlsIds, hundred);
  // GetSetSize should handle self-references e.g. if a popup button controls
  // itself.
  tree_update.nodes[7].id = 8;
  tree_update.nodes[7].role = ax::mojom::Role::kPopUpButton;
  tree_update.nodes[7].AddIntListAttribute(
      ax::mojom::IntListAttribute::kControlsIds, eight);
  AXTree tree(tree_update);

  AXNode* button = tree.GetFromId(2);
  EXPECT_OPTIONAL_EQ(3, button->GetSetSize());
  EXPECT_FALSE(button->GetPosInSet());
  AXNode* menu = tree.GetFromId(3);
  EXPECT_OPTIONAL_EQ(3, menu->GetSetSize());
  AXNode* item = tree.GetFromId(4);
  EXPECT_OPTIONAL_EQ(1, item->GetPosInSet());
  EXPECT_OPTIONAL_EQ(3, item->GetSetSize());
  item = tree.GetFromId(5);
  EXPECT_OPTIONAL_EQ(2, item->GetPosInSet());
  EXPECT_OPTIONAL_EQ(3, item->GetSetSize());
  item = tree.GetFromId(6);
  EXPECT_OPTIONAL_EQ(3, item->GetPosInSet());
  EXPECT_OPTIONAL_EQ(3, item->GetSetSize());
  button = tree.GetFromId(7);
  EXPECT_OPTIONAL_EQ(0, button->GetSetSize());
  button = tree.GetFromId(8);
  EXPECT_OPTIONAL_EQ(0, button->GetSetSize());
}

// Tests GetPosInSet and GetSetSize return the assigned int attribute values
// when a pop-up button is a leaf node.
TEST(AXTreeTest, SetSizePosInSetLeafPopUpButton) {
  AXTreeUpdate tree_update;
  tree_update.root_id = 1;
  tree_update.nodes.resize(2);
  tree_update.nodes[0].id = 1;
  tree_update.nodes[0].role = ax::mojom::Role::kGenericContainer;
  tree_update.nodes[0].child_ids = {2};
  tree_update.nodes[1].id = 2;
  tree_update.nodes[1].role = ax::mojom::Role::kPopUpButton;
  tree_update.nodes[1].AddIntAttribute(ax::mojom::IntAttribute::kPosInSet, 3);
  tree_update.nodes[1].AddIntAttribute(ax::mojom::IntAttribute::kSetSize, 77);
  AXTree tree(tree_update);

  AXNode* pop_up_button = tree.GetFromId(2);
  EXPECT_OPTIONAL_EQ(3, pop_up_button->GetPosInSet());
  EXPECT_OPTIONAL_EQ(77, pop_up_button->GetSetSize());
}

TEST(AXTreeTest, OnNodeWillBeDeletedHasValidUnignoredParent) {
  AXTreeUpdate initial_state;
  initial_state.root_id = 1;
  initial_state.nodes.resize(3);
  initial_state.nodes[0].id = 1;
  initial_state.nodes[0].role = ax::mojom::Role::kRootWebArea;
  initial_state.nodes[0].child_ids = {2};
  initial_state.nodes[1].id = 2;
  initial_state.nodes[1].role = ax::mojom::Role::kGenericContainer;
  initial_state.nodes[1].child_ids = {3};
  initial_state.nodes[2].id = 3;
  initial_state.nodes[2].role = ax::mojom::Role::kGenericContainer;
  AXTree tree(initial_state);

  AXTreeUpdate tree_update;
  tree_update.nodes.resize(1);
  // Remove child from node:2, and add State::kIgnored
  tree_update.nodes[0] = initial_state.nodes[1];
  tree_update.nodes[0].AddState(ax::mojom::State::kIgnored);
  tree_update.nodes[0].child_ids.clear();

  // Before node:3 is deleted, the unignored parent is node:2.
  // Assert that this is the case in |OnNodeWillBeDeleted|.
  TestAXTreeObserver test_observer(&tree);
  test_observer.unignored_parent_id_before_node_deleted = 2;
  ASSERT_TRUE(tree.Unserialize(tree_update));
}

TEST(AXTreeTest, OnNodeHasBeenDeleted) {
  AXTreeUpdate initial_state;

  initial_state.root_id = 1;
  initial_state.nodes.resize(6);
  initial_state.nodes[0].id = 1;
  initial_state.nodes[0].role = ax::mojom::Role::kRootWebArea;
  initial_state.nodes[0].child_ids = {2};
  initial_state.nodes[1].id = 2;
  initial_state.nodes[1].role = ax::mojom::Role::kButton;
  initial_state.nodes[1].child_ids = {3, 4};
  initial_state.nodes[2].id = 3;
  initial_state.nodes[2].role = ax::mojom::Role::kCheckBox;
  initial_state.nodes[3].id = 4;
  initial_state.nodes[3].role = ax::mojom::Role::kStaticText;
  initial_state.nodes[3].child_ids = {5, 6};
  initial_state.nodes[4].id = 5;
  initial_state.nodes[4].role = ax::mojom::Role::kInlineTextBox;
  initial_state.nodes[5].id = 6;
  initial_state.nodes[5].role = ax::mojom::Role::kInlineTextBox;

  AXTree tree(initial_state);

  AXTreeUpdate update;
  update.nodes.resize(2);
  update.nodes[0] = initial_state.nodes[1];
  update.nodes[0].child_ids = {4};
  update.nodes[1] = initial_state.nodes[3];
  update.nodes[1].child_ids = {};

  TestAXTreeObserver test_observer(&tree);
  ASSERT_TRUE(tree.Unserialize(update));

  EXPECT_EQ(3U, test_observer.deleted_ids().size());
  EXPECT_EQ(3, test_observer.deleted_ids()[0]);
  EXPECT_EQ(5, test_observer.deleted_ids()[1]);
  EXPECT_EQ(6, test_observer.deleted_ids()[2]);

  // Verify that the nodes we intend to delete in the update are actually
  // absent from the tree.
  for (auto id : test_observer.deleted_ids()) {
    SCOPED_TRACE(testing::Message()
                 << "Node with id=" << id << ", should not exist in the tree");
    EXPECT_EQ(nullptr, tree.GetFromId(id));
  }
}

// Tests a fringe scenario that may happen if multiple AXTreeUpdates are merged.
// Make sure that we correctly Unserialize if a newly created node is deleted,
// and possibly recreated later.
TEST(AXTreeTest, SingleUpdateDeletesNewlyCreatedChildNode) {
  AXTreeUpdate initial_state;
  initial_state.root_id = 1;
  initial_state.nodes.resize(1);
  initial_state.nodes[0].id = 1;
  initial_state.nodes[0].role = ax::mojom::Role::kRootWebArea;
  AXTree tree(initial_state);

  AXTreeUpdate tree_update;
  tree_update.nodes.resize(6);
  // Add child node:2
  tree_update.nodes[0] = initial_state.nodes[0];
  tree_update.nodes[0].child_ids = {2};
  tree_update.nodes[1].id = 2;
  tree_update.nodes[1].role = ax::mojom::Role::kGenericContainer;
  // Remove child node:2
  tree_update.nodes[2] = initial_state.nodes[0];
  // Add child node:2
  tree_update.nodes[3] = initial_state.nodes[0];
  tree_update.nodes[3].child_ids = {2};
  tree_update.nodes[4].id = 2;
  tree_update.nodes[4].role = ax::mojom::Role::kGenericContainer;
  // Remove child node:2
  tree_update.nodes[5] = initial_state.nodes[0];

  ASSERT_TRUE(tree.Unserialize(tree_update)) << tree.error();

  ASSERT_EQ(
      "AXTree\n"
      "id=1 rootWebArea (0, 0)-(0, 0)\n",
      tree.ToString());

  // Unserialize again, but with another add child.
  tree_update.nodes.resize(8);
  tree_update.nodes[6] = initial_state.nodes[0];
  tree_update.nodes[6].child_ids = {2};
  tree_update.nodes[7].id = 2;
  tree_update.nodes[7].role = ax::mojom::Role::kGenericContainer;
  ASSERT_TRUE(tree.Unserialize(tree_update)) << tree.error();

  ASSERT_EQ(
      "AXTree\n"
      "id=1 rootWebArea (0, 0)-(0, 0) child_ids=2\n"
      "  id=2 genericContainer (0, 0)-(0, 0)\n",
      tree.ToString());
}

// Tests a fringe scenario that may happen if multiple AXTreeUpdates are merged.
// Make sure that we correctly Unserialize if a node is reparented multiple
// times.
TEST(AXTreeTest, SingleUpdateReparentsNodeMultipleTimes) {
  // ++{kRootWebArea, 1}
  // ++++{kList, 2}
  // ++++++{kListItem, 4}
  // ++++{kList, 3}
  AXTreeUpdate initial_state;
  initial_state.root_id = 1;
  initial_state.nodes.resize(4);
  initial_state.nodes[0].id = 1;
  initial_state.nodes[0].role = ax::mojom::Role::kRootWebArea;
  initial_state.nodes[0].child_ids = {2, 3};
  initial_state.nodes[1].id = 2;
  initial_state.nodes[1].role = ax::mojom::Role::kList;
  initial_state.nodes[1].child_ids = {4};
  initial_state.nodes[2].id = 3;
  initial_state.nodes[2].role = ax::mojom::Role::kList;
  initial_state.nodes[3].id = 4;
  initial_state.nodes[3].role = ax::mojom::Role::kListItem;
  AXTree tree(initial_state);

  AXTreeUpdate tree_update;
  tree_update.nodes.resize(6);
  // Remove child node:4
  tree_update.nodes[0].id = 2;
  tree_update.nodes[0].role = ax::mojom::Role::kList;
  // Reparent child node:4 onto node:3
  tree_update.nodes[1].id = 3;
  tree_update.nodes[1].role = ax::mojom::Role::kList;
  tree_update.nodes[1].child_ids = {4};
  tree_update.nodes[2].id = 4;
  tree_update.nodes[2].role = ax::mojom::Role::kListItem;
  // Remove child ndoe:4
  tree_update.nodes[3].id = 3;
  tree_update.nodes[3].role = ax::mojom::Role::kList;
  // Reparent child node:4 onto node:2
  tree_update.nodes[4].id = 2;
  tree_update.nodes[4].role = ax::mojom::Role::kList;
  tree_update.nodes[4].child_ids = {4};
  tree_update.nodes[5].id = 4;
  tree_update.nodes[5].role = ax::mojom::Role::kListItem;

  ASSERT_TRUE(tree.Unserialize(tree_update)) << tree.error();
  EXPECT_EQ(
      "AXTree\nid=1 rootWebArea (0, 0)-(0, 0) child_ids=2,3\n"
      "  id=2 list (0, 0)-(0, 0) child_ids=4\n"
      "    id=4 listItem (0, 0)-(0, 0)\n"
      "  id=3 list (0, 0)-(0, 0)\n",
      tree.ToString());

  // Unserialize again, but with another reparent.
  tree_update.nodes.resize(9);
  tree_update.nodes[6] = tree_update.nodes[0];
  tree_update.nodes[7] = tree_update.nodes[1];
  tree_update.nodes[8] = tree_update.nodes[2];

  ASSERT_TRUE(tree.Unserialize(tree_update)) << tree.error();
  EXPECT_EQ(
      "AXTree\nid=1 rootWebArea (0, 0)-(0, 0) child_ids=2,3\n"
      "  id=2 list (0, 0)-(0, 0)\n"
      "  id=3 list (0, 0)-(0, 0) child_ids=4\n"
      "    id=4 listItem (0, 0)-(0, 0)\n",
      tree.ToString());
}

// Tests a fringe scenario that may happen if multiple AXTreeUpdates are merged.
// Make sure that we correctly Unserialize if a newly created node toggles its
// ignored state.
TEST(AXTreeTest, SingleUpdateIgnoresNewlyCreatedUnignoredChildNode) {
  AXTreeUpdate initial_state;
  initial_state.root_id = 1;
  initial_state.nodes.resize(1);
  initial_state.nodes[0].id = 1;
  initial_state.nodes[0].role = ax::mojom::Role::kRootWebArea;
  AXTree tree(initial_state);

  AXTreeUpdate tree_update;
  tree_update.nodes.resize(3);
  // Add child node:2
  tree_update.nodes[0] = initial_state.nodes[0];
  tree_update.nodes[0].child_ids = {2};
  tree_update.nodes[1].id = 2;
  tree_update.nodes[1].role = ax::mojom::Role::kGenericContainer;
  // Add State::kIgnored to node:2
  tree_update.nodes[2] = tree_update.nodes[1];
  tree_update.nodes[2].AddState(ax::mojom::State::kIgnored);

  ASSERT_TRUE(tree.Unserialize(tree_update)) << tree.error();

  ASSERT_EQ(
      "AXTree\n"
      "id=1 rootWebArea (0, 0)-(0, 0) child_ids=2\n"
      "  id=2 genericContainer IGNORED (0, 0)-(0, 0)\n",
      tree.ToString());
}

// Tests a fringe scenario that may happen if multiple AXTreeUpdates are merged.
// Make sure that we correctly Unserialize if a newly created node toggles its
// ignored state.
TEST(AXTreeTest, SingleUpdateTogglesIgnoredStateAfterCreatingNode) {
  AXTreeUpdate initial_state;
  initial_state.root_id = 1;
  initial_state.nodes.resize(1);
  initial_state.nodes[0].id = 1;
  initial_state.nodes[0].role = ax::mojom::Role::kRootWebArea;
  AXTree tree(initial_state);

  ASSERT_EQ(
      "AXTree\n"
      "id=1 rootWebArea (0, 0)-(0, 0)\n",
      tree.ToString());

  AXTreeUpdate tree_update;
  tree_update.nodes.resize(5);
  // Add child node:2, node:3
  tree_update.nodes[0] = initial_state.nodes[0];
  tree_update.nodes[0].child_ids = {2, 3};
  tree_update.nodes[1].id = 2;
  tree_update.nodes[1].role = ax::mojom::Role::kGenericContainer;
  tree_update.nodes[2].id = 3;
  tree_update.nodes[2].role = ax::mojom::Role::kGenericContainer;
  tree_update.nodes[2].AddState(ax::mojom::State::kIgnored);
  // Add State::kIgnored to node:2
  tree_update.nodes[3] = tree_update.nodes[1];
  tree_update.nodes[3].AddState(ax::mojom::State::kIgnored);
  // Remove State::kIgnored from node:3
  tree_update.nodes[4] = tree_update.nodes[2];
  tree_update.nodes[4].RemoveState(ax::mojom::State::kIgnored);

  ASSERT_TRUE(tree.Unserialize(tree_update)) << tree.error();

  ASSERT_EQ(
      "AXTree\n"
      "id=1 rootWebArea (0, 0)-(0, 0) child_ids=2,3\n"
      "  id=2 genericContainer IGNORED (0, 0)-(0, 0)\n"
      "  id=3 genericContainer (0, 0)-(0, 0)\n",
      tree.ToString());
}

// Tests a fringe scenario that may happen if multiple AXTreeUpdates are merged.
// Make sure that we correctly Unserialize if a node toggles its ignored state
// and is then removed from the tree.
TEST(AXTreeTest, SingleUpdateTogglesIgnoredStateBeforeDestroyingNode) {
  AXTreeUpdate initial_state;
  initial_state.root_id = 1;
  initial_state.nodes.resize(3);
  initial_state.nodes[0].id = 1;
  initial_state.nodes[0].role = ax::mojom::Role::kRootWebArea;
  initial_state.nodes[0].child_ids = {2, 3};
  initial_state.nodes[1].id = 2;
  initial_state.nodes[1].role = ax::mojom::Role::kGenericContainer;
  initial_state.nodes[2].id = 3;
  initial_state.nodes[2].role = ax::mojom::Role::kGenericContainer;
  initial_state.nodes[2].AddState(ax::mojom::State::kIgnored);
  AXTree tree(initial_state);

  ASSERT_EQ(
      "AXTree\n"
      "id=1 rootWebArea (0, 0)-(0, 0) child_ids=2,3\n"
      "  id=2 genericContainer (0, 0)-(0, 0)\n"
      "  id=3 genericContainer IGNORED (0, 0)-(0, 0)\n",
      tree.ToString());

  AXTreeUpdate tree_update;
  tree_update.nodes.resize(3);
  // Add State::kIgnored to node:2
  tree_update.nodes[0] = initial_state.nodes[1];
  tree_update.nodes[0].AddState(ax::mojom::State::kIgnored);
  // Remove State::kIgnored from node:3
  tree_update.nodes[1] = initial_state.nodes[2];
  tree_update.nodes[1].RemoveState(ax::mojom::State::kIgnored);
  // Remove child node:2, node:3
  tree_update.nodes[2] = initial_state.nodes[0];
  tree_update.nodes[2].child_ids.clear();

  ASSERT_TRUE(tree.Unserialize(tree_update)) << tree.error();

  ASSERT_EQ(
      "AXTree\n"
      "id=1 rootWebArea (0, 0)-(0, 0)\n",
      tree.ToString());
}

// Tests that the IsInListMarker() method returns true if the current node is a
// list marker or if it's a descendant node of a list marker.
TEST(AXTreeTest, TestIsInListMarker) {
  // This test uses the template of a list of one element: "1. List item"
  AXTreeUpdate tree_update;
  tree_update.root_id = 1;
  tree_update.nodes.resize(8);
  tree_update.nodes[0].id = 1;
  tree_update.nodes[0].role = ax::mojom::Role::kList;
  tree_update.nodes[0].child_ids = {2, 3};
  tree_update.nodes[1].id = 2;
  tree_update.nodes[1].role = ax::mojom::Role::kListItem;
  tree_update.nodes[2].id = 3;
  tree_update.nodes[2].child_ids = {4, 7};
  tree_update.nodes[3].id = 4;
  tree_update.nodes[3].role = ax::mojom::Role::kListMarker;
  tree_update.nodes[3].child_ids = {5};
  tree_update.nodes[4].id = 5;
  tree_update.nodes[4].role = ax::mojom::Role::kStaticText;  // "1. "
  tree_update.nodes[4].child_ids = {6};
  tree_update.nodes[5].id = 6;
  tree_update.nodes[5].role = ax::mojom::Role::kInlineTextBox;  // "1. "
  tree_update.nodes[6].id = 7;
  tree_update.nodes[6].role = ax::mojom::Role::kStaticText;  // "List item"
  tree_update.nodes[6].child_ids = {8};
  tree_update.nodes[7].id = 8;
  tree_update.nodes[7].role = ax::mojom::Role::kInlineTextBox;  // "List item"
  AXTree tree(tree_update);

  AXNode* list_node = tree.GetFromId(1);
  ASSERT_EQ(false, list_node->IsInListMarker());

  AXNode* list_item_node = tree.GetFromId(2);
  ASSERT_EQ(false, list_item_node->IsInListMarker());

  AXNode* list_marker1 = tree.GetFromId(4);
  ASSERT_EQ(true, list_marker1->IsInListMarker());

  AXNode* static_node1 = tree.GetFromId(5);
  ASSERT_EQ(true, static_node1->IsInListMarker());

  AXNode* inline_node1 = tree.GetFromId(6);
  ASSERT_EQ(true, inline_node1->IsInListMarker());

  AXNode* static_node2 = tree.GetFromId(7);
  ASSERT_EQ(false, static_node2->IsInListMarker());

  AXNode* inline_node2 = tree.GetFromId(8);
  ASSERT_EQ(false, inline_node2->IsInListMarker());
}

}  // namespace ui
