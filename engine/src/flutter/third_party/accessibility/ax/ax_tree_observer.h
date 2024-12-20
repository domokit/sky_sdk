// Copyright 2018 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef UI_ACCESSIBILITY_AX_TREE_OBSERVER_H_
#define UI_ACCESSIBILITY_AX_TREE_OBSERVER_H_

#include <cstdint>
#include <string>
#include <vector>

#include "ax_enums.h"
#include "ax_export.h"

namespace ui {

class AXNode;
struct AXNodeData;
class AXTree;
struct AXTreeData;

// Used when you want to be notified when changes happen to an AXTree.
//
// |OnAtomicUpdateFinished| is notified at the end of an atomic update.
// It provides a vector of nodes that were added or changed, for final
// postprocessing.
class AX_EXPORT AXTreeObserver {
 public:
  AXTreeObserver();
  virtual ~AXTreeObserver();

  // Called before any tree modifications have occurred, notifying that a single
  // node will change its data. Its id and data will be valid, but its links to
  // parents and children are only valid within this callstack. Do not hold a
  // reference to the node or any relative nodes such as ancestors or
  // descendants described by the node data outside of this event.
  virtual void OnNodeDataWillChange(AXTree* tree,
                                    const AXNodeData& old_node_data,
                                    const AXNodeData& new_node_data) {}

  // Called after all tree modifications have occurred, notifying that a single
  // node has changed its data. Its id, data, and links to parent and children
  // will all be valid, since the tree is in a stable state after updating.
  virtual void OnNodeDataChanged(AXTree* tree,
                                 const AXNodeData& old_node_data,
                                 const AXNodeData& new_node_data) {}

  // Individual callbacks for every attribute of AXNodeData that can change.
  // Called after all tree mutations have occurred, notifying that a single node
  // changed its data. Its id, data, and links to parent and children will all
  // be valid, since the tree is in a stable state after updating.
  virtual void OnRoleChanged(AXTree* tree,
                             AXNode* node,
                             ax::mojom::Role old_role,
                             ax::mojom::Role new_role) {}
  virtual void OnStateChanged(AXTree* tree,
                              AXNode* node,
                              ax::mojom::State state,
                              bool new_value) {}
  virtual void OnStringAttributeChanged(AXTree* tree,
                                        AXNode* node,
                                        ax::mojom::StringAttribute attr,
                                        const std::string& old_value,
                                        const std::string& new_value) {}
  virtual void OnIntAttributeChanged(AXTree* tree,
                                     AXNode* node,
                                     ax::mojom::IntAttribute attr,
                                     int32_t old_value,
                                     int32_t new_value) {}
  virtual void OnFloatAttributeChanged(AXTree* tree,
                                       AXNode* node,
                                       ax::mojom::FloatAttribute attr,
                                       float old_value,
                                       float new_value) {}
  virtual void OnBoolAttributeChanged(AXTree* tree,
                                      AXNode* node,
                                      ax::mojom::BoolAttribute attr,
                                      bool new_value) {}
  virtual void OnIntListAttributeChanged(
      AXTree* tree,
      AXNode* node,
      ax::mojom::IntListAttribute attr,
      const std::vector<int32_t>& old_value,
      const std::vector<int32_t>& new_value) {}
  virtual void OnStringListAttributeChanged(
      AXTree* tree,
      AXNode* node,
      ax::mojom::StringListAttribute attr,
      const std::vector<std::string>& old_value,
      const std::vector<std::string>& new_value) {}

  // Called when tree data changes, after all nodes have been updated.
  virtual void OnTreeDataChanged(AXTree* tree,
                                 const AXTreeData& old_data,
                                 const AXTreeData& new_data) {}

  // Called before any tree modifications have occurred, notifying that a single
  // node will be deleted. Its id and data will be valid, but its links to
  // parents and children are only valid within this callstack. Do not hold
  // a reference to node outside of the event.
  virtual void OnNodeWillBeDeleted(AXTree* tree, AXNode* node) {}

  // Same as OnNodeWillBeDeleted, but only called once for an entire subtree.
  virtual void OnSubtreeWillBeDeleted(AXTree* tree, AXNode* node) {}

  // Called just before a node is deleted for reparenting. See
  // |OnNodeWillBeDeleted| for additional information.
  virtual void OnNodeWillBeReparented(AXTree* tree, AXNode* node) {}

  // Called just before a subtree is deleted for reparenting. See
  // |OnSubtreeWillBeDeleted| for additional information.
  virtual void OnSubtreeWillBeReparented(AXTree* tree, AXNode* node) {}

  // Called after all tree mutations have occurred, notifying that a single node
  // has been created. Its id, data, and links to parent and children will all
  // be valid, since the tree is in a stable state after updating.
  virtual void OnNodeCreated(AXTree* tree, AXNode* node) {}

  // Called after all tree mutations have occurred or during tree teardown,
  // notifying that a single node has been deleted from the tree.
  virtual void OnNodeDeleted(AXTree* tree, int32_t node_id) {}

  // Same as |OnNodeCreated|, but called for nodes that have been reparented.
  virtual void OnNodeReparented(AXTree* tree, AXNode* node) {}

  // Called after all tree mutations have occurred, notifying that a single node
  // has updated its data or children. Its id, data, and links to parent and
  // children will all be valid, since the tree is in a stable state after
  // updating.
  virtual void OnNodeChanged(AXTree* tree, AXNode* node) {}

  enum ChangeType {
    NODE_CREATED,
    SUBTREE_CREATED,
    NODE_CHANGED,
    NODE_REPARENTED,
    SUBTREE_REPARENTED
  };

  struct Change {
    Change(AXNode* node, ChangeType type) {
      this->node = node;
      this->type = type;
    }
    AXNode* node;
    ChangeType type;
  };

  // Called at the end of the update operation. Every node that was added
  // or changed will be included in |changes|, along with an enum indicating
  // the type of change - either (1) a node was created, (2) a node was created
  // and it's the root of a new subtree, or (3) a node was changed. Finally,
  // a bool indicates if the root of the tree was changed or not.
  virtual void OnAtomicUpdateFinished(AXTree* tree,
                                      bool root_changed,
                                      const std::vector<Change>& changes) {}
};

}  // namespace ui

#endif  // UI_ACCESSIBILITY_AX_TREE_OBSERVER_H_
