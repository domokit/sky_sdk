// Copyright 2016 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef UI_ACCESSIBILITY_AX_NODE_POSITION_H_
#define UI_ACCESSIBILITY_AX_NODE_POSITION_H_

#include <cstdint>
#include <vector>

#include "ax_enums.h"
#include "ax_export.h"
#include "ax_node.h"
#include "ax_position.h"
#include "ax_tree_id.h"

namespace ui {

// AXNodePosition includes implementations of AXPosition methods which require
// knowledge of the AXPosition AXNodeType (which is unknown by AXPosition).
class AX_EXPORT AXNodePosition : public AXPosition<AXNodePosition, AXNode> {
 public:
  // Creates either a text or a tree position, depending on the type of the node
  // provided.
  static AXPositionInstance CreatePosition(
      const AXNode& node,
      int child_index_or_text_offset,
      ax::mojom::TextAffinity affinity = ax::mojom::TextAffinity::kDownstream);

  AXNodePosition();
  ~AXNodePosition() override;
  AXNodePosition(const AXNodePosition& other);

  AXPositionInstance Clone() const override;

  std::u16string GetText() const override;
  bool IsInLineBreak() const override;
  bool IsInTextObject() const override;
  bool IsInWhiteSpace() const override;
  int MaxTextOffset() const override;

 protected:
  void AnchorChild(int child_index,
                   AXTreeID* tree_id,
                   AXNode::AXID* child_id) const override;
  int AnchorChildCount() const override;
  int AnchorUnignoredChildCount() const override;
  int AnchorIndexInParent() const override;
  int AnchorSiblingCount() const override;
  std::stack<AXNode*> GetAncestorAnchors() const override;
  AXNode* GetLowestUnignoredAncestor() const override;
  void AnchorParent(AXTreeID* tree_id, AXNode::AXID* parent_id) const override;
  AXNode* GetNodeInTree(AXTreeID tree_id, AXNode::AXID node_id) const override;
  AXNode::AXID GetAnchorID(AXNode* node) const override;
  AXTreeID GetTreeID(AXNode* node) const override;

  bool IsEmbeddedObjectInParent() const override;
  bool IsInLineBreakingObject() const override;
  ax::mojom::Role GetAnchorRole() const override;
  ax::mojom::Role GetRole(AXNode* node) const override;
  AXNodeTextStyles GetTextStyles() const override;
  std::vector<int32_t> GetWordStartOffsets() const override;
  std::vector<int32_t> GetWordEndOffsets() const override;
  AXNode::AXID GetNextOnLineID(AXNode::AXID node_id) const override;
  AXNode::AXID GetPreviousOnLineID(AXNode::AXID node_id) const override;

 private:
  // Returns the parent node of the provided child. Returns the parent
  // node's tree id and node id through the provided output parameters,
  // |parent_tree_id| and |parent_id|.
  static AXNode* GetParent(AXNode* child,
                           AXTreeID child_tree_id,
                           AXTreeID* parent_tree_id,
                           AXNode::AXID* parent_id);
};

}  // namespace ui

#endif  // UI_ACCESSIBILITY_AX_NODE_POSITION_H_
