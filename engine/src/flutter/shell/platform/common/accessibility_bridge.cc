// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "accessibility_bridge.h"

#include <functional>
#include <utility>

#include "flutter/third_party/accessibility/ax/ax_tree_manager_map.h"
#include "flutter/third_party/accessibility/ax/ax_tree_update.h"
#include "flutter/third_party/accessibility/base/logging.h"

namespace flutter {  // namespace

constexpr int kHasScrollingAction =
    FlutterSemanticsAction::kFlutterSemanticsActionScrollLeft |
    FlutterSemanticsAction::kFlutterSemanticsActionScrollRight |
    FlutterSemanticsAction::kFlutterSemanticsActionScrollUp |
    FlutterSemanticsAction::kFlutterSemanticsActionScrollDown;

// AccessibilityBridge
AccessibilityBridge::AccessibilityBridge()
    : tree_(std::make_unique<ui::AXTree>()) {
  event_generator_.SetTree(tree_.get());
  tree_->AddObserver(static_cast<ui::AXTreeObserver*>(this));
  ui::AXTreeData data = tree_->data();
  data.tree_id = ui::AXTreeID::CreateNewAXTreeID();
  tree_->UpdateData(data);
  ui::AXTreeManagerMap::GetInstance().AddTreeManager(tree_->GetAXTreeID(),
                                                     this);
}

AccessibilityBridge::~AccessibilityBridge() {
  event_generator_.ReleaseTree();
  tree_->RemoveObserver(static_cast<ui::AXTreeObserver*>(this));
}

void AccessibilityBridge::AddFlutterSemanticsNodeUpdate(
    const FlutterSemanticsNode2& node) {
  pending_semantics_node_updates_[node.id] = FromFlutterSemanticsNode(node);
}

void AccessibilityBridge::AddFlutterSemanticsCustomActionUpdate(
    const FlutterSemanticsCustomAction2& action) {
  pending_semantics_custom_action_updates_[action.id] =
      FromFlutterSemanticsCustomAction(action);
}

void AccessibilityBridge::CommitUpdates() {
  // AXTree cannot move a node in a single update.
  // This must be split across two updates:
  //
  // * Update 1: remove nodes from their old parents.
  // * Update 2: re-add nodes (including their children) to their new parents.
  //
  // First, start by removing nodes if necessary.
  std::optional<ui::AXTreeUpdate> remove_reparented =
      CreateRemoveReparentedNodesUpdate();
  if (remove_reparented.has_value()) {
    tree_->Unserialize(remove_reparented.value());

    std::string error = tree_->error();
    if (!error.empty()) {
      FML_LOG(ERROR) << "Failed to update ui::AXTree, error: " << error;
      assert(false);
      return;
    }
  }

  // Second, apply the pending node updates. This also moves reparented nodes to
  // their new parents if needed.
  ui::AXTreeUpdate update{.tree_data = tree_->data()};

  // Figure out update order, ui::AXTree only accepts update in tree order,
  // where parent node must come before the child node in
  // ui::AXTreeUpdate.nodes. We start with picking a random node and turn the
  // entire subtree into a list. We pick another node from the remaining update,
  // and keep doing so until the update map is empty. We then concatenate the
  // lists in the reversed order, this guarantees parent updates always come
  // before child updates. If the root is in the update, it is guaranteed to
  // be the first node of the last list.
  std::vector<std::vector<SemanticsNode>> results;
  while (!pending_semantics_node_updates_.empty()) {
    auto begin = pending_semantics_node_updates_.begin();
    SemanticsNode target = begin->second;
    std::vector<SemanticsNode> sub_tree_list;
    GetSubTreeList(target, sub_tree_list);
    results.push_back(sub_tree_list);
    pending_semantics_node_updates_.erase(begin);
  }

  for (size_t i = results.size(); i > 0; i--) {
    for (const SemanticsNode& node : results[i - 1]) {
      ConvertFlutterUpdate(node, update);
    }
  }

  // The first update must set the tree's root, which is guaranteed to be the
  // last list's first node. A tree's root node never changes, though it can be
  // modified.
  if (!results.empty() && GetRootAsAXNode()->id() == ui::AXNode::kInvalidAXID) {
    FML_DCHECK(!results.back().empty());

    update.root_id = results.back().front().id;
  }

  tree_->Unserialize(update);
  pending_semantics_node_updates_.clear();
  pending_semantics_custom_action_updates_.clear();

  std::string error = tree_->error();
  if (!error.empty()) {
    FML_LOG(ERROR) << "Failed to update ui::AXTree, error: " << error;
    return;
  }
  // Handles accessibility events as the result of the semantics update.
  for (const auto& targeted_event : event_generator_) {
    auto event_target =
        GetFlutterPlatformNodeDelegateFromID(targeted_event.node->id());
    if (event_target.expired()) {
      continue;
    }

    OnAccessibilityEvent(targeted_event);
  }
  event_generator_.ClearEvents();
}

std::weak_ptr<FlutterPlatformNodeDelegate>
AccessibilityBridge::GetFlutterPlatformNodeDelegateFromID(
    AccessibilityNodeId id) const {
  const auto iter = id_wrapper_map_.find(id);
  if (iter != id_wrapper_map_.end()) {
    return iter->second;
  }

  return std::weak_ptr<FlutterPlatformNodeDelegate>();
}

const ui::AXTreeData& AccessibilityBridge::GetAXTreeData() const {
  return tree_->data();
}

const std::vector<ui::AXEventGenerator::TargetedEvent>
AccessibilityBridge::GetPendingEvents() const {
  std::vector<ui::AXEventGenerator::TargetedEvent> result(
      event_generator_.begin(), event_generator_.end());
  return result;
}

void AccessibilityBridge::OnNodeWillBeDeleted(ui::AXTree* tree,
                                              ui::AXNode* node) {}

void AccessibilityBridge::OnSubtreeWillBeDeleted(ui::AXTree* tree,
                                                 ui::AXNode* node) {}

void AccessibilityBridge::OnNodeReparented(ui::AXTree* tree, ui::AXNode* node) {
}

void AccessibilityBridge::OnRoleChanged(ui::AXTree* tree,
                                        ui::AXNode* node,
                                        ax::mojom::Role old_role,
                                        ax::mojom::Role new_role) {}

void AccessibilityBridge::OnNodeDataChanged(
    ui::AXTree* tree,
    const ui::AXNodeData& old_node_data,
    const ui::AXNodeData& new_node_data) {
  auto platform_view =
      GetFlutterPlatformNodeDelegateFromID(new_node_data.id).lock();
  if (platform_view) {
    platform_view->NodeDataChanged(old_node_data, new_node_data);
  }
}

void AccessibilityBridge::OnNodeCreated(ui::AXTree* tree, ui::AXNode* node) {
  BASE_DCHECK(node);
  id_wrapper_map_[node->id()] = CreateFlutterPlatformNodeDelegate();
  id_wrapper_map_[node->id()]->Init(
      std::static_pointer_cast<FlutterPlatformNodeDelegate::OwnerBridge>(
          shared_from_this()),
      node);
}

void AccessibilityBridge::OnNodeDeleted(ui::AXTree* tree,
                                        AccessibilityNodeId node_id) {
  BASE_DCHECK(node_id != ui::AXNode::kInvalidAXID);
  if (id_wrapper_map_.find(node_id) != id_wrapper_map_.end()) {
    id_wrapper_map_.erase(node_id);
  }
}

void AccessibilityBridge::OnAtomicUpdateFinished(
    ui::AXTree* tree,
    bool root_changed,
    const std::vector<ui::AXTreeObserver::Change>& changes) {
  // The Flutter semantics update does not include child->parent relationship
  // We have to update the relative bound offset container id here in order
  // to calculate the screen bound correctly.
  for (const auto& change : changes) {
    ui::AXNode* node = change.node;
    const ui::AXNodeData& data = node->data();
    AccessibilityNodeId offset_container_id = -1;
    if (node->parent()) {
      offset_container_id = node->parent()->id();
    }
    node->SetLocation(offset_container_id, data.relative_bounds.bounds,
                      data.relative_bounds.transform.get());
  }
}

std::optional<ui::AXTreeUpdate>
AccessibilityBridge::CreateRemoveReparentedNodesUpdate() {
  std::unordered_map<int32_t, ui::AXNodeData> updates;

  for (const auto& node_update : pending_semantics_node_updates_) {
    for (int32_t child_id : node_update.second.children_in_traversal_order) {
      // Skip nodes that don't exist or have a parent in the current tree.
      ui::AXNode* child = tree_->GetFromId(child_id);
      if (!child) {
        continue;
      }

      // Flutter's root node should never be reparented.
      assert(child->parent());

      // Skip nodes whose parents are unchanged.
      if (child->parent()->id() == node_update.second.id) {
        continue;
      }

      // This pending update moves the current child node.
      // That new child must have a corresponding pending update.
      assert(pending_semantics_node_updates_.find(child_id) !=
             pending_semantics_node_updates_.end());

      // Create an update to remove the child from its previous parent.
      int32_t parent_id = child->parent()->id();
      if (updates.find(parent_id) == updates.end()) {
        updates[parent_id] = tree_->GetFromId(parent_id)->data();
      }

      ui::AXNodeData* parent = &updates[parent_id];
      auto iter = std::find(parent->child_ids.begin(), parent->child_ids.end(),
                            child_id);

      assert(iter != parent->child_ids.end());
      parent->child_ids.erase(iter);
    }
  }

  if (updates.empty()) {
    return std::nullopt;
  }

  ui::AXTreeUpdate update{
      .tree_data = tree_->data(),
      .nodes = std::vector<ui::AXNodeData>(),
  };

  for (std::pair<int32_t, ui::AXNodeData> data : updates) {
    update.nodes.push_back(std::move(data.second));
  }

  return update;
}

// Private method.
void AccessibilityBridge::GetSubTreeList(const SemanticsNode& target,
                                         std::vector<SemanticsNode>& result) {
  result.push_back(target);
  for (int32_t child : target.children_in_traversal_order) {
    auto iter = pending_semantics_node_updates_.find(child);
    if (iter != pending_semantics_node_updates_.end()) {
      SemanticsNode node = iter->second;
      GetSubTreeList(node, result);
      pending_semantics_node_updates_.erase(iter);
    }
  }
}

void AccessibilityBridge::ConvertFlutterUpdate(const SemanticsNode& node,
                                               ui::AXTreeUpdate& tree_update) {
  ui::AXNodeData node_data;
  node_data.id = node.id;
  SetRoleFromFlutterUpdate(node_data, node);
  SetStateFromFlutterUpdate(node_data, node);
  SetActionsFromFlutterUpdate(node_data, node);
  SetBooleanAttributesFromFlutterUpdate(node_data, node);
  SetIntAttributesFromFlutterUpdate(node_data, node);
  SetIntListAttributesFromFlutterUpdate(node_data, node);
  SetStringListAttributesFromFlutterUpdate(node_data, node);
  SetNameFromFlutterUpdate(node_data, node);
  SetValueFromFlutterUpdate(node_data, node);
  SetTooltipFromFlutterUpdate(node_data, node);
  node_data.relative_bounds.bounds.SetRect(node.rect.left, node.rect.top,
                                           node.rect.right - node.rect.left,
                                           node.rect.bottom - node.rect.top);
  node_data.relative_bounds.transform = std::make_unique<gfx::Transform>(
      node.transform.scaleX, node.transform.skewX, node.transform.transX, 0,
      node.transform.skewY, node.transform.scaleY, node.transform.transY, 0,
      node.transform.pers0, node.transform.pers1, node.transform.pers2, 0, 0, 0,
      0, 0);
  for (auto child : node.children_in_traversal_order) {
    node_data.child_ids.push_back(child);
  }
  SetTreeData(node, tree_update);
  tree_update.nodes.push_back(node_data);
}

void AccessibilityBridge::SetRoleFromFlutterUpdate(ui::AXNodeData& node_data,
                                                   const SemanticsNode& node) {
  FlutterSemanticsFlag flags = node.flags;
  if (flags & FlutterSemanticsFlag::kFlutterSemanticsFlagIsButton) {
    node_data.role = ax::mojom::Role::kButton;
    return;
  }
  if (flags & FlutterSemanticsFlag::kFlutterSemanticsFlagIsTextField &&
      !(flags & FlutterSemanticsFlag::kFlutterSemanticsFlagIsReadOnly)) {
    node_data.role = ax::mojom::Role::kTextField;
    return;
  }
  if (flags & FlutterSemanticsFlag::kFlutterSemanticsFlagIsHeader) {
    node_data.role = ax::mojom::Role::kHeader;
    return;
  }
  if (flags & FlutterSemanticsFlag::kFlutterSemanticsFlagIsImage) {
    node_data.role = ax::mojom::Role::kImage;
    return;
  }
  if (flags & FlutterSemanticsFlag::kFlutterSemanticsFlagIsLink) {
    node_data.role = ax::mojom::Role::kLink;
    return;
  }

  if (flags & kFlutterSemanticsFlagIsInMutuallyExclusiveGroup &&
      flags & kFlutterSemanticsFlagHasCheckedState) {
    node_data.role = ax::mojom::Role::kRadioButton;
    return;
  }
  if (flags & kFlutterSemanticsFlagHasCheckedState) {
    node_data.role = ax::mojom::Role::kCheckBox;
    return;
  }
  if (flags & kFlutterSemanticsFlagHasToggledState) {
    node_data.role = ax::mojom::Role::kSwitch;
    return;
  }
  if (flags & kFlutterSemanticsFlagIsSlider) {
    node_data.role = ax::mojom::Role::kSlider;
    return;
  }
  // If the state cannot be derived from the flutter flags, we fallback to group
  // or static text.
  if (node.children_in_traversal_order.empty()) {
    node_data.role = ax::mojom::Role::kStaticText;
  } else {
    node_data.role = ax::mojom::Role::kGroup;
  }
}

void AccessibilityBridge::SetStateFromFlutterUpdate(ui::AXNodeData& node_data,
                                                    const SemanticsNode& node) {
  FlutterSemanticsFlag flags = node.flags;
  FlutterSemanticsAction actions = node.actions;
  if (flags & FlutterSemanticsFlag::kFlutterSemanticsFlagHasExpandedState &&
      flags & FlutterSemanticsFlag::kFlutterSemanticsFlagIsExpanded) {
    node_data.AddState(ax::mojom::State::kExpanded);
  } else if (flags &
             FlutterSemanticsFlag::kFlutterSemanticsFlagHasExpandedState) {
    node_data.AddState(ax::mojom::State::kCollapsed);
  }
  if (flags & FlutterSemanticsFlag::kFlutterSemanticsFlagIsTextField &&
      (flags & FlutterSemanticsFlag::kFlutterSemanticsFlagIsReadOnly) == 0) {
    node_data.AddState(ax::mojom::State::kEditable);
  }
  if (node_data.role == ax::mojom::Role::kStaticText &&
      (actions & kHasScrollingAction) == 0 && node.value.empty() &&
      node.label.empty() && node.hint.empty()) {
    node_data.AddState(ax::mojom::State::kIgnored);
  } else {
    // kFlutterSemanticsFlagIsFocusable means a keyboard focusable, it is
    // different from semantics focusable.
    // TODO(chunhtai): figure out whether something is not semantics focusable.
    node_data.AddState(ax::mojom::State::kFocusable);
  }
}

void AccessibilityBridge::SetActionsFromFlutterUpdate(
    ui::AXNodeData& node_data,
    const SemanticsNode& node) {
  FlutterSemanticsAction actions = node.actions;
  if (actions & FlutterSemanticsAction::kFlutterSemanticsActionTap) {
    node_data.AddAction(ax::mojom::Action::kDoDefault);
  }
  if (actions & FlutterSemanticsAction::kFlutterSemanticsActionScrollLeft) {
    node_data.AddAction(ax::mojom::Action::kScrollLeft);
  }
  if (actions & FlutterSemanticsAction::kFlutterSemanticsActionScrollRight) {
    node_data.AddAction(ax::mojom::Action::kScrollRight);
  }
  if (actions & FlutterSemanticsAction::kFlutterSemanticsActionScrollUp) {
    node_data.AddAction(ax::mojom::Action::kScrollUp);
  }
  if (actions & FlutterSemanticsAction::kFlutterSemanticsActionScrollDown) {
    node_data.AddAction(ax::mojom::Action::kScrollDown);
  }
  if (actions & FlutterSemanticsAction::kFlutterSemanticsActionIncrease) {
    node_data.AddAction(ax::mojom::Action::kIncrement);
  }
  if (actions & FlutterSemanticsAction::kFlutterSemanticsActionDecrease) {
    node_data.AddAction(ax::mojom::Action::kDecrement);
  }
  // Every node has show on screen action.
  node_data.AddAction(ax::mojom::Action::kScrollToMakeVisible);

  if (actions & FlutterSemanticsAction::kFlutterSemanticsActionSetSelection) {
    node_data.AddAction(ax::mojom::Action::kSetSelection);
  }
  if (actions & FlutterSemanticsAction::
                    kFlutterSemanticsActionDidGainAccessibilityFocus) {
    node_data.AddAction(ax::mojom::Action::kSetAccessibilityFocus);
  }
  if (actions & FlutterSemanticsAction::
                    kFlutterSemanticsActionDidLoseAccessibilityFocus) {
    node_data.AddAction(ax::mojom::Action::kClearAccessibilityFocus);
  }
  if (actions & FlutterSemanticsAction::kFlutterSemanticsActionCustomAction) {
    node_data.AddAction(ax::mojom::Action::kCustomAction);
  }
}

void AccessibilityBridge::SetBooleanAttributesFromFlutterUpdate(
    ui::AXNodeData& node_data,
    const SemanticsNode& node) {
  FlutterSemanticsAction actions = node.actions;
  FlutterSemanticsFlag flags = node.flags;
  node_data.AddBoolAttribute(ax::mojom::BoolAttribute::kScrollable,
                             actions & kHasScrollingAction);
  node_data.AddBoolAttribute(
      ax::mojom::BoolAttribute::kClickable,
      actions & FlutterSemanticsAction::kFlutterSemanticsActionTap);
  // TODO(chunhtai): figure out if there is a node that does not clip overflow.
  node_data.AddBoolAttribute(ax::mojom::BoolAttribute::kClipsChildren,
                             !node.children_in_traversal_order.empty());
  node_data.AddBoolAttribute(
      ax::mojom::BoolAttribute::kSelected,
      flags & FlutterSemanticsFlag::kFlutterSemanticsFlagIsSelected);
  node_data.AddBoolAttribute(
      ax::mojom::BoolAttribute::kEditableRoot,
      flags & FlutterSemanticsFlag::kFlutterSemanticsFlagIsTextField &&
          (flags & FlutterSemanticsFlag::kFlutterSemanticsFlagIsReadOnly) == 0);
  // Mark nodes as line breaking so that screen readers don't
  // merge all consecutive objects into one.
  // TODO(schectman): When should a node have this attribute set?
  // https://github.com/flutter/flutter/issues/118184
  node_data.AddBoolAttribute(ax::mojom::BoolAttribute::kIsLineBreakingObject,
                             true);
}

void AccessibilityBridge::SetIntAttributesFromFlutterUpdate(
    ui::AXNodeData& node_data,
    const SemanticsNode& node) {
  FlutterSemanticsFlag flags = node.flags;
  node_data.AddIntAttribute(ax::mojom::IntAttribute::kTextDirection,
                            node.text_direction);

  int sel_start = node.text_selection_base;
  int sel_end = node.text_selection_extent;
  if (flags & FlutterSemanticsFlag::kFlutterSemanticsFlagIsTextField &&
      (flags & FlutterSemanticsFlag::kFlutterSemanticsFlagIsReadOnly) == 0 &&
      !node.value.empty()) {
    // By default the text field selection should be at the end.
    sel_start = sel_start == -1 ? node.value.length() : sel_start;
    sel_end = sel_end == -1 ? node.value.length() : sel_end;
  }
  node_data.AddIntAttribute(ax::mojom::IntAttribute::kTextSelStart, sel_start);
  node_data.AddIntAttribute(ax::mojom::IntAttribute::kTextSelEnd, sel_end);

  if (node_data.role == ax::mojom::Role::kRadioButton ||
      node_data.role == ax::mojom::Role::kCheckBox) {
    node_data.AddIntAttribute(
        ax::mojom::IntAttribute::kCheckedState,
        static_cast<int32_t>(
            flags & FlutterSemanticsFlag::kFlutterSemanticsFlagIsCheckStateMixed
                ? ax::mojom::CheckedState::kMixed
            : flags & FlutterSemanticsFlag::kFlutterSemanticsFlagIsChecked
                ? ax::mojom::CheckedState::kTrue
                : ax::mojom::CheckedState::kFalse));
  } else if (node_data.role == ax::mojom::Role::kSwitch) {
    node_data.AddIntAttribute(
        ax::mojom::IntAttribute::kCheckedState,
        static_cast<int32_t>(
            flags & FlutterSemanticsFlag::kFlutterSemanticsFlagIsToggled
                ? ax::mojom::CheckedState::kTrue
                : ax::mojom::CheckedState::kFalse));
  }
}

void AccessibilityBridge::SetIntListAttributesFromFlutterUpdate(
    ui::AXNodeData& node_data,
    const SemanticsNode& node) {
  FlutterSemanticsAction actions = node.actions;
  if (actions & FlutterSemanticsAction::kFlutterSemanticsActionCustomAction) {
    std::vector<int32_t> custom_action_ids;
    for (size_t i = 0; i < node.custom_accessibility_actions.size(); i++) {
      custom_action_ids.push_back(node.custom_accessibility_actions[i]);
    }
    node_data.AddIntListAttribute(ax::mojom::IntListAttribute::kCustomActionIds,
                                  custom_action_ids);
  }
}

void AccessibilityBridge::SetStringListAttributesFromFlutterUpdate(
    ui::AXNodeData& node_data,
    const SemanticsNode& node) {
  FlutterSemanticsAction actions = node.actions;
  if (actions & FlutterSemanticsAction::kFlutterSemanticsActionCustomAction) {
    std::vector<std::string> custom_action_description;
    for (size_t i = 0; i < node.custom_accessibility_actions.size(); i++) {
      auto iter = pending_semantics_custom_action_updates_.find(
          node.custom_accessibility_actions[i]);
      BASE_DCHECK(iter != pending_semantics_custom_action_updates_.end());
      custom_action_description.push_back(iter->second.label);
    }
    node_data.AddStringListAttribute(
        ax::mojom::StringListAttribute::kCustomActionDescriptions,
        custom_action_description);
  }
}

void AccessibilityBridge::SetNameFromFlutterUpdate(ui::AXNodeData& node_data,
                                                   const SemanticsNode& node) {
  node_data.SetName(node.label);
}

void AccessibilityBridge::SetValueFromFlutterUpdate(ui::AXNodeData& node_data,
                                                    const SemanticsNode& node) {
  node_data.SetValue(node.value);
}

void AccessibilityBridge::SetTooltipFromFlutterUpdate(
    ui::AXNodeData& node_data,
    const SemanticsNode& node) {
  node_data.SetTooltip(node.tooltip);
}

void AccessibilityBridge::SetTreeData(const SemanticsNode& node,
                                      ui::AXTreeUpdate& tree_update) {
  FlutterSemanticsFlag flags = node.flags;
  // Set selection of the focused node if:
  // 1. this text field has a valid selection
  // 2. this text field doesn't have a valid selection but had selection stored
  //    in the tree.
  if (flags & FlutterSemanticsFlag::kFlutterSemanticsFlagIsTextField &&
      flags & FlutterSemanticsFlag::kFlutterSemanticsFlagIsFocused) {
    if (node.text_selection_base != -1) {
      tree_update.tree_data.sel_anchor_object_id = node.id;
      tree_update.tree_data.sel_anchor_offset = node.text_selection_base;
      tree_update.tree_data.sel_focus_object_id = node.id;
      tree_update.tree_data.sel_focus_offset = node.text_selection_extent;
      tree_update.has_tree_data = true;
    } else if (tree_update.tree_data.sel_anchor_object_id == node.id) {
      tree_update.tree_data.sel_anchor_object_id = ui::AXNode::kInvalidAXID;
      tree_update.tree_data.sel_anchor_offset = -1;
      tree_update.tree_data.sel_focus_object_id = ui::AXNode::kInvalidAXID;
      tree_update.tree_data.sel_focus_offset = -1;
      tree_update.has_tree_data = true;
    }
  }

  if (flags & FlutterSemanticsFlag::kFlutterSemanticsFlagIsFocused &&
      tree_update.tree_data.focus_id != node.id) {
    tree_update.tree_data.focus_id = node.id;
    tree_update.has_tree_data = true;
  } else if ((flags & FlutterSemanticsFlag::kFlutterSemanticsFlagIsFocused) ==
                 0 &&
             tree_update.tree_data.focus_id == node.id) {
    tree_update.tree_data.focus_id = ui::AXNode::kInvalidAXID;
    tree_update.has_tree_data = true;
  }
}

AccessibilityBridge::SemanticsNode
AccessibilityBridge::FromFlutterSemanticsNode(
    const FlutterSemanticsNode2& flutter_node) {
  SemanticsNode result;
  result.id = flutter_node.id;
  result.flags = flutter_node.flags;
  result.actions = flutter_node.actions;
  result.text_selection_base = flutter_node.text_selection_base;
  result.text_selection_extent = flutter_node.text_selection_extent;
  result.scroll_child_count = flutter_node.scroll_child_count;
  result.scroll_index = flutter_node.scroll_index;
  result.scroll_position = flutter_node.scroll_position;
  result.scroll_extent_max = flutter_node.scroll_extent_max;
  result.scroll_extent_min = flutter_node.scroll_extent_min;
  result.elevation = flutter_node.elevation;
  result.thickness = flutter_node.thickness;
  if (flutter_node.label) {
    result.label = std::string(flutter_node.label);
  }
  if (flutter_node.hint) {
    result.hint = std::string(flutter_node.hint);
  }
  if (flutter_node.value) {
    result.value = std::string(flutter_node.value);
  }
  if (flutter_node.increased_value) {
    result.increased_value = std::string(flutter_node.increased_value);
  }
  if (flutter_node.decreased_value) {
    result.decreased_value = std::string(flutter_node.decreased_value);
  }
  if (flutter_node.tooltip) {
    result.tooltip = std::string(flutter_node.tooltip);
  }
  result.text_direction = flutter_node.text_direction;
  result.rect = flutter_node.rect;
  result.transform = flutter_node.transform;
  if (flutter_node.child_count > 0) {
    result.children_in_traversal_order = std::vector<int32_t>(
        flutter_node.children_in_traversal_order,
        flutter_node.children_in_traversal_order + flutter_node.child_count);
  }
  if (flutter_node.custom_accessibility_actions_count > 0) {
    result.custom_accessibility_actions = std::vector<int32_t>(
        flutter_node.custom_accessibility_actions,
        flutter_node.custom_accessibility_actions +
            flutter_node.custom_accessibility_actions_count);
  }
  return result;
}

AccessibilityBridge::SemanticsCustomAction
AccessibilityBridge::FromFlutterSemanticsCustomAction(
    const FlutterSemanticsCustomAction2& flutter_custom_action) {
  SemanticsCustomAction result;
  result.id = flutter_custom_action.id;
  result.override_action = flutter_custom_action.override_action;
  if (flutter_custom_action.label) {
    result.label = std::string(flutter_custom_action.label);
  }
  if (flutter_custom_action.hint) {
    result.hint = std::string(flutter_custom_action.hint);
  }
  return result;
}

void AccessibilityBridge::SetLastFocusedId(AccessibilityNodeId node_id) {
  if (last_focused_id_ != node_id) {
    auto last_focused_child =
        GetFlutterPlatformNodeDelegateFromID(last_focused_id_);
    if (!last_focused_child.expired()) {
      DispatchAccessibilityAction(
          last_focused_id_,
          FlutterSemanticsAction::
              kFlutterSemanticsActionDidLoseAccessibilityFocus,
          {});
    }
    last_focused_id_ = node_id;
  }
}

AccessibilityNodeId AccessibilityBridge::GetLastFocusedId() {
  return last_focused_id_;
}

gfx::NativeViewAccessible AccessibilityBridge::GetNativeAccessibleFromId(
    AccessibilityNodeId id) {
  auto platform_node_delegate = GetFlutterPlatformNodeDelegateFromID(id).lock();
  if (!platform_node_delegate) {
    return nullptr;
  }
  return platform_node_delegate->GetNativeViewAccessible();
}

gfx::RectF AccessibilityBridge::RelativeToGlobalBounds(const ui::AXNode* node,
                                                       bool& offscreen,
                                                       bool clip_bounds) {
  return tree_->RelativeToTreeBounds(node, gfx::RectF(), &offscreen,
                                     clip_bounds);
}

ui::AXNode* AccessibilityBridge::GetNodeFromTree(
    ui::AXTreeID tree_id,
    ui::AXNode::AXID node_id) const {
  return GetNodeFromTree(node_id);
}

ui::AXNode* AccessibilityBridge::GetNodeFromTree(
    ui::AXNode::AXID node_id) const {
  return tree_->GetFromId(node_id);
}

ui::AXTreeID AccessibilityBridge::GetTreeID() const {
  return tree_->GetAXTreeID();
}

ui::AXTreeID AccessibilityBridge::GetParentTreeID() const {
  return ui::AXTreeIDUnknown();
}

ui::AXNode* AccessibilityBridge::GetRootAsAXNode() const {
  return tree_->root();
}

ui::AXNode* AccessibilityBridge::GetParentNodeFromParentTreeAsAXNode() const {
  return nullptr;
}

ui::AXTree* AccessibilityBridge::GetTree() const {
  return tree_.get();
}

ui::AXPlatformNode* AccessibilityBridge::GetPlatformNodeFromTree(
    const ui::AXNode::AXID node_id) const {
  auto platform_delegate_weak = GetFlutterPlatformNodeDelegateFromID(node_id);
  auto platform_delegate = platform_delegate_weak.lock();
  if (!platform_delegate) {
    return nullptr;
  }
  return platform_delegate->GetPlatformNode();
}

ui::AXPlatformNode* AccessibilityBridge::GetPlatformNodeFromTree(
    const ui::AXNode& node) const {
  return GetPlatformNodeFromTree(node.id());
}

ui::AXPlatformNodeDelegate* AccessibilityBridge::RootDelegate() const {
  return GetFlutterPlatformNodeDelegateFromID(GetRootAsAXNode()->id())
      .lock()
      .get();
}

}  // namespace flutter
