// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/fuchsia/flutter/accessibility_bridge.h"

#include <lib/inspect/cpp/inspector.h>
#include <lib/zx/process.h>
#include <zircon/status.h>
#include <zircon/types.h>

#include <algorithm>
#include <deque>

#include "flutter/fml/logging.h"
#include "flutter/lib/ui/semantics/semantics_node.h"

#include "../runtime/dart/utils/root_inspect_node.h"

namespace flutter_runner {
namespace {

#if !FLUTTER_RELEASE
static constexpr char kTreeDumpInspectRootName[] = "semantic_tree_root";

// Converts flutter semantic node flags to a string representation.
std::string NodeFlagsToString(const flutter::SemanticsNode& node) {
  std::string output;
  if (node.HasFlag(flutter::SemanticsFlags::kHasCheckedState)) {
    output += "kHasCheckedState|";
  }
  if (node.HasFlag(flutter::SemanticsFlags::kHasEnabledState)) {
    output += "kHasEnabledState|";
  }
  if (node.HasFlag(flutter::SemanticsFlags::kHasImplicitScrolling)) {
    output += "kHasImplicitScrolling|";
  }
  if (node.HasFlag(flutter::SemanticsFlags::kHasToggledState)) {
    output += "kHasToggledState|";
  }
  if (node.HasFlag(flutter::SemanticsFlags::kIsButton)) {
    output += "kIsButton|";
  }
  if (node.HasFlag(flutter::SemanticsFlags::kIsChecked)) {
    output += "kIsChecked|";
  }
  if (node.HasFlag(flutter::SemanticsFlags::kIsEnabled)) {
    output += "kIsEnabled|";
  }
  if (node.HasFlag(flutter::SemanticsFlags::kIsFocusable)) {
    output += "kIsFocusable|";
  }
  if (node.HasFlag(flutter::SemanticsFlags::kIsFocused)) {
    output += "kIsFocused|";
  }
  if (node.HasFlag(flutter::SemanticsFlags::kIsHeader)) {
    output += "kIsHeader|";
  }
  if (node.HasFlag(flutter::SemanticsFlags::kIsHidden)) {
    output += "kIsHidden|";
  }
  if (node.HasFlag(flutter::SemanticsFlags::kIsImage)) {
    output += "kIsImage|";
  }
  if (node.HasFlag(flutter::SemanticsFlags::kIsInMutuallyExclusiveGroup)) {
    output += "kIsInMutuallyExclusiveGroup|";
  }
  if (node.HasFlag(flutter::SemanticsFlags::kIsKeyboardKey)) {
    output += "kIsKeyboardKey|";
  }
  if (node.HasFlag(flutter::SemanticsFlags::kIsLink)) {
    output += "kIsLink|";
  }
  if (node.HasFlag(flutter::SemanticsFlags::kIsLiveRegion)) {
    output += "kIsLiveRegion|";
  }
  if (node.HasFlag(flutter::SemanticsFlags::kIsObscured)) {
    output += "kIsObscured|";
  }
  if (node.HasFlag(flutter::SemanticsFlags::kIsReadOnly)) {
    output += "kIsReadOnly|";
  }
  if (node.HasFlag(flutter::SemanticsFlags::kIsSelected)) {
    output += "kIsSelected|";
  }
  if (node.HasFlag(flutter::SemanticsFlags::kIsSlider)) {
    output += "kIsSlider|";
  }
  if (node.HasFlag(flutter::SemanticsFlags::kIsTextField)) {
    output += "kIsTextField|";
  }
  if (node.HasFlag(flutter::SemanticsFlags::kIsToggled)) {
    output += "kIsToggled|";
  }
  if (node.HasFlag(flutter::SemanticsFlags::kNamesRoute)) {
    output += "kNamesRoute|";
  }
  if (node.HasFlag(flutter::SemanticsFlags::kScopesRoute)) {
    output += "kScopesRoute|";
  }

  return output;
}

// Converts flutter semantic node actions to a string representation.
std::string NodeActionsToString(const flutter::SemanticsNode& node) {
  std::string output;

  if (node.HasAction(flutter::SemanticsAction::kCopy)) {
    output += "kCopy|";
  }
  if (node.HasAction(flutter::SemanticsAction::kCustomAction)) {
    output += "kCustomAction|";
  }
  if (node.HasAction(flutter::SemanticsAction::kCut)) {
    output += "kCut|";
  }
  if (node.HasAction(flutter::SemanticsAction::kDecrease)) {
    output += "kDecrease|";
  }
  if (node.HasAction(flutter::SemanticsAction::kDidGainAccessibilityFocus)) {
    output += "kDidGainAccessibilityFocus|";
  }
  if (node.HasAction(flutter::SemanticsAction::kDidLoseAccessibilityFocus)) {
    output += "kDidLoseAccessibilityFocus|";
  }
  if (node.HasAction(flutter::SemanticsAction::kDismiss)) {
    output += "kDismiss|";
  }
  if (node.HasAction(flutter::SemanticsAction::kIncrease)) {
    output += "kIncrease|";
  }
  if (node.HasAction(flutter::SemanticsAction::kLongPress)) {
    output += "kLongPress|";
  }
  if (node.HasAction(
          flutter::SemanticsAction::kMoveCursorBackwardByCharacter)) {
    output += "kMoveCursorBackwardByCharacter|";
  }
  if (node.HasAction(flutter::SemanticsAction::kMoveCursorBackwardByWord)) {
    output += "kMoveCursorBackwardByWord|";
  }
  if (node.HasAction(flutter::SemanticsAction::kMoveCursorForwardByCharacter)) {
    output += "kMoveCursorForwardByCharacter|";
  }
  if (node.HasAction(flutter::SemanticsAction::kMoveCursorForwardByWord)) {
    output += "kMoveCursorForwardByWord|";
  }
  if (node.HasAction(flutter::SemanticsAction::kPaste)) {
    output += "kPaste|";
  }
  if (node.HasAction(flutter::SemanticsAction::kScrollDown)) {
    output += "kScrollDown|";
  }
  if (node.HasAction(flutter::SemanticsAction::kScrollLeft)) {
    output += "kScrollLeft|";
  }
  if (node.HasAction(flutter::SemanticsAction::kScrollRight)) {
    output += "kScrollRight|";
  }
  if (node.HasAction(flutter::SemanticsAction::kScrollUp)) {
    output += "kScrollUp|";
  }
  if (node.HasAction(flutter::SemanticsAction::kScrollToOffset)) {
    output += "kScrollToOffset|";
  }
  if (node.HasAction(flutter::SemanticsAction::kSetSelection)) {
    output += "kSetSelection|";
  }
  if (node.HasAction(flutter::SemanticsAction::kSetText)) {
    output += "kSetText|";
  }
  if (node.HasAction(flutter::SemanticsAction::kShowOnScreen)) {
    output += "kShowOnScreen|";
  }
  if (node.HasAction(flutter::SemanticsAction::kTap)) {
    output += "kTap|";
  }
  if (node.HasAction(flutter::SemanticsAction::kFocus)) {
    output += "kFocus|";
  }

  return output;
}

// Returns a string representation of the flutter semantic node absolut
// location.
std::string NodeLocationToString(const SkRect& rect) {
  auto min_x = rect.fLeft;
  auto min_y = rect.fTop;
  auto max_x = rect.fRight;
  auto max_y = rect.fBottom;
  std::string location =
      "min(" + std::to_string(min_x) + ", " + std::to_string(min_y) + ") max(" +
      std::to_string(max_x) + ", " + std::to_string(max_y) + ")";

  return location;
}

// Returns a string representation of the node's different types of children.
std::string NodeChildrenToString(const flutter::SemanticsNode& node) {
  std::stringstream output;
  if (!node.childrenInTraversalOrder.empty()) {
    output << "children in traversal order:[";
    for (const auto child_id : node.childrenInTraversalOrder) {
      output << child_id << ", ";
    }
    output << "]\n";
  }
  if (!node.childrenInHitTestOrder.empty()) {
    output << "children in hit test order:[";
    for (const auto child_id : node.childrenInHitTestOrder) {
      output << child_id << ", ";
    }
    output << ']';
  }

  return output.str();
}
#endif  // !FLUTTER_RELEASE

}  // namespace

AccessibilityBridge::AccessibilityBridge(
    SetSemanticsEnabledCallback set_semantics_enabled_callback,
    DispatchSemanticsActionCallback dispatch_semantics_action_callback,
    fuchsia::accessibility::semantics::SemanticsManagerHandle semantics_manager,
    fuchsia::ui::views::ViewRef view_ref,
    inspect::Node inspect_node)
    : set_semantics_enabled_callback_(
          std::move(set_semantics_enabled_callback)),
      dispatch_semantics_action_callback_(
          std::move(dispatch_semantics_action_callback)),
      binding_(this),
      fuchsia_semantics_manager_(semantics_manager.Bind()),
      atomic_updates_(std::make_shared<std::queue<FuchsiaAtomicUpdate>>()),
      inspect_node_(std::move(inspect_node)) {
  fuchsia_semantics_manager_.set_error_handler([](zx_status_t status) {
    FML_LOG(ERROR) << "Flutter cannot connect to SemanticsManager with status: "
                   << zx_status_get_string(status) << ".";
  });
  fuchsia_semantics_manager_->RegisterViewForSemantics(
      std::move(view_ref), binding_.NewBinding(), tree_ptr_.NewRequest());

#if !FLUTTER_RELEASE
  // The first argument to |CreateLazyValues| is the name of the lazy node, and
  // will only be displayed if the callback used to generate the node's content
  // fails. Therefore, we use an error message for this node name.
  inspect_node_tree_dump_ =
      inspect_node_.CreateLazyValues("dump_fail", [this]() {
        inspect::Inspector inspector;
        if (auto it = nodes_.find(kRootNodeId); it == nodes_.end()) {
          inspector.GetRoot().CreateString(
              "empty_tree", "this semantic tree is empty", &inspector);
        } else {
          FillInspectTree(
              kRootNodeId, /*current_level=*/1,
              inspector.GetRoot().CreateChild(kTreeDumpInspectRootName),
              &inspector);
        }
        return fpromise::make_ok_promise(std::move(inspector));
      });
#endif  // !FLUTTER_RELEASE
}

bool AccessibilityBridge::GetSemanticsEnabled() const {
  return semantics_enabled_;
}

void AccessibilityBridge::SetSemanticsEnabled(bool enabled) {
  semantics_enabled_ = enabled;
  if (!enabled) {
    nodes_.clear();
  }
}

fuchsia::ui::gfx::BoundingBox AccessibilityBridge::GetNodeLocation(
    const flutter::SemanticsNode& node) const {
  fuchsia::ui::gfx::BoundingBox box;
  box.min.x = node.rect.fLeft;
  box.min.y = node.rect.fTop;
  box.min.z = static_cast<float>(node.elevation);
  box.max.x = node.rect.fRight;
  box.max.y = node.rect.fBottom;
  box.max.z = static_cast<float>(node.thickness);
  return box;
}

fuchsia::ui::gfx::mat4 AccessibilityBridge::GetNodeTransform(
    const flutter::SemanticsNode& node) const {
  return ConvertSkiaTransformToMat4(node.transform);
}

fuchsia::ui::gfx::mat4 AccessibilityBridge::ConvertSkiaTransformToMat4(
    const SkM44 transform) const {
  fuchsia::ui::gfx::mat4 value;
  float* m = value.matrix.data();
  transform.getColMajor(m);
  return value;
}

fuchsia::accessibility::semantics::Attributes
AccessibilityBridge::GetNodeAttributes(const flutter::SemanticsNode& node,
                                       size_t* added_size) const {
  fuchsia::accessibility::semantics::Attributes attributes;
  // TODO(MI4-2531): Don't truncate.
  if (node.label.size() > fuchsia::accessibility::semantics::MAX_LABEL_SIZE) {
    attributes.set_label(node.label.substr(
        0, fuchsia::accessibility::semantics::MAX_LABEL_SIZE));
    *added_size += fuchsia::accessibility::semantics::MAX_LABEL_SIZE;
  } else {
    attributes.set_label(node.label);
    *added_size += node.label.size();
  }

  if (node.tooltip.size() > fuchsia::accessibility::semantics::MAX_LABEL_SIZE) {
    attributes.set_secondary_label(node.tooltip.substr(
        0, fuchsia::accessibility::semantics::MAX_LABEL_SIZE));
    *added_size += fuchsia::accessibility::semantics::MAX_LABEL_SIZE;
  } else {
    attributes.set_secondary_label(node.tooltip);
    *added_size += node.tooltip.size();
  }

  if (node.HasFlag(flutter::SemanticsFlags::kIsKeyboardKey)) {
    attributes.set_is_keyboard_key(true);
  }

  return attributes;
}

fuchsia::accessibility::semantics::States AccessibilityBridge::GetNodeStates(
    const flutter::SemanticsNode& node,
    size_t* additional_size) const {
  fuchsia::accessibility::semantics::States states;
  (*additional_size) += sizeof(fuchsia::accessibility::semantics::States);

  // Set checked state.
  if (!node.HasFlag(flutter::SemanticsFlags::kHasCheckedState)) {
    states.set_checked_state(
        fuchsia::accessibility::semantics::CheckedState::NONE);
  } else {
    states.set_checked_state(
        node.HasFlag(flutter::SemanticsFlags::kIsChecked)
            ? fuchsia::accessibility::semantics::CheckedState::CHECKED
            : fuchsia::accessibility::semantics::CheckedState::UNCHECKED);
  }

  // Set enabled state.
  if (node.HasFlag(flutter::SemanticsFlags::kHasEnabledState)) {
    states.set_enabled_state(
        node.HasFlag(flutter::SemanticsFlags::kIsEnabled)
            ? fuchsia::accessibility::semantics::EnabledState::ENABLED
            : fuchsia::accessibility::semantics::EnabledState::DISABLED);
  }

  // Set selected state.
  states.set_selected(node.HasFlag(flutter::SemanticsFlags::kIsSelected));

  // Flutter's definition of a hidden node is different from Fuchsia, so it must
  // not be set here.

  // Set value.
  if (node.value.size() > fuchsia::accessibility::semantics::MAX_VALUE_SIZE) {
    states.set_value(node.value.substr(
        0, fuchsia::accessibility::semantics::MAX_VALUE_SIZE));
    (*additional_size) += fuchsia::accessibility::semantics::MAX_VALUE_SIZE;
  } else {
    states.set_value(node.value);
    (*additional_size) += node.value.size();
  }

  // Set toggled state.
  if (node.HasFlag(flutter::SemanticsFlags::kHasToggledState)) {
    states.set_toggled_state(
        node.HasFlag(flutter::SemanticsFlags::kIsToggled)
            ? fuchsia::accessibility::semantics::ToggledState::ON
            : fuchsia::accessibility::semantics::ToggledState::OFF);
  }

  return states;
}

std::vector<fuchsia::accessibility::semantics::Action>
AccessibilityBridge::GetNodeActions(const flutter::SemanticsNode& node,
                                    size_t* additional_size) const {
  std::vector<fuchsia::accessibility::semantics::Action> node_actions;

  if (node.HasAction(flutter::SemanticsAction::kTap)) {
    node_actions.push_back(fuchsia::accessibility::semantics::Action::DEFAULT);
  }
  if (node.HasAction(flutter::SemanticsAction::kLongPress)) {
    node_actions.push_back(
        fuchsia::accessibility::semantics::Action::SECONDARY);
  }
  if (node.HasAction(flutter::SemanticsAction::kShowOnScreen)) {
    node_actions.push_back(
        fuchsia::accessibility::semantics::Action::SHOW_ON_SCREEN);
  }
  if (node.HasAction(flutter::SemanticsAction::kIncrease)) {
    node_actions.push_back(
        fuchsia::accessibility::semantics::Action::INCREMENT);
  }
  if (node.HasAction(flutter::SemanticsAction::kDecrease)) {
    node_actions.push_back(
        fuchsia::accessibility::semantics::Action::DECREMENT);
  }

  *additional_size +=
      node_actions.size() * sizeof(fuchsia::accessibility::semantics::Action);
  return node_actions;
}

fuchsia::accessibility::semantics::Role AccessibilityBridge::GetNodeRole(
    const flutter::SemanticsNode& node) const {
  if (node.HasFlag(flutter::SemanticsFlags::kIsButton)) {
    return fuchsia::accessibility::semantics::Role::BUTTON;
  }

  if (node.HasFlag(flutter::SemanticsFlags::kIsTextField)) {
    return fuchsia::accessibility::semantics::Role::TEXT_FIELD;
  }

  if (node.HasFlag(flutter::SemanticsFlags::kIsLink)) {
    return fuchsia::accessibility::semantics::Role::LINK;
  }

  if (node.HasFlag(flutter::SemanticsFlags::kIsSlider)) {
    return fuchsia::accessibility::semantics::Role::SLIDER;
  }

  if (node.HasFlag(flutter::SemanticsFlags::kIsHeader)) {
    return fuchsia::accessibility::semantics::Role::HEADER;
  }
  if (node.HasFlag(flutter::SemanticsFlags::kIsImage)) {
    return fuchsia::accessibility::semantics::Role::IMAGE;
  }

  // If a flutter node supports the kIncrease or kDecrease actions, it can be
  // treated as a slider control by assistive technology. This is important
  // because users have special gestures to deal with sliders, and Fuchsia API
  // requires nodes that can receive this kind of action to be a slider control.
  if (node.HasAction(flutter::SemanticsAction::kIncrease) ||
      node.HasAction(flutter::SemanticsAction::kDecrease)) {
    return fuchsia::accessibility::semantics::Role::SLIDER;
  }

  // If a flutter node has a checked state, then we assume it is either a
  // checkbox or a radio button. We distinguish between checkboxes and
  // radio buttons based on membership in a mutually exclusive group.
  if (node.HasFlag(flutter::SemanticsFlags::kHasCheckedState)) {
    if (node.HasFlag(flutter::SemanticsFlags::kIsInMutuallyExclusiveGroup)) {
      return fuchsia::accessibility::semantics::Role::RADIO_BUTTON;
    } else {
      return fuchsia::accessibility::semantics::Role::CHECK_BOX;
    }
  }

  if (node.HasFlag(flutter::SemanticsFlags::kHasToggledState)) {
    return fuchsia::accessibility::semantics::Role::TOGGLE_SWITCH;
  }
  return fuchsia::accessibility::semantics::Role::UNKNOWN;
}

std::unordered_set<int32_t> AccessibilityBridge::GetDescendants(
    int32_t node_id) const {
  std::unordered_set<int32_t> descendents;
  std::deque<int32_t> to_process = {node_id};
  while (!to_process.empty()) {
    int32_t id = to_process.front();
    to_process.pop_front();
    descendents.emplace(id);

    auto it = nodes_.find(id);
    if (it != nodes_.end()) {
      const auto& node = it->second.data;
      for (const auto& child : node.childrenInHitTestOrder) {
        if (descendents.find(child) == descendents.end()) {
          to_process.push_back(child);
        } else {
          // This indicates either a cycle or a child with multiple parents.
          // Flutter should never let this happen, but the engine API does not
          // explicitly forbid it right now.
          // TODO(http://fxbug.dev/75905): Crash flutter accessibility bridge
          // when a cycle in the tree is found.
          FML_LOG(ERROR) << "Semantics Node " << child
                         << " has already been listed as a child of another "
                            "node, ignoring for parent "
                         << id << ".";
        }
      }
    }
  }
  return descendents;
}

// The only known usage of a negative number for a node ID is in the embedder
// API as a sentinel value, which is not expected here. No valid producer of
// nodes should give us a negative ID.
static uint32_t FlutterIdToFuchsiaId(int32_t flutter_node_id) {
  FML_DCHECK(flutter_node_id >= 0)
      << "Unexpectedly received a negative semantics node ID.";
  return static_cast<uint32_t>(flutter_node_id);
}

void AccessibilityBridge::PruneUnreachableNodes(
    FuchsiaAtomicUpdate* atomic_update) {
  const auto& reachable_nodes = GetDescendants(kRootNodeId);
  auto iter = nodes_.begin();
  while (iter != nodes_.end()) {
    int32_t id = iter->first;
    if (reachable_nodes.find(id) == reachable_nodes.end()) {
      atomic_update->AddNodeDeletion(FlutterIdToFuchsiaId(id));
      iter = nodes_.erase(iter);
    } else {
      iter++;
    }
  }
}

// TODO(FIDL-718) - remove this, handle the error instead in something like
// set_error_handler.
static void PrintNodeSizeError(uint32_t node_id) {
  FML_LOG(ERROR) << "Semantics node with ID " << node_id
                 << " exceeded the maximum FIDL message size and may not "
                    "be delivered to the accessibility manager service.";
}

void AccessibilityBridge::AddSemanticsNodeUpdate(
    const flutter::SemanticsNodeUpdates update,
    float view_pixel_ratio) {
  if (update.empty()) {
    return;
  }
  FML_DCHECK(nodes_.find(kRootNodeId) != nodes_.end() ||
             update.find(kRootNodeId) != update.end())
      << "AccessibilityBridge received an update with out ever getting a root "
         "node.";

  FuchsiaAtomicUpdate atomic_update;
  bool has_root_node_update = false;
  // TODO(MI4-2498): Actions, Roles, hit test children, additional
  // flags/states/attr

  // TODO(MI4-1478): Support for partial updates for nodes > 64kb
  // e.g. if a node has a long label or more than 64k children.
  for (const auto& [flutter_node_id, flutter_node] : update) {
    size_t this_node_size = sizeof(fuchsia::accessibility::semantics::Node);
    // We handle root update separately in GetRootNodeUpdate.
    // TODO(chunhtai): remove this special case after we remove the inverse
    // view pixel ratio transformation in scenic view.
    // TODO(http://fxbug.dev/75908): Investigate flutter a11y bridge refactor
    // after removal of the inverse view pixel ratio transformation in scenic
    // view).
    if (flutter_node.id == kRootNodeId) {
      root_flutter_semantics_node_ = flutter_node;
      has_root_node_update = true;
      continue;
    }
    // Store the nodes for later hit testing and logging.
    nodes_[flutter_node.id].data = flutter_node;

    fuchsia::accessibility::semantics::Node fuchsia_node;
    std::vector<uint32_t> child_ids;
    // Send the nodes in traversal order, so the manager can figure out
    // traversal.
    for (int32_t flutter_child_id : flutter_node.childrenInTraversalOrder) {
      child_ids.push_back(FlutterIdToFuchsiaId(flutter_child_id));
    }
    // TODO(http://fxbug.dev/75910): check the usage of FlutterIdToFuchsiaId in
    // the flutter accessibility bridge.
    fuchsia_node.set_node_id(flutter_node.id)
        .set_role(GetNodeRole(flutter_node))
        .set_location(GetNodeLocation(flutter_node))
        .set_transform(GetNodeTransform(flutter_node))
        .set_attributes(GetNodeAttributes(flutter_node, &this_node_size))
        .set_states(GetNodeStates(flutter_node, &this_node_size))
        .set_actions(GetNodeActions(flutter_node, &this_node_size))
        .set_child_ids(child_ids);
    this_node_size +=
        kNodeIdSize * flutter_node.childrenInTraversalOrder.size();

    atomic_update.AddNodeUpdate(std::move(fuchsia_node), this_node_size);
  }

  // Handles root node update.
  if (has_root_node_update || last_seen_view_pixel_ratio_ != view_pixel_ratio) {
    last_seen_view_pixel_ratio_ = view_pixel_ratio;
    size_t root_node_size;
    fuchsia::accessibility::semantics::Node root_update =
        GetRootNodeUpdate(root_node_size);
    atomic_update.AddNodeUpdate(std::move(root_update), root_node_size);
  }

  PruneUnreachableNodes(&atomic_update);
  UpdateScreenRects();

  atomic_updates_->push(std::move(atomic_update));
  if (atomic_updates_->size() == 1) {
    // There were no commits in the queue, so send this one.
    Apply(&atomic_updates_->front());
  }
}

fuchsia::accessibility::semantics::Node AccessibilityBridge::GetRootNodeUpdate(
    size_t& node_size) {
  fuchsia::accessibility::semantics::Node root_fuchsia_node;
  std::vector<uint32_t> child_ids;
  node_size = sizeof(fuchsia::accessibility::semantics::Node);
  for (int32_t flutter_child_id :
       root_flutter_semantics_node_.childrenInTraversalOrder) {
    child_ids.push_back(FlutterIdToFuchsiaId(flutter_child_id));
  }
  // Applies the inverse view pixel ratio transformation to the root node.
  float inverse_view_pixel_ratio = 1.f / last_seen_view_pixel_ratio_;
  SkM44 inverse_view_pixel_ratio_transform;
  inverse_view_pixel_ratio_transform.setScale(inverse_view_pixel_ratio,
                                              inverse_view_pixel_ratio, 1.f);

  SkM44 result = root_flutter_semantics_node_.transform *
                 inverse_view_pixel_ratio_transform;
  nodes_[root_flutter_semantics_node_.id].data = root_flutter_semantics_node_;

  // TODO(http://fxbug.dev/75910): check the usage of FlutterIdToFuchsiaId in
  // the flutter accessibility bridge.
  root_fuchsia_node.set_node_id(root_flutter_semantics_node_.id)
      .set_role(GetNodeRole(root_flutter_semantics_node_))
      .set_location(GetNodeLocation(root_flutter_semantics_node_))
      .set_transform(ConvertSkiaTransformToMat4(result))
      .set_attributes(
          GetNodeAttributes(root_flutter_semantics_node_, &node_size))
      .set_states(GetNodeStates(root_flutter_semantics_node_, &node_size))
      .set_actions(GetNodeActions(root_flutter_semantics_node_, &node_size))
      .set_child_ids(child_ids);
  node_size += kNodeIdSize *
               root_flutter_semantics_node_.childrenInTraversalOrder.size();
  return root_fuchsia_node;
}

void AccessibilityBridge::RequestAnnounce(const std::string message) {
  fuchsia::accessibility::semantics::SemanticEvent semantic_event;
  fuchsia::accessibility::semantics::AnnounceEvent announce_event;
  announce_event.set_message(message);
  semantic_event.set_announce(std::move(announce_event));

  tree_ptr_->SendSemanticEvent(std::move(semantic_event), []() {});
}

void AccessibilityBridge::UpdateScreenRects() {
  std::unordered_set<int32_t> visited_nodes;

  // The embedder applies a special pixel ratio transform to the root of the
  // view, and the accessibility bridge applies the inverse of this transform
  // to the root node. However, this transform is not persisted in the flutter
  // representation of the root node, so we need to account for it explicitly
  // here.
  float inverse_view_pixel_ratio = 1.f / last_seen_view_pixel_ratio_;
  SkM44 inverse_view_pixel_ratio_transform;
  inverse_view_pixel_ratio_transform.setScale(inverse_view_pixel_ratio,
                                              inverse_view_pixel_ratio, 1.f);

  UpdateScreenRects(kRootNodeId, inverse_view_pixel_ratio_transform,
                    &visited_nodes);
}

void AccessibilityBridge::UpdateScreenRects(
    int32_t node_id,
    SkM44 parent_transform,
    std::unordered_set<int32_t>* visited_nodes) {
  auto it = nodes_.find(node_id);
  if (it == nodes_.end()) {
    FML_LOG(ERROR) << "UpdateScreenRects called on unknown node";
    return;
  }
  auto& node = it->second;
  const auto& current_transform = parent_transform * node.data.transform;

  const auto& rect = node.data.rect;
  SkV4 dst[2] = {
      current_transform.map(rect.left(), rect.top(), 0, 1),
      current_transform.map(rect.right(), rect.bottom(), 0, 1),
  };
  node.screen_rect.setLTRB(dst[0].x, dst[0].y, dst[1].x, dst[1].y);
  node.screen_rect.sort();

  visited_nodes->emplace(node_id);

  for (uint32_t child_id : node.data.childrenInHitTestOrder) {
    if (visited_nodes->find(child_id) == visited_nodes->end()) {
      UpdateScreenRects(child_id, current_transform, visited_nodes);
    }
  }
}

std::optional<flutter::SemanticsAction>
AccessibilityBridge::GetFlutterSemanticsAction(
    fuchsia::accessibility::semantics::Action fuchsia_action,
    uint32_t node_id) {
  switch (fuchsia_action) {
    // The default action associated with the element.
    case fuchsia::accessibility::semantics::Action::DEFAULT:
      return flutter::SemanticsAction::kTap;
    // The secondary action associated with the element. This may correspond to
    // a long press (touchscreens) or right click (mouse).
    case fuchsia::accessibility::semantics::Action::SECONDARY:
      return flutter::SemanticsAction::kLongPress;
    // Set (input/non-accessibility) focus on this element.
    case fuchsia::accessibility::semantics::Action::SET_FOCUS:
      FML_LOG(WARNING)
          << "Unsupported action SET_FOCUS sent for accessibility node "
          << node_id;
      return {};
    // Set the element's value.
    case fuchsia::accessibility::semantics::Action::SET_VALUE:
      FML_LOG(WARNING)
          << "Unsupported action SET_VALUE sent for accessibility node "
          << node_id;
      return {};
    // Scroll node to make it visible.
    case fuchsia::accessibility::semantics::Action::SHOW_ON_SCREEN:
      return flutter::SemanticsAction::kShowOnScreen;
    case fuchsia::accessibility::semantics::Action::INCREMENT:
      return flutter::SemanticsAction::kIncrease;
    case fuchsia::accessibility::semantics::Action::DECREMENT:
      return flutter::SemanticsAction::kDecrease;
    default:
      FML_LOG(WARNING) << "Unexpected action "
                       << static_cast<int32_t>(fuchsia_action)
                       << " sent for accessibility node " << node_id;
      return {};
  }
}

// |fuchsia::accessibility::semantics::SemanticListener|
void AccessibilityBridge::OnAccessibilityActionRequested(
    uint32_t node_id,
    fuchsia::accessibility::semantics::Action action,
    fuchsia::accessibility::semantics::SemanticListener::
        OnAccessibilityActionRequestedCallback callback) {
  // TODO(http://fxbug.dev/75910): check the usage of FlutterIdToFuchsiaId in
  // the flutter accessibility bridge.
  if (nodes_.find(node_id) == nodes_.end()) {
    FML_LOG(ERROR) << "Attempted to send accessibility action "
                   << static_cast<int32_t>(action)
                   << " to unknown node id: " << node_id;
    callback(false);
    return;
  }

  std::optional<flutter::SemanticsAction> flutter_action =
      GetFlutterSemanticsAction(action, node_id);
  if (!flutter_action.has_value()) {
    callback(false);
    return;
  }
  dispatch_semantics_action_callback_(static_cast<int32_t>(node_id),
                                      flutter_action.value());
  callback(true);
}

// |fuchsia::accessibility::semantics::SemanticListener|
void AccessibilityBridge::HitTest(
    fuchsia::math::PointF local_point,
    fuchsia::accessibility::semantics::SemanticListener::HitTestCallback
        callback) {
  auto hit_node_id = GetHitNode(kRootNodeId, local_point.x, local_point.y);
  FML_DCHECK(hit_node_id.has_value());
  fuchsia::accessibility::semantics::Hit hit;
  // TODO(http://fxbug.dev/75910): check the usage of FlutterIdToFuchsiaId in
  // the flutter accessibility bridge.
  hit.set_node_id(hit_node_id.value_or(kRootNodeId));
  callback(std::move(hit));
}

std::optional<int32_t> AccessibilityBridge::GetHitNode(int32_t node_id,
                                                       float x,
                                                       float y) {
  auto it = nodes_.find(node_id);
  if (it == nodes_.end()) {
    FML_LOG(ERROR) << "Attempted to hit test unknown node id: " << node_id;
    return {};
  }
  auto const& node = it->second;
  if (node.data.flags &
          static_cast<int32_t>(flutter::SemanticsFlags::kIsHidden) ||  //
      !node.screen_rect.contains(x, y)) {
    return {};
  }
  for (int32_t child_id : node.data.childrenInHitTestOrder) {
    auto candidate = GetHitNode(child_id, x, y);
    if (candidate) {
      return candidate;
    }
  }

  if (IsFocusable(node.data)) {
    return node_id;
  }

  return {};
}

bool AccessibilityBridge::IsFocusable(
    const flutter::SemanticsNode& node) const {
  if (node.HasFlag(flutter::SemanticsFlags::kScopesRoute)) {
    return false;
  }

  if (node.HasFlag(flutter::SemanticsFlags::kIsFocusable)) {
    return true;
  }

  // Always consider platform views focusable.
  if (node.IsPlatformViewNode()) {
    return true;
  }

  // Always consider actionable nodes focusable.
  if (node.actions != 0) {
    return true;
  }

  // Consider text nodes focusable.
  return !node.label.empty() || !node.value.empty() || !node.hint.empty();
}

// |fuchsia::accessibility::semantics::SemanticListener|
void AccessibilityBridge::OnSemanticsModeChanged(
    bool enabled,
    OnSemanticsModeChangedCallback callback) {
  set_semantics_enabled_callback_(enabled);
}

#if !FLUTTER_RELEASE
void AccessibilityBridge::FillInspectTree(int32_t flutter_node_id,
                                          int32_t current_level,
                                          inspect::Node inspect_node,
                                          inspect::Inspector* inspector) const {
  const auto it = nodes_.find(flutter_node_id);
  if (it == nodes_.end()) {
    inspect_node.CreateString(
        "missing_child",
        "This node has a parent in the semantic tree but has no value",
        inspector);
    inspector->emplace(std::move(inspect_node));
    return;
  }
  const auto& semantic_node = it->second;
  const auto& data = semantic_node.data;

  inspect_node.CreateInt("id", data.id, inspector);

  // Even with an empty label, we still want to create the property to
  // explicetly show that it is empty.
  inspect_node.CreateString("label", data.label, inspector);
  if (!data.hint.empty()) {
    inspect_node.CreateString("hint", data.hint, inspector);
  }
  if (!data.value.empty()) {
    inspect_node.CreateString("value", data.value, inspector);
  }
  if (!data.increasedValue.empty()) {
    inspect_node.CreateString("increased_value", data.increasedValue,
                              inspector);
  }
  if (!data.decreasedValue.empty()) {
    inspect_node.CreateString("decreased_value", data.decreasedValue,
                              inspector);
  }

  if (data.textDirection) {
    inspect_node.CreateString(
        "text_direction", data.textDirection == 1 ? "RTL" : "LTR", inspector);
  }

  if (data.flags) {
    inspect_node.CreateString("flags", NodeFlagsToString(data), inspector);
  }
  if (data.actions) {
    inspect_node.CreateString("actions", NodeActionsToString(data), inspector);
  }

  inspect_node.CreateString(
      "location", NodeLocationToString(semantic_node.screen_rect), inspector);
  if (!data.childrenInTraversalOrder.empty() ||
      !data.childrenInHitTestOrder.empty()) {
    inspect_node.CreateString("children", NodeChildrenToString(data),
                              inspector);
  }

  inspect_node.CreateInt("current_level", current_level, inspector);

  for (int32_t flutter_child_id : semantic_node.data.childrenInTraversalOrder) {
    const auto inspect_name = "node_" + std::to_string(flutter_child_id);
    FillInspectTree(flutter_child_id, current_level + 1,
                    inspect_node.CreateChild(inspect_name), inspector);
  }

  inspector->emplace(std::move(inspect_node));
}
#endif  // !FLUTTER_RELEASE

void AccessibilityBridge::Apply(FuchsiaAtomicUpdate* atomic_update) {
  size_t begin = 0;
  auto it = atomic_update->deletions.begin();

  // Process up to kMaxDeletionsPerUpdate deletions at a time.
  while (it != atomic_update->deletions.end()) {
    std::vector<uint32_t> to_delete;
    size_t end = std::min(atomic_update->deletions.size() - begin,
                          kMaxDeletionsPerUpdate);
    std::copy(std::make_move_iterator(it), std::make_move_iterator(it + end),
              std::back_inserter(to_delete));
    tree_ptr_->DeleteSemanticNodes(std::move(to_delete));
    begin = end;
    it += end;
  }

  std::vector<fuchsia::accessibility::semantics::Node> to_update;
  size_t current_size = 0;
  for (auto& node_and_size : atomic_update->updates) {
    if (current_size + node_and_size.second > kMaxMessageSize) {
      tree_ptr_->UpdateSemanticNodes(std::move(to_update));
      current_size = 0;
      to_update.clear();
    }
    current_size += node_and_size.second;
    to_update.push_back(std::move(node_and_size.first));
  }
  if (!to_update.empty()) {
    tree_ptr_->UpdateSemanticNodes(std::move(to_update));
  }

  // Commit this update and subsequent ones; for flow control wait for a
  // response between each commit.
  tree_ptr_->CommitUpdates(
      [this, atomic_updates = std::weak_ptr<std::queue<FuchsiaAtomicUpdate>>(
                 atomic_updates_)]() {
        auto atomic_updates_ptr = atomic_updates.lock();
        if (!atomic_updates_ptr) {
          // The queue no longer exists, which means that is no longer
          // necessary.
          return;
        }
        // Removes the update that just went through.
        atomic_updates_ptr->pop();
        if (!atomic_updates_ptr->empty()) {
          Apply(&atomic_updates_ptr->front());
        }
      });

  atomic_update->deletions.clear();
  atomic_update->updates.clear();
}

void AccessibilityBridge::FuchsiaAtomicUpdate::AddNodeUpdate(
    fuchsia::accessibility::semantics::Node node,
    size_t size) {
  if (size > kMaxMessageSize) {
    // TODO(MI4-2531, FIDL-718): Remove this
    // This is defensive. If, despite our best efforts, we ended up with a node
    // that is larger than the max fidl size, we send no updates.
    PrintNodeSizeError(node.node_id());
    return;
  }
  updates.emplace_back(std::move(node), size);
}

void AccessibilityBridge::FuchsiaAtomicUpdate::AddNodeDeletion(uint32_t id) {
  deletions.push_back(id);
}
}  // namespace flutter_runner
