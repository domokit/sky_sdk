// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_ACCESSIBILITY_BRIDGE_H_
#define FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_ACCESSIBILITY_BRIDGE_H_

// Work around symbol conflicts with ICU.
#undef TRUE
#undef FALSE

#include <fuchsia/accessibility/semantics/cpp/fidl.h>
#include <fuchsia/ui/gfx/cpp/fidl.h>
#include <lib/fidl/cpp/binding_set.h>
#include <lib/inspect/component/cpp/component.h>
#include <zircon/types.h>

#include <memory>
#include <optional>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "flutter/fml/macros.h"
#include "flutter/lib/ui/semantics/semantics_node.h"

namespace flutter_runner {

// Accessibility bridge.
//
// This class intermediates accessibility-related calls between Fuchsia and
// Flutter. It serves to resolve the impedance mismatch between Flutter's
// platform-agnostic accessibility APIs and Fuchsia's APIs and behaviour.
//
// This bridge performs the following functions, among others:
//
// * Translates Flutter's semantics node updates to events Fuchsia requires
//   (e.g. Flutter only sends updates for changed nodes, but Fuchsia requires
//   the entire flattened subtree to be sent when a node changes.
class AccessibilityBridge
    : public fuchsia::accessibility::semantics::SemanticListener {
 public:
  using SetSemanticsEnabledCallback = std::function<void(bool)>;
  using DispatchSemanticsActionCallback =
      std::function<void(int32_t, flutter::SemanticsAction)>;

  // TODO(MI4-2531, FIDL-718): Remove this. We shouldn't be worried about
  // batching messages at this level.
  // FIDL may encode a C++ struct as larger than the sizeof the C++ struct.
  // This is to make sure we don't send updates that are too large.
  static constexpr uint32_t kMaxMessageSize = ZX_CHANNEL_MAX_MSG_BYTES / 2;

  static_assert(fuchsia::accessibility::semantics::MAX_LABEL_SIZE <
                kMaxMessageSize - 1);

  // Flutter uses signed 32 bit integers for node IDs, while Fuchsia uses
  // unsigned 32 bit integers. A change in the size on either one would break
  // casts and size tracking logic in the implementation.
  static constexpr size_t kNodeIdSize = sizeof(flutter::SemanticsNode::id);
  static_assert(
      kNodeIdSize ==
          sizeof(fuchsia::accessibility::semantics::Node().node_id()),
      "flutter::SemanticsNode::id and "
      "fuchsia::accessibility::semantics::Node::node_id differ in size.");

  // Maximum number of node ids to be deleted through the Semantics API.
  static constexpr size_t kMaxDeletionsPerUpdate =
      kMaxMessageSize / kNodeIdSize;

  AccessibilityBridge(
      SetSemanticsEnabledCallback set_semantics_enabled_callback,
      DispatchSemanticsActionCallback dispatch_semantics_action_callback,
      fuchsia::accessibility::semantics::SemanticsManagerHandle
          semantics_manager,
      fuchsia::ui::views::ViewRef view_ref,
      inspect::Node inspect_node);

  // Returns true if accessible navigation is enabled.
  bool GetSemanticsEnabled() const;

  // Enables Flutter accessibility navigation features.
  //
  // Once enabled, any semantics updates in the Flutter application will
  // trigger |FuchsiaAccessibility::DispatchAccessibilityEvent| callbacks
  // to send events back to the Fuchsia SemanticsManager.
  void SetSemanticsEnabled(bool enabled);

  // Adds a semantics node update to the buffer of node updates to apply.
  void AddSemanticsNodeUpdate(const flutter::SemanticsNodeUpdates update,
                              float view_pixel_ratio);

  // Requests a message announcement from the accessibility TTS system.
  void RequestAnnounce(const std::string message);

  // Notifies the bridge of a 'hover move' touch exploration event.
  zx_status_t OnHoverMove(double x, double y);

  // |fuchsia::accessibility::semantics::SemanticListener|
  void HitTest(
      fuchsia::math::PointF local_point,
      fuchsia::accessibility::semantics::SemanticListener::HitTestCallback
          callback) override;

  // |fuchsia::accessibility::semantics::SemanticListener|
  void OnAccessibilityActionRequested(
      uint32_t node_id,
      fuchsia::accessibility::semantics::Action action,
      fuchsia::accessibility::semantics::SemanticListener::
          OnAccessibilityActionRequestedCallback callback) override;

 private:
  // Fuchsia's default root semantic node id.
  static constexpr int32_t kRootNodeId = 0;

  // Represents an atomic semantic update to Fuchsia, which can contain deletes
  // and updates of semantic nodes.
  //
  // An atomic update is a set of operations that take a semantic tree in a
  // valid state to another valid state. Please check the semantics FIDL
  // documentation for details.
  struct FuchsiaAtomicUpdate {
    FuchsiaAtomicUpdate() = default;
    ~FuchsiaAtomicUpdate() = default;
    FuchsiaAtomicUpdate(FuchsiaAtomicUpdate&&) = default;

    // Adds a node to be updated. |size| contains the
    // size in bytes of the node to be updated.
    void AddNodeUpdate(fuchsia::accessibility::semantics::Node node,
                       size_t size);

    // Adds a node to be deleted.
    void AddNodeDeletion(uint32_t id);

    std::vector<std::pair<fuchsia::accessibility::semantics::Node, size_t>>
        updates;
    std::vector<uint32_t> deletions;
  };

  // Holds a flutter semantic node and some extra info.
  // In particular, it adds a screen_rect field to flutter::SemanticsNode.
  struct SemanticsNode {
    flutter::SemanticsNode data;
    SkRect screen_rect;
  };

  fuchsia::accessibility::semantics::Node GetRootNodeUpdate(size_t& node_size);

  // Derives the BoundingBox of a Flutter semantics node from its
  // rect and elevation.
  fuchsia::ui::gfx::BoundingBox GetNodeLocation(
      const flutter::SemanticsNode& node) const;

  // Gets mat4 transformation from a Flutter semantics node.
  fuchsia::ui::gfx::mat4 GetNodeTransform(
      const flutter::SemanticsNode& node) const;

  // Converts a Flutter semantics node's transformation to a mat4.
  fuchsia::ui::gfx::mat4 ConvertSkiaTransformToMat4(
      const SkM44 transform) const;

  // Derives the attributes for a Fuchsia semantics node from a Flutter
  // semantics node.
  fuchsia::accessibility::semantics::Attributes GetNodeAttributes(
      const flutter::SemanticsNode& node,
      size_t* added_size) const;

  // Derives the states for a Fuchsia semantics node from a Flutter semantics
  // node.
  fuchsia::accessibility::semantics::States GetNodeStates(
      const flutter::SemanticsNode& node,
      size_t* additional_size) const;

  // Derives the set of supported actions for a Fuchsia semantics node from
  // a Flutter semantics node.
  std::vector<fuchsia::accessibility::semantics::Action> GetNodeActions(
      const flutter::SemanticsNode& node,
      size_t* additional_size) const;

  // Derives the role for a Fuchsia semantics node from a Flutter
  // semantics node.
  fuchsia::accessibility::semantics::Role GetNodeRole(
      const flutter::SemanticsNode& node) const;

  // Gets the set of reachable descendants from the given node id.
  std::unordered_set<int32_t> GetDescendants(int32_t node_id) const;

  // Removes internal references to any dangling nodes from previous
  // updates, and adds the nodes to be deleted to the current |atomic_update|.
  //
  // The node ids to be deleted are only collected at this point, and will be
  // committed in the next call to |Apply()|.
  void PruneUnreachableNodes(FuchsiaAtomicUpdate* atomic_update);

  // Updates the on-screen positions of accessibility elements,
  // starting from the root element with an identity matrix.
  //
  // This should be called from Update.
  void UpdateScreenRects();

  // Updates the on-screen positions of accessibility elements, starting
  // from node_id and using the specified transform.
  //
  // Update calls this via UpdateScreenRects().
  void UpdateScreenRects(int32_t node_id,
                         SkM44 parent_transform,
                         std::unordered_set<int32_t>* visited_nodes);

  // Traverses the semantics tree to find the node_id hit by the given x,y
  // point.
  //
  // Assumes that SemanticsNode::screen_rect is up to date.
  std::optional<int32_t> GetHitNode(int32_t node_id, float x, float y);

  // Returns whether the node is considered focusable.
  bool IsFocusable(const flutter::SemanticsNode& node) const;

  // Converts a fuchsia::accessibility::semantics::Action to a
  // flutter::SemanticsAction.
  //
  // The node_id parameter is used for printing warnings about unsupported
  // action types.
  std::optional<flutter::SemanticsAction> GetFlutterSemanticsAction(
      fuchsia::accessibility::semantics::Action fuchsia_action,
      uint32_t node_id);

  // Applies the updates and deletions in |atomic_update|, sending them via
  // |tree_ptr|.
  void Apply(FuchsiaAtomicUpdate* atomic_update);

  // |fuchsia::accessibility::semantics::SemanticListener|
  void OnSemanticsModeChanged(bool enabled,
                              OnSemanticsModeChangedCallback callback) override;

#if !FLUTTER_RELEASE
  // Fills the inspect tree with debug information about the semantic tree.
  void FillInspectTree(int32_t flutter_node_id,
                       int32_t current_level,
                       inspect::Node inspect_node,
                       inspect::Inspector* inspector) const;
#endif  // !FLUTTER_RELEASE

  SetSemanticsEnabledCallback set_semantics_enabled_callback_;
  DispatchSemanticsActionCallback dispatch_semantics_action_callback_;
  flutter::SemanticsNode root_flutter_semantics_node_;
  float last_seen_view_pixel_ratio_ = 1.f;

  fidl::Binding<fuchsia::accessibility::semantics::SemanticListener> binding_;
  fuchsia::accessibility::semantics::SemanticsManagerPtr
      fuchsia_semantics_manager_;
  fuchsia::accessibility::semantics::SemanticTreePtr tree_ptr_;

  // This is the cache of all nodes we've sent to Fuchsia's SemanticsManager.
  // Assists with pruning unreachable nodes and hit testing.
  std::unordered_map<int32_t, SemanticsNode> nodes_;
  bool semantics_enabled_;

  // Queue of atomic updates to be sent to Fuchsia.
  std::shared_ptr<std::queue<FuchsiaAtomicUpdate>> atomic_updates_;

  // Node to publish inspect data.
  inspect::Node inspect_node_;

#if !FLUTTER_RELEASE
  // Inspect node to store a dump of the semantic tree. Note that this only gets
  // computed if requested, so it does not use memory to store the dump unless
  // an explicit request is made.
  inspect::LazyNode inspect_node_tree_dump_;
#endif  // !FLUTTER_RELEASE

  FML_DISALLOW_COPY_AND_ASSIGN(AccessibilityBridge);
};
}  // namespace flutter_runner

#endif  // FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_ACCESSIBILITY_BRIDGE_H_
