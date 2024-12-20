// Copyright 2017 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef UI_ACCESSIBILITY_AX_EVENT_GENERATOR_H_
#define UI_ACCESSIBILITY_AX_EVENT_GENERATOR_H_

#include <bitset>
#include <map>
#include <ostream>
#include <set>
#include <vector>

#include "ax_event_intent.h"
#include "ax_export.h"
#include "ax_tree.h"
#include "ax_tree_observer.h"

namespace ui {

// Subclass of AXTreeObserver that automatically generates AXEvents to fire
// based on changes to an accessibility tree.  Every platform
// tends to want different events, so this class lets each platform
// handle the events it wants and ignore the others.
class AX_EXPORT AXEventGenerator : public AXTreeObserver {
 public:
  enum class Event : int32_t {
    ACCESS_KEY_CHANGED,
    ACTIVE_DESCENDANT_CHANGED,
    ALERT,
    // ATK treats alignment, indentation, and other format-related attributes as
    // text attributes even when they are only applicable to the entire object.
    // And it lacks an event for object attributes changing.
    ATK_TEXT_OBJECT_ATTRIBUTE_CHANGED,
    ATOMIC_CHANGED,
    AUTO_COMPLETE_CHANGED,
    BUSY_CHANGED,
    CHECKED_STATE_CHANGED,
    CHILDREN_CHANGED,
    CLASS_NAME_CHANGED,
    COLLAPSED,
    CONTROLS_CHANGED,
    DESCRIBED_BY_CHANGED,
    DESCRIPTION_CHANGED,
    DOCUMENT_SELECTION_CHANGED,
    DOCUMENT_TITLE_CHANGED,
    DROPEFFECT_CHANGED,
    ENABLED_CHANGED,
    EXPANDED,
    FOCUS_CHANGED,
    FLOW_FROM_CHANGED,
    FLOW_TO_CHANGED,
    GRABBED_CHANGED,
    HASPOPUP_CHANGED,
    HIERARCHICAL_LEVEL_CHANGED,
    IGNORED_CHANGED,
    IMAGE_ANNOTATION_CHANGED,
    INVALID_STATUS_CHANGED,
    KEY_SHORTCUTS_CHANGED,
    LABELED_BY_CHANGED,
    LANGUAGE_CHANGED,
    LAYOUT_INVALIDATED,   // Fired when aria-busy goes false
    LIVE_REGION_CHANGED,  // Fired on the root of a live region.
    LIVE_REGION_CREATED,
    LIVE_REGION_NODE_CHANGED,  // Fired on a node within a live region.
    LIVE_RELEVANT_CHANGED,
    LIVE_STATUS_CHANGED,
    LOAD_COMPLETE,
    LOAD_START,
    MENU_ITEM_SELECTED,
    MULTILINE_STATE_CHANGED,
    MULTISELECTABLE_STATE_CHANGED,
    NAME_CHANGED,
    OBJECT_ATTRIBUTE_CHANGED,
    OTHER_ATTRIBUTE_CHANGED,
    PLACEHOLDER_CHANGED,
    PORTAL_ACTIVATED,
    POSITION_IN_SET_CHANGED,
    RELATED_NODE_CHANGED,
    READONLY_CHANGED,
    REQUIRED_STATE_CHANGED,
    ROLE_CHANGED,
    ROW_COUNT_CHANGED,
    SCROLL_HORIZONTAL_POSITION_CHANGED,
    SCROLL_VERTICAL_POSITION_CHANGED,
    SELECTED_CHANGED,
    SELECTED_CHILDREN_CHANGED,
    SET_SIZE_CHANGED,
    SORT_CHANGED,
    STATE_CHANGED,
    SUBTREE_CREATED,
    TEXT_ATTRIBUTE_CHANGED,
    VALUE_CHANGED,
    VALUE_MAX_CHANGED,
    VALUE_MIN_CHANGED,
    VALUE_STEP_CHANGED,

    // This event is for the exact set of attributes that affect
    // the MSAA/IAccessible state on Windows. Not needed on other platforms,
    // but very natural to compute here.
    WIN_IACCESSIBLE_STATE_CHANGED,
  };

  // For distinguishing between show and hide state when a node has
  // IGNORED_CHANGED event.
  enum class IgnoredChangedState : uint8_t { kShow, kHide, kCount = 2 };

  struct EventParams {
    EventParams(Event event,
                ax::mojom::EventFrom event_from,
                const std::vector<AXEventIntent>& event_intents);
    ~EventParams();
    Event event;
    ax::mojom::EventFrom event_from;
    std::vector<AXEventIntent> event_intents;

    bool operator==(const EventParams& rhs);
    bool operator<(const EventParams& rhs) const;
  };

  struct TargetedEvent {
    // |node| must not be null
    TargetedEvent(ui::AXNode* node, const EventParams& event_params);
    ui::AXNode* node;
    const EventParams& event_params;
  };

  class AX_EXPORT Iterator
      : public std::iterator<std::input_iterator_tag, TargetedEvent> {
   public:
    Iterator(
        const std::map<AXNode*, std::set<EventParams>>& map,
        const std::map<AXNode*, std::set<EventParams>>::const_iterator& head);
    Iterator(const Iterator& other);
    ~Iterator();

    bool operator!=(const Iterator& rhs) const;
    Iterator& operator++();
    TargetedEvent operator*() const;

   private:
    const std::map<AXNode*, std::set<EventParams>>& map_;
    std::map<AXNode*, std::set<EventParams>>::const_iterator map_iter_;
    std::set<EventParams>::const_iterator set_iter_;
  };

  // For storing ignored changed states for a particular node. We use bitset as
  // the underlying data structure to improve memory usage.
  // We use the index of AXEventGenerator::IgnoredChangedState enum
  // to access the bitset data.
  // e.g. AXEventGenerator::IgnoredChangedState::kShow has index 0 in the
  // IgnoredChangedState enum. If |IgnoredChangedStatesBitset[0]| is set, it
  // means IgnoredChangedState::kShow is present. Similarly, kHide has index 1
  // in the enum, and it corresponds to |IgnoredChangedStatesBitset[1]|.
  using IgnoredChangedStatesBitset =
      std::bitset<static_cast<size_t>(IgnoredChangedState::kCount)>;
  using const_iterator = Iterator;
  using iterator = Iterator;
  using value_type = TargetedEvent;

  // If you use this constructor, you must call SetTree
  // before using this class.
  AXEventGenerator();

  // Automatically registers itself as the observer of |tree| and
  // clears it on destruction. |tree| must be valid for the lifetime
  // of this object.
  explicit AXEventGenerator(AXTree* tree);

  ~AXEventGenerator() override;

  // Clears this class as the observer of the previous tree that was
  // being monitored, if any, and starts monitoring |new_tree|, if not
  // nullptr. Note that |new_tree| must be valid for the lifetime of
  // this object or until you call SetTree again.
  void SetTree(AXTree* new_tree);

  // Null |tree_| without accessing it or destroying it.
  void ReleaseTree();

  Iterator begin() const {
    return Iterator(tree_events_, tree_events_.begin());
  }
  Iterator end() const { return Iterator(tree_events_, tree_events_.end()); }

  // Clear any previously added events.
  void ClearEvents();

  // This is called automatically based on changes to the tree observed
  // by AXTreeObserver, but you can also call it directly to add events
  // and retrieve them later.
  //
  // Note that events are organized by node and then by event id to
  // efficiently remove duplicates, so events won't be retrieved in the
  // same order they were added.
  void AddEvent(ui::AXNode* node, Event event);

  void set_always_fire_load_complete(bool val) {
    always_fire_load_complete_ = val;
  }

 protected:
  // AXTreeObserver overrides.
  void OnNodeDataChanged(AXTree* tree,
                         const AXNodeData& old_node_data,
                         const AXNodeData& new_node_data) override;
  void OnRoleChanged(AXTree* tree,
                     AXNode* node,
                     ax::mojom::Role old_role,
                     ax::mojom::Role new_role) override;
  void OnStateChanged(AXTree* tree,
                      AXNode* node,
                      ax::mojom::State state,
                      bool new_value) override;
  void OnStringAttributeChanged(AXTree* tree,
                                AXNode* node,
                                ax::mojom::StringAttribute attr,
                                const std::string& old_value,
                                const std::string& new_value) override;
  void OnIntAttributeChanged(AXTree* tree,
                             AXNode* node,
                             ax::mojom::IntAttribute attr,
                             int32_t old_value,
                             int32_t new_value) override;
  void OnFloatAttributeChanged(AXTree* tree,
                               AXNode* node,
                               ax::mojom::FloatAttribute attr,
                               float old_value,
                               float new_value) override;
  void OnBoolAttributeChanged(AXTree* tree,
                              AXNode* node,
                              ax::mojom::BoolAttribute attr,
                              bool new_value) override;
  void OnIntListAttributeChanged(
      AXTree* tree,
      AXNode* node,
      ax::mojom::IntListAttribute attr,
      const std::vector<int32_t>& old_value,
      const std::vector<int32_t>& new_value) override;
  void OnTreeDataChanged(AXTree* tree,
                         const ui::AXTreeData& old_data,
                         const ui::AXTreeData& new_data) override;
  void OnNodeWillBeDeleted(AXTree* tree, AXNode* node) override;
  void OnSubtreeWillBeDeleted(AXTree* tree, AXNode* node) override;
  void OnNodeWillBeReparented(AXTree* tree, AXNode* node) override;
  void OnSubtreeWillBeReparented(AXTree* tree, AXNode* node) override;
  void OnAtomicUpdateFinished(AXTree* tree,
                              bool root_changed,
                              const std::vector<Change>& changes) override;

 private:
  void FireLiveRegionEvents(AXNode* node);
  void FireActiveDescendantEvents();
  void FireRelationSourceEvents(AXTree* tree, AXNode* target_node);
  bool ShouldFireLoadEvents(AXNode* node);
  // Remove excessive events for a tree update containing node.
  // We remove certain events on a node when it flips its IGNORED state to
  // either show/hide and one of the node's ancestor has also flipped its
  // IGNORED state in the same way (show/hide) in the tree update.
  // |ancestor_has_ignored_map| contains if a node's ancestor has changed to
  // IGNORED state.
  // Map's key is an AXNode.
  // Map's value is a std::bitset containing IgnoredChangedStates(kShow/kHide).
  // - Map's value IgnoredChangedStatesBitset contains kShow if an ancestor
  //   of node removed its IGNORED state.
  // - Map's value IgnoredChangedStatesBitset contains kHide if an ancestor
  //   of node changed to IGNORED state.
  // - When IgnoredChangedStatesBitset is not set, it means neither the
  //   node nor its ancestor has IGNORED_CHANGED.
  void TrimEventsDueToAncestorIgnoredChanged(
      AXNode* node,
      std::map<AXNode*, IgnoredChangedStatesBitset>&
          ancestor_ignored_changed_map);
  void PostprocessEvents();
  static void GetRestrictionStates(ax::mojom::Restriction restriction,
                                   bool* is_enabled,
                                   bool* is_readonly);

  // Returns a vector of values unique to either |lhs| or |rhs|
  static std::vector<int32_t> ComputeIntListDifference(
      const std::vector<int32_t>& lhs,
      const std::vector<int32_t>& rhs);

  AXTree* tree_ = nullptr;  // Not owned.
  std::map<AXNode*, std::set<EventParams>> tree_events_;

  // Valid between the call to OnIntAttributeChanged and the call to
  // OnAtomicUpdateFinished. List of nodes whose active descendant changed.
  std::vector<AXNode*> active_descendant_changed_;

  bool always_fire_load_complete_ = false;
};

AX_EXPORT std::ostream& operator<<(std::ostream& os,
                                   AXEventGenerator::Event event);
AX_EXPORT const char* ToString(AXEventGenerator::Event event);

}  // namespace ui

#endif  // UI_ACCESSIBILITY_AX_EVENT_GENERATOR_H_
