// Copyright 2017 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "ax_event_generator.h"

#include <algorithm>

#include "ax_enums.h"
#include "ax_node.h"
#include "ax_role_properties.h"
#include "base/container_utils.h"

namespace ui {
namespace {

bool IsActiveLiveRegion(const AXTreeObserver::Change& change) {
  return change.node->data().HasStringAttribute(
             ax::mojom::StringAttribute::kLiveStatus) &&
         change.node->data().GetStringAttribute(
             ax::mojom::StringAttribute::kLiveStatus) != "off";
}

bool IsContainedInLiveRegion(const AXTreeObserver::Change& change) {
  return change.node->data().HasStringAttribute(
             ax::mojom::StringAttribute::kContainerLiveStatus) &&
         change.node->data().HasStringAttribute(
             ax::mojom::StringAttribute::kName);
}

bool HasEvent(const std::set<AXEventGenerator::EventParams>& node_events,
              AXEventGenerator::Event event) {
  for (auto& iter : node_events) {
    if (iter.event == event)
      return true;
  }
  return false;
}

void RemoveEvent(std::set<AXEventGenerator::EventParams>* node_events,
                 AXEventGenerator::Event event) {
  for (auto& iter : *node_events) {
    if (iter.event == event) {
      node_events->erase(iter);
      return;
    }
  }
}

// If a node toggled its ignored state, don't also fire children-changed because
// platforms likely will do that in response to ignored-changed.
// Suppress name- and description-changed because those can be emitted as a side
// effect of calculating alternative text values for a newly-displayed object.
// Ditto for text attributes such as foreground and background colors, or
// display changing from "none" to "block."
void RemoveEventsDueToIgnoredChanged(
    std::set<AXEventGenerator::EventParams>* node_events) {
  RemoveEvent(node_events,
              AXEventGenerator::Event::ATK_TEXT_OBJECT_ATTRIBUTE_CHANGED);
  RemoveEvent(node_events, AXEventGenerator::Event::CHILDREN_CHANGED);
  RemoveEvent(node_events, AXEventGenerator::Event::DESCRIPTION_CHANGED);
  RemoveEvent(node_events, AXEventGenerator::Event::NAME_CHANGED);
  RemoveEvent(node_events, AXEventGenerator::Event::OBJECT_ATTRIBUTE_CHANGED);
  RemoveEvent(node_events, AXEventGenerator::Event::SORT_CHANGED);
  RemoveEvent(node_events, AXEventGenerator::Event::TEXT_ATTRIBUTE_CHANGED);
  RemoveEvent(node_events,
              AXEventGenerator::Event::WIN_IACCESSIBLE_STATE_CHANGED);
}

// Add a particular AXEventGenerator::IgnoredChangedState to
// |ignored_changed_states|.
void AddIgnoredChangedState(
    AXEventGenerator::IgnoredChangedStatesBitset& ignored_changed_states,
    AXEventGenerator::IgnoredChangedState state) {
  ignored_changed_states.set(static_cast<size_t>(state));
}

// Returns true if |ignored_changed_states| contains a particular
// AXEventGenerator::IgnoredChangedState.
bool HasIgnoredChangedState(
    AXEventGenerator::IgnoredChangedStatesBitset& ignored_changed_states,
    AXEventGenerator::IgnoredChangedState state) {
  return ignored_changed_states[static_cast<size_t>(state)];
}

}  // namespace

AXEventGenerator::EventParams::EventParams(
    Event event,
    ax::mojom::EventFrom event_from,
    const std::vector<AXEventIntent>& event_intents)
    : event(event), event_from(event_from), event_intents(event_intents) {}

AXEventGenerator::EventParams::~EventParams() = default;

AXEventGenerator::TargetedEvent::TargetedEvent(AXNode* node,
                                               const EventParams& event_params)
    : node(node), event_params(event_params) {
  BASE_DCHECK(node);
}

bool AXEventGenerator::EventParams::operator==(const EventParams& rhs) {
  return rhs.event == event;
}

bool AXEventGenerator::EventParams::operator<(const EventParams& rhs) const {
  return event < rhs.event;
}

AXEventGenerator::Iterator::Iterator(
    const std::map<AXNode*, std::set<EventParams>>& map,
    const std::map<AXNode*, std::set<EventParams>>::const_iterator& head)
    : map_(map), map_iter_(head) {
  if (map_iter_ != map.end())
    set_iter_ = map_iter_->second.begin();
}

AXEventGenerator::Iterator::Iterator(const AXEventGenerator::Iterator& other) =
    default;

AXEventGenerator::Iterator::~Iterator() = default;

bool AXEventGenerator::Iterator::operator!=(
    const AXEventGenerator::Iterator& rhs) const {
  return map_iter_ != rhs.map_iter_ ||
         (map_iter_ != map_.end() && set_iter_ != rhs.set_iter_);
}

AXEventGenerator::Iterator& AXEventGenerator::Iterator::operator++() {
  if (map_iter_ == map_.end())
    return *this;

  set_iter_++;
  while (map_iter_ != map_.end() && set_iter_ == map_iter_->second.end()) {
    map_iter_++;
    if (map_iter_ != map_.end())
      set_iter_ = map_iter_->second.begin();
  }

  return *this;
}

AXEventGenerator::TargetedEvent AXEventGenerator::Iterator::operator*() const {
  BASE_DCHECK(map_iter_ != map_.end() && set_iter_ != map_iter_->second.end());
  return AXEventGenerator::TargetedEvent(map_iter_->first, *set_iter_);
}

AXEventGenerator::AXEventGenerator() = default;

AXEventGenerator::AXEventGenerator(AXTree* tree) : tree_(tree) {
  if (tree_)
    tree_->AddObserver(this);
}

AXEventGenerator::~AXEventGenerator() = default;

void AXEventGenerator::SetTree(AXTree* new_tree) {
  if (tree_)
    tree_->RemoveObserver(this);
  tree_ = new_tree;
  if (tree_)
    tree_->AddObserver(this);
}

void AXEventGenerator::ReleaseTree() {
  tree_->RemoveObserver(this);
  tree_ = nullptr;
}

void AXEventGenerator::ClearEvents() {
  tree_events_.clear();
}

void AXEventGenerator::AddEvent(AXNode* node, AXEventGenerator::Event event) {
  BASE_DCHECK(node);

  if (node->data().role == ax::mojom::Role::kInlineTextBox)
    return;

  std::set<EventParams>& node_events = tree_events_[node];
  node_events.emplace(event, ax::mojom::EventFrom::kNone,
                      tree_->event_intents());
}

void AXEventGenerator::OnNodeDataChanged(AXTree* tree,
                                         const AXNodeData& old_node_data,
                                         const AXNodeData& new_node_data) {
  BASE_DCHECK(tree_ == tree);
  // Fire CHILDREN_CHANGED events when the list of children updates.
  // Internally we store inline text box nodes as children of a static text
  // node or a line break node, which enables us to determine character bounds
  // and line layout. We don't expose those to platform APIs, though, so
  // suppress CHILDREN_CHANGED events on static text nodes.
  if (new_node_data.child_ids != old_node_data.child_ids &&
      !ui::IsText(new_node_data.role)) {
    AXNode* node = tree_->GetFromId(new_node_data.id);
    tree_events_[node].emplace(Event::CHILDREN_CHANGED,
                               ax::mojom::EventFrom::kNone,
                               tree_->event_intents());
  }
}

void AXEventGenerator::OnRoleChanged(AXTree* tree,
                                     AXNode* node,
                                     ax::mojom::Role old_role,
                                     ax::mojom::Role new_role) {
  BASE_DCHECK(tree_ == tree);
  AddEvent(node, Event::ROLE_CHANGED);
}

void AXEventGenerator::OnStateChanged(AXTree* tree,
                                      AXNode* node,
                                      ax::mojom::State state,
                                      bool new_value) {
  BASE_DCHECK(tree_ == tree);

  if (state != ax::mojom::State::kIgnored) {
    AddEvent(node, Event::STATE_CHANGED);
    AddEvent(node, Event::WIN_IACCESSIBLE_STATE_CHANGED);
  }

  switch (state) {
    case ax::mojom::State::kExpanded:
      AddEvent(node, new_value ? Event::EXPANDED : Event::COLLAPSED);

      if (node->data().role == ax::mojom::Role::kRow ||
          node->data().role == ax::mojom::Role::kTreeItem) {
        AXNode* container = node;
        while (container && !IsRowContainer(container->data().role))
          container = container->parent();
        if (container)
          AddEvent(container, Event::ROW_COUNT_CHANGED);
      }
      break;
    case ax::mojom::State::kIgnored: {
      AXNode* unignored_parent = node->GetUnignoredParent();
      if (unignored_parent)
        AddEvent(unignored_parent, Event::CHILDREN_CHANGED);
      AddEvent(node, Event::IGNORED_CHANGED);
      if (!new_value)
        AddEvent(node, Event::SUBTREE_CREATED);
      break;
    }
    case ax::mojom::State::kMultiline:
      AddEvent(node, Event::MULTILINE_STATE_CHANGED);
      break;
    case ax::mojom::State::kMultiselectable:
      AddEvent(node, Event::MULTISELECTABLE_STATE_CHANGED);
      break;
    case ax::mojom::State::kRequired:
      AddEvent(node, Event::REQUIRED_STATE_CHANGED);
      break;
    default:
      break;
  }
}

void AXEventGenerator::OnStringAttributeChanged(AXTree* tree,
                                                AXNode* node,
                                                ax::mojom::StringAttribute attr,
                                                const std::string& old_value,
                                                const std::string& new_value) {
  BASE_DCHECK(tree_ == tree);

  switch (attr) {
    case ax::mojom::StringAttribute::kAccessKey:
      AddEvent(node, Event::ACCESS_KEY_CHANGED);
      break;
    case ax::mojom::StringAttribute::kAriaInvalidValue:
      AddEvent(node, Event::INVALID_STATUS_CHANGED);
      break;
    case ax::mojom::StringAttribute::kAutoComplete:
      AddEvent(node, Event::AUTO_COMPLETE_CHANGED);
      break;
    case ax::mojom::StringAttribute::kClassName:
      AddEvent(node, Event::CLASS_NAME_CHANGED);
      break;
    case ax::mojom::StringAttribute::kDescription:
      AddEvent(node, Event::DESCRIPTION_CHANGED);
      break;
    case ax::mojom::StringAttribute::kKeyShortcuts:
      AddEvent(node, Event::KEY_SHORTCUTS_CHANGED);
      break;
    case ax::mojom::StringAttribute::kLanguage:
      AddEvent(node, Event::LANGUAGE_CHANGED);
      break;
    case ax::mojom::StringAttribute::kLiveRelevant:
      AddEvent(node, Event::LIVE_RELEVANT_CHANGED);
      break;
    case ax::mojom::StringAttribute::kLiveStatus:
      AddEvent(node, Event::LIVE_STATUS_CHANGED);

      // Fire a LIVE_REGION_CREATED if the previous value was off, and the new
      // value is not-off.
      if (!IsAlert(node->data().role)) {
        bool old_state = !old_value.empty() && old_value != "off";
        bool new_state = !new_value.empty() && new_value != "off";
        if (!old_state && new_state)
          AddEvent(node, Event::LIVE_REGION_CREATED);
      }
      break;
    case ax::mojom::StringAttribute::kName:
      // If the name of the root node changes, we expect OnTreeDataChanged to
      // add a DOCUMENT_TITLE_CHANGED event instead.
      if (node != tree->root())
        AddEvent(node, Event::NAME_CHANGED);

      if (node->data().HasStringAttribute(
              ax::mojom::StringAttribute::kContainerLiveStatus)) {
        FireLiveRegionEvents(node);
      }
      break;
    case ax::mojom::StringAttribute::kPlaceholder:
      AddEvent(node, Event::PLACEHOLDER_CHANGED);
      break;
    case ax::mojom::StringAttribute::kValue:
      AddEvent(node, Event::VALUE_CHANGED);
      break;
    case ax::mojom::StringAttribute::kImageAnnotation:
      // The image annotation is reported as part of the accessible name.
      AddEvent(node, Event::IMAGE_ANNOTATION_CHANGED);
      break;
    case ax::mojom::StringAttribute::kFontFamily:
      AddEvent(node, Event::TEXT_ATTRIBUTE_CHANGED);
      break;
    default:
      AddEvent(node, Event::OTHER_ATTRIBUTE_CHANGED);
      break;
  }
}

void AXEventGenerator::OnIntAttributeChanged(AXTree* tree,
                                             AXNode* node,
                                             ax::mojom::IntAttribute attr,
                                             int32_t old_value,
                                             int32_t new_value) {
  BASE_DCHECK(tree_ == tree);

  switch (attr) {
    case ax::mojom::IntAttribute::kActivedescendantId:
      // Don't fire on invisible containers, as it confuses some screen readers,
      // such as NVDA.
      if (!node->data().HasState(ax::mojom::State::kInvisible)) {
        AddEvent(node, Event::ACTIVE_DESCENDANT_CHANGED);
        active_descendant_changed_.push_back(node);
      }
      break;
    case ax::mojom::IntAttribute::kCheckedState:
      AddEvent(node, Event::CHECKED_STATE_CHANGED);
      AddEvent(node, Event::WIN_IACCESSIBLE_STATE_CHANGED);
      break;
    case ax::mojom::IntAttribute::kDropeffect:
      AddEvent(node, Event::DROPEFFECT_CHANGED);
      break;
    case ax::mojom::IntAttribute::kHasPopup:
      AddEvent(node, Event::HASPOPUP_CHANGED);
      AddEvent(node, Event::WIN_IACCESSIBLE_STATE_CHANGED);
      break;
    case ax::mojom::IntAttribute::kHierarchicalLevel:
      AddEvent(node, Event::HIERARCHICAL_LEVEL_CHANGED);
      break;
    case ax::mojom::IntAttribute::kInvalidState:
      AddEvent(node, Event::INVALID_STATUS_CHANGED);
      break;
    case ax::mojom::IntAttribute::kPosInSet:
      AddEvent(node, Event::POSITION_IN_SET_CHANGED);
      break;
    case ax::mojom::IntAttribute::kRestriction: {
      bool was_enabled;
      bool was_readonly;
      GetRestrictionStates(static_cast<ax::mojom::Restriction>(old_value),
                           &was_enabled, &was_readonly);
      bool is_enabled;
      bool is_readonly;
      GetRestrictionStates(static_cast<ax::mojom::Restriction>(new_value),
                           &is_enabled, &is_readonly);

      if (was_enabled != is_enabled) {
        AddEvent(node, Event::ENABLED_CHANGED);
        AddEvent(node, Event::WIN_IACCESSIBLE_STATE_CHANGED);
      }
      if (was_readonly != is_readonly) {
        AddEvent(node, Event::READONLY_CHANGED);
        AddEvent(node, Event::WIN_IACCESSIBLE_STATE_CHANGED);
      }
      break;
    }
    case ax::mojom::IntAttribute::kScrollX:
      AddEvent(node, Event::SCROLL_HORIZONTAL_POSITION_CHANGED);
      break;
    case ax::mojom::IntAttribute::kScrollY:
      AddEvent(node, Event::SCROLL_VERTICAL_POSITION_CHANGED);
      break;
    case ax::mojom::IntAttribute::kSortDirection:
      // Ignore sort direction changes on roles other than table headers and
      // grid headers.
      if (IsTableHeader(node->data().role))
        AddEvent(node, Event::SORT_CHANGED);
      break;
    case ax::mojom::IntAttribute::kImageAnnotationStatus:
      // The image annotation is reported as part of the accessible name.
      AddEvent(node, Event::IMAGE_ANNOTATION_CHANGED);
      break;
    case ax::mojom::IntAttribute::kSetSize:
      AddEvent(node, Event::SET_SIZE_CHANGED);
      break;
    case ax::mojom::IntAttribute::kBackgroundColor:
    case ax::mojom::IntAttribute::kColor:
    case ax::mojom::IntAttribute::kTextDirection:
    case ax::mojom::IntAttribute::kTextPosition:
    case ax::mojom::IntAttribute::kTextStyle:
    case ax::mojom::IntAttribute::kTextOverlineStyle:
    case ax::mojom::IntAttribute::kTextStrikethroughStyle:
    case ax::mojom::IntAttribute::kTextUnderlineStyle:
      AddEvent(node, Event::TEXT_ATTRIBUTE_CHANGED);
      break;
    case ax::mojom::IntAttribute::kTextAlign:
      // Alignment is exposed as an object attribute because it cannot apply to
      // a substring. However, for some platforms (e.g. ATK), alignment is a
      // text attribute. Therefore fire both events to ensure platforms get the
      // expected notifications.
      AddEvent(node, Event::ATK_TEXT_OBJECT_ATTRIBUTE_CHANGED);
      AddEvent(node, Event::OBJECT_ATTRIBUTE_CHANGED);
      break;
    default:
      AddEvent(node, Event::OTHER_ATTRIBUTE_CHANGED);
      break;
  }
}

void AXEventGenerator::OnFloatAttributeChanged(AXTree* tree,
                                               AXNode* node,
                                               ax::mojom::FloatAttribute attr,
                                               float old_value,
                                               float new_value) {
  BASE_DCHECK(tree_ == tree);

  switch (attr) {
    case ax::mojom::FloatAttribute::kMaxValueForRange:
      AddEvent(node, Event::VALUE_MAX_CHANGED);
      break;
    case ax::mojom::FloatAttribute::kMinValueForRange:
      AddEvent(node, Event::VALUE_MIN_CHANGED);
      break;
    case ax::mojom::FloatAttribute::kStepValueForRange:
      AddEvent(node, Event::VALUE_STEP_CHANGED);
      break;
    case ax::mojom::FloatAttribute::kValueForRange:
      AddEvent(node, Event::VALUE_CHANGED);
      break;
    case ax::mojom::FloatAttribute::kFontSize:
    case ax::mojom::FloatAttribute::kFontWeight:
      AddEvent(node, Event::TEXT_ATTRIBUTE_CHANGED);
      break;
    case ax::mojom::FloatAttribute::kTextIndent:
      // Indentation is exposed as an object attribute because it cannot apply
      // to a substring. However, for some platforms (e.g. ATK), alignment is a
      // text attribute. Therefore fire both events to ensure platforms get the
      // expected notifications.
      AddEvent(node, Event::ATK_TEXT_OBJECT_ATTRIBUTE_CHANGED);
      AddEvent(node, Event::OBJECT_ATTRIBUTE_CHANGED);
      break;
    default:
      AddEvent(node, Event::OTHER_ATTRIBUTE_CHANGED);
      break;
  }
}

void AXEventGenerator::OnBoolAttributeChanged(AXTree* tree,
                                              AXNode* node,
                                              ax::mojom::BoolAttribute attr,
                                              bool new_value) {
  BASE_DCHECK(tree_ == tree);

  switch (attr) {
    case ax::mojom::BoolAttribute::kBusy:
      AddEvent(node, Event::BUSY_CHANGED);
      AddEvent(node, Event::WIN_IACCESSIBLE_STATE_CHANGED);
      // Fire an 'invalidated' event when aria-busy becomes false
      if (!new_value)
        AddEvent(node, Event::LAYOUT_INVALIDATED);
      break;
    case ax::mojom::BoolAttribute::kGrabbed:
      AddEvent(node, Event::GRABBED_CHANGED);
      break;
    case ax::mojom::BoolAttribute::kLiveAtomic:
      AddEvent(node, Event::ATOMIC_CHANGED);
      break;
    case ax::mojom::BoolAttribute::kSelected: {
      AddEvent(node, Event::SELECTED_CHANGED);
      AddEvent(node, Event::WIN_IACCESSIBLE_STATE_CHANGED);
      AXNode* container = node;
      while (container &&
             !IsContainerWithSelectableChildren(container->data().role))
        container = container->parent();
      if (container)
        AddEvent(container, Event::SELECTED_CHILDREN_CHANGED);
      break;
    }
    default:
      AddEvent(node, Event::OTHER_ATTRIBUTE_CHANGED);
      break;
  }
}

void AXEventGenerator::OnIntListAttributeChanged(
    AXTree* tree,
    AXNode* node,
    ax::mojom::IntListAttribute attr,
    const std::vector<int32_t>& old_value,
    const std::vector<int32_t>& new_value) {
  BASE_DCHECK(tree_ == tree);

  switch (attr) {
    case ax::mojom::IntListAttribute::kControlsIds:
      AddEvent(node, Event::CONTROLS_CHANGED);
      break;
    case ax::mojom::IntListAttribute::kDescribedbyIds:
      AddEvent(node, Event::DESCRIBED_BY_CHANGED);
      break;
    case ax::mojom::IntListAttribute::kFlowtoIds: {
      AddEvent(node, Event::FLOW_TO_CHANGED);

      // Fire FLOW_FROM_CHANGED for all nodes added or removed
      for (int32_t id : ComputeIntListDifference(old_value, new_value)) {
        if (auto* target_node = tree->GetFromId(id))
          AddEvent(target_node, Event::FLOW_FROM_CHANGED);
      }
      break;
    }
    case ax::mojom::IntListAttribute::kLabelledbyIds:
      AddEvent(node, Event::LABELED_BY_CHANGED);
      break;
    case ax::mojom::IntListAttribute::kMarkerEnds:
    case ax::mojom::IntListAttribute::kMarkerStarts:
    case ax::mojom::IntListAttribute::kMarkerTypes:
      // On a native text field, the spelling- and grammar-error markers are
      // associated with children not exposed on any platform. Therefore, we
      // adjust the node we fire that event on here.
      if (AXNode* text_field = node->GetTextFieldAncestor())
        AddEvent(text_field, Event::TEXT_ATTRIBUTE_CHANGED);
      else
        AddEvent(node, Event::TEXT_ATTRIBUTE_CHANGED);
      break;
    default:
      AddEvent(node, Event::OTHER_ATTRIBUTE_CHANGED);
      break;
  }
}

void AXEventGenerator::OnTreeDataChanged(AXTree* tree,
                                         const AXTreeData& old_tree_data,
                                         const AXTreeData& new_tree_data) {
  BASE_DCHECK(tree_ == tree);

  if (new_tree_data.loaded && !old_tree_data.loaded &&
      ShouldFireLoadEvents(tree->root())) {
    AddEvent(tree->root(), Event::LOAD_COMPLETE);
  }

  if (new_tree_data.sel_is_backward != old_tree_data.sel_is_backward ||
      new_tree_data.sel_anchor_object_id !=
          old_tree_data.sel_anchor_object_id ||
      new_tree_data.sel_anchor_offset != old_tree_data.sel_anchor_offset ||
      new_tree_data.sel_anchor_affinity != old_tree_data.sel_anchor_affinity ||
      new_tree_data.sel_focus_object_id != old_tree_data.sel_focus_object_id ||
      new_tree_data.sel_focus_offset != old_tree_data.sel_focus_offset ||
      new_tree_data.sel_focus_affinity != old_tree_data.sel_focus_affinity) {
    AddEvent(tree->root(), Event::DOCUMENT_SELECTION_CHANGED);
  }
  if (new_tree_data.title != old_tree_data.title)
    AddEvent(tree->root(), Event::DOCUMENT_TITLE_CHANGED);
  if (new_tree_data.focus_id != old_tree_data.focus_id) {
    AXNode* focus_node = tree->GetFromId(new_tree_data.focus_id);
    if (focus_node) {
      AddEvent(focus_node, Event::FOCUS_CHANGED);
    }
  }
}

void AXEventGenerator::OnNodeWillBeDeleted(AXTree* tree, AXNode* node) {
  BASE_DCHECK(tree_ == tree);
  tree_events_.erase(node);
}

void AXEventGenerator::OnSubtreeWillBeDeleted(AXTree* tree, AXNode* node) {
  BASE_DCHECK(tree_ == tree);
}

void AXEventGenerator::OnNodeWillBeReparented(AXTree* tree, AXNode* node) {
  BASE_DCHECK(tree_ == tree);
  tree_events_.erase(node);
}

void AXEventGenerator::OnSubtreeWillBeReparented(AXTree* tree, AXNode* node) {
  BASE_DCHECK(tree_ == tree);
}

void AXEventGenerator::OnAtomicUpdateFinished(
    AXTree* tree,
    bool root_changed,
    const std::vector<Change>& changes) {
  BASE_DCHECK(tree_ == tree);

  if (root_changed && ShouldFireLoadEvents(tree->root())) {
    if (tree->data().loaded)
      AddEvent(tree->root(), Event::LOAD_COMPLETE);
    else
      AddEvent(tree->root(), Event::LOAD_START);
  }

  for (const auto& change : changes) {
    if (change.type == SUBTREE_CREATED) {
      AddEvent(change.node, Event::SUBTREE_CREATED);
    } else if (change.type != NODE_CREATED) {
      FireRelationSourceEvents(tree, change.node);
      continue;
    }

    if (IsAlert(change.node->data().role))
      AddEvent(change.node, Event::ALERT);
    else if (IsActiveLiveRegion(change))
      AddEvent(change.node, Event::LIVE_REGION_CREATED);
    else if (IsContainedInLiveRegion(change))
      FireLiveRegionEvents(change.node);
  }

  FireActiveDescendantEvents();

  PostprocessEvents();
}

void AXEventGenerator::FireLiveRegionEvents(AXNode* node) {
  AXNode* live_root = node;
  while (live_root && !live_root->data().HasStringAttribute(
                          ax::mojom::StringAttribute::kLiveStatus))
    live_root = live_root->parent();

  if (live_root &&
      !live_root->data().GetBoolAttribute(ax::mojom::BoolAttribute::kBusy) &&
      live_root->data().GetStringAttribute(
          ax::mojom::StringAttribute::kLiveStatus) != "off") {
    // Fire LIVE_REGION_NODE_CHANGED on each node that changed.
    if (!node->data()
             .GetStringAttribute(ax::mojom::StringAttribute::kName)
             .empty())
      AddEvent(node, Event::LIVE_REGION_NODE_CHANGED);
    // Fire LIVE_REGION_NODE_CHANGED on the root of the live region.
    AddEvent(live_root, Event::LIVE_REGION_CHANGED);
  }
}

void AXEventGenerator::FireActiveDescendantEvents() {
  for (AXNode* node : active_descendant_changed_) {
    AXNode* descendant = tree_->GetFromId(node->data().GetIntAttribute(
        ax::mojom::IntAttribute::kActivedescendantId));
    if (!descendant)
      continue;
    switch (descendant->data().role) {
      case ax::mojom::Role::kMenuItem:
      case ax::mojom::Role::kMenuItemCheckBox:
      case ax::mojom::Role::kMenuItemRadio:
      case ax::mojom::Role::kMenuListOption:
        AddEvent(descendant, Event::MENU_ITEM_SELECTED);
        break;
      default:
        break;
    }
  }
  active_descendant_changed_.clear();
}

void AXEventGenerator::FireRelationSourceEvents(AXTree* tree,
                                                AXNode* target_node) {
  int32_t target_id = target_node->id();
  std::set<AXNode*> source_nodes;
  auto callback = [&](const auto& entry) {
    const auto& target_to_sources = entry.second;
    auto sources_it = target_to_sources.find(target_id);
    if (sources_it == target_to_sources.end())
      return;

    auto sources = sources_it->second;
    std::for_each(sources.begin(), sources.end(), [&](int32_t source_id) {
      AXNode* source_node = tree->GetFromId(source_id);

      if (!source_node || source_nodes.count(source_node) > 0)
        return;

      source_nodes.insert(source_node);

      // GCC < 6.4 requires this pointer when calling a member
      // function in anonymous function
      this->AddEvent(source_node, Event::RELATED_NODE_CHANGED);
    });
  };

  std::for_each(tree->int_reverse_relations().begin(),
                tree->int_reverse_relations().end(), callback);
  std::for_each(
      tree->intlist_reverse_relations().begin(),
      tree->intlist_reverse_relations().end(), [&](auto& entry) {
        // Explicitly exclude relationships for which an additional event on the
        // source node would cause extra noise. For example, kRadioGroupIds
        // forms relations among all radio buttons and serves little value for
        // AT to get events on the previous radio button in the group.
        if (entry.first != ax::mojom::IntListAttribute::kRadioGroupIds)
          callback(entry);
      });
}

// Attempts to suppress load-related events that we presume no AT will be
// interested in under any circumstances, such as pages which have no size.
bool AXEventGenerator::ShouldFireLoadEvents(AXNode* node) {
  if (always_fire_load_complete_)
    return true;

  const AXNodeData& data = node->data();
  return data.relative_bounds.bounds.width() ||
         data.relative_bounds.bounds.height();
}

void AXEventGenerator::TrimEventsDueToAncestorIgnoredChanged(
    AXNode* node,
    std::map<AXNode*, IgnoredChangedStatesBitset>&
        ancestor_ignored_changed_map) {
  BASE_DCHECK(node);

  // Recursively compute and cache ancestor ignored changed results in
  // |ancestor_ignored_changed_map|, if |node|'s ancestors have become ignored
  // and the ancestor's ignored changed results have not been cached.
  if (node->parent() &&
      !base::Contains(ancestor_ignored_changed_map, node->parent())) {
    TrimEventsDueToAncestorIgnoredChanged(node->parent(),
                                          ancestor_ignored_changed_map);
  }

  // If an ancestor of |node| changed to ignored state (hide), append hide state
  // to the corresponding entry in the map for |node|. Similarly, if an ancestor
  // of |node| removed its ignored state (show), we append show state to the
  // corresponding entry in map for |node| as well. If |node| flipped its
  // ignored state as well, we want to remove various events related to
  // IGNORED_CHANGED event.
  const auto& parent_map_iter =
      ancestor_ignored_changed_map.find(node->parent());
  const auto& curr_events_iter = tree_events_.find(node);

  // Initialize |ancestor_ignored_changed_map[node]| with an empty bitset,
  // representing neither |node| nor its ancestor has IGNORED_CHANGED.
  IgnoredChangedStatesBitset& ancestor_ignored_changed_states =
      ancestor_ignored_changed_map[node];

  // If |ancestor_ignored_changed_map| contains an entry for |node|'s
  // ancestor's and the ancestor has either show/hide state, we want to populate
  // |node|'s show/hide state in the map based on its cached ancestor result.
  // An empty entry in |ancestor_ignored_changed_map| for |node| means that
  // neither |node| nor its ancestor has IGNORED_CHANGED.
  if (parent_map_iter != ancestor_ignored_changed_map.end()) {
    // Propagate ancestor's show/hide states to |node|'s entry in the map.
    if (HasIgnoredChangedState(parent_map_iter->second,
                               IgnoredChangedState::kHide)) {
      AddIgnoredChangedState(ancestor_ignored_changed_states,
                             IgnoredChangedState::kHide);
    }
    if (HasIgnoredChangedState(parent_map_iter->second,
                               IgnoredChangedState::kShow)) {
      AddIgnoredChangedState(ancestor_ignored_changed_states,
                             IgnoredChangedState::kShow);
    }

    // If |node| has IGNORED changed with show/hide state that matches one of
    // its ancestors' IGNORED changed show/hide states, we want to remove
    // |node|'s IGNORED_CHANGED related events.
    if (curr_events_iter != tree_events_.end() &&
        HasEvent(curr_events_iter->second, Event::IGNORED_CHANGED)) {
      if ((HasIgnoredChangedState(parent_map_iter->second,
                                  IgnoredChangedState::kHide) &&
           node->IsIgnored()) ||
          (HasIgnoredChangedState(parent_map_iter->second,
                                  IgnoredChangedState::kShow) &&
           !node->IsIgnored())) {
        RemoveEvent(&(curr_events_iter->second), Event::IGNORED_CHANGED);
        RemoveEventsDueToIgnoredChanged(&(curr_events_iter->second));
      }

      if (node->IsIgnored()) {
        AddIgnoredChangedState(ancestor_ignored_changed_states,
                               IgnoredChangedState::kHide);
      } else {
        AddIgnoredChangedState(ancestor_ignored_changed_states,
                               IgnoredChangedState::kShow);
      }
    }

    return;
  }

  // If ignored changed results for ancestors are not cached, calculate the
  // corresponding entry for |node| in the map using the ignored states and
  // events of |node|.
  if (curr_events_iter != tree_events_.end() &&
      HasEvent(curr_events_iter->second, Event::IGNORED_CHANGED)) {
    if (node->IsIgnored()) {
      AddIgnoredChangedState(ancestor_ignored_changed_states,
                             IgnoredChangedState::kHide);
    } else {
      AddIgnoredChangedState(ancestor_ignored_changed_states,
                             IgnoredChangedState::kShow);
    }

    return;
  }
}

void AXEventGenerator::PostprocessEvents() {
  std::map<AXNode*, IgnoredChangedStatesBitset> ancestor_ignored_changed_map;
  std::set<AXNode*> removed_subtree_created_nodes;
  auto iter = tree_events_.begin();
  while (iter != tree_events_.end()) {
    AXNode* node = iter->first;
    std::set<EventParams>& node_events = iter->second;

    // A newly created live region or alert should not *also* fire a
    // live region changed event.
    if (HasEvent(node_events, Event::ALERT) ||
        HasEvent(node_events, Event::LIVE_REGION_CREATED)) {
      RemoveEvent(&node_events, Event::LIVE_REGION_CHANGED);
    }

    if (HasEvent(node_events, Event::IGNORED_CHANGED)) {
      // If a node toggled its ignored state, we only want to fire
      // IGNORED_CHANGED event on the top most ancestor where this ignored state
      // change takes place and suppress all the descendants's IGNORED_CHANGED
      // events.
      TrimEventsDueToAncestorIgnoredChanged(node, ancestor_ignored_changed_map);
      RemoveEventsDueToIgnoredChanged(&node_events);
    }

    // When the selected option in an expanded select element changes, the
    // foreground and background colors change. But we don't want to treat
    // those as text attribute changes. This can also happen when a widget
    // such as a button becomes enabled/disabled.
    if (HasEvent(node_events, Event::SELECTED_CHANGED) ||
        HasEvent(node_events, Event::ENABLED_CHANGED)) {
      RemoveEvent(&node_events, Event::TEXT_ATTRIBUTE_CHANGED);
    }

    AXNode* parent = node->GetUnignoredParent();

    // Don't fire text attribute changed on this node if its immediate parent
    // also has text attribute changed.
    if (parent && HasEvent(node_events, Event::TEXT_ATTRIBUTE_CHANGED) &&
        tree_events_.find(parent) != tree_events_.end() &&
        HasEvent(tree_events_[parent], Event::TEXT_ATTRIBUTE_CHANGED)) {
      RemoveEvent(&node_events, Event::TEXT_ATTRIBUTE_CHANGED);
    }

    // Don't fire subtree created on this node if any of its ancestors also has
    // subtree created.
    if (HasEvent(node_events, Event::SUBTREE_CREATED)) {
      while (parent &&
             (tree_events_.find(parent) != tree_events_.end() ||
              base::Contains(removed_subtree_created_nodes, parent))) {
        if (base::Contains(removed_subtree_created_nodes, parent) ||
            HasEvent(tree_events_[parent], Event::SUBTREE_CREATED)) {
          RemoveEvent(&node_events, Event::SUBTREE_CREATED);
          removed_subtree_created_nodes.insert(node);
          break;
        }
        parent = parent->GetUnignoredParent();
      }
    }

    // If this was the only event, remove the node entirely from the
    // tree events.
    if (node_events.empty())
      iter = tree_events_.erase(iter);
    else
      ++iter;
  }
}

// static
void AXEventGenerator::GetRestrictionStates(ax::mojom::Restriction restriction,
                                            bool* is_enabled,
                                            bool* is_readonly) {
  switch (restriction) {
    case ax::mojom::Restriction::kDisabled:
      *is_enabled = false;
      *is_readonly = true;
      break;
    case ax::mojom::Restriction::kReadOnly:
      *is_enabled = true;
      *is_readonly = true;
      break;
    case ax::mojom::Restriction::kNone:
      *is_enabled = true;
      *is_readonly = false;
      break;
  }
}

// static
std::vector<int32_t> AXEventGenerator::ComputeIntListDifference(
    const std::vector<int32_t>& lhs,
    const std::vector<int32_t>& rhs) {
  std::set<int32_t> sorted_lhs(lhs.cbegin(), lhs.cend());
  std::set<int32_t> sorted_rhs(rhs.cbegin(), rhs.cend());

  std::vector<int32_t> result;
  std::set_symmetric_difference(sorted_lhs.cbegin(), sorted_lhs.cend(),
                                sorted_rhs.cbegin(), sorted_rhs.cend(),
                                std::back_inserter(result));
  return result;
}

std::ostream& operator<<(std::ostream& os, AXEventGenerator::Event event) {
  return os << ToString(event);
}

const char* ToString(AXEventGenerator::Event event) {
  switch (event) {
    case AXEventGenerator::Event::ACCESS_KEY_CHANGED:
      return "ACCESS_KEY_CHANGED";
    case AXEventGenerator::Event::ATOMIC_CHANGED:
      return "ATOMIC_CHANGED";
    case AXEventGenerator::Event::ACTIVE_DESCENDANT_CHANGED:
      return "ACTIVE_DESCENDANT_CHANGED";
    case AXEventGenerator::Event::ALERT:
      return "ALERT";
    case AXEventGenerator::Event::ATK_TEXT_OBJECT_ATTRIBUTE_CHANGED:
      return "ATK_TEXT_OBJECT_ATTRIBUTE_CHANGED";
    case AXEventGenerator::Event::BUSY_CHANGED:
      return "BUSY_CHANGED";
    case AXEventGenerator::Event::CHECKED_STATE_CHANGED:
      return "CHECKED_STATE_CHANGED";
    case AXEventGenerator::Event::CHILDREN_CHANGED:
      return "CHILDREN_CHANGED";
    case AXEventGenerator::Event::CLASS_NAME_CHANGED:
      return "CLASS_NAME_CHANGED";
    case AXEventGenerator::Event::COLLAPSED:
      return "COLLAPSED";
    case AXEventGenerator::Event::CONTROLS_CHANGED:
      return "CONTROLS_CHANGED";
    case AXEventGenerator::Event::DESCRIBED_BY_CHANGED:
      return "DESCRIBED_BY_CHANGED";
    case AXEventGenerator::Event::DESCRIPTION_CHANGED:
      return "DESCRIPTION_CHANGED";
    case AXEventGenerator::Event::DOCUMENT_SELECTION_CHANGED:
      return "DOCUMENT_SELECTION_CHANGED";
    case AXEventGenerator::Event::DOCUMENT_TITLE_CHANGED:
      return "DOCUMENT_TITLE_CHANGED";
    case AXEventGenerator::Event::DROPEFFECT_CHANGED:
      return "DROPEFFECT_CHANGED";
    case AXEventGenerator::Event::ENABLED_CHANGED:
      return "ENABLED_CHANGED";
    case AXEventGenerator::Event::EXPANDED:
      return "EXPANDED";
    case AXEventGenerator::Event::FLOW_FROM_CHANGED:
      return "FLOW_FROM_CHANGED";
    case AXEventGenerator::Event::FLOW_TO_CHANGED:
      return "FLOW_TO_CHANGED";
    case AXEventGenerator::Event::GRABBED_CHANGED:
      return "GRABBED_CHANGED";
    case AXEventGenerator::Event::HASPOPUP_CHANGED:
      return "HASPOPUP_CHANGED";
    case AXEventGenerator::Event::HIERARCHICAL_LEVEL_CHANGED:
      return "HIERARCHICAL_LEVEL_CHANGED";
    case ui::AXEventGenerator::Event::IGNORED_CHANGED:
      return "IGNORED_CHANGED";
    case AXEventGenerator::Event::IMAGE_ANNOTATION_CHANGED:
      return "IMAGE_ANNOTATION_CHANGED";
    case AXEventGenerator::Event::INVALID_STATUS_CHANGED:
      return "INVALID_STATUS_CHANGED";
    case AXEventGenerator::Event::KEY_SHORTCUTS_CHANGED:
      return "KEY_SHORTCUTS_CHANGED";
    case AXEventGenerator::Event::LABELED_BY_CHANGED:
      return "LABELED_BY_CHANGED";
    case AXEventGenerator::Event::LANGUAGE_CHANGED:
      return "LANGUAGE_CHANGED";
    case AXEventGenerator::Event::LAYOUT_INVALIDATED:
      return "LAYOUT_INVALIDATED";
    case AXEventGenerator::Event::LIVE_REGION_CHANGED:
      return "LIVE_REGION_CHANGED";
    case AXEventGenerator::Event::LIVE_REGION_CREATED:
      return "LIVE_REGION_CREATED";
    case AXEventGenerator::Event::LIVE_REGION_NODE_CHANGED:
      return "LIVE_REGION_NODE_CHANGED";
    case AXEventGenerator::Event::LIVE_RELEVANT_CHANGED:
      return "LIVE_RELEVANT_CHANGED";
    case AXEventGenerator::Event::LIVE_STATUS_CHANGED:
      return "LIVE_STATUS_CHANGED";
    case AXEventGenerator::Event::LOAD_COMPLETE:
      return "LOAD_COMPLETE";
    case AXEventGenerator::Event::LOAD_START:
      return "LOAD_START";
    case AXEventGenerator::Event::MENU_ITEM_SELECTED:
      return "MENU_ITEM_SELECTED";
    case AXEventGenerator::Event::MULTILINE_STATE_CHANGED:
      return "MULTILINE_STATE_CHANGED";
    case AXEventGenerator::Event::MULTISELECTABLE_STATE_CHANGED:
      return "MULTISELECTABLE_STATE_CHANGED";
    case AXEventGenerator::Event::NAME_CHANGED:
      return "NAME_CHANGED";
    case AXEventGenerator::Event::OBJECT_ATTRIBUTE_CHANGED:
      return "OBJECT_ATTRIBUTE_CHANGED";
    case AXEventGenerator::Event::OTHER_ATTRIBUTE_CHANGED:
      return "OTHER_ATTRIBUTE_CHANGED";
    case AXEventGenerator::Event::PLACEHOLDER_CHANGED:
      return "PLACEHOLDER_CHANGED";
    case AXEventGenerator::Event::PORTAL_ACTIVATED:
      return "PORTAL_ACTIVATED";
    case AXEventGenerator::Event::POSITION_IN_SET_CHANGED:
      return "POSITION_IN_SET_CHANGED";
    case AXEventGenerator::Event::READONLY_CHANGED:
      return "READONLY_CHANGED";
    case AXEventGenerator::Event::RELATED_NODE_CHANGED:
      return "RELATED_NODE_CHANGED";
    case AXEventGenerator::Event::REQUIRED_STATE_CHANGED:
      return "REQUIRED_STATE_CHANGED";
    case AXEventGenerator::Event::ROLE_CHANGED:
      return "ROLE_CHANGED";
    case AXEventGenerator::Event::ROW_COUNT_CHANGED:
      return "ROW_COUNT_CHANGED";
    case AXEventGenerator::Event::SCROLL_HORIZONTAL_POSITION_CHANGED:
      return "SCROLL_HORIZONTAL_POSITION_CHANGED";
    case AXEventGenerator::Event::SCROLL_VERTICAL_POSITION_CHANGED:
      return "SCROLL_VERTICAL_POSITION_CHANGED";
    case AXEventGenerator::Event::SELECTED_CHANGED:
      return "SELECTED_CHANGED";
    case AXEventGenerator::Event::SELECTED_CHILDREN_CHANGED:
      return "SELECTED_CHILDREN_CHANGED";
    case AXEventGenerator::Event::SET_SIZE_CHANGED:
      return "SET_SIZE_CHANGED";
    case AXEventGenerator::Event::STATE_CHANGED:
      return "STATE_CHANGED";
    case AXEventGenerator::Event::SUBTREE_CREATED:
      return "SUBTREE_CREATED";
    case AXEventGenerator::Event::TEXT_ATTRIBUTE_CHANGED:
      return "TEXT_ATTRIBUTE_CHANGED";
    case AXEventGenerator::Event::VALUE_CHANGED:
      return "VALUE_CHANGED";
    case AXEventGenerator::Event::VALUE_MAX_CHANGED:
      return "VALUE_MAX_CHANGED";
    case AXEventGenerator::Event::VALUE_MIN_CHANGED:
      return "VALUE_MIN_CHANGED";
    case AXEventGenerator::Event::VALUE_STEP_CHANGED:
      return "VALUE_STEP_CHANGED";
    case AXEventGenerator::Event::AUTO_COMPLETE_CHANGED:
      return "AUTO_COMPLETE_CHANGED";
    case AXEventGenerator::Event::FOCUS_CHANGED:
      return "FOCUS_CHANGED";
    case AXEventGenerator::Event::SORT_CHANGED:
      return "SORT_CHANGED";
    case AXEventGenerator::Event::WIN_IACCESSIBLE_STATE_CHANGED:
      return "WIN_IACCESSIBLE_STATE_CHANGED";
  }
  BASE_UNREACHABLE();
}

}  // namespace ui
