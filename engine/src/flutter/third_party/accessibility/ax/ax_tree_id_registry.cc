// Copyright 2014 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "ax_tree_id_registry.h"

#include "ax_action_handler_base.h"
#include "base/logging.h"

namespace ui {

// static
AXTreeIDRegistry& AXTreeIDRegistry::GetInstance() {
  static AXTreeIDRegistry INSTANCE;
  return INSTANCE;
}

void AXTreeIDRegistry::SetFrameIDForAXTreeID(const FrameID& frame_id,
                                             const AXTreeID& ax_tree_id) {
  auto it = frame_to_ax_tree_id_map_.find(frame_id);
  if (it != frame_to_ax_tree_id_map_.end()) {
    BASE_UNREACHABLE();
    return;
  }

  frame_to_ax_tree_id_map_[frame_id] = ax_tree_id;
  ax_tree_to_frame_id_map_[ax_tree_id] = frame_id;
}

AXTreeIDRegistry::FrameID AXTreeIDRegistry::GetFrameID(
    const AXTreeID& ax_tree_id) {
  auto it = ax_tree_to_frame_id_map_.find(ax_tree_id);
  if (it != ax_tree_to_frame_id_map_.end())
    return it->second;

  return FrameID(-1, -1);
}

AXTreeID AXTreeIDRegistry::GetAXTreeID(AXTreeIDRegistry::FrameID frame_id) {
  auto it = frame_to_ax_tree_id_map_.find(frame_id);
  if (it != frame_to_ax_tree_id_map_.end())
    return it->second;

  return ui::AXTreeIDUnknown();
}

AXTreeID AXTreeIDRegistry::GetOrCreateAXTreeID(AXActionHandlerBase* handler) {
  for (auto it : id_to_action_handler_) {
    if (it.second == handler)
      return it.first;
  }
  AXTreeID new_id = AXTreeID::CreateNewAXTreeID();
  SetAXTreeID(new_id, handler);
  return new_id;
}

AXActionHandlerBase* AXTreeIDRegistry::GetActionHandler(AXTreeID ax_tree_id) {
  auto it = id_to_action_handler_.find(ax_tree_id);
  if (it == id_to_action_handler_.end())
    return nullptr;
  return it->second;
}

void AXTreeIDRegistry::SetAXTreeID(const ui::AXTreeID& id,
                                   AXActionHandlerBase* action_handler) {
  BASE_DCHECK(id_to_action_handler_.find(id) == id_to_action_handler_.end());
  id_to_action_handler_[id] = action_handler;
}

void AXTreeIDRegistry::RemoveAXTreeID(AXTreeID ax_tree_id) {
  auto frame_it = ax_tree_to_frame_id_map_.find(ax_tree_id);
  if (frame_it != ax_tree_to_frame_id_map_.end()) {
    frame_to_ax_tree_id_map_.erase(frame_it->second);
    ax_tree_to_frame_id_map_.erase(frame_it);
  }

  auto action_it = id_to_action_handler_.find(ax_tree_id);
  if (action_it != id_to_action_handler_.end())
    id_to_action_handler_.erase(action_it);
}

AXTreeIDRegistry::AXTreeIDRegistry() {}

AXTreeIDRegistry::~AXTreeIDRegistry() {}

}  // namespace ui
