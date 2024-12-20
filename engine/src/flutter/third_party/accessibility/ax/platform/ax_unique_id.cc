// Copyright 2017 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "ax_unique_id.h"

#include <memory>
#include <unordered_set>

#include "base/container_utils.h"
#include "base/logging.h"

namespace ui {

namespace {

std::unordered_set<int32_t> g_assigned_ids;

}  // namespace

AXUniqueId::AXUniqueId() : AXUniqueId(INT32_MAX) {}

AXUniqueId::AXUniqueId(const int32_t max_id) : id_(GetNextAXUniqueId(max_id)) {}

AXUniqueId::~AXUniqueId() {
  g_assigned_ids.erase(id_);
}

bool AXUniqueId::operator==(const AXUniqueId& other) const {
  return Get() == other.Get();
}

bool AXUniqueId::operator!=(const AXUniqueId& other) const {
  return !(*this == other);
}

bool AXUniqueId::IsAssigned(const int32_t id) const {
  return base::Contains(g_assigned_ids, id);
}

int32_t AXUniqueId::GetNextAXUniqueId(const int32_t max_id) {
  static int32_t current_id = 0;
  static bool has_wrapped = false;

  const int32_t prev_id = current_id;
  do {
    if (current_id >= max_id) {
      current_id = 1;
      has_wrapped = true;
    } else {
      ++current_id;
    }
    if (current_id == prev_id) {
      BASE_LOG() << "There are over 2 billion available IDs, so the newly "
                    "created ID cannot be equal to the most recently created "
                    "ID.";
    }
    // If it |has_wrapped| then we need to continue until we find the first
    // unassigned ID.
  } while (has_wrapped && IsAssigned(current_id));

  g_assigned_ids.insert(current_id);
  return current_id;
}

}  // namespace ui
