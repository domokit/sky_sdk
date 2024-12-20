// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "check_view.h"

#include "flutter/fml/logging.h"

namespace fuchsia_test_utils {

bool CheckViewExistsInSnapshot(
    const fuchsia::ui::observation::geometry::ViewTreeSnapshot& snapshot,
    zx_koid_t view_ref_koid) {
  if (!snapshot.has_views()) {
    return false;
  }

  auto snapshot_count =
      std::count_if(snapshot.views().begin(), snapshot.views().end(),
                    [view_ref_koid](const auto& view) {
                      return view.view_ref_koid() == view_ref_koid;
                    });

  return snapshot_count > 0;
}

bool CheckViewExistsInUpdates(
    const std::vector<fuchsia::ui::observation::geometry::ViewTreeSnapshot>&
        updates,
    zx_koid_t view_ref_koid) {
  auto update_count = std::count_if(
      updates.begin(), updates.end(), [view_ref_koid](auto& snapshot) {
        return CheckViewExistsInSnapshot(snapshot, view_ref_koid);
      });

  return update_count > 0;
}

}  // namespace fuchsia_test_utils
