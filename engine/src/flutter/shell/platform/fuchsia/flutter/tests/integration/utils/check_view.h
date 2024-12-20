// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_TESTS_INTEGRATION_UTILS_CHECK_VIEW_H_
#define FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_TESTS_INTEGRATION_UTILS_CHECK_VIEW_H_

#include <vector>

#include <fuchsia/ui/observation/geometry/cpp/fidl.h>
#include <zircon/status.h>

namespace fuchsia_test_utils {

/// Returns true if a view with the given |view_ref_koid| exists in a |snapshot|
/// of the view tree, false otherwise.
bool CheckViewExistsInSnapshot(
    const fuchsia::ui::observation::geometry::ViewTreeSnapshot& snapshot,
    zx_koid_t view_ref_koid);

/// Returns true if any of the snapshots of the view tree in |updates| contain a
/// view with the given |view_ref_koid|, false otherwise.
bool CheckViewExistsInUpdates(
    const std::vector<fuchsia::ui::observation::geometry::ViewTreeSnapshot>&
        updates,
    zx_koid_t view_ref_koid);

}  // namespace fuchsia_test_utils

#endif  // FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_TESTS_INTEGRATION_UTILS_CHECK_VIEW_H_
