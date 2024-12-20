// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/common/snapshot_controller.h"

#include "flutter/shell/common/snapshot_controller_skia.h"

#if IMPELLER_SUPPORTS_RENDERING
#include "flutter/shell/common/snapshot_controller_impeller.h"
#endif  // IMPELLER_SUPPORTS_RENDERING

namespace flutter {

std::unique_ptr<SnapshotController> SnapshotController::Make(
    const Delegate& delegate,
    const Settings& settings) {
#if IMPELLER_SUPPORTS_RENDERING
  if (settings.enable_impeller) {
    return std::make_unique<SnapshotControllerImpeller>(delegate);
  }
#endif  // IMPELLER_SUPPORTS_RENDERING
#if !SLIMPELLER
  return std::make_unique<SnapshotControllerSkia>(delegate);
#else   //  !SLIMPELLER
  FML_LOG(FATAL)
      << "Cannot create a Skia snapshot controller in an Impeller build.";
  return nullptr;
#endif  //  !SLIMPELLER
}

SnapshotController::SnapshotController(const Delegate& delegate)
    : delegate_(delegate) {}

}  // namespace flutter
