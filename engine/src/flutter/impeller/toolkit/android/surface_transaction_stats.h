// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_TOOLKIT_ANDROID_SURFACE_TRANSACTION_STATS_H_
#define FLUTTER_IMPELLER_TOOLKIT_ANDROID_SURFACE_TRANSACTION_STATS_H_

#include "flutter/fml/unique_fd.h"
#include "impeller/toolkit/android/surface_control.h"

namespace impeller::android {

fml::UniqueFD CreatePreviousReleaseFence(const SurfaceControl& control,
                                         ASurfaceTransactionStats* stats);

}  // namespace impeller::android

#endif  // FLUTTER_IMPELLER_TOOLKIT_ANDROID_SURFACE_TRANSACTION_STATS_H_
