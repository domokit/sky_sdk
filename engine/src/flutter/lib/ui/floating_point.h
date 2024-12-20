// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_UI_FLOATING_POINT_H_
#define FLUTTER_LIB_UI_FLOATING_POINT_H_

#include <algorithm>
#include <limits>

namespace flutter {

/// Converts a double to a float, truncating finite values that are larger than
/// FLT_MAX or smaller than FLT_MIN to those values.
static float SafeNarrow(double value) {
  if (std::isinf(value) || std::isnan(value)) {
    return static_cast<float>(value);
  } else {
    // Avoid truncation to inf/-inf.
    return std::clamp(static_cast<float>(value),
                      std::numeric_limits<float>::lowest(),
                      std::numeric_limits<float>::max());
  }
}

}  // namespace flutter

#endif  // FLUTTER_LIB_UI_FLOATING_POINT_H_
