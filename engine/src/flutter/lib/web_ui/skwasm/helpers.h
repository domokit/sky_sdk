// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_WEB_UI_SKWASM_HELPERS_H_
#define FLUTTER_LIB_WEB_UI_SKWASM_HELPERS_H_

#include "third_party/skia/include/core/SkMatrix.h"
#include "third_party/skia/include/core/SkRRect.h"
#include "third_party/skia/include/core/SkSamplingOptions.h"

namespace Skwasm {

inline SkMatrix createMatrix(const SkScalar* f) {
  return SkMatrix::MakeAll(f[0], f[1], f[2], f[3], f[4], f[5], f[6], f[7],
                           f[8]);
}

inline SkRRect createRRect(const SkScalar* f) {
  const SkRect* rect = reinterpret_cast<const SkRect*>(f);
  const SkVector* radiiValues = reinterpret_cast<const SkVector*>(f + 4);

  SkRRect rr;
  rr.setRectRadii(*rect, radiiValues);
  return rr;
}

// This needs to be kept in sync with the "FilterQuality" enum in dart:ui
enum class FilterQuality {
  none,
  low,
  medium,
  high,
};

inline SkFilterMode filterModeForQuality(FilterQuality quality) {
  switch (quality) {
    case FilterQuality::none:
    case FilterQuality::low:
      return SkFilterMode::kNearest;
    case FilterQuality::medium:
    case FilterQuality::high:
      return SkFilterMode::kLinear;
  }
}

inline SkSamplingOptions samplingOptionsForQuality(FilterQuality quality) {
  switch (quality) {
    case FilterQuality::none:
      return SkSamplingOptions(SkFilterMode::kNearest, SkMipmapMode::kNone);
    case FilterQuality::low:
      return SkSamplingOptions(SkFilterMode::kLinear, SkMipmapMode::kNone);
    case FilterQuality::medium:
      return SkSamplingOptions(SkFilterMode::kLinear, SkMipmapMode::kLinear);
    case FilterQuality::high:
      // Cubic equation coefficients recommended by Mitchell & Netravali
      // in their paper on cubic interpolation.
      return SkSamplingOptions(SkCubicResampler::Mitchell());
  }
}
}  // namespace Skwasm

#endif  // FLUTTER_LIB_WEB_UI_SKWASM_HELPERS_H_
