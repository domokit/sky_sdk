// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/display_list/effects/dl_color_filter.h"

#include "flutter/display_list/dl_color.h"
#include "flutter/display_list/effects/color_filters/dl_blend_color_filter.h"
#include "flutter/display_list/effects/color_filters/dl_linear_to_srgb_gamma_color_filter.h"
#include "flutter/display_list/effects/color_filters/dl_matrix_color_filter.h"
#include "flutter/display_list/effects/color_filters/dl_srgb_to_linear_gamma_color_filter.h"

namespace flutter {

std::shared_ptr<const DlColorFilter> DlColorFilter::MakeBlend(
    DlColor color,
    DlBlendMode mode) {
  // Delegate to a method private to DlBlendColorFilter due to private
  // constructor preventing |make_shared| from here.
  return DlBlendColorFilter::Make(color, mode);
}

std::shared_ptr<const DlColorFilter> DlColorFilter::MakeMatrix(
    const float matrix[20]) {
  // Delegate to a method private to DlBlendColorFilter due to private
  // constructor preventing |make_shared| from here.
  return DlMatrixColorFilter::Make(matrix);
}

std::shared_ptr<const DlColorFilter> DlColorFilter::MakeSrgbToLinearGamma() {
  return DlSrgbToLinearGammaColorFilter::kInstance;
}

std::shared_ptr<const DlColorFilter> DlColorFilter::MakeLinearToSrgbGamma() {
  return DlLinearToSrgbGammaColorFilter::kInstance;
}

}  // namespace flutter
