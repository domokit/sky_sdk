// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/display_list/effects/image_filters/dl_blur_image_filter.h"

namespace flutter {

std::shared_ptr<DlImageFilter> DlBlurImageFilter::Make(DlScalar sigma_x,
                                                       DlScalar sigma_y,
                                                       DlTileMode tile_mode) {
  if (!std::isfinite(sigma_x) || !std::isfinite(sigma_y)) {
    return nullptr;
  }
  if (sigma_x < SK_ScalarNearlyZero && sigma_y < SK_ScalarNearlyZero) {
    return nullptr;
  }
  sigma_x = (sigma_x < SK_ScalarNearlyZero) ? 0 : sigma_x;
  sigma_y = (sigma_y < SK_ScalarNearlyZero) ? 0 : sigma_y;
  return std::make_shared<DlBlurImageFilter>(sigma_x, sigma_y, tile_mode);
}

DlRect* DlBlurImageFilter::map_local_bounds(const DlRect& input_bounds,
                                            DlRect& output_bounds) const {
  output_bounds = input_bounds.Expand(sigma_x_ * 3.0f, sigma_y_ * 3.0f);
  return &output_bounds;
}

DlIRect* DlBlurImageFilter::map_device_bounds(const DlIRect& input_bounds,
                                              const DlMatrix& ctm,
                                              DlIRect& output_bounds) const {
  return outset_device_bounds(input_bounds, sigma_x_ * 3.0f, sigma_y_ * 3.0f,
                              ctm, output_bounds);
}

DlIRect* DlBlurImageFilter::get_input_device_bounds(
    const DlIRect& output_bounds,
    const DlMatrix& ctm,
    DlIRect& input_bounds) const {
  // Blurs are symmetric in terms of output-for-input and input-for-output
  return map_device_bounds(output_bounds, ctm, input_bounds);
}

bool DlBlurImageFilter::equals_(const DlImageFilter& other) const {
  FML_DCHECK(other.type() == DlImageFilterType::kBlur);
  auto that = static_cast<const DlBlurImageFilter*>(&other);
  return (DlScalarNearlyEqual(sigma_x_, that->sigma_x_) &&
          DlScalarNearlyEqual(sigma_y_, that->sigma_y_) &&
          tile_mode_ == that->tile_mode_);
}

}  // namespace flutter
