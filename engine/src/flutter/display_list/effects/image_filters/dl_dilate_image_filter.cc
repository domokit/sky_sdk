// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/display_list/effects/image_filters/dl_dilate_image_filter.h"

namespace flutter {

std::shared_ptr<DlImageFilter> DlDilateImageFilter::Make(DlScalar radius_x,
                                                         DlScalar radius_y) {
  if (std::isfinite(radius_x) && radius_x > SK_ScalarNearlyZero &&
      std::isfinite(radius_y) && radius_y > SK_ScalarNearlyZero) {
    return std::make_shared<DlDilateImageFilter>(radius_x, radius_y);
  }
  return nullptr;
}

DlRect* DlDilateImageFilter::map_local_bounds(const DlRect& input_bounds,
                                              DlRect& output_bounds) const {
  output_bounds = input_bounds.Expand(radius_x_, radius_y_);
  return &output_bounds;
}

DlIRect* DlDilateImageFilter::map_device_bounds(const DlIRect& input_bounds,
                                                const DlMatrix& ctm,
                                                DlIRect& output_bounds) const {
  return outset_device_bounds(input_bounds, radius_x_, radius_y_, ctm,
                              output_bounds);
}

DlIRect* DlDilateImageFilter::get_input_device_bounds(
    const DlIRect& output_bounds,
    const DlMatrix& ctm,
    DlIRect& input_bounds) const {
  return inset_device_bounds(output_bounds, radius_x_, radius_y_, ctm,
                             input_bounds);
}

bool DlDilateImageFilter::equals_(const DlImageFilter& other) const {
  FML_DCHECK(other.type() == DlImageFilterType::kDilate);
  auto that = static_cast<const DlDilateImageFilter*>(&other);
  return (radius_x_ == that->radius_x_ && radius_y_ == that->radius_y_);
}

}  // namespace flutter
