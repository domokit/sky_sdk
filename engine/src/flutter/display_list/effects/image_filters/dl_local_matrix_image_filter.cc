// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/display_list/effects/image_filters/dl_local_matrix_image_filter.h"

#include "flutter/display_list/utils/dl_comparable.h"

namespace flutter {

std::shared_ptr<DlImageFilter> DlLocalMatrixImageFilter::Make(
    const DlMatrix& matrix,
    const std::shared_ptr<DlImageFilter>& filter) {
  return std::make_shared<DlLocalMatrixImageFilter>(matrix, filter);
}

bool DlLocalMatrixImageFilter::modifies_transparent_black() const {
  if (!image_filter_) {
    return false;
  }
  return image_filter_->modifies_transparent_black();
}

DlRect* DlLocalMatrixImageFilter::map_local_bounds(
    const DlRect& input_bounds,
    DlRect& output_bounds) const {
  if (!image_filter_) {
    output_bounds = input_bounds;
    return &output_bounds;
  }
  return image_filter_->map_local_bounds(input_bounds, output_bounds);
}

DlIRect* DlLocalMatrixImageFilter::map_device_bounds(
    const DlIRect& input_bounds,
    const DlMatrix& ctm,
    DlIRect& output_bounds) const {
  if (!image_filter_) {
    output_bounds = input_bounds;
    return &output_bounds;
  }
  return image_filter_->map_device_bounds(input_bounds, ctm * matrix_,
                                          output_bounds);
}

DlIRect* DlLocalMatrixImageFilter::get_input_device_bounds(
    const DlIRect& output_bounds,
    const DlMatrix& ctm,
    DlIRect& input_bounds) const {
  if (!image_filter_) {
    input_bounds = output_bounds;
    return &input_bounds;
  }
  return image_filter_->get_input_device_bounds(output_bounds, ctm * matrix_,
                                                input_bounds);
}

bool DlLocalMatrixImageFilter::equals_(const DlImageFilter& other) const {
  FML_DCHECK(other.type() == DlImageFilterType::kLocalMatrix);
  auto that = static_cast<const DlLocalMatrixImageFilter*>(&other);
  return (matrix_ == that->matrix_ &&
          Equals(image_filter_, that->image_filter_));
}

}  // namespace flutter
