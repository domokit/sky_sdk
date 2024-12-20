// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/display_list/effects/image_filters/dl_matrix_image_filter.h"

namespace flutter {

std::shared_ptr<DlImageFilter> DlMatrixImageFilter::Make(
    const DlMatrix& matrix,
    DlImageSampling sampling) {
  if (matrix.IsFinite() && !matrix.IsIdentity()) {
    return std::make_shared<DlMatrixImageFilter>(matrix, sampling);
  }
  return nullptr;
}

DlRect* DlMatrixImageFilter::map_local_bounds(const DlRect& input_bounds,
                                              DlRect& output_bounds) const {
  output_bounds = input_bounds.TransformAndClipBounds(matrix_);
  return &output_bounds;
}

DlIRect* DlMatrixImageFilter::map_device_bounds(const DlIRect& input_bounds,
                                                const DlMatrix& ctm,
                                                DlIRect& output_bounds) const {
  if (!ctm.IsInvertible()) {
    output_bounds = input_bounds;
    return nullptr;
  }
  DlMatrix matrix = ctm * matrix_ * ctm.Invert();
  DlRect device_rect =
      DlRect::Make(input_bounds).TransformAndClipBounds(matrix);
  output_bounds = DlIRect::RoundOut(device_rect);
  return &output_bounds;
}

DlIRect* DlMatrixImageFilter::get_input_device_bounds(
    const DlIRect& output_bounds,
    const DlMatrix& ctm,
    DlIRect& input_bounds) const {
  DlMatrix matrix = ctm * matrix_;
  if (!matrix.IsInvertible()) {
    input_bounds = output_bounds;
    return nullptr;
  }
  DlMatrix inverse = ctm * matrix.Invert();
  DlRect bounds = DlRect::Make(output_bounds);
  bounds = bounds.TransformAndClipBounds(inverse);
  input_bounds = DlIRect::RoundOut(bounds);
  return &input_bounds;
}

bool DlMatrixImageFilter::equals_(const DlImageFilter& other) const {
  FML_DCHECK(other.type() == DlImageFilterType::kMatrix);
  auto that = static_cast<const DlMatrixImageFilter*>(&other);
  return (matrix_ == that->matrix_ && sampling_ == that->sampling_);
}

}  // namespace flutter
