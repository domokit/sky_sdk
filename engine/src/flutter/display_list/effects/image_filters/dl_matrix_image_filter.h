// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_DISPLAY_LIST_EFFECTS_IMAGE_FILTERS_DL_MATRIX_IMAGE_FILTER_H_
#define FLUTTER_DISPLAY_LIST_EFFECTS_IMAGE_FILTERS_DL_MATRIX_IMAGE_FILTER_H_

#include "display_list/effects/dl_image_filter.h"

#include "flutter/display_list/dl_sampling_options.h"

namespace flutter {

class DlMatrixImageFilter final : public DlImageFilter {
 public:
  DlMatrixImageFilter(const DlMatrix& matrix, DlImageSampling sampling)
      : matrix_(matrix), sampling_(sampling) {}
  explicit DlMatrixImageFilter(const DlMatrixImageFilter* filter)
      : DlMatrixImageFilter(filter->matrix_, filter->sampling_) {}
  DlMatrixImageFilter(const DlMatrixImageFilter& filter)
      : DlMatrixImageFilter(&filter) {}

  static std::shared_ptr<DlImageFilter> Make(const DlMatrix& matrix,
                                             DlImageSampling sampling);

  std::shared_ptr<DlImageFilter> shared() const override {
    return std::make_shared<DlMatrixImageFilter>(this);
  }

  DlImageFilterType type() const override { return DlImageFilterType::kMatrix; }
  size_t size() const override { return sizeof(*this); }

  const DlMatrix& matrix() const { return matrix_; }
  DlImageSampling sampling() const { return sampling_; }

  const DlMatrixImageFilter* asMatrix() const override { return this; }

  bool modifies_transparent_black() const override { return false; }

  DlRect* map_local_bounds(const DlRect& input_bounds,
                           DlRect& output_bounds) const override;

  DlIRect* map_device_bounds(const DlIRect& input_bounds,
                             const DlMatrix& ctm,
                             DlIRect& output_bounds) const override;

  DlIRect* get_input_device_bounds(const DlIRect& output_bounds,
                                   const DlMatrix& ctm,
                                   DlIRect& input_bounds) const override;

 protected:
  bool equals_(const DlImageFilter& other) const override;

 private:
  DlMatrix matrix_;
  DlImageSampling sampling_;
};

}  // namespace flutter

#endif  // FLUTTER_DISPLAY_LIST_EFFECTS_IMAGE_FILTERS_DL_MATRIX_IMAGE_FILTER_H_
