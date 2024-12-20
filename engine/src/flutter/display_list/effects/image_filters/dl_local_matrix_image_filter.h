// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_DISPLAY_LIST_EFFECTS_IMAGE_FILTERS_DL_LOCAL_MATRIX_IMAGE_FILTER_H_
#define FLUTTER_DISPLAY_LIST_EFFECTS_IMAGE_FILTERS_DL_LOCAL_MATRIX_IMAGE_FILTER_H_

#include "display_list/effects/dl_image_filter.h"

namespace flutter {

class DlLocalMatrixImageFilter final : public DlImageFilter {
 public:
  explicit DlLocalMatrixImageFilter(
      const DlMatrix& matrix,
      const std::shared_ptr<DlImageFilter>& filter)
      : matrix_(matrix), image_filter_(filter) {}
  explicit DlLocalMatrixImageFilter(const DlLocalMatrixImageFilter* filter)
      : DlLocalMatrixImageFilter(filter->matrix_, filter->image_filter_) {}
  DlLocalMatrixImageFilter(const DlLocalMatrixImageFilter& filter)
      : DlLocalMatrixImageFilter(&filter) {}

  std::shared_ptr<DlImageFilter> shared() const override {
    return std::make_shared<DlLocalMatrixImageFilter>(this);
  }

  static std::shared_ptr<DlImageFilter> Make(
      const DlMatrix& matrix,
      const std::shared_ptr<DlImageFilter>& filter);

  DlImageFilterType type() const override {
    return DlImageFilterType::kLocalMatrix;
  }
  size_t size() const override { return sizeof(*this); }

  const DlMatrix& matrix() const { return matrix_; }

  const std::shared_ptr<DlImageFilter> image_filter() const {
    return image_filter_;
  }

  const DlLocalMatrixImageFilter* asLocalMatrix() const override {
    return this;
  }

  bool modifies_transparent_black() const override;

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
  std::shared_ptr<DlImageFilter> image_filter_;
};

}  // namespace flutter

#endif  // FLUTTER_DISPLAY_LIST_EFFECTS_IMAGE_FILTERS_DL_LOCAL_MATRIX_IMAGE_FILTER_H_
