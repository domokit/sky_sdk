// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_DISPLAY_LIST_EFFECTS_IMAGE_FILTERS_DL_ERODE_IMAGE_FILTER_H_
#define FLUTTER_DISPLAY_LIST_EFFECTS_IMAGE_FILTERS_DL_ERODE_IMAGE_FILTER_H_

#include <utility>

#include "display_list/effects/dl_image_filter.h"

namespace flutter {

class DlErodeImageFilter final : public DlImageFilter {
 public:
  DlErodeImageFilter(DlScalar radius_x, DlScalar radius_y)
      : radius_x_(radius_x), radius_y_(radius_y) {}
  explicit DlErodeImageFilter(const DlErodeImageFilter* filter)
      : DlErodeImageFilter(filter->radius_x_, filter->radius_y_) {}
  DlErodeImageFilter(const DlErodeImageFilter& filter)
      : DlErodeImageFilter(&filter) {}

  static std::shared_ptr<DlImageFilter> Make(DlScalar radius_x,
                                             DlScalar radius_y);

  std::shared_ptr<DlImageFilter> shared() const override {
    return std::make_shared<DlErodeImageFilter>(this);
  }

  DlImageFilterType type() const override { return DlImageFilterType::kErode; }
  size_t size() const override { return sizeof(*this); }

  const DlErodeImageFilter* asErode() const override { return this; }

  bool modifies_transparent_black() const override { return false; }

  DlRect* map_local_bounds(const DlRect& input_bounds,
                           DlRect& output_bounds) const override;

  DlIRect* map_device_bounds(const DlIRect& input_bounds,
                             const DlMatrix& ctm,
                             DlIRect& output_bounds) const override;

  DlIRect* get_input_device_bounds(const DlIRect& output_bounds,
                                   const DlMatrix& ctm,
                                   DlIRect& input_bounds) const override;

  DlScalar radius_x() const { return radius_x_; }
  DlScalar radius_y() const { return radius_y_; }

 protected:
  bool equals_(const DlImageFilter& other) const override;

 private:
  DlScalar radius_x_;
  DlScalar radius_y_;
};

}  // namespace flutter

#endif  // FLUTTER_DISPLAY_LIST_EFFECTS_IMAGE_FILTERS_DL_ERODE_IMAGE_FILTER_H_
