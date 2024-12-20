// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_DISPLAY_LIST_EFFECTS_IMAGE_FILTERS_DL_COMPOSE_IMAGE_FILTER_H_
#define FLUTTER_DISPLAY_LIST_EFFECTS_IMAGE_FILTERS_DL_COMPOSE_IMAGE_FILTER_H_

#include "display_list/effects/dl_image_filter.h"

namespace flutter {

class DlComposeImageFilter final : public DlImageFilter {
 public:
  DlComposeImageFilter(const std::shared_ptr<DlImageFilter>& outer,
                       const std::shared_ptr<DlImageFilter>& inner)
      : outer_(outer), inner_(inner) {}
  DlComposeImageFilter(const DlImageFilter* outer, const DlImageFilter* inner)
      : outer_(outer->shared()), inner_(inner->shared()) {}
  DlComposeImageFilter(const DlImageFilter& outer, const DlImageFilter& inner)
      : DlComposeImageFilter(&outer, &inner) {}
  explicit DlComposeImageFilter(const DlComposeImageFilter* filter)
      : DlComposeImageFilter(filter->outer_, filter->inner_) {}
  DlComposeImageFilter(const DlComposeImageFilter& filter)
      : DlComposeImageFilter(&filter) {}

  static std::shared_ptr<DlImageFilter> Make(
      const std::shared_ptr<DlImageFilter>& outer,
      const std::shared_ptr<DlImageFilter>& inner);

  std::shared_ptr<DlImageFilter> shared() const override {
    return std::make_shared<DlComposeImageFilter>(this);
  }

  DlImageFilterType type() const override {
    return DlImageFilterType::kCompose;
  }
  size_t size() const override { return sizeof(*this); }

  std::shared_ptr<DlImageFilter> outer() const { return outer_; }
  std::shared_ptr<DlImageFilter> inner() const { return inner_; }

  const DlComposeImageFilter* asCompose() const override { return this; }

  bool modifies_transparent_black() const override;

  DlRect* map_local_bounds(const DlRect& input_bounds,
                           DlRect& output_bounds) const override;

  DlIRect* map_device_bounds(const DlIRect& input_bounds,
                             const DlMatrix& ctm,
                             DlIRect& output_bounds) const override;

  DlIRect* get_input_device_bounds(const DlIRect& output_bounds,
                                   const DlMatrix& ctm,
                                   DlIRect& input_bounds) const override;

  MatrixCapability matrix_capability() const override;

 protected:
  bool equals_(const DlImageFilter& other) const override;

 private:
  const std::shared_ptr<DlImageFilter> outer_;
  const std::shared_ptr<DlImageFilter> inner_;
};

}  // namespace flutter

#endif  // FLUTTER_DISPLAY_LIST_EFFECTS_IMAGE_FILTERS_DL_COMPOSE_IMAGE_FILTER_H_
