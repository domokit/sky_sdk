// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_DISPLAY_LIST_EFFECTS_COLOR_SOURCES_DL_CONICAL_GRADIENT_COLOR_SOURCE_H_
#define FLUTTER_DISPLAY_LIST_EFFECTS_COLOR_SOURCES_DL_CONICAL_GRADIENT_COLOR_SOURCE_H_

#include "flutter/display_list/effects/color_sources/dl_gradient_color_source_base.h"

namespace flutter {

class DlConicalGradientColorSource final : public DlGradientColorSourceBase {
 public:
  const DlConicalGradientColorSource* asConicalGradient() const override {
    return this;
  }

  bool isUIThreadSafe() const override { return true; }

  std::shared_ptr<DlColorSource> shared() const override;

  DlColorSourceType type() const override {
    return DlColorSourceType::kConicalGradient;
  }
  size_t size() const override { return sizeof(*this) + vector_sizes(); }

  DlPoint start_center() const { return start_center_; }
  DlScalar start_radius() const { return start_radius_; }
  DlPoint end_center() const { return end_center_; }
  DlScalar end_radius() const { return end_radius_; }

 protected:
  virtual const void* pod() const override { return this + 1; }

  bool equals_(DlColorSource const& other) const override;

 private:
  template <typename Colors>
  DlConicalGradientColorSource(DlPoint start_center,
                               DlScalar start_radius,
                               DlPoint end_center,
                               DlScalar end_radius,
                               uint32_t stop_count,
                               Colors colors,
                               const float* stops,
                               DlTileMode tile_mode,
                               const DlMatrix* matrix = nullptr)
      : DlGradientColorSourceBase(stop_count, tile_mode, matrix),
        start_center_(start_center),
        start_radius_(start_radius),
        end_center_(end_center),
        end_radius_(end_radius) {
    store_color_stops(this + 1, colors, stops);
  }

  explicit DlConicalGradientColorSource(
      const DlConicalGradientColorSource* source);

  DlPoint start_center_;
  DlScalar start_radius_;
  DlPoint end_center_;
  DlScalar end_radius_;

  friend class DlColorSource;
  friend class DisplayListBuilder;

  FML_DISALLOW_COPY_ASSIGN_AND_MOVE(DlConicalGradientColorSource);
};

}  // namespace flutter

#endif  // FLUTTER_DISPLAY_LIST_EFFECTS_COLOR_SOURCES_DL_CONICAL_GRADIENT_COLOR_SOURCE_H_
