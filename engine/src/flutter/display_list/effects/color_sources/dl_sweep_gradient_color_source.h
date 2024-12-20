// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_DISPLAY_LIST_EFFECTS_COLOR_SOURCES_DL_SWEEP_GRADIENT_COLOR_SOURCE_H_
#define FLUTTER_DISPLAY_LIST_EFFECTS_COLOR_SOURCES_DL_SWEEP_GRADIENT_COLOR_SOURCE_H_

#include "flutter/display_list/effects/color_sources/dl_gradient_color_source_base.h"

namespace flutter {

class DlSweepGradientColorSource final : public DlGradientColorSourceBase {
 public:
  const DlSweepGradientColorSource* asSweepGradient() const override {
    return this;
  }

  bool isUIThreadSafe() const override { return true; }

  std::shared_ptr<DlColorSource> shared() const override;

  DlColorSourceType type() const override {
    return DlColorSourceType::kSweepGradient;
  }
  size_t size() const override { return sizeof(*this) + vector_sizes(); }

  DlPoint center() const { return center_; }
  DlScalar start() const { return start_; }
  DlScalar end() const { return end_; }

 protected:
  virtual const void* pod() const override { return this + 1; }

  bool equals_(DlColorSource const& other) const override;

 private:
  template <typename Colors>
  DlSweepGradientColorSource(DlPoint center,
                             DlScalar start,
                             DlScalar end,
                             uint32_t stop_count,
                             Colors colors,
                             const float* stops,
                             DlTileMode tile_mode,
                             const DlMatrix* matrix = nullptr)
      : DlGradientColorSourceBase(stop_count, tile_mode, matrix),
        center_(center),
        start_(start),
        end_(end) {
    store_color_stops(this + 1, colors, stops);
  }

  explicit DlSweepGradientColorSource(const DlSweepGradientColorSource* source);

  DlPoint center_;
  DlScalar start_;
  DlScalar end_;

  friend class DlColorSource;
  friend class DisplayListBuilder;

  FML_DISALLOW_COPY_ASSIGN_AND_MOVE(DlSweepGradientColorSource);
};

}  // namespace flutter

#endif  // FLUTTER_DISPLAY_LIST_EFFECTS_COLOR_SOURCES_DL_SWEEP_GRADIENT_COLOR_SOURCE_H_
