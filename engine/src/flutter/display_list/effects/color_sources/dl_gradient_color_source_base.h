// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_DISPLAY_LIST_EFFECTS_COLOR_SOURCES_DL_GRADIENT_COLOR_SOURCE_BASE_H_
#define FLUTTER_DISPLAY_LIST_EFFECTS_COLOR_SOURCES_DL_GRADIENT_COLOR_SOURCE_BASE_H_

#include "flutter/display_list/effects/color_sources/dl_matrix_color_source_base.h"

namespace flutter {

// Utility base class common to all gradient DlColorSource implementations
class DlGradientColorSourceBase : public DlMatrixColorSourceBase {
 public:
  bool is_opaque() const override;

  bool isGradient() const override { return true; }

  DlTileMode tile_mode() const { return mode_; }
  int stop_count() const { return stop_count_; }
  const DlColor* colors() const {
    return reinterpret_cast<const DlColor*>(pod());
  }
  const float* stops() const {
    return reinterpret_cast<const float*>(colors() + stop_count());
  }

 protected:
  DlGradientColorSourceBase(uint32_t stop_count,
                            DlTileMode tile_mode,
                            const DlMatrix* matrix = nullptr);

  size_t vector_sizes() const {
    return stop_count_ * (sizeof(DlColor) + sizeof(float));
  }

  virtual const void* pod() const = 0;

  bool base_equals_(DlGradientColorSourceBase const* other_base) const;

  void store_color_stops(void* pod,
                         const DlColor* color_data,
                         const float* stop_data);

  void store_color_stops(void* pod,
                         const DlScalar* color_data_argb,
                         const float* stop_data);

 private:
  DlTileMode mode_;
  uint32_t stop_count_;

  FML_DISALLOW_COPY_ASSIGN_AND_MOVE(DlGradientColorSourceBase);
};

}  // namespace flutter

#endif  // FLUTTER_DISPLAY_LIST_EFFECTS_COLOR_SOURCES_DL_GRADIENT_COLOR_SOURCE_BASE_H_
