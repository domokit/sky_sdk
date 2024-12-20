// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_DISPLAY_LIST_EFFECTS_COLOR_SOURCES_DL_IMAGE_COLOR_SOURCE_H_
#define FLUTTER_DISPLAY_LIST_EFFECTS_COLOR_SOURCES_DL_IMAGE_COLOR_SOURCE_H_

#include "flutter/display_list/effects/color_sources/dl_matrix_color_source_base.h"

namespace flutter {

class DlImageColorSource final : public DlMatrixColorSourceBase {
 public:
  DlImageColorSource(sk_sp<const DlImage> image,
                     DlTileMode horizontal_tile_mode,
                     DlTileMode vertical_tile_mode,
                     DlImageSampling sampling = DlImageSampling::kLinear,
                     const DlMatrix* matrix = nullptr);

  bool isUIThreadSafe() const override {
    return image_ ? image_->isUIThreadSafe() : true;
  }

  const DlImageColorSource* asImage() const override { return this; }

  std::shared_ptr<DlColorSource> shared() const override {
    return WithSampling(sampling_);
  }

  std::shared_ptr<DlColorSource> WithSampling(DlImageSampling sampling) const;

  DlColorSourceType type() const override { return DlColorSourceType::kImage; }
  size_t size() const override { return sizeof(*this); }

  bool is_opaque() const override { return image_->isOpaque(); }

  sk_sp<const DlImage> image() const { return image_; }
  DlTileMode horizontal_tile_mode() const { return horizontal_tile_mode_; }
  DlTileMode vertical_tile_mode() const { return vertical_tile_mode_; }
  DlImageSampling sampling() const { return sampling_; }

 protected:
  bool equals_(DlColorSource const& other) const override;

 private:
  sk_sp<const DlImage> image_;
  DlTileMode horizontal_tile_mode_;
  DlTileMode vertical_tile_mode_;
  DlImageSampling sampling_;

  FML_DISALLOW_COPY_ASSIGN_AND_MOVE(DlImageColorSource);
};

}  // namespace flutter

#endif  // FLUTTER_DISPLAY_LIST_EFFECTS_COLOR_SOURCES_DL_IMAGE_COLOR_SOURCE_H_
