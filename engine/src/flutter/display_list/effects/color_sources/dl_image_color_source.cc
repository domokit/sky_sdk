// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/display_list/effects/color_sources/dl_image_color_source.h"

namespace flutter {

DlImageColorSource::DlImageColorSource(sk_sp<const DlImage> image,
                                       DlTileMode horizontal_tile_mode,
                                       DlTileMode vertical_tile_mode,
                                       DlImageSampling sampling,
                                       const DlMatrix* matrix)
    : DlMatrixColorSourceBase(matrix),
      image_(std::move(image)),
      horizontal_tile_mode_(horizontal_tile_mode),
      vertical_tile_mode_(vertical_tile_mode),
      sampling_(sampling) {}

std::shared_ptr<DlColorSource> DlImageColorSource::WithSampling(
    DlImageSampling sampling) const {
  return std::make_shared<DlImageColorSource>(image_, horizontal_tile_mode_,
                                              vertical_tile_mode_, sampling,
                                              matrix_ptr());
}

bool DlImageColorSource::equals_(DlColorSource const& other) const {
  FML_DCHECK(other.type() == DlColorSourceType::kImage);
  auto that = static_cast<DlImageColorSource const*>(&other);
  return (image_->Equals(that->image_) && matrix() == that->matrix() &&
          horizontal_tile_mode_ == that->horizontal_tile_mode_ &&
          vertical_tile_mode_ == that->vertical_tile_mode_ &&
          sampling_ == that->sampling_);
}

}  // namespace flutter
