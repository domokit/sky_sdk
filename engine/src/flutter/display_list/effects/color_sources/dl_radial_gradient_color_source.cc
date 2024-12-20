// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/display_list/effects/color_sources/dl_radial_gradient_color_source.h"

namespace flutter {

DlRadialGradientColorSource::DlRadialGradientColorSource(
    const DlRadialGradientColorSource* source)
    : DlGradientColorSourceBase(source->stop_count(),
                                source->tile_mode(),
                                source->matrix_ptr()),
      center_(source->center()),
      radius_(source->radius()) {
  store_color_stops(this + 1, source->colors(), source->stops());
}

std::shared_ptr<DlColorSource> DlRadialGradientColorSource::shared() const {
  return MakeRadial(center_, radius_, stop_count(), colors(), stops(),
                    tile_mode(), matrix_ptr());
}

bool DlRadialGradientColorSource::equals_(DlColorSource const& other) const {
  FML_DCHECK(other.type() == DlColorSourceType::kRadialGradient);
  auto that = static_cast<DlRadialGradientColorSource const*>(&other);
  return (center_ == that->center_ && radius_ == that->radius_ &&
          base_equals_(that));
}

}  // namespace flutter
