// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/display_list/effects/color_sources/dl_conical_gradient_color_source.h"

namespace flutter {

DlConicalGradientColorSource::DlConicalGradientColorSource(
    const DlConicalGradientColorSource* source)
    : DlGradientColorSourceBase(source->stop_count(),
                                source->tile_mode(),
                                source->matrix_ptr()),
      start_center_(source->start_center()),
      start_radius_(source->start_radius()),
      end_center_(source->end_center()),
      end_radius_(source->end_radius()) {
  store_color_stops(this + 1, source->colors(), source->stops());
}

std::shared_ptr<DlColorSource> DlConicalGradientColorSource::shared() const {
  return MakeConical(start_center_, start_radius_, end_center_, end_radius_,
                     stop_count(), colors(), stops(), tile_mode(),
                     matrix_ptr());
}

bool DlConicalGradientColorSource::equals_(DlColorSource const& other) const {
  FML_DCHECK(other.type() == DlColorSourceType::kConicalGradient);
  auto that = static_cast<DlConicalGradientColorSource const*>(&other);
  return (start_center_ == that->start_center_ &&
          start_radius_ == that->start_radius_ &&
          end_center_ == that->end_center_ &&
          end_radius_ == that->end_radius_ && base_equals_(that));
}

}  // namespace flutter
