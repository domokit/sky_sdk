// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/display_list/effects/color_sources/dl_linear_gradient_color_source.h"

namespace flutter {

DlLinearGradientColorSource::DlLinearGradientColorSource(
    const DlLinearGradientColorSource* source)
    : DlGradientColorSourceBase(source->stop_count(),
                                source->tile_mode(),
                                source->matrix_ptr()),
      start_point_(source->start_point()),
      end_point_(source->end_point()) {
  store_color_stops(this + 1, source->colors(), source->stops());
}

std::shared_ptr<DlColorSource> DlLinearGradientColorSource::shared() const {
  return MakeLinear(start_point_, end_point_, stop_count(), colors(), stops(),
                    tile_mode(), matrix_ptr());
}

bool DlLinearGradientColorSource::equals_(DlColorSource const& other) const {
  FML_DCHECK(other.type() == DlColorSourceType::kLinearGradient);
  auto that = static_cast<DlLinearGradientColorSource const*>(&other);
  return (start_point_ == that->start_point_ &&
          end_point_ == that->end_point_ && base_equals_(that));
}

}  // namespace flutter
