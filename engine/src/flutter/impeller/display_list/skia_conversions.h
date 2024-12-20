// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_DISPLAY_LIST_SKIA_CONVERSIONS_H_
#define FLUTTER_IMPELLER_DISPLAY_LIST_SKIA_CONVERSIONS_H_

#include "flutter/display_list/dl_blend_mode.h"
#include "flutter/display_list/dl_color.h"
#include "flutter/display_list/effects/dl_color_sources.h"
#include "impeller/core/formats.h"
#include "impeller/core/sampler_descriptor.h"
#include "impeller/geometry/color.h"
#include "impeller/geometry/path.h"
#include "impeller/geometry/path_builder.h"
#include "impeller/geometry/point.h"
#include "impeller/geometry/rect.h"
#include "third_party/skia/include/core/SkColorType.h"
#include "third_party/skia/include/core/SkPath.h"
#include "third_party/skia/include/core/SkPoint.h"
#include "third_party/skia/include/core/SkRRect.h"
#include "third_party/skia/include/core/SkRSXform.h"
#include "third_party/skia/include/core/SkTextBlob.h"

namespace impeller {
namespace skia_conversions {

/// @brief Like SkRRect.isSimple, but allows the corners to differ by
///        kEhCloseEnough.
///
///        An RRect is simple if all corner radii are approximately
///        equal.
bool IsNearlySimpleRRect(const SkRRect& rr);

Rect ToRect(const SkRect& rect);

std::optional<Rect> ToRect(const SkRect* rect);
std::optional<const Rect> ToRect(const flutter::DlRect* rect);

std::vector<Rect> ToRects(const SkRect tex[], int count);
std::vector<Rect> ToRects(const flutter::DlRect tex[], int count);

std::vector<Point> ToPoints(const SkPoint points[], int count);
std::vector<Point> ToPoints(const flutter::DlPoint points[], int count);

Point ToPoint(const SkPoint& point);

Size ToSize(const SkPoint& point);

Color ToColor(const flutter::DlColor& color);

Matrix ToRSXForm(const SkRSXform& form);

std::optional<impeller::PixelFormat> ToPixelFormat(SkColorType type);

impeller::SamplerDescriptor ToSamplerDescriptor(
    const flutter::DlImageSampling options);

Matrix ToMatrix(const SkMatrix& m);

BlendMode ToBlendMode(flutter::DlBlendMode mode);

/// @brief Convert display list colors + stops into impeller colors and stops,
/// taking care to ensure that the stops monotonically increase from 0.0 to 1.0.
///
/// The general process is:
/// * Ensure that the first gradient stop value is 0.0. If not, insert a new
///   stop with a value of 0.0 and use the first gradient color as this new
///   stops color.
/// * Ensure the last gradient stop value is 1.0. If not, insert a new stop
///   with a value of 1.0 and use the last gradient color as this stops color.
/// * Clamp all gradient values between the values of 0.0 and 1.0.
/// * For all stop values, ensure that the values are monotonically increasing
///   by clamping each value to a minimum of the previous stop value and itself.
///   For example, with stop values of 0.0, 0.5, 0.4, 1.0, we would clamp such
///   that the values were 0.0, 0.5, 0.5, 1.0.
void ConvertStops(const flutter::DlGradientColorSourceBase* gradient,
                  std::vector<Color>& colors,
                  std::vector<float>& stops);

}  // namespace skia_conversions
}  // namespace impeller

#endif  // FLUTTER_IMPELLER_DISPLAY_LIST_SKIA_CONVERSIONS_H_
