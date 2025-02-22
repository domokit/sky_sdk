// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/impeller/geometry/round_superellipse.h"

#include "flutter/impeller/geometry/round_rect.h"
#include "flutter/impeller/geometry/round_superellipse_param.h"

namespace impeller {

RoundSuperellipse RoundSuperellipse::MakeRectRadii(
    const Rect& in_bounds,
    const RoundingRadii& in_radii) {
  if (!in_bounds.IsFinite()) {
    return {};
  }
  Rect bounds = in_bounds.GetPositive();
  // RoundingRadii::Scaled might return an empty radii if bounds or in_radii is
  // empty, which is expected. Pass along the bounds even if the radii is empty
  // as it would still have a valid location and/or 1-dimensional size which
  // might appear when stroked
  return RoundSuperellipse(bounds, in_radii.Scaled(bounds));
}

[[nodiscard]] bool RoundSuperellipse::Contains(const Point& p) const {
  if (!bounds_.Contains(p)) {
    return false;
  }
  auto param = RoundSuperellipseParam::MakeBoundsRadii(bounds_, radii_);
  return param.Contains(p);
}

RoundRect RoundSuperellipse::ToApproximateRoundRect() const {
  // Experiments have shown that using the same corner radii for the RRect
  // provides an approximation that is close to optimal, as achieving a perfect
  // match is not feasible.
  return RoundRect::MakeRectRadii(GetBounds(), GetRadii());
}

}  // namespace impeller
