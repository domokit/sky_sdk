// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/impeller/geometry/round_superellipse.h"

#include <cmath>

namespace impeller {

namespace {
// The distance from the middle of the curve corners (i.e. the intersection of
// the circular arc with its respective quadrant bisector, or point M in the
// figure in DrawOctantSquareLikeSquircle) to either closeby side of the
// bounding box.
constexpr Scalar CalculateGap(Scalar corner_radius) {
  // The formula should be kept in sync with a few files, as documented in
  // `CalculateGap` in round_superellipse_geometry.cc.

  // Heuristic formula derived from experimentation.
  return 0.2924066406 * corner_radius;
}
}  // namespace

RoundSuperellipse RoundSuperellipse::MakeRectRadius(const Rect& rect,
                                                    Scalar corner_radius) {
  if (rect.IsEmpty() || !rect.IsFinite() ||  //
      !std::isfinite(corner_radius)) {
    // preserve the empty bounds as they might be strokable
    return RoundSuperellipse(rect, 0);
  }

  return RoundSuperellipse(rect, corner_radius);
}

Rect RoundSuperellipse::EstimateInner() const {
  return bounds_.Expand(-CalculateGap(corner_radius_));
}

}  // namespace impeller
