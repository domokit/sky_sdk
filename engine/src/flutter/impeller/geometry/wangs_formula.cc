// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/geometry/wangs_formula.h"

namespace impeller {

namespace {

// Don't allow linearized segments to be off by more than 1/4th of a pixel from
// the true curve. This value should be scaled by the max basis of the
// X and Y directions.
constexpr static Scalar kPrecision = 4;

constexpr Scalar length(Point n) {
  Point nn = n * n;
  return std::sqrt(nn.x + nn.y);
}

}  // namespace

Scalar ComputeCubicSubdivisions(Scalar scale_factor,
                                Point p0,
                                Point p1,
                                Point p2,
                                Point p3) {
  Scalar k = scale_factor * .75f * kPrecision;
  Point a = (p0 - p1 * 2 + p2).Abs();
  Point b = (p1 - p2 * 2 + p3).Abs();
  return std::sqrt(k * length(a.Max(b)));
}

Scalar ComputeQuadradicSubdivisions(Scalar scale_factor,
                                    Point p0,
                                    Point p1,
                                    Point p2) {
  Scalar k = scale_factor * .25f * kPrecision;
  return std::sqrt(k * length(p0 - p1 * 2 + p2));
}

Scalar ComputeQuadradicSubdivisions(Scalar scale_factor,
                                    const QuadraticPathComponent& quad) {
  return ComputeQuadradicSubdivisions(scale_factor, quad.p1, quad.cp, quad.p2);
}

Scalar ComputeCubicSubdivisions(float scale_factor,
                                const CubicPathComponent& cub) {
  return ComputeCubicSubdivisions(scale_factor, cub.p1, cub.cp1, cub.cp2,
                                  cub.p2);
}

}  // namespace impeller
