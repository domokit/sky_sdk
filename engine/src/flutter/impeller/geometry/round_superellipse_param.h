// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_GEOMETRY_ROUND_SUPERELLIPSE_PARAM_H_
#define FLUTTER_IMPELLER_GEOMETRY_ROUND_SUPERELLIPSE_PARAM_H_

#include "flutter/impeller/geometry/point.h"
#include "flutter/impeller/geometry/rect.h"
#include "flutter/impeller/geometry/rounding_radii.h"
#include "flutter/impeller/geometry/size.h"

namespace impeller {

// A utility struct that expands input parameters for a rounded superellipse to
// drawing variables.
struct RoundSuperellipseParam {
  // Parameters for drawing a square-like rounded superellipse.
  //
  // This structure is used to define an octant of an arbitrary rounded
  // superellipse.
  struct Octant {
    // The offset of the square-like rounded superellipse's center from the
    // origin.
    //
    // All other coordinates in this structure are relative to this point.
    Point offset;

    // The coordinate of the midpoint of the top edge, relative to the `offset`
    // point.
    //
    // This is the starting point of the octant curve.
    Point edge_mid;

    // The coordinate of the superellipse's center, relative to the `offset`
    // point.
    Point se_center;
    // The semi-axis length of the superellipse.
    Scalar se_a;
    // The degree of the superellipse.
    Scalar se_n;
    // The range of the parameter "theta" used to define the superellipse curve.
    //
    // The "theta" is not the angle of the curve but the implicit parameter
    // used in the curve's parametric equation.
    Scalar se_max_theta;

    Scalar ratio;

    // The coordinate of the top left end of the circular arc, relative to the
    // `offset` point.
    Point circle_start;
    // The center of the circular arc, relative to the `offset` point.
    Point circle_center;
    // The angular span of the circular arc, measured in radians.
    Radians circle_max_angle;
  };

  // Parameters for drawing a rounded superellipse with equal radius size for
  // all corners.
  //
  // This structure is used to define a quadrant of an arbitrary rounded
  // superellipse.
  struct Quadrant {
    // The offset of the rounded superellipse's center from the origin.
    //
    // All other coordinates in this structure are relative to this point.
    Point offset;

    // The scaling factor used to transform a normalized rounded superellipse
    // back to its original, unnormalized shape.
    //
    // Normalization refers to adjusting the original curve, which may have
    // asymmetrical corner sizes, into a symmetrical one by reducing the longer
    // radius to match the shorter one. For instance, to draw a rounded
    // superellipse with size (200, 300) and radii (20, 10), the function first
    // draws a normalized RSE with size (100, 300) and radii (10, 10), then
    // scales it by (2x, 1x) to restore the original proportions.
    //
    // Normalization also flips the curve to the first quadrant (positive x and
    // y) if it originally resides in another quadrant. This affects the signs
    // of `signed_scale`.
    Point signed_scale;

    // The parameters for the two octants that make up this quadrant after
    // normalization.
    Octant top;
    Octant right;
  };

  // The parameters for the four quadrants that make up the full contour.
  //
  // If `all_corners_same` is true, then only `top_right` is popularized.
  Quadrant top_right;
  Quadrant bottom_right;
  Quadrant bottom_left;
  Quadrant top_left;

  // If true, all corners are the same and only `top_right` is popularized.
  bool all_corners_same;

  // Create a param for a rounded superellipse with the specific bounds and
  // radii.
  [[nodiscard]] static RoundSuperellipseParam MakeBoundsRadii(
      const Rect& bounds,
      const RoundingRadii& radii);

  // Returns whether this rounded superellipse contains the point.
  //
  // This method does not perform any prescreening such as comparing the point
  // with the bounds, which is recommended for callers.
  bool Contains(const Point& point) const;

  // A factor used to calculate the "gap", defined as the distance from the
  // midpoint of the curved corners to the nearest sides of the bounding box.
  //
  // When the corner radius is symmetrical on both dimensions, the midpoint of
  // the corner is where the circular arc intersects its quadrant bisector. When
  // the corner radius is asymmetrical, since the corner can be considered
  // "elongated" from a symmetrical corner, the midpoint is transformed in the
  // same way.
  //
  // Experiments indicate that the gap is linear with respect to the corner
  // radius on that dimension.
  static constexpr Scalar kGapFactor = 0.29289321881f;  // 1-cos(pi/4)

  static void SuperellipseBezierArc(
      Point* output,
      const RoundSuperellipseParam::Octant& param);
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_GEOMETRY_ROUND_SUPERELLIPSE_PARAM_H_
