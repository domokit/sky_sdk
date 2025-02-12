// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/impeller/geometry/round_superellipse_param.h"

namespace impeller {

namespace {

// Return the value that splits the range from `left` to `right` into two
// portions whose ratio equals to `ratio_left` : `ratio_right`.
Scalar Split(Scalar left, Scalar right, Scalar ratio_left, Scalar ratio_right) {
  if (ratio_left == 0 && ratio_right == 0) {
    return (left + right) / 2;
  }
  return (left * ratio_right + right * ratio_left) / (ratio_left + ratio_right);
}

// Return the same Point, but each NaN coordinate is replaced by 1.
inline Point ReplanceNaNWithOne(Point in) {
  return Point{std::isnan(in.x) ? 1 : in.x, std::isnan(in.y) ? 1 : in.y};
}

// Swap the x and y coordinate of a point.
//
// Effectively mirrors the point by the y=x line.
inline Point Flip(Point a) {
  return Point{a.y, a.x};
}

// A look up table with precomputed variables.
//
// The columns represent the following variabls respectively:
//
//  * n
//  * sin(thetaJ)
//
// For definition of the variables, see ComputeOctant.
constexpr Scalar kPrecomputedVariables[][2] = {
    /*ratio=2.00*/ {2.00000000, 0.117205737},
    /*ratio=2.02*/ {2.03999083, 0.117205737},
    /*ratio=2.04*/ {2.07976152, 0.119418745},
    /*ratio=2.06*/ {2.11195967, 0.136274515},
    /*ratio=2.08*/ {2.14721808, 0.141289310},
    /*ratio=2.10*/ {2.18349805, 0.143410679},
    /*ratio=2.12*/ {2.21858213, 0.146668334},
    /*ratio=2.14*/ {2.24861661, 0.154985392},
    /*ratio=2.16*/ {2.28146030, 0.158932848},
    /*ratio=2.18*/ {2.30842385, 0.168182439},
    /*ratio=2.20*/ {2.33888662, 0.172911853},
    /*ratio=2.22*/ {2.36937163, 0.177039959},
    /*ratio=2.24*/ {2.40317673, 0.177839181},
    /*ratio=2.26*/ {2.42840031, 0.185615110},
    /*ratio=2.28*/ {2.45838300, 0.188905374},
    /*ratio=2.30*/ {2.48660575, 0.193273145}};
constexpr Scalar kRatioStepInverse = 50;  // = 1 / 0.02

constexpr size_t kNumRecords =
    sizeof(kPrecomputedVariables) / sizeof(kPrecomputedVariables[0]);
constexpr Scalar kMinRatio = 2.00f;
constexpr Scalar kMaxRatio = kMinRatio + (kNumRecords - 1) / kRatioStepInverse;

// Linear interpolation for `kPrecomputedVariables`.
//
// The `column` is a 0-based index that decides the target variable.
Scalar LerpPrecomputedVariable(size_t column, Scalar ratio) {
  Scalar steps = std::clamp<Scalar>((ratio - kMinRatio) * kRatioStepInverse, 0,
                                    kNumRecords - 1);
  size_t left = std::clamp<size_t>(static_cast<size_t>(std::floor(steps)), 0,
                                   kNumRecords - 2);
  Scalar frac = steps - left;

  return (1 - frac) * kPrecomputedVariables[left][column] +
         frac * kPrecomputedVariables[left + 1][column];
}

// Find the center of the circle that passes the given two points and have the
// given radius.
Point FindCircleCenter(Point a, Point b, Scalar r) {
  /* Denote the middle point of A and B as M. The key is to find the center of
   * the circle.
   *         A --__
   *          /  ⟍ `、
   *         /   M  ⟍\
   *        /       ⟋  B
   *       /     ⟋   ↗
   *      /   ⟋
   *     / ⟋    r
   *  C ᜱ  ↙
   */

  Point a_to_b = b - a;
  Point m = (a + b) / 2;
  Point c_to_m = Point(-a_to_b.y, a_to_b.x);
  Scalar distance_am = a_to_b.GetLength() / 2;
  Scalar distance_cm = sqrt(r * r - distance_am * distance_am);
  return m - distance_cm * c_to_m.Normalize();
}

// Compute parameters for a square-like rounded superellipse with a symmetrical
// radius.
RoundSuperellipseParam::Octant ComputeOctant(Point center,
                                             Scalar half_size,
                                             Scalar radius) {
  /* The following figure shows the first quadrant of a square-like rounded
   * superellipse. The target arc consists of the "stretch" (AB), a
   * superellipsoid arc (BJ), and a circular arc (JM).
   *
   *     straight   superelipse
   *          ↓     ↓
   *        A    B       J    circular arc
   *        ---------...._   ↙
   *        |    |      /  `⟍ M
   *        |    |     /    ⟋ ⟍
   *        |    |    /  ⟋     \
   *        |    |   / ⟋        |
   *        |    |  ᜱD          |
   *        |    | /             |
   *    ↑   +----+ S             |
   *    s   |    |               |
   *    ↓   +----+---------------| A'
   *       O
   *        ← s →
   *        ←---- half_size -----→
   */

  Scalar ratio =
      radius == 0 ? kMaxRatio : std::min(half_size * 2 / radius, kMaxRatio);
  Scalar a = ratio * radius / 2;
  Scalar s = half_size - a;
  Scalar g = RoundSuperellipseParam::kGapFactor * radius;

  Scalar n = LerpPrecomputedVariable(0, ratio);
  Scalar sin_thetaJ = radius == 0 ? 0 : LerpPrecomputedVariable(1, ratio);

  Scalar sin_thetaJ_sq = sin_thetaJ * sin_thetaJ;
  Scalar cos_thetaJ_sq = 1 - sin_thetaJ_sq;
  Scalar tan_thetaJ_sq = sin_thetaJ_sq / cos_thetaJ_sq;

  Scalar xJ = a * pow(sin_thetaJ_sq, 1 / n);
  Scalar yJ = a * pow(cos_thetaJ_sq, 1 / n);
  Scalar tan_phiJ = pow(tan_thetaJ_sq, (n - 1) / n);
  Scalar d = (xJ - tan_phiJ * yJ) / (1 - tan_phiJ);
  Scalar R = (a - d - g) * sqrt(2);

  Point pointA{0, half_size};
  Point pointM{half_size - g, half_size - g};
  Point pointS{s, s};
  Point pointJ = Point{xJ, yJ} + pointS;
  Point circle_center =
      radius == 0 ? pointM : FindCircleCenter(pointJ, pointM, R);
  Radians circle_max_angle =
      radius == 0 ? Radians(0)
                  : (pointM - circle_center).AngleTo(pointJ - circle_center);

  return RoundSuperellipseParam::Octant{
      .offset = center,

      .edge_mid = pointA,

      .se_center = pointS,
      .se_a = a,
      .se_n = n,
      .se_max_theta = asin(sin_thetaJ),

      .ratio = ratio,

      .circle_start = pointJ,
      .circle_center = circle_center,
      .circle_max_angle = circle_max_angle,
  };
}

// Compute parameters for a quadrant of a rounded superellipse with asymmetrical
// radii.
//
// The `corner` is the coordinate of the corner point in the same coordinate
// space as `center`, which specifies both the half size of the bounding box and
// which quadrant the curve should be.
RoundSuperellipseParam::Quadrant ComputeQuadrant(Point center,
                                                 Point corner,
                                                 Size in_radii) {
  Point corner_vector = corner - center;
  Size radii = in_radii.Abs();

  // The prefix "norm" is short for "normalized".
  //
  // Be extra careful to avoid NaNs in cases that some coordinates of `in_radii`
  // or `corner_vector` are zero.
  Scalar norm_radius = radii.MinDimension();
  Size forward_scale = norm_radius == 0 ? Size{1, 1} : radii / norm_radius;
  Point norm_half_size = corner_vector.Abs() / forward_scale;
  Point signed_scale = ReplanceNaNWithOne(corner_vector / norm_half_size);

  // Each quadrant curve is composed of two octant curves, each of which belongs
  // to a square-like rounded rectangle. For the two octants to connect at the
  // circular arc, the centers these two square-like rounded rectangle must be
  // offset from the quadrant center by a same distance in different directions.
  // The distance is denoted as `c`.
  Scalar c = norm_half_size.x - norm_half_size.y;

  return RoundSuperellipseParam::Quadrant{
      .offset = center,
      .signed_scale = signed_scale,
      .top = ComputeOctant(Point{0, -c}, norm_half_size.x, norm_radius),
      .right = ComputeOctant(Point{c, 0}, norm_half_size.y, norm_radius),
  };
}

// Checks whether the given point is contained in the first octant of the given
// square-like rounded superellipse.
//
// The first octant refers to the region that spans from 0 to pi/4 starting from
// positive Y axis clockwise.
//
// If the point is not within this octant at all, then this function always
// returns true.  Otherwise this function returns whether the point is contained
// within the rounded superellipse.
//
// The `param.offset` is ignored. The input point should have been transformed
// to the coordinate space where the rounded superellipse is centered at the
// origin.
bool OctantContains(const RoundSuperellipseParam::Octant& param,
                    const Point& p) {
  // Check whether the point is within the octant.
  if (p.x < 0 || p.y < 0 || p.y < p.x) {
    return true;
  }
  // Check if the point is within the stretch segment.
  if (p.x <= param.se_center.x) {
    return p.y <= param.edge_mid.y;
  }
  // Check if the point is within the superellipsoid segment.
  if (p.x <= param.circle_start.x) {
    Point p_se = (p - param.se_center) / param.se_a;
    return powf(p_se.x, param.se_n) + powf(p_se.y, param.se_n) <= 1;
  }
  Scalar circle_radius =
      param.circle_start.GetDistanceSquared(param.circle_center);
  Point p_circle = p - param.circle_center;
  return p_circle.GetDistanceSquared(Point()) < circle_radius;
}

// Determine if p is inside the corner curve defined by the indicated corner
// param.
//
// The coordinates of p should be within the same coordinate space with
// `param.offset`.
//
// If `check_quadrant` is true, then this function first checks if the point is
// within the quadrant of given corner. If not, this function returns true,
// otherwise this method continues to check whether the point is contained in
// the rounded superellipse.
//
// If `check_quadrant` is false, then the first step above is skipped, and the
// function checks whether the absolute (relative to the center) coordinate of p
// is contained in the rounded superellipse.
bool CornerContains(const RoundSuperellipseParam::Quadrant& param,
                    const Point& p,
                    bool check_quadrant = true) {
  Point norm_point = (p - param.offset) / param.signed_scale;
  if (check_quadrant) {
    if (norm_point.x < 0 || norm_point.y < 0) {
      return true;
    }
  } else {
    norm_point = norm_point.Abs();
  }
  return OctantContains(param.top, norm_point - param.top.offset) &&
         OctantContains(param.right, Flip(norm_point - param.right.offset));
}

}  // namespace

RoundSuperellipseParam RoundSuperellipseParam::MakeBoundsRadii(
    const Rect& bounds_,
    const RoundingRadii& radii_) {
  if (radii_.AreAllCornersSame()) {
    return RoundSuperellipseParam{
        .top_right = ComputeQuadrant(bounds_.GetCenter(), bounds_.GetRightTop(),
                                     radii_.top_right),
        .all_corners_same = true,
    };
  }
  Scalar top_split = Split(bounds_.GetLeft(), bounds_.GetRight(),
                           radii_.top_left.width, radii_.top_right.width);
  Scalar right_split =
      Split(bounds_.GetTop(), bounds_.GetBottom(), radii_.top_right.height,
            radii_.bottom_right.height);
  Scalar bottom_split =
      Split(bounds_.GetLeft(), bounds_.GetRight(), radii_.bottom_left.width,
            radii_.bottom_right.width);
  Scalar left_split = Split(bounds_.GetTop(), bounds_.GetBottom(),
                            radii_.top_left.height, radii_.bottom_left.height);

  return RoundSuperellipseParam{
      .top_right = ComputeQuadrant(Point{top_split, right_split},
                                   bounds_.GetRightTop(), radii_.top_right),
      .bottom_right =
          ComputeQuadrant(Point{bottom_split, right_split},
                          bounds_.GetRightBottom(), radii_.bottom_right),
      .bottom_left =
          ComputeQuadrant(Point{bottom_split, left_split},
                          bounds_.GetLeftBottom(), radii_.bottom_left),
      .top_left = ComputeQuadrant(Point{top_split, left_split},
                                  bounds_.GetLeftTop(), radii_.top_left),
      .all_corners_same = false,
  };
}

bool RoundSuperellipseParam::Contains(const Point& point) const {
  if (all_corners_same) {
    return CornerContains(top_right, point, /*check_quadrant=*/false);
  }
  return CornerContains(top_right, point) &&
         CornerContains(bottom_right, point) &&
         CornerContains(bottom_left, point) && CornerContains(top_left, point);
}

void RoundSuperellipseParam::SuperellipseBezierArc(
    Point* output,
    const RoundSuperellipseParam::Octant& param) {
  Point start = {param.se_center.x, param.edge_mid.y};
  const Point& end = param.circle_start;
  constexpr Point start_tangent = {1, 0};
  Point circle_start_vector = param.circle_start - param.circle_center;
  Point end_tangent =
      Point{-circle_start_vector.y, circle_start_vector.x}.Normalize();

  Scalar start_factor = LerpPrecomputedVariable(0, param.ratio);
  Scalar end_factor = LerpPrecomputedVariable(1, param.ratio);

  output[0] = start;
  output[1] = start + start_tangent * start_factor * param.se_a;
  output[2] = end + end_tangent * end_factor * param.se_a;
  output[3] = end;
}

}  // namespace impeller
