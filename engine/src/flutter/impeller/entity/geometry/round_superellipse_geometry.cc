// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <cmath>

#include "flutter/impeller/entity/geometry/round_superellipse_geometry.h"

#include "impeller/geometry/constants.h"

namespace impeller {

namespace {
// A look up table with precomputed variables.
//
// The columns represent the following variabls respectively:
//
//  * ratio = size / a
//  * n
//  * d / a
//  * thetaJ
//
// For definition of the variables, see DrawOctantSquareLikeSquircle.
constexpr Scalar kPrecomputedVariables[][4] = {
    {2.000, 2.00000, 0.00000, 0.24040},  //
    {2.020, 2.03340, 0.01447, 0.24040},  //
    {2.040, 2.06540, 0.02575, 0.21167},  //
    {2.060, 2.09800, 0.03668, 0.20118},  //
    {2.080, 2.13160, 0.04719, 0.19367},  //
    {2.100, 2.17840, 0.05603, 0.16233},  //
    {2.120, 2.19310, 0.06816, 0.20020},  //
    {2.140, 2.22990, 0.07746, 0.19131},  //
    {2.160, 2.26360, 0.08693, 0.19008},  //
    {2.180, 2.30540, 0.09536, 0.17935},  //
    {2.200, 2.32900, 0.10541, 0.19136},  //
    {2.220, 2.38330, 0.11237, 0.17130},  //
    {2.240, 2.39770, 0.12271, 0.18956},  //
    {2.260, 2.41770, 0.13251, 0.20254},  //
    {2.280, 2.47180, 0.13879, 0.18454},  //
    {2.300, 2.50910, 0.14658, 0.18261}   //
};

constexpr size_t kNumRecords =
    sizeof(kPrecomputedVariables) / sizeof(kPrecomputedVariables[0]);
constexpr Scalar kMinRatio = kPrecomputedVariables[0][0];
constexpr Scalar kMaxRatio = kPrecomputedVariables[kNumRecords - 1][0];
constexpr Scalar kRatioStep =
    kPrecomputedVariables[1][0] - kPrecomputedVariables[0][0];

// Linear interpolation for `kPrecomputedVariables`.
//
// The `column` is a 0-based index that decides the target variable, where 1
// corresponds to the 2nd element of each row, etc.
//
// The `ratio` corresponds to column 0, on which the lerp is calculated.
Scalar LerpPrecomputedVariable(size_t column, Scalar ratio) {
  Scalar steps =
      std::clamp<Scalar>((ratio - kMinRatio) / kRatioStep, 0, kNumRecords - 1);
  size_t left = std::clamp<size_t>(static_cast<size_t>(std::floor(steps)), 0,
                                   kNumRecords - 2);
  Scalar frac = steps - left;

  return (1 - frac) * kPrecomputedVariables[left][column] +
         frac * kPrecomputedVariables[left + 1][column];
}

// Return the shortest of `corner_radius`, height/2, and width/2.
//
// Corner radii longer than 1/2 of the side length does not make sense, and will
// be limited to the longest possible.
Scalar LimitRadius(Scalar corner_radius, const Rect& bounds) {
  return std::min(corner_radius,
                  std::min(bounds.GetWidth() / 2, bounds.GetHeight() / 2));
}

// The max angular step that the algorithm will traverse a quadrant of the
// curve.
//
// This limits the max number of points of the curve.
constexpr Scalar kMaxQuadrantSteps = 40;

// Calculates the angular step size for a smooth curve.
//
// Returns the angular step needed to ensure a curve appears smooth
// based on the smallest dimension of a shape. Smaller dimensions require
// larger steps as less detail is needed for smoothness.
//
// The `minDimension` is the smallest dimension (e.g., width or height) of the
// shape.
//
// The `fullAngle` is the total angular range to traverse.
Scalar CalculateStep(Scalar minDimension, Scalar fullAngle) {
  constexpr Scalar kMinAngleStep = kPiOver2 / kMaxQuadrantSteps;

  // Assumes at least 1 point is needed per pixel to achieve sufficient
  // smoothness.
  constexpr Scalar pointsPerPixel = 1.0;
  size_t pointsByDimension =
      static_cast<size_t>(std::ceil(minDimension * pointsPerPixel));
  Scalar angleByDimension = fullAngle / pointsByDimension;

  return std::min(kMinAngleStep, angleByDimension);
}

// The distance from point M (the 45deg point) to either side of the closer
// bounding box is defined as `CalculateGap`.
constexpr Scalar CalculateGap(Scalar corner_radius) {
  // Heuristic formula derived from experimentation.
  return 0.2924066406 * corner_radius;
}

// Draw a circular arc from `start` to `end` with a radius of `r`.
//
// It is assumed that `start` is north-west to `end`, and the center
// of the circle is south-west to both points.
//
// The resulting points are appended to `output` and include the starting point
// but exclude the ending point.
//
// Returns the number of the
size_t DrawCircularArc(Point* output, Point start, Point end, Scalar r) {
  /* Denote the middle point of S and E as M. The key is to find the center of
   * the circle.
   *         S --__
   *          /  ⟍ `、
   *         /   M  ⟍\
   *        /       ⟋  E
   *       /     ⟋   ↗
   *      /   ⟋
   *     / ⟋    r
   *  C ᜱ  ↙
   */

  Point s_to_e = end - start;
  Point m = (start + end) / 2;
  Point c_to_m = Point(-s_to_e.y, s_to_e.x);
  Scalar distance_sm = s_to_e.GetLength() / 2;
  Scalar distance_cm = sqrt(r * r - distance_sm * distance_sm);
  Point c = m - distance_cm * c_to_m.Normalize();
  Scalar angle_sce = asinf(distance_sm / r) * 2;
  Point c_to_s = start - c;

  Scalar step = CalculateStep(std::abs(s_to_e.y), angle_sce);

  Point* next = output;
  Scalar angle = 0;
  while (angle < angle_sce) {
    *(next++) = c_to_s.Rotate(Radians(-angle)) + c;
    angle += step;
  }
  return next - output;
}

// Draws an arc representing the top 1/8 segment of a square-like rounded
// superellipse.
//
// The resulting arc centers at the origin, spanning from 0 to pi/4, moving
// clockwise starting from the positive Y-axis, and includes the starting point
// (the middle of the top flat side) while excluding the ending point (the x=y
// point).
//
// The full square-like rounded superellipse has a width and height specified by
// `size` and features rounded corners determined by `corner_radius`. The
// `corner_radius` corresponds to the `cornerRadius` parameter in SwiftUI,
// rather than the literal radius of corner circles.
//
// Returns the number of points generated.
size_t DrawOctantSquareLikeSquircle(Point* output,
                                    Scalar size,
                                    Scalar corner_radius) {
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
   *    ↑   +----+               |
   *    s   |    |               |
   *    ↓   +----+---------------| A'
   *       O     S
   *        ← s →
   *        ←------ size/2 ------→
   *
   * Define gap (g) as the distance between point M and the bounding box,
   * therefore point M is at (size/2 - g, size/2 - g).
   *
   * The superellipsoid curve can be drawn with an implicit parameter θ:
   *   x = a * sinθ ^ (2/n)
   *   y = a * cosθ ^ (2/n)
   * https://math.stackexchange.com/questions/2573746/superellipse-parametric-equation
   *
   * Define thetaJ as the θ at point J.
   */

  Scalar ratio = {std::min(size / corner_radius, kMaxRatio)};
  Scalar a = ratio * corner_radius / 2;
  Scalar s = size / 2 - a;
  Scalar g = CalculateGap(corner_radius);

  Scalar n = LerpPrecomputedVariable(1, ratio);
  Scalar d = LerpPrecomputedVariable(2, ratio) * a;
  Scalar thetaJ = LerpPrecomputedVariable(3, ratio);

  Scalar R = (a - d - g) * sqrt(2);

  Point pointM(size / 2 - g, size / 2 - g);

  Scalar xJ = a * pow(abs(sinf(thetaJ)), 2 / n);
  Scalar yJ = a * pow(abs(cosf(thetaJ)), 2 / n);

  Point* next = output;
  // A
  *(next++) = Point(0, size / 2);
  // Superellipsoid arc BJ (B inclusive, J exclusive)
  {
    Scalar step = CalculateStep(a - yJ, thetaJ);
    Scalar angle = 0;
    while (angle < thetaJ) {
      Scalar x = a * pow(abs(sinf(angle)), 2 / n);
      Scalar y = a * pow(abs(cosf(angle)), 2 / n);
      *(next++) = Point(x + s, y + s);
      angle += step;
    }
  }

  // Circular arc JM (B inclusive, M exclusive)
  next += DrawCircularArc(next, {xJ + s, yJ + s}, pointM, R);
  return next - output;
}

// Optionally `flip` the input points before offsetting it by `center`, and
// append the result to `output`.
//
// If `flip` is true, then the entire input list is reversed, and the x and y
// coordinate of each point is swapped as well. This effectively mirrors the
// input point list by the y=x line.
size_t FlipAndOffset(Point* output,
                     const Point* input,
                     size_t input_length,
                     bool flip,
                     const Point& center) {
  if (!flip) {
    for (size_t i = 0; i < input_length; i++) {
      output[i] = input[i] + center;
    }
  } else {
    for (size_t i = 0; i < input_length; i++) {
      const Point& point = input[input_length - i - 1];
      output[i] = Point(point.y + center.x, point.x + center.y);
    }
  }
  return input_length;
}

constexpr Point kReflection[4] = {{1, 1}, {1, -1}, {-1, -1}, {-1, 1}};

// Mirror the point list `quad` into other quadrants and output as a triangle
// strip.
//
// The input arc `quad` should reside in the first quadrant, starting at
// positive Y axis and ending at positive X axis (both ends inclusive), for a
// total of `quad_length` points. This function mirrors the arc into 4
// quadrants, offset the result by `center`, and rearrange it as a triangle
// strip, which is appended to `output`.
//
// A total of (quad_length - 1) * 4 points will be appended, and `output` must
// have sufficient memory allocated before this call.
void MirrorIntoTriangleStrip(const Point* quad,
                             size_t quad_length,
                             const Point& center,
                             Point* output) {
  // The length of 1/4 arc including the starting point but excluding the
  // ending point.
  const size_t arc_length = quad_length - 1;
  auto GetPoint = [quad, arc_length](size_t i) -> Point {
    if (i < arc_length) {
      return quad[i];
    }
    i = i - arc_length;
    if (i < arc_length) {
      return quad[arc_length - i] * kReflection[1];
    }
    i = i - arc_length;
    if (i < arc_length) {
      return quad[i] * kReflection[2];
    }
    i = i - arc_length;
    if (i < arc_length) {
      return quad[arc_length - i] * kReflection[3];
    } else {
      // Unreachable
      return Point();
    }
  };

  size_t index_count = 0;

  output[index_count++] = GetPoint(0) + center;

  size_t a = 1;
  size_t b = arc_length * 4 - 1;
  while (a < b) {
    output[index_count++] = GetPoint(a) + center;
    output[index_count++] = GetPoint(b) + center;
    a++;
    b--;
  }
  if (a == b) {
    output[index_count++] = GetPoint(b) + center;
  }
}

}  // namespace

RoundSuperellipseGeometry::RoundSuperellipseGeometry(const Rect& bounds,
                                                     Scalar corner_radius)
    : bounds_(bounds), corner_radius_(LimitRadius(corner_radius, bounds)) {}

RoundSuperellipseGeometry::~RoundSuperellipseGeometry() {}

GeometryResult RoundSuperellipseGeometry::GetPositionBuffer(
    const ContentContext& renderer,
    const Entity& entity,
    RenderPass& pass) const {
  const Size size = bounds_.GetSize();
  const Point center = bounds_.GetCenter();

  // The full shape is divided into 4 segments: the top and bottom edges come
  // from two square-like rounded superellipses (called "width-aligned"), while
  // the left and right squircles come from another two ("height-aligned").
  //
  // Denote the distance from the center of the square-like squircles to the
  // origin as `c`. The width-aligned square-like squircle and the
  // height-aligned one have the same offset in different directions.
  const Scalar c = (size.width - size.height) / 2;

  // The cache is allocated as follows:
  //
  //  * The first chunk stores the quadrant arc.
  //  * The second chunk stores an octant arc before flipping and translation.
  Point* cache = renderer.GetTessellator().GetStrokePointCache().data();

  // The memory size (in units of Points) allocated to store the first chunk.
  constexpr size_t kMaxQuadrantLength = kPointArenaSize / 4;
  // Since the curve is traversed in steps bounded by kMaxQuadrantSteps, the
  // curving part will have fewer points than kMaxQuadrantSteps. Multiply it by
  // 2 for storing other sporatic points (an extremely conservative estimate).
  static_assert(kMaxQuadrantLength > 2 * kMaxQuadrantSteps);

  // Draw the first quadrant of the shape and store in `quadrant`, including
  // both ends. It will be mirrored to other quadrants later.
  Point* quadrant = cache;
  size_t quadrant_length;
  {
    Point* next = quadrant;

    Point* octant_cache = cache + kMaxQuadrantLength;
    size_t octant_length;

    octant_length =
        DrawOctantSquareLikeSquircle(octant_cache, size.width, corner_radius_);
    next += FlipAndOffset(next, octant_cache, octant_length, /*flip=*/false,
                          Point(0, -c));

    *(next++) = Point(size / 2) - CalculateGap(corner_radius_);  // Point M

    octant_length =
        DrawOctantSquareLikeSquircle(octant_cache, size.height, corner_radius_);
    next += FlipAndOffset(next, octant_cache, octant_length, /*flip=*/true,
                          Point(c, 0));

    quadrant_length = next - quadrant;
  }

  // The `contour_point_count` include all points on the border. The "-1" comes
  // from duplicate ends from the mirrored arcs.
  size_t contour_length = 4 * (quadrant_length - 1);
  BufferView vertex_buffer = renderer.GetTransientsBuffer().Emplace(
      nullptr, sizeof(Point) * contour_length, alignof(Point));
  Point* vertex_data =
      reinterpret_cast<Point*>(vertex_buffer.GetBuffer()->OnGetContents() +
                               vertex_buffer.GetRange().offset);

  MirrorIntoTriangleStrip(quadrant, quadrant_length, center, vertex_data);

  return GeometryResult{
      .type = PrimitiveType::kTriangleStrip,
      .vertex_buffer =
          {
              .vertex_buffer = vertex_buffer,
              .vertex_count = contour_length,
              .index_type = IndexType::kNone,
          },
      .transform = entity.GetShaderTransform(pass),
  };
}

std::optional<Rect> RoundSuperellipseGeometry::GetCoverage(
    const Matrix& transform) const {
  return bounds_.TransformBounds(transform);
}

bool RoundSuperellipseGeometry::CoversArea(const Matrix& transform,
                                           const Rect& rect) const {
  if (!transform.IsTranslationScaleOnly()) {
    return false;
  }
  // Use the rectangle formed by the four 45deg points (point M) as a
  // conservative estimate of the inner rectangle.
  Scalar g = CalculateGap(corner_radius_);
  Rect coverage =
      Rect::MakeLTRB(bounds_.GetLeft() + g, bounds_.GetTop() + g,
                     bounds_.GetRight() - g, bounds_.GetBottom() - g)
          .TransformBounds(transform);
  return coverage.Contains(rect);
}

bool RoundSuperellipseGeometry::IsAxisAlignedRect() const {
  return false;
}

}  // namespace impeller
