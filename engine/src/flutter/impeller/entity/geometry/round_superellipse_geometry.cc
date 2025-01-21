// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <cmath>
#include <variant>

#include "flutter/impeller/entity/geometry/round_superellipse_geometry.h"

#include "impeller/geometry/constants.h"

namespace impeller {

namespace {

// An interface for classes that arranges a point list that forms a convex
// contour into a triangle strip.
class ConvexRearranger {
 public:
  ConvexRearranger() {}

  virtual ~ConvexRearranger() {}

  virtual size_t ContourLength() const = 0;

  virtual Point GetPoint(size_t i) const = 0;

  void RearrangeIntoTriangleStrip(Point* output) {
    size_t index_count = 0;

    output[index_count++] = GetPoint(0);

    size_t a = 1;
    size_t contour_length = ContourLength();
    size_t b = contour_length - 1;
    while (a < b) {
      output[index_count++] = GetPoint(a);
      output[index_count++] = GetPoint(b);
      a++;
      b--;
    }
    if (a == b) {
      output[index_count++] = GetPoint(b);
    }
  }

 private:
  ConvexRearranger(const ConvexRearranger&) = delete;
  ConvexRearranger& operator=(const ConvexRearranger&) = delete;
};

// A convex rearranger whose contour is concatenated from 4 quadrant segments.
//
// The input quadrant curves must travel from the Y axis to the X axis, and
// include both ends. This means that the points on the axes are duplicate
// between segments, and will be omitted by this class.
class UnevenQuadrantsRearranger : public ConvexRearranger {
 public:
  UnevenQuadrantsRearranger(Point* cache, size_t segment_capacity)
      : cache_(cache), segment_capacity_(segment_capacity) {}

  Point* QuadCache(size_t i) { return cache_ + segment_capacity_ * i; }

  const Point* QuadCache(size_t i) const {
    return cache_ + segment_capacity_ * i;
  }

  size_t& QuadSize(size_t i) { return lengths_[i]; }

  size_t ContourLength() const override {
    return lengths_[0] + lengths_[1] + lengths_[2] + lengths_[3] - 4;
  }

  Point GetPoint(size_t i) const override {
    //   output            from       index
    //      0 ... l0-2    quads[0]   0    ... l0-2
    // next 0 ... l1-2    quads[1]   l1-1 ... 1
    // next 0 ... l2-2    quads[2]   0    ... l2-2
    // next 0 ... l3-2    quads[3]   l3-1 ... 1
    size_t high = lengths_[0] - 1;
    if (i < high) {
      return QuadCache(0)[i];
    }
    high += lengths_[1] - 1;
    if (i < high) {
      return QuadCache(1)[high - i];
    }
    size_t low = high;
    high += lengths_[2] - 1;
    if (i < high) {
      return QuadCache(2)[i - low];
    }
    high += lengths_[3] - 1;
    if (i < high) {
      return QuadCache(3)[high - i];
    } else {
      // Unreachable
      return Point();
    }
  }

 private:
  Point* cache_;
  size_t segment_capacity_;
  size_t lengths_[4];
};

// A convex rearranger whose contour is concatenated from 4 identical quadrant
// segments.
//
// The input curve must travel from the Y axis to the X axis and include both
// ends. This means that the points on the axes are duplicate between segments,
// and will be omitted by this class.
class MirroredQuadrantRearranger : public ConvexRearranger {
 public:
  MirroredQuadrantRearranger(Point center, Point* cache)
      : center_(center), cache_(cache) {}

  size_t& QuadSize() { return l_; }

  size_t ContourLength() const override { return l_ * 4 - 4; }

  Point GetPoint(size_t i) const override {
    //   output          from   index
    //      0 ... l-2    quad   0   ... l-2
    // next 0 ... l-2    quad   l-1 ... 1
    // next 0 ... l-2    quad   0   ... l-2
    // next 0 ... l-2    quad   l-1 ... 1
    size_t high = l_ - 1;
    if (i < high) {
      return cache_[i] + center_;
    }
    high += l_ - 1;
    if (i < high) {
      return cache_[high - i] * Point{1, -1} + center_;
    }
    size_t low = high;
    high += l_ - 1;
    if (i < high) {
      return cache_[i - low] * Point{-1, -1} + center_;
    }
    high += l_ - 1;
    if (i < high) {
      return cache_[high - i] * Point{-1, 1} + center_;
    } else {
      // Unreachable
      return Point();
    }
  }

 private:
  Point center_;
  Point* cache_;
  size_t l_ = 0;
};

// A matrix that swaps the coordinates of a point.
// clang-format off
constexpr Matrix kFlip = Matrix(
  0.0f, 1.0f, 0.0f, 0.0f,
  1.0f, 0.0f, 0.0f, 0.0f,
  0.0f, 0.0f, 1.0f, 0.0f,
  0.0f, 0.0f, 0.0f, 1.0f);
// clang-format on

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

// A factor used to calculate the "gap", defined as the distance from the
// midpoint of the curved corners to the nearest sides of the bounding box.
//
// When the corner radius is symmetrical on both dimensions, the midpoint of the
// corner is where the circular arc intersects its quadrant bisector. When the
// corner radius is asymmetrical, since the corner can be considered "elongated"
// from a symmetrical corner, the midpoint is transformed in the same way.
//
// Experiments indicate that the gap is linear with respect to the corner
// radius on that dimension.
//
// The formula should be kept in sync with a few files, as documented in
// `CalculateGap` in round_superellipse_geometry.cc.
constexpr Scalar kGapFactor = 0.2924066406;

// Return the value that splits the range from `left` to `right` into two
// portions whose ratio equals to `ratio_left` : `ratio_right`.
static Scalar Split(Scalar left,
                    Scalar right,
                    Scalar ratio_left,
                    Scalar ratio_right) {
  return (left * ratio_right + right * ratio_left) / (ratio_left + ratio_right);
}

// Draw a circular arc from `start` to `end` with a radius of `r`.
//
// It is assumed that `start` is north-west to `end`, and the center of the
// circle is south-west to both points. If `reverse` is true, then the curve
// goes from `end` to `start` instead.
//
// The resulting points, after applying `transform`, are appended to `output`
// and include the effective starting point but exclude the effective ending
// point.
//
// Returns the number of generated points.
size_t DrawCircularArc(Point* output,
                       Point start,
                       Point end,
                       Scalar r,
                       bool reverse,
                       const Matrix& transform) {
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
  Matrix full_transform = transform * Matrix::MakeTranslation(c);

  Point* next = output;
  Scalar angle = reverse ? angle_sce : 0.0f;
  Scalar step =
      (reverse ? -1 : 1) * CalculateStep(std::abs(s_to_e.y), angle_sce);
  Scalar end_angle = reverse ? 0.0f : angle_sce;

  while ((angle < end_angle) != reverse) {
    *(next++) = full_transform * c_to_s.Rotate(Radians(-angle));
    angle += step;
  }
  return next - output;
}

// Draw a superellipsoid arc.
//
// The superellipse is centered at the origin and has degree `n` and both
// semi-axes equal to `a`. The arc starts from positive Y axis and spans from 0
// to `max_theta` radiance clockwise if `reverse` is false, or from `max_theta`
// to 0 otherwise.
//
// The resulting points, after applying `transform`, are appended to `output`
// and include the starting point but exclude the ending point.
//
// Returns the number of generated points.
size_t DrawSuperellipsoidArc(Point* output,
                             Scalar a,
                             Scalar n,
                             Scalar max_theta,
                             bool reverse,
                             const Matrix& transform) {
  Point* next = output;
  Scalar angle = reverse ? max_theta : 0.0f;
  Scalar step =
      (reverse ? -1 : 1) *
      CalculateStep(a - a * pow(abs(cosf(max_theta)), 2 / n), max_theta);
  Scalar end = reverse ? 0.0f : max_theta;
  while ((angle < end) != reverse) {
    Scalar x = a * pow(abs(sinf(angle)), 2 / n);
    Scalar y = a * pow(abs(cosf(angle)), 2 / n);
    *(next++) = transform * Point(x, y);
    angle += step;
  }
  return next - output;
}

// Draws an arc representing the top 1/8 segment of a square-like rounded
// superellipse centered at the origin.
//
// The square-like rounded superellipse that this arc belongs to has a width and
// height specified by `size` and features rounded corners determined by
// `corner_radius`. The `corner_radius` corresponds to the `cornerRadius`
// parameter in SwiftUI, rather than the literal radius of corner circles.
//
// If `reverse` is false, the resulting arc spans from 0 (inclusive) to pi/4
// (exclusive), moving clockwise starting from the positive Y-axis. If `reverse`
// is true, the curve spans from pi/4 (inclusive) to 0 (inclusive)
// counterclockwise instead.
//
// Returns the number of points generated.
size_t DrawOctantSquareLikeSquircle(Point* output,
                                    Scalar size,
                                    Scalar corner_radius,
                                    bool reverse,
                                    const Matrix& transform) {
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
  Scalar g = kGapFactor * corner_radius;

  Scalar n = LerpPrecomputedVariable(1, ratio);
  Scalar d = LerpPrecomputedVariable(2, ratio) * a;
  Scalar thetaJ = LerpPrecomputedVariable(3, ratio);

  Scalar R = (a - d - g) * sqrt(2);

  Point pointA{0, size / 2};
  Point pointM{size / 2 - g, size / 2 - g};
  Point pointS{s, s};
  Point pointJ =
      Point{pow(abs(sinf(thetaJ)), 2 / n), pow(abs(cosf(thetaJ)), 2 / n)} * a +
      pointS;
  Matrix translationS = Matrix::MakeTranslation(pointS);

  Point* next = output;
  if (!reverse) {
    // Point A
    *(next++) = transform * pointA;
    // Arc [B, J)
    next += DrawSuperellipsoidArc(next, a, n, thetaJ, reverse,
                                  transform * translationS);
    // Arc [J, M)
    next += DrawCircularArc(next, pointJ, pointM, R, reverse, transform);
  } else {
    // Arc [M, J)
    next += DrawCircularArc(next, pointJ, pointM, R, reverse, transform);
    // Arc [J, B)
    next += DrawSuperellipsoidArc(next, a, n, thetaJ, reverse,
                                  transform * translationS);
    // Point B
    *(next++) = transform * Point{s, size / 2};
    // Point A
    *(next++) = transform * pointA;
  }
  return next - output;
}

// Draw a quadrant curve, both ends included.
//
// Returns the number of points.
//
// The eact quadrant is specified by the direction of `outer` relative to
// `center`. The curve goes from the X axis to the Y axis.
static size_t DrawQuadrant(Point* output,
                           Point center,
                           Point outer,
                           Size radii) {
  if (radii.width == 0 || radii.height == 0) {
    // Degrade to rectangle. (A zero radius causes error below.)
    output[0] = {center.x, outer.y};
    output[1] = outer;
    output[2] = {outer.x, center.y};
    return 3;
  }
  // Normalize sizes and radii into symmetrical radius by scaling the longer of
  // `radii` to the shorter. For example, to draw a RSE with size (200, 300)
  // and radii (20, 10), this function draws one with size (100, 300) and radii
  // (10, 10) and then scales it by (2x, 1x).
  Scalar norm_radius = radii.MinDimension();
  Size radius_scale = radii / norm_radius;
  Point signed_size = (outer - center) * 2;
  Point norm_size = signed_size.Abs() / radius_scale;
  Point signed_scale = signed_size / norm_size;

  // Each quadrant curve is composed of two octant curves, each of which belongs
  // to a square-like rounded rectangle. When `norm_size`'s width != height, the
  // centers of such square-like rounded rectangles are offset from the origin
  // by a distance denoted as `c`.
  Scalar c = (norm_size.x - norm_size.y) / 2;

  Point* next = output;

  next += DrawOctantSquareLikeSquircle(
      next, norm_size.x, norm_radius, /*reverse=*/false,
      Matrix::MakeTranslateScale(signed_scale, center) *
          Matrix::MakeTranslation(Size{0, -c}));

  next += DrawOctantSquareLikeSquircle(
      next, norm_size.y, norm_radius, /*reverse=*/true,
      Matrix::MakeTranslateScale(signed_scale, center) *
          Matrix::MakeTranslation(Size{c, 0}) * kFlip);

  return next - output;
}

}  // namespace

RoundSuperellipseGeometry::RoundSuperellipseGeometry(const Rect& bounds,
                                                     const RoundingRadii& radii)
    : bounds_(bounds.GetPositive()), radii_(radii.Scaled(bounds_)) {}

RoundSuperellipseGeometry::RoundSuperellipseGeometry(const Rect& bounds,
                                                     float corner_radius)
    : RoundSuperellipseGeometry(bounds,
                                RoundingRadii::MakeRadius(corner_radius)) {}

RoundSuperellipseGeometry::~RoundSuperellipseGeometry() {}

GeometryResult RoundSuperellipseGeometry::GetPositionBuffer(
    const ContentContext& renderer,
    const Entity& entity,
    RenderPass& pass) const {
  Point* cache = renderer.GetTessellator().GetStrokePointCache().data();

  // The memory size (in units of Points) allocated to store each quadrants.
  constexpr size_t kMaxQuadSize = kPointArenaSize / 4;
  // Since the curve is traversed in steps bounded by kMaxQuadrantSteps, the
  // curving part will have fewer points than kMaxQuadrantSteps. Multiply it by
  // 2 for storing other sporatic points (an extremely conservative estimate).
  static_assert(kMaxQuadSize > 2 * kMaxQuadrantSteps);

  ConvexRearranger* rearranger;
  std::variant<std::monostate, MirroredQuadrantRearranger,
               UnevenQuadrantsRearranger>
      rearranger_holder;

  if (radii_.AreAllCornersSame()) {
    rearranger_holder.emplace<MirroredQuadrantRearranger>(bounds_.GetCenter(),
                                                          cache);
    auto& t = std::get<MirroredQuadrantRearranger>(rearranger_holder);
    rearranger = &t;

    // The quadrant must be drawn at the origin so that it can be rotated later.
    t.QuadSize() = DrawQuadrant(cache, Point(),
                                bounds_.GetRightTop() - bounds_.GetCenter(),
                                radii_.top_right);
  } else {
    rearranger_holder.emplace<UnevenQuadrantsRearranger>(cache, kMaxQuadSize);
    auto& t = std::get<UnevenQuadrantsRearranger>(rearranger_holder);
    rearranger = &t;

    Scalar top_split = Split(bounds_.GetLeft(), bounds_.GetRight(),
                             radii_.top_left.width, radii_.top_right.width);
    Scalar right_split =
        Split(bounds_.GetTop(), bounds_.GetBottom(), radii_.top_right.height,
              radii_.bottom_right.height);
    Scalar bottom_split =
        Split(bounds_.GetLeft(), bounds_.GetRight(), radii_.bottom_left.width,
              radii_.bottom_right.width);
    Scalar left_split =
        Split(bounds_.GetTop(), bounds_.GetBottom(), radii_.top_left.height,
              radii_.bottom_left.height);

    t.QuadSize(0) = DrawQuadrant(t.QuadCache(0), Point{top_split, right_split},
                                 bounds_.GetRightTop(), radii_.top_right);
    t.QuadSize(1) =
        DrawQuadrant(t.QuadCache(1), Point{bottom_split, right_split},
                     bounds_.GetRightBottom(), radii_.bottom_right);
    t.QuadSize(2) =
        DrawQuadrant(t.QuadCache(2), Point{bottom_split, left_split},
                     bounds_.GetLeftBottom(), radii_.bottom_left);
    t.QuadSize(3) = DrawQuadrant(t.QuadCache(3), Point{top_split, left_split},
                                 bounds_.GetLeftTop(), radii_.top_left);
  }

  size_t contour_length = rearranger->ContourLength();
  BufferView vertex_buffer = renderer.GetTransientsBuffer().Emplace(
      nullptr, sizeof(Point) * contour_length, alignof(Point));
  Point* vertex_data =
      reinterpret_cast<Point*>(vertex_buffer.GetBuffer()->OnGetContents() +
                               vertex_buffer.GetRange().offset);
  rearranger->RearrangeIntoTriangleStrip(vertex_data);

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
  Scalar left_inset = std::max(radii_.top_left.width, radii_.bottom_left.width);
  Scalar right_inset =
      std::max(radii_.top_right.width, radii_.bottom_right.width);
  Scalar top_inset = std::max(radii_.top_left.height, radii_.top_right.height);
  Scalar bottom_inset =
      std::max(radii_.bottom_left.height, radii_.bottom_right.height);
  Rect coverage =
      Rect::MakeLTRB(bounds_.GetLeft() + left_inset * kGapFactor,
                     bounds_.GetTop() + top_inset * kGapFactor,
                     bounds_.GetRight() - right_inset * kGapFactor,
                     bounds_.GetBottom() - bottom_inset * kGapFactor);
  return coverage.TransformBounds(transform).Contains(rect);
}

bool RoundSuperellipseGeometry::IsAxisAlignedRect() const {
  return false;
}

}  // namespace impeller
