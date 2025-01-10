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

size_t DrawSuperellipsoidArc(Point* output,
                             Point center,
                             Scalar a,
                             Scalar n,
                             Scalar max_theta) {
  Scalar step =
      CalculateStep(a - a * pow(abs(cosf(max_theta)), 2 / n), max_theta);

  Point* next = output;
  Scalar angle = 0;
  while (angle < max_theta) {
    Scalar x = a * pow(abs(sinf(angle)), 2 / n);
    Scalar y = a * pow(abs(cosf(angle)), 2 / n);
    *(next++) = Point(x, y) + center;
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

  Point center = {s, s};

  Point* next = output;
  // A
  *(next++) = Point(0, size / 2);

  // Superellipsoid arc BJ (B inclusive, J exclusive)
  next += DrawSuperellipsoidArc(next, center, a, n, thetaJ);

  // Circular arc JM (B inclusive, M exclusive)
  next += DrawCircularArc(next, next[-1], pointM, R);
  return next - output;
}

// Optionally `flip` the input points before transforming it with `transform`,
// and append the result to `output`.
//
// If `flip` is true, then the entire input list is reversed, and the x and y
// coordinate of each point is swapped as well. This effectively mirrors the
// input point list by the y=x line.
size_t FlipAndTransform(Point* output,
                        const Point* input,
                        size_t input_length,
                        bool flip,
                        const Matrix& transform) {
  if (!flip) {
    for (size_t i = 0; i < input_length; i++) {
      output[i] = transform * input[i];
    }
  } else {
    for (size_t i = 0; i < input_length; i++) {
      const Point& point = input[input_length - i - 1];
      output[i] = transform * Point(point.y, point.x);
    }
  }
  return input_length;
}

// Draw a quadrant curve, both ends included.
//
// Returns the number of points.
//
// The eact quadrant is specified by the direction of `outer` relative to
// `center`. The curve goes from the X axis to the Y axis.
static size_t DrawQuadrant(Point* output,
                           Point* octant_cache,
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
  size_t octant_length;

  octant_length =
      DrawOctantSquareLikeSquircle(octant_cache, norm_size.x, norm_radius);
  next += FlipAndTransform(next, octant_cache, octant_length, /*flip=*/false,
                           Matrix::MakeTranslateScale(signed_scale, center) *
                               Matrix::MakeTranslation(Size{0, -c}));

  *(next++) = Point(outer) -
              CalculateGap(norm_radius) * signed_scale;  // Middle of corner

  octant_length =
      DrawOctantSquareLikeSquircle(octant_cache, norm_size.y, norm_radius);
  next += FlipAndTransform(next, octant_cache, octant_length, /*flip=*/true,
                           Matrix::MakeTranslateScale(signed_scale, center) *
                               Matrix::MakeTranslation(Size{c, 0}));

  return next - output;
}

static inline void NormalizeEmptyToZero(Size& radii) {
  if (radii.IsEmpty()) {
    radii = Size();
  }
}

static inline void AdjustScale(Scalar& radius1,
                               Scalar& radius2,
                               Scalar dimension,
                               Scalar& scale) {
  FML_DCHECK(radius1 >= 0.0f && radius2 >= 0.0f);
  FML_DCHECK(dimension > 0.0f);
  if (radius1 + radius2 > dimension) {
    scale = std::min(scale, dimension / (radius1 + radius2));
  }
}

RoundingRadii LimitRadii(const Rect& bounds, const RoundingRadii& in_radii) {
  if (bounds.IsEmpty() || !bounds.IsFinite() ||  //
      in_radii.AreAllCornersEmpty() || !in_radii.IsFinite()) {
    // preserve the empty bounds as they might be strokable
    return RoundingRadii();
  }

  // Copy the incoming radii so that we can work on normalizing them to the
  // particular rectangle they are paired with without disturbing the caller.
  RoundingRadii radii = in_radii;

  // If any corner is flat or has a negative value, normalize it to zeros
  // We do this first so that the unnecessary non-flat part of that radius
  // does not contribute to the global scaling below.
  NormalizeEmptyToZero(radii.top_left);
  NormalizeEmptyToZero(radii.top_right);
  NormalizeEmptyToZero(radii.bottom_left);
  NormalizeEmptyToZero(radii.bottom_right);

  // Now determine a global scale to apply to all of the radii to ensure
  // that none of the adjacent pairs of radius values sum to larger than
  // the corresponding dimension of the rectangle.
  Size size = bounds.GetSize();
  Scalar scale = 1.0f;
  // clang-format off
  AdjustScale(radii.top_left.width,    radii.top_right.width,     size.width,
              scale);
  AdjustScale(radii.bottom_left.width, radii.bottom_right.width,  size.width,
              scale);
  AdjustScale(radii.top_left.height,   radii.bottom_left.height,  size.height,
              scale);
  AdjustScale(radii.top_right.height,  radii.bottom_right.height, size.height,
              scale);
  // clang-format on
  if (scale < 1.0f) {
    radii = radii * scale;
  }

  return radii;
}

// A class that arranges a point list that forms a convex contour into a
// triangle strip.
class ConvexTessellator {
 public:
  ConvexTessellator() {}

  virtual ~ConvexTessellator() {}

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
};

// A convex tessellator whose contour is concatenated from 4 quadrant segments.
//
// All quadrant curves travel from the Y axis to the X axis, and include both
// ends. This means that the points on the axes are duplicate between segments,
// and will be omitted by this class.
class UnevenQuadrantsTessellator : public ConvexTessellator {
 public:
  UnevenQuadrantsTessellator(Point* cache, size_t segment_capacity)
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

// A convex tessellator whose contour is concatenated from 4 identical quadrant
// segments.
//
// The input curve must travel from the Y axis to the X axis and include both
// ends. This means that the points on the axes are duplicate between segments,
// and will be omitted by this class.
class MirroredQuadrantTessellator : public ConvexTessellator {
 public:
  MirroredQuadrantTessellator(Point center, Point* cache)
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

}  // namespace

RoundSuperellipseGeometry::RoundSuperellipseGeometry(const Rect& bounds,
                                                     const RoundingRadii& radii)
    : bounds_(bounds), radii_(LimitRadii(bounds, radii)) {}

RoundSuperellipseGeometry::RoundSuperellipseGeometry(const Rect& bounds,
                                                     float corner_radius)
    : RoundSuperellipseGeometry(bounds,
                                RoundingRadii::MakeRadius(corner_radius)) {}

RoundSuperellipseGeometry::~RoundSuperellipseGeometry() {}

GeometryResult RoundSuperellipseGeometry::GetPositionBuffer(
    const ContentContext& renderer,
    const Entity& entity,
    RenderPass& pass) const {
  // The cache is allocated as follows:
  //
  //  * The first chunk stores the quadrant arc.
  //  * The second chunk stores an octant arc before flipping and translation.
  Point* cache = renderer.GetTessellator().GetStrokePointCache().data();

  // The memory size (in units of Points) allocated to store the first chunk.
  constexpr size_t kMaxQuadSize = kPointArenaSize / 5;
  // Since the curve is traversed in steps bounded by kMaxQuadrantSteps, the
  // curving part will have fewer points than kMaxQuadrantSteps. Multiply it by
  // 2 for storing other sporatic points (an extremely conservative estimate).
  static_assert(kMaxQuadSize > 2 * kMaxQuadrantSteps);

  Point* octant_cache = cache + kMaxQuadSize * 4;

  // === SPLITS ===

  ConvexTessellator* tessellator;
  std::variant<std::monostate, MirroredQuadrantTessellator,
               UnevenQuadrantsTessellator>
      tessellator_holder;

  if (radii_.AreAllCornersSame()) {
    tessellator_holder.emplace<MirroredQuadrantTessellator>(bounds_.GetCenter(),
                                                            cache);
    MirroredQuadrantTessellator& t =
        std::get<MirroredQuadrantTessellator>(tessellator_holder);
    tessellator = &t;

    // The quadrant must be drawn at the origin so that it can be rotated later.
    t.QuadSize() = DrawQuadrant(cache, octant_cache, Point(),
                                bounds_.GetRightTop() - bounds_.GetCenter(),
                                radii_.top_right);
  } else {
    tessellator_holder.emplace<UnevenQuadrantsTessellator>(cache, kMaxQuadSize);
    UnevenQuadrantsTessellator& t =
        std::get<UnevenQuadrantsTessellator>(tessellator_holder);
    tessellator = &t;

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

    t.QuadSize(0) = DrawQuadrant(t.QuadCache(0), octant_cache,
                                 Point{top_split, right_split},
                                 bounds_.GetRightTop(), radii_.top_right);
    t.QuadSize(1) = DrawQuadrant(t.QuadCache(1), octant_cache,
                                 Point{bottom_split, right_split},
                                 bounds_.GetRightBottom(), radii_.bottom_right);
    t.QuadSize(2) = DrawQuadrant(t.QuadCache(2), octant_cache,
                                 Point{bottom_split, left_split},
                                 bounds_.GetLeftBottom(), radii_.bottom_left);
    t.QuadSize(3) =
        DrawQuadrant(t.QuadCache(3), octant_cache, Point{top_split, left_split},
                     bounds_.GetLeftTop(), radii_.top_left);
  }

  size_t contour_length = tessellator->ContourLength();
  BufferView vertex_buffer = renderer.GetTransientsBuffer().Emplace(
      nullptr, sizeof(Point) * contour_length, alignof(Point));
  Point* vertex_data =
      reinterpret_cast<Point*>(vertex_buffer.GetBuffer()->OnGetContents() +
                               vertex_buffer.GetRange().offset);
  tessellator->RearrangeIntoTriangleStrip(vertex_data);

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
  if (!radii_.AreAllCornersSame() || !radii_.top_left.IsSquare()) {
    // TODO(dkwingsmt): At least try some estimates here.
    return false;
  }
  // Use the rectangle formed by the four 45deg points (point M) as a
  // conservative estimate of the inner rectangle.
  Scalar g = CalculateGap(radii_.top_left.width);
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
