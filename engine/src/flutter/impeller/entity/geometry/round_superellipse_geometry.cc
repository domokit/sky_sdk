// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <cmath>
#include <variant>

#include "flutter/impeller/entity/geometry/round_superellipse_geometry.h"
#include "flutter/impeller/geometry/round_superellipse_param.h"

#include "impeller/geometry/constants.h"

namespace impeller {

namespace {

constexpr auto kGapFactor = RoundSuperellipseParam::kGapFactor;

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

// Draw a superellipsoid arc.
//
// The superellipse is centered at the origin and has degree `n` and both
// semi-axes equal to `a`. The arc starts from positive Y axis and spans from 0
// to `max_theta` radiance clockwise if `reverse` is false, or from `max_theta`
// to 0 otherwise.
//
// The resulting points, transformed by `transform`, are appended to `output`.
// The starting point is included, but the ending point is excluded.
//
// Returns the number of points generated.
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

// Draws a circular arc centered at the origin with a radius of `r`, starting at
// `start`, and spanning `max_angle` clockwise.
//
// If `reverse` is false, points are generated from `start` to `start +
// max_angle`.  If `reverse` is true, points are generated from `start +
// max_angle` back to `start`.
//
// The generated points, transformed by `transform`, are appended to `output`.
// The starting point is included, but the ending point is excluded.
//
// Returns the number of points generated.
size_t DrawCircularArc(Point* output,
                       Point start,
                       Scalar max_angle,
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

  Point end = start.Rotate(Radians(-max_angle));

  Point* next = output;
  Scalar angle = reverse ? max_angle : 0.0f;
  Scalar step =
      (reverse ? -1 : 1) * CalculateStep(std::abs(start.y - end.y), max_angle);
  Scalar end_angle = reverse ? 0.0f : max_angle;

  while ((angle < end_angle) != reverse) {
    *(next++) = transform * start.Rotate(Radians(-angle));
    angle += step;
  }
  return next - output;
}

// Draws an arc representing the top 1/8 segment of a square-like rounded
// superellipse centered at the origin.
//
// If `reverse_and_flip` is false, the resulting arc spans from 0 (inclusive) to
// pi/4 (exclusive), moving clockwise starting from the positive Y-axis. If
// `reverse` is true, the curve spans from pi/4 (inclusive) to 0 (inclusive)
// counterclockwise instead, and all points have their x and y coordinates
// flipped.
//
// Either way, each point is then transformed by `external_transform` and
// appended to `output`.
//
// Returns the number of points generated.
size_t DrawOctantSquareLikeSquircle(Point* output,
                                    const RoundSuperellipseParam::Octant& param,
                                    bool reverse_and_flip,
                                    const Matrix& external_transform) {
  Matrix transform = external_transform * Matrix::MakeTranslation(param.offset);
  if (reverse_and_flip) {
    transform = transform * kFlip;
  }

  /* The following figure shows the first quadrant of a square-like rounded
   * superellipse. The target arc consists of the "stretch" (AB), a
   * superellipsoid arc (BJ), and a circular arc (JM).
   *
   *     straight   superelipse
   *          ↓     ↓
   *        A    B       J    circular arc
   *        ---------...._   ↙
   *        |    |      /  `⟍ M (where y=x)
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
   */

  Point* next = output;
  if (!reverse_and_flip) {
    // Point A
    *(next++) = transform * param.edge_mid;
    // Arc [B, J)
    next += DrawSuperellipsoidArc(
        next, param.se_a, param.se_n, param.se_max_theta, reverse_and_flip,
        transform * Matrix::MakeTranslation(param.se_center));
    // Arc [J, M)
    next += DrawCircularArc(
        next, param.circle_start - param.circle_center,
        param.circle_max_angle.radians, reverse_and_flip,
        transform * Matrix::MakeTranslation(param.circle_center));
  } else {
    // Arc [M, J)
    next += DrawCircularArc(
        next, param.circle_start - param.circle_center,
        param.circle_max_angle.radians, reverse_and_flip,
        transform * Matrix::MakeTranslation(param.circle_center));
    // Arc [J, B)
    next += DrawSuperellipsoidArc(
        next, param.se_a, param.se_n, param.se_max_theta, reverse_and_flip,
        transform * Matrix::MakeTranslation(param.se_center));
    // Point B
    *(next++) = transform * (param.se_center + Point{0, param.se_a});
    // Point A
    *(next++) = transform * param.edge_mid;
  }
  return next - output;
}

// Draw a quadrant curve, both ends included.
//
// Returns the number of points.
static size_t DrawQuadrant(Point* output,
                           const RoundSuperellipseParam::Quadrant& param) {
  Point* next = output;
  auto transform = Matrix::MakeTranslateScale(param.signed_scale, param.offset);

  next += DrawOctantSquareLikeSquircle(next, param.top,
                                       /*reverse_and_flip=*/false, transform);

  next += DrawOctantSquareLikeSquircle(next, param.right,
                                       /*reverse_and_flip=*/true, transform);

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

  auto param = RoundSuperellipseParam::MakeBoundsRadii(bounds_, radii_);

  if (param.all_corners_same) {
    rearranger_holder.emplace<MirroredQuadrantRearranger>(bounds_.GetCenter(),
                                                          cache);
    auto& t = std::get<MirroredQuadrantRearranger>(rearranger_holder);
    rearranger = &t;

    // The quadrant must be drawn at the origin so that it can be rotated later.
    param.top_right.offset = Point();
    t.QuadSize() = DrawQuadrant(cache, param.top_right);
  } else {
    rearranger_holder.emplace<UnevenQuadrantsRearranger>(cache, kMaxQuadSize);
    auto& t = std::get<UnevenQuadrantsRearranger>(rearranger_holder);
    rearranger = &t;

    t.QuadSize(0) = DrawQuadrant(t.QuadCache(0), param.top_right);
    t.QuadSize(1) = DrawQuadrant(t.QuadCache(1), param.bottom_right);
    t.QuadSize(2) = DrawQuadrant(t.QuadCache(2), param.bottom_left);
    t.QuadSize(3) = DrawQuadrant(t.QuadCache(3), param.top_left);
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
