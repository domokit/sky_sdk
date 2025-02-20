// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_GEOMETRY_PATH_COMPONENT_H_
#define FLUTTER_IMPELLER_GEOMETRY_PATH_COMPONENT_H_

#include <array>
#include <functional>
#include <optional>
#include <type_traits>
#include <vector>

#include "impeller/geometry/point.h"
#include "impeller/geometry/scalar.h"

namespace impeller {

/// @brief An interface for generating a multi contour polyline as a triangle
///        strip.
class VertexWriter {
 public:
  virtual void EndContour() = 0;

  virtual void Write(Point point) = 0;
};

/// @brief A vertex writer that generates a triangle fan and requires primitive
/// restart.
class FanVertexWriter : public VertexWriter {
 public:
  explicit FanVertexWriter(Point* point_buffer, uint16_t* index_buffer);

  ~FanVertexWriter();

  size_t GetIndexCount() const;

  void EndContour() override;

  void Write(Point point) override;

 private:
  size_t count_ = 0;
  size_t index_count_ = 0;
  Point* point_buffer_ = nullptr;
  uint16_t* index_buffer_ = nullptr;
};

/// @brief A vertex writer that generates a triangle strip and requires
///        primitive restart.
class StripVertexWriter : public VertexWriter {
 public:
  explicit StripVertexWriter(Point* point_buffer, uint16_t* index_buffer);

  ~StripVertexWriter();

  size_t GetIndexCount() const;

  void EndContour() override;

  void Write(Point point) override;

 private:
  size_t count_ = 0;
  size_t index_count_ = 0;
  size_t contour_start_ = 0;
  Point* point_buffer_ = nullptr;
  uint16_t* index_buffer_ = nullptr;
};

/// @brief A vertex writer that generates a line strip topology.
class LineStripVertexWriter : public VertexWriter {
 public:
  explicit LineStripVertexWriter(std::vector<Point>& points);

  ~LineStripVertexWriter() = default;

  void EndContour() override;

  void Write(Point point) override;

  std::pair<size_t, size_t> GetVertexCount() const;

  const std::vector<Point>& GetOversizedBuffer() const;

 private:
  size_t offset_ = 0u;
  std::vector<Point>& points_;
  std::vector<Point> overflow_;
};

/// @brief A vertex writer that has no hardware requirements.
class GLESVertexWriter : public VertexWriter {
 public:
  explicit GLESVertexWriter(std::vector<Point>& points,
                            std::vector<uint16_t>& indices);

  ~GLESVertexWriter() = default;

  void EndContour() override;

  void Write(Point point) override;

 private:
  bool previous_contour_odd_points_ = false;
  size_t contour_start_ = 0u;
  std::vector<Point>& points_;
  std::vector<uint16_t>& indices_;
};

struct LinearPathComponent {
  Point p1;
  Point p2;

  LinearPathComponent() {}

  LinearPathComponent(Point ap1, Point ap2) : p1(ap1), p2(ap2) {}

  Point Solve(Scalar time) const;

  void AppendPolylinePoints(std::vector<Point>& points) const;

  std::vector<Point> Extrema() const;

  bool operator==(const LinearPathComponent& other) const {
    return p1 == other.p1 && p2 == other.p2;
  }

  std::optional<Vector2> GetStartDirection() const;

  std::optional<Vector2> GetEndDirection() const;
};

// A component that represets a Quadratic Bézier curve.
struct QuadraticPathComponent {
  // Start point.
  Point p1;
  // Control point.
  Point cp;
  // End point.
  Point p2;

  QuadraticPathComponent() {}

  QuadraticPathComponent(Point ap1, Point acp, Point ap2)
      : p1(ap1), cp(acp), p2(ap2) {}

  Point Solve(Scalar time) const;

  Point SolveDerivative(Scalar time) const;

  void AppendPolylinePoints(Scalar scale_factor,
                            std::vector<Point>& points) const;

  using PointProc = std::function<void(const Point& point)>;

  void ToLinearPathComponents(Scalar scale_factor, const PointProc& proc) const;

  void ToLinearPathComponents(Scalar scale, VertexWriter& writer) const;

  size_t CountLinearPathComponents(Scalar scale) const;

  std::vector<Point> Extrema() const;

  bool operator==(const QuadraticPathComponent& other) const {
    return p1 == other.p1 && cp == other.cp && p2 == other.p2;
  }

  std::optional<Vector2> GetStartDirection() const;

  std::optional<Vector2> GetEndDirection() const;
};

// A component that represets a Conic section curve.
//
// A conic section is basically nearly a quadratic bezier curve, but it
// has an additional weight that is applied to the middle term (the control
// point term).
//
// Starting with the equation for a quadratic curve which is:
//   (A) P1 * (1 - t) * (1 - t)
//     + CP * 2 * t * (1 - t)
//     + P2 * t * t
// One thing to note is that the quadratic coefficients always sum to 1:
//   (B) (1-t)(1-t) + 2t(1-t) + tt
//    == 1 - 2t + tt + 2t - 2tt + tt
//    == 1
// which means that the resulting point is always a weighted average of
// the 3 points without having to "divide by the sum of the coefficients"
// that is normally done when computing weighted averages.
//
// The conic equation, though, would then be:
//   (C) P1 * (1 - t) * (1 - t)
//     + CP * 2 * t * (1 - t) * w
//     + P2 * t * t
// That would be the final equation, but if we look at the coefficients:
//   (D) (1-t)(1-t) + 2wt(1-t) + tt
//    == 1 - 2t + tt + 2wt - 2wtt + tt
//    == 1 + (2w - 2)t + (2 - 2w)tt
// These only sum to 1 if the weight (w) is 1. In order for this math to
// produce a point that is the weighted average of the 3 points, we have
// to compute both and divide the equation (C) by the equation (D).
//
// Note that there are important potential optimizations we could apply.
// If w is 0,
//   then this equation devolves into a straight line from P1 to P2.
//   Note that the "progress" from P1 to P2, as a function of t, is
//   quadratic if you compute it as the indicated numerator and denominator,
//   but the actual points generated are all on the line from P1 to P2
// If w is (sqrt(2) / 2),
//   then this math is exactly an elliptical section, provided the 3 points
//   P1, CP, P2 form a right angle, and a circular section if they are also
//   of equal length (|P1,CP| == |CP,P2|)
// If w is 1,
//   then we really don't need the division since the denominator will always
//   be 1 and we could just treat that curve as a quadratic.
// If w is (infinity/large enough),
//   then the equation devolves into 2 straight lines P1->CP->P2, but
//   the straightforward math may encounter infinity/NaN values in the
//   intermediate stages. The limit as w approaches infinity are those
//   two lines.
//
// Some examples: https://fiddle.skia.org/c/986b521feb3b832f04cbdfeefc9fbc58
// Note that the quadratic drawn in red in the center is identical to the
// conic with a weight of 1, drawn in green in the lower left.
struct ConicPathComponent {
  // Start point.
  Point p1;
  // Control point.
  Point cp;
  // End point.
  Point p2;

  // Weight
  //
  // We only need one value, but the underlying storage allocation is always
  // a multiple of Point objects. To avoid confusion over which field the
  // weight is stored in, and what the value of the other field may be, we
  // store it in both x,y components of the |weight| field.
  //
  // This may also be an advantage eventually for code that can vectorize
  // the conic calculations on both X & Y simultaneously.
  Point weight;

  ConicPathComponent() {}

  ConicPathComponent(Point ap1, Point acp, Point ap2, Scalar weight)
      : p1(ap1), cp(acp), p2(ap2), weight(weight, weight) {}

  Point Solve(Scalar time) const;

  void AppendPolylinePoints(Scalar scale_factor,
                            std::vector<Point>& points) const;

  void ToLinearPathComponents(Scalar scale, VertexWriter& writer) const;

  size_t CountLinearPathComponents(Scalar scale) const;

  std::vector<Point> Extrema() const;

  bool operator==(const ConicPathComponent& other) const {
    return p1 == other.p1 && cp == other.cp && p2 == other.p2 &&
           weight == other.weight;
  }

  std::optional<Vector2> GetStartDirection() const;

  std::optional<Vector2> GetEndDirection() const;

  std::array<QuadraticPathComponent, 2> ToQuadraticPathComponents() const;

  void SubdivideToQuadraticPoints(std::array<Point, 5>& points) const;
};

// A component that represets a Cubic Bézier curve.
struct CubicPathComponent {
  // Start point.
  Point p1;
  // The first control point.
  Point cp1;
  // The second control point.
  Point cp2;
  // End point.
  Point p2;

  CubicPathComponent() {}

  explicit CubicPathComponent(const QuadraticPathComponent& q)
      : p1(q.p1),
        cp1(q.p1 + (q.cp - q.p1) * (2.0 / 3.0)),
        cp2(q.p2 + (q.cp - q.p2) * (2.0 / 3.0)),
        p2(q.p2) {}

  CubicPathComponent(Point ap1, Point acp1, Point acp2, Point ap2)
      : p1(ap1), cp1(acp1), cp2(acp2), p2(ap2) {}

  Point Solve(Scalar time) const;

  Point SolveDerivative(Scalar time) const;

  void AppendPolylinePoints(Scalar scale, std::vector<Point>& points) const;

  std::vector<Point> Extrema() const;

  using PointProc = std::function<void(const Point& point)>;

  void ToLinearPathComponents(Scalar scale, const PointProc& proc) const;

  void ToLinearPathComponents(Scalar scale, VertexWriter& writer) const;

  size_t CountLinearPathComponents(Scalar scale) const;

  CubicPathComponent Subsegment(Scalar t0, Scalar t1) const;

  bool operator==(const CubicPathComponent& other) const {
    return p1 == other.p1 && cp1 == other.cp1 && cp2 == other.cp2 &&
           p2 == other.p2;
  }

  std::optional<Vector2> GetStartDirection() const;

  std::optional<Vector2> GetEndDirection() const;

 private:
  QuadraticPathComponent Lower() const;
};

struct ContourComponent {
  Point destination;

  // 0, 0 for closed, anything else for open.
  Point closed = Point(1, 1);

  ContourComponent() {}

  constexpr bool IsClosed() const { return closed == Point{0, 0}; }

  explicit ContourComponent(Point p, Point closed)
      : destination(p), closed(closed) {}

  bool operator==(const ContourComponent& other) const {
    return destination == other.destination && IsClosed() == other.IsClosed();
  }
};

static_assert(!std::is_polymorphic<LinearPathComponent>::value);
static_assert(!std::is_polymorphic<QuadraticPathComponent>::value);
static_assert(!std::is_polymorphic<CubicPathComponent>::value);

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_GEOMETRY_PATH_COMPONENT_H_
