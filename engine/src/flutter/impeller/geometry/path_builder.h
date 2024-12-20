// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_GEOMETRY_PATH_BUILDER_H_
#define FLUTTER_IMPELLER_GEOMETRY_PATH_BUILDER_H_

#include "impeller/geometry/path.h"
#include "impeller/geometry/rect.h"
#include "impeller/geometry/round_rect.h"
#include "impeller/geometry/scalar.h"

namespace impeller {

class PathBuilder {
 public:
  /// Used for approximating quarter circle arcs with cubic curves. This is the
  /// control point distance which results in the smallest possible unit circle
  /// integration for a right angle arc. It can be used to approximate arcs less
  /// than 90 degrees to great effect by simply reducing it proportionally to
  /// the angle. However, accuracy rapidly diminishes if magnified for obtuse
  /// angle arcs, and so multiple cubic curves should be used when approximating
  /// arcs greater than 90 degrees.
  constexpr static const Scalar kArcApproximationMagic = 0.551915024494f;

  PathBuilder();

  ~PathBuilder();

  Path CopyPath(FillType fill = FillType::kNonZero);

  Path TakePath(FillType fill = FillType::kNonZero);

  /// @brief Reserve [point_size] points and [verb_size] verbs in the underlying
  ///        path buffer.
  void Reserve(size_t point_size, size_t verb_size);

  PathBuilder& SetConvexity(Convexity value);

  PathBuilder& MoveTo(Point point, bool relative = false);

  PathBuilder& Close();

  /// @brief Insert a line from the current position to `point`.
  ///
  /// If `relative` is true, then `point` is relative to the current location.
  PathBuilder& LineTo(Point point, bool relative = false);

  PathBuilder& HorizontalLineTo(Scalar x, bool relative = false);

  PathBuilder& VerticalLineTo(Scalar y, bool relative = false);

  /// @brief Insert a quadradic curve from the current position to `point` using
  /// the control point `controlPoint`.
  ///
  /// If `relative` is true the `point` and `controlPoint` are relative to
  /// current location.
  PathBuilder& QuadraticCurveTo(Point controlPoint,
                                Point point,
                                bool relative = false);

  /// @brief Insert a cubic curve from the curren position to `point` using the
  /// control points `controlPoint1` and `controlPoint2`.
  ///
  /// If `relative` is true the `point`, `controlPoint1`, and `controlPoint2`
  /// are relative to current location.
  PathBuilder& CubicCurveTo(Point controlPoint1,
                            Point controlPoint2,
                            Point point,
                            bool relative = false);

  PathBuilder& AddRect(Rect rect);

  PathBuilder& AddCircle(const Point& center, Scalar radius);

  PathBuilder& AddArc(const Rect& oval_bounds,
                      Radians start,
                      Radians sweep,
                      bool use_center = false);

  PathBuilder& AddOval(const Rect& rect);

  /// @brief Move to point `p1`, then insert a line from `p1` to `p2`.
  PathBuilder& AddLine(const Point& p1, const Point& p2);

  /// @brief Move to point `p1`, then insert a quadradic curve from `p1` to `p2`
  /// with the control point `cp`.
  PathBuilder& AddQuadraticCurve(Point p1, Point cp, Point p2);

  /// @brief Move to point `p1`, then insert a cubic curve from `p1` to `p2`
  /// with control points `cp1` and `cp2`.
  PathBuilder& AddCubicCurve(Point p1, Point cp1, Point cp2, Point p2);

  /// @brief Transform the existing path segments and contours by the given
  /// `offset`.
  PathBuilder& Shift(Point offset);

  /// @brief Set the bounding box that will be used by `Path.GetBoundingBox` in
  /// place of performing the computation.
  ///
  ///        When Impeller recieves Skia Path objects, many of these already
  ///        have computed bounds. This method is used to avoid needlessly
  ///        recomputing these bounds.
  PathBuilder& SetBounds(Rect bounds);

  PathBuilder& AddRoundRect(RoundRect rect);

  PathBuilder& AddPath(const Path& path);

 private:
  Point subpath_start_;
  Point current_;
  size_t current_contour_location_ = 0u;
  size_t contour_count_ = 0u;
  Path::Data prototype_;

  PathBuilder& AddRoundedRectTopLeft(Rect rect, RoundingRadii radii);

  PathBuilder& AddRoundedRectTopRight(Rect rect, RoundingRadii radii);

  PathBuilder& AddRoundedRectBottomRight(Rect rect, RoundingRadii radii);

  PathBuilder& AddRoundedRectBottomLeft(Rect rect, RoundingRadii radii);

  void AddContourComponent(const Point& destination, bool is_closed = false);

  void SetContourClosed(bool is_closed);

  void AddLinearComponent(const Point& p1, const Point& p2);

  void AddLinearComponentIfNeeded(const Point& p1, const Point& p2);

  void AddQuadraticComponent(const Point& p1, const Point& cp, const Point& p2);

  void AddCubicComponent(const Point& p1,
                         const Point& cp1,
                         const Point& cp2,
                         const Point& p2);

  /// Compute the bounds of the path unless they are already computed or
  /// set by an external source, such as |SetBounds|. Any call which mutates
  /// the path data can invalidate the computed/set bounds.
  void UpdateBounds();

  std::optional<std::pair<Point, Point>> GetMinMaxCoveragePoints() const;

  PathBuilder(const PathBuilder&) = delete;
  PathBuilder& operator=(const PathBuilder&&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_GEOMETRY_PATH_BUILDER_H_
