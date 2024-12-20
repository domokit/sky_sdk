// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_ENTITY_GEOMETRY_LINE_GEOMETRY_H_
#define FLUTTER_IMPELLER_ENTITY_GEOMETRY_LINE_GEOMETRY_H_

#include "impeller/entity/geometry/geometry.h"

namespace impeller {

class LineGeometry final : public Geometry {
 public:
  explicit LineGeometry(Point p0, Point p1, Scalar width, Cap cap);

  ~LineGeometry() override;

  static Scalar ComputePixelHalfWidth(const Matrix& transform, Scalar width);

  // |Geometry|
  bool CoversArea(const Matrix& transform, const Rect& rect) const override;

  // |Geometry|
  bool IsAxisAlignedRect() const override;

  Scalar ComputeAlphaCoverage(const Matrix& transform) const override;

 private:
  // Computes the 4 corners of a rectangle that defines the line and
  // possibly extended endpoints which will be rendered under the given
  // transform, and returns true if such a rectangle is defined.
  //
  // The coordinates will be generated in the original coordinate system
  // of the line end points and the transform will only be used to determine
  // the minimum line width.
  //
  // For kButt and kSquare end caps the ends should always be exteded as
  // per that decoration, but for kRound caps the ends might be extended
  // if the goal is to get a conservative bounds and might not be extended
  // if the calling code is planning to draw the round caps on the ends.
  //
  // @return true if the transform and width were not degenerate
  bool ComputeCorners(Point corners[4],
                      const Matrix& transform,
                      bool extend_endpoints) const;

  Vector2 ComputeAlongVector(const Matrix& transform,
                             bool allow_zero_length) const;

  // |Geometry|
  GeometryResult GetPositionBuffer(const ContentContext& renderer,
                                   const Entity& entity,
                                   RenderPass& pass) const override;

  // |Geometry|
  std::optional<Rect> GetCoverage(const Matrix& transform) const override;

  Point p0_;
  Point p1_;
  Scalar width_;
  Cap cap_;

  LineGeometry(const LineGeometry&) = delete;

  LineGeometry& operator=(const LineGeometry&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_ENTITY_GEOMETRY_LINE_GEOMETRY_H_
