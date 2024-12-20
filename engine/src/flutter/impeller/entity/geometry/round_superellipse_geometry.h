// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_ENTITY_GEOMETRY_ROUND_SUPERELLIPSE_GEOMETRY_H_
#define FLUTTER_IMPELLER_ENTITY_GEOMETRY_ROUND_SUPERELLIPSE_GEOMETRY_H_

#include "impeller/entity/geometry/geometry.h"

namespace impeller {

/// Geometry class that can generate vertices for a rounded superellipse.
///
/// A superellipse is an ellipse-like shape that is defined by the parameters N,
/// alpha, and beta:
///
///  1 = |x / b| ^n + |y / a| ^n
///
/// A rounded superellipse is a square-like superellipse (a=b) with its four
/// corners replaced by circular arcs. It replicates the `RoundedRectangle`
/// shape in SwiftUI with corner style `.continuous`.
///
/// The `bounds` defines the position and size of the shape. The `corner_radius`
/// corresponds to SwiftUI's `cornerRadius` parameter, which is close to, but
/// not exactly equals to, the radius of the corner circles.
class RoundSuperellipseGeometry final : public Geometry {
 public:
  explicit RoundSuperellipseGeometry(const Rect& bounds, Scalar corner_radius);

  ~RoundSuperellipseGeometry() override;

  // |Geometry|
  bool CoversArea(const Matrix& transform, const Rect& rect) const override;

  // |Geometry|
  bool IsAxisAlignedRect() const override;

 private:
  // |Geometry|
  GeometryResult GetPositionBuffer(const ContentContext& renderer,
                                   const Entity& entity,
                                   RenderPass& pass) const override;

  // |Geometry|
  std::optional<Rect> GetCoverage(const Matrix& transform) const override;

  const Rect bounds_;
  double corner_radius_;

  RoundSuperellipseGeometry(const RoundSuperellipseGeometry&) = delete;

  RoundSuperellipseGeometry& operator=(const RoundSuperellipseGeometry&) =
      delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_ENTITY_GEOMETRY_ROUND_SUPERELLIPSE_GEOMETRY_H_
