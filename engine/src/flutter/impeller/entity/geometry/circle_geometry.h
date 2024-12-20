// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_ENTITY_GEOMETRY_CIRCLE_GEOMETRY_H_
#define FLUTTER_IMPELLER_ENTITY_GEOMETRY_CIRCLE_GEOMETRY_H_

#include "impeller/entity/geometry/geometry.h"

namespace impeller {

// Geometry class that can generate vertices (with or without texture
// coordinates) for either filled or stroked circles
class CircleGeometry final : public Geometry {
 public:
  explicit CircleGeometry(const Point& center, Scalar radius);

  explicit CircleGeometry(const Point& center,
                          Scalar radius,
                          Scalar stroke_width);

  ~CircleGeometry() override;

  // |Geometry|
  bool CoversArea(const Matrix& transform, const Rect& rect) const override;

  // |Geometry|
  bool IsAxisAlignedRect() const override;

  // |Geometry|
  Scalar ComputeAlphaCoverage(const Matrix& transform) const override;

 private:
  // |Geometry|
  GeometryResult GetPositionBuffer(const ContentContext& renderer,
                                   const Entity& entity,
                                   RenderPass& pass) const override;

  // |Geometry|
  std::optional<Rect> GetCoverage(const Matrix& transform) const override;

  Point center_;
  Scalar radius_;
  Scalar stroke_width_;

  CircleGeometry(const CircleGeometry&) = delete;

  CircleGeometry& operator=(const CircleGeometry&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_ENTITY_GEOMETRY_CIRCLE_GEOMETRY_H_
