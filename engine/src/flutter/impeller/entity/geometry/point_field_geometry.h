// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_ENTITY_GEOMETRY_POINT_FIELD_GEOMETRY_H_
#define FLUTTER_IMPELLER_ENTITY_GEOMETRY_POINT_FIELD_GEOMETRY_H_

#include "impeller/entity/geometry/geometry.h"

namespace impeller {

/// @brief A geometry class specialized for Canvas::DrawPoints.
///
/// Does not hold ownership of the allocated point data, which is expected to be
/// maintained via the display list structure.
class PointFieldGeometry final : public Geometry {
 public:
  PointFieldGeometry(const Point* points,
                     size_t point_count,
                     Scalar radius,
                     bool round);

  ~PointFieldGeometry() override;

  // |Geometry|
  std::optional<Rect> GetCoverage(const Matrix& transform) const override;

 private:
  // |Geometry|
  GeometryResult GetPositionBuffer(const ContentContext& renderer,
                                   const Entity& entity,
                                   RenderPass& pass) const override;

  size_t point_count_;
  Scalar radius_;
  bool round_;
  const Point* points_;

  PointFieldGeometry(const PointFieldGeometry&) = delete;

  PointFieldGeometry& operator=(const PointFieldGeometry&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_ENTITY_GEOMETRY_POINT_FIELD_GEOMETRY_H_
