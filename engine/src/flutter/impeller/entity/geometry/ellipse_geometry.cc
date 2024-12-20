// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <algorithm>

#include "flutter/impeller/entity/geometry/ellipse_geometry.h"

#include "flutter/impeller/entity/geometry/line_geometry.h"

namespace impeller {

EllipseGeometry::EllipseGeometry(Rect bounds) : bounds_(bounds) {}

GeometryResult EllipseGeometry::GetPositionBuffer(
    const ContentContext& renderer,
    const Entity& entity,
    RenderPass& pass) const {
  return ComputePositionGeometry(
      renderer,
      renderer.GetTessellator().FilledEllipse(entity.GetTransform(), bounds_),
      entity, pass);
}

std::optional<Rect> EllipseGeometry::GetCoverage(
    const Matrix& transform) const {
  return bounds_.TransformBounds(transform);
}

bool EllipseGeometry::CoversArea(const Matrix& transform,
                                 const Rect& rect) const {
  return false;
}

bool EllipseGeometry::IsAxisAlignedRect() const {
  return false;
}

}  // namespace impeller
