// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <algorithm>

#include "flutter/impeller/entity/geometry/circle_geometry.h"

#include "flutter/impeller/entity/geometry/line_geometry.h"
#include "impeller/core/formats.h"
#include "impeller/entity/geometry/geometry.h"

namespace impeller {

CircleGeometry::CircleGeometry(const Point& center, Scalar radius)
    : center_(center), radius_(radius), stroke_width_(-1.0f) {
  FML_DCHECK(radius >= 0);
}

CircleGeometry::~CircleGeometry() = default;

CircleGeometry::CircleGeometry(const Point& center,
                               Scalar radius,
                               Scalar stroke_width)
    : center_(center),
      radius_(radius),
      stroke_width_(std::max(stroke_width, 0.0f)) {
  FML_DCHECK(radius >= 0);
  FML_DCHECK(stroke_width >= 0);
}

// |Geometry|
Scalar CircleGeometry::ComputeAlphaCoverage(const Matrix& transform) const {
  if (stroke_width_ < 0) {
    return 1;
  }
  return Geometry::ComputeStrokeAlphaCoverage(transform, stroke_width_);
}

GeometryResult CircleGeometry::GetPositionBuffer(const ContentContext& renderer,
                                                 const Entity& entity,
                                                 RenderPass& pass) const {
  auto& transform = entity.GetTransform();

  Scalar half_width = stroke_width_ < 0 ? 0.0
                                        : LineGeometry::ComputePixelHalfWidth(
                                              transform, stroke_width_);

  // We call the StrokedCircle method which will simplify to a
  // FilledCircleGenerator if the inner_radius is <= 0.
  auto generator = renderer.GetTessellator().StrokedCircle(transform, center_,
                                                           radius_, half_width);

  return ComputePositionGeometry(renderer, generator, entity, pass);
}

std::optional<Rect> CircleGeometry::GetCoverage(const Matrix& transform) const {
  Point corners[4]{
      {center_.x, center_.y - radius_},
      {center_.x + radius_, center_.y},
      {center_.x, center_.y + radius_},
      {center_.x - radius_, center_.y},
  };

  for (int i = 0; i < 4; i++) {
    corners[i] = transform * corners[i];
  }
  return Rect::MakePointBounds(std::begin(corners), std::end(corners));
}

bool CircleGeometry::CoversArea(const Matrix& transform,
                                const Rect& rect) const {
  return false;
}

bool CircleGeometry::IsAxisAlignedRect() const {
  return false;
}

}  // namespace impeller
