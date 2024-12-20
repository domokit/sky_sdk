// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/entity/geometry/rect_geometry.h"

namespace impeller {

RectGeometry::RectGeometry(Rect rect) : rect_(rect) {}

RectGeometry::~RectGeometry() = default;

GeometryResult RectGeometry::GetPositionBuffer(const ContentContext& renderer,
                                               const Entity& entity,
                                               RenderPass& pass) const {
  auto& host_buffer = renderer.GetTransientsBuffer();
  return GeometryResult{
      .type = PrimitiveType::kTriangleStrip,
      .vertex_buffer =
          {
              .vertex_buffer = host_buffer.Emplace(
                  rect_.GetPoints().data(), 8 * sizeof(float), alignof(float)),
              .vertex_count = 4,
              .index_type = IndexType::kNone,
          },
      .transform = entity.GetShaderTransform(pass),
  };
}

std::optional<Rect> RectGeometry::GetCoverage(const Matrix& transform) const {
  return rect_.TransformBounds(transform);
}

bool RectGeometry::CoversArea(const Matrix& transform, const Rect& rect) const {
  if (!transform.IsTranslationScaleOnly()) {
    return false;
  }
  Rect coverage = rect_.TransformBounds(transform);
  return coverage.Contains(rect);
}

bool RectGeometry::IsAxisAlignedRect() const {
  return true;
}

}  // namespace impeller
