// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/entity/geometry/cover_geometry.h"

#include "impeller/renderer/render_pass.h"

namespace impeller {

CoverGeometry::CoverGeometry() = default;

GeometryResult CoverGeometry::GetPositionBuffer(const ContentContext& renderer,
                                                const Entity& entity,
                                                RenderPass& pass) const {
  auto rect = Rect::MakeSize(pass.GetRenderTargetSize());
  constexpr uint16_t kRectIndicies[4] = {0, 1, 2, 3};
  auto& host_buffer = renderer.GetTransientsBuffer();
  return GeometryResult{
      .type = PrimitiveType::kTriangleStrip,
      .vertex_buffer =
          {
              .vertex_buffer = host_buffer.Emplace(
                  rect.GetTransformedPoints(entity.GetTransform().Invert())
                      .data(),
                  8 * sizeof(float), alignof(float)),
              .index_buffer = host_buffer.Emplace(
                  kRectIndicies, 4 * sizeof(uint16_t), alignof(uint16_t)),
              .vertex_count = 4,
              .index_type = IndexType::k16bit,
          },
      .transform = entity.GetShaderTransform(pass),
  };
}

std::optional<Rect> CoverGeometry::GetCoverage(const Matrix& transform) const {
  return Rect::MakeMaximum();
}

bool CoverGeometry::CoversArea(const Matrix& transform,
                               const Rect& rect) const {
  return true;
}

bool CoverGeometry::CanApplyMaskFilter() const {
  return false;
}

}  // namespace impeller
