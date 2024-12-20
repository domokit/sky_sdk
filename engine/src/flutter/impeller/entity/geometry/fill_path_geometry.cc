// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/entity/geometry/fill_path_geometry.h"

#include "fml/logging.h"
#include "impeller/core/formats.h"
#include "impeller/core/vertex_buffer.h"
#include "impeller/entity/contents/content_context.h"
#include "impeller/entity/geometry/geometry.h"

namespace impeller {

FillPathGeometry::FillPathGeometry(const Path& path,
                                   std::optional<Rect> inner_rect)
    : path_(path), inner_rect_(inner_rect) {}

FillPathGeometry::~FillPathGeometry() {}

GeometryResult FillPathGeometry::GetPositionBuffer(
    const ContentContext& renderer,
    const Entity& entity,
    RenderPass& pass) const {
  auto& host_buffer = renderer.GetTransientsBuffer();

  const auto& bounding_box = path_.GetBoundingBox();
  if (bounding_box.has_value() && bounding_box->IsEmpty()) {
    return GeometryResult{
        .type = PrimitiveType::kTriangle,
        .vertex_buffer =
            VertexBuffer{
                .vertex_buffer = {},
                .vertex_count = 0,
                .index_type = IndexType::k16bit,
            },
        .transform = pass.GetOrthographicTransform() * entity.GetTransform(),
    };
  }

  bool supports_primitive_restart =
      renderer.GetDeviceCapabilities().SupportsPrimitiveRestart();
  bool supports_triangle_fan =
      renderer.GetDeviceCapabilities().SupportsTriangleFan() &&
      supports_primitive_restart;
  VertexBuffer vertex_buffer = renderer.GetTessellator().TessellateConvex(
      path_, host_buffer, entity.GetTransform().GetMaxBasisLengthXY(),
      /*supports_primitive_restart=*/supports_primitive_restart,
      /*supports_triangle_fan=*/supports_triangle_fan);

  return GeometryResult{
      .type = supports_triangle_fan ? PrimitiveType::kTriangleFan
                                    : PrimitiveType::kTriangleStrip,
      .vertex_buffer = std::move(vertex_buffer),
      .transform = entity.GetShaderTransform(pass),
      .mode = GetResultMode(),
  };
}

GeometryResult::Mode FillPathGeometry::GetResultMode() const {
  const auto& bounding_box = path_.GetBoundingBox();
  if (path_.IsConvex() ||
      (bounding_box.has_value() && bounding_box->IsEmpty())) {
    return GeometryResult::Mode::kNormal;
  }

  switch (path_.GetFillType()) {
    case FillType::kNonZero:
      return GeometryResult::Mode::kNonZero;
    case FillType::kOdd:
      return GeometryResult::Mode::kEvenOdd;
  }

  FML_UNREACHABLE();
}

std::optional<Rect> FillPathGeometry::GetCoverage(
    const Matrix& transform) const {
  return path_.GetTransformedBoundingBox(transform);
}

bool FillPathGeometry::CoversArea(const Matrix& transform,
                                  const Rect& rect) const {
  if (!inner_rect_.has_value()) {
    return false;
  }
  if (!transform.IsTranslationScaleOnly()) {
    return false;
  }
  Rect coverage = inner_rect_->TransformBounds(transform);
  return coverage.Contains(rect);
}

}  // namespace impeller
