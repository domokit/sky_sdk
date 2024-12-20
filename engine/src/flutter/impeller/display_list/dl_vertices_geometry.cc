// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/display_list/dl_vertices_geometry.h"

#include "display_list/dl_vertices.h"
#include "impeller/core/formats.h"
#include "impeller/display_list/skia_conversions.h"
#include "impeller/geometry/point.h"
#include "third_party/skia/include/core/SkPoint.h"

namespace impeller {

namespace {

// Fan mode isn't natively supported on Metal backends. Unroll into triangle
// mode by manipulating the index array.
//
// In Triangle fan, the first vertex is shared across all triangles, and then
// each sliding window of two vertices plus that first vertex defines a
// triangle.
static std::vector<uint16_t> fromFanIndices(size_t vertex_count,
                                            size_t index_count,
                                            const uint16_t* indices) {
  std::vector<uint16_t> unrolled_indices;

  // Un-fan index buffer if provided.
  if (index_count > 0u) {
    if (index_count < 3u) {
      return {};
    }

    auto center_point = indices[0];
    for (auto i = 1u; i < index_count - 1; i++) {
      unrolled_indices.push_back(center_point);
      unrolled_indices.push_back(indices[i]);
      unrolled_indices.push_back(indices[i + 1]);
    }
  } else {
    if (vertex_count < 3u) {
      return {};
    }

    // If indices were not provided, create an index buffer that unfans
    // triangles instead of re-writing points, colors, et cetera.
    for (auto i = 1u; i < vertex_count - 1; i++) {
      unrolled_indices.push_back(0);
      unrolled_indices.push_back(i);
      unrolled_indices.push_back(i + 1);
    }
  }
  return unrolled_indices;
}

}  // namespace

/////// Vertices Geometry ///////

DlVerticesGeometry::DlVerticesGeometry(
    const std::shared_ptr<const flutter::DlVertices>& vertices,
    const ContentContext& renderer)
    : vertices_(vertices) {
  performed_normalization_ = MaybePerformIndexNormalization(renderer);
  bounds_ = vertices_->GetBounds();
}

PrimitiveType DlVerticesGeometry::GetPrimitiveType() const {
  switch (vertices_->mode()) {
    case flutter::DlVertexMode::kTriangleFan:
      // Unrolled into triangle mode.
      if (performed_normalization_) {
        return PrimitiveType::kTriangle;
      }
      return PrimitiveType::kTriangleFan;
    case flutter::DlVertexMode::kTriangleStrip:
      return PrimitiveType::kTriangleStrip;
    case flutter::DlVertexMode::kTriangles:
      return PrimitiveType::kTriangle;
  }
}

bool DlVerticesGeometry::HasVertexColors() const {
  return vertices_->colors() != nullptr;
}

bool DlVerticesGeometry::HasTextureCoordinates() const {
  return vertices_->texture_coordinate_data() != nullptr;
}

std::optional<Rect> DlVerticesGeometry::GetTextureCoordinateCoverge() const {
  if (!HasTextureCoordinates()) {
    return std::nullopt;
  }
  auto vertex_count = vertices_->vertex_count();
  if (vertex_count == 0) {
    return std::nullopt;
  }

  auto first = vertices_->texture_coordinate_data();
  auto left = first->x;
  auto top = first->y;
  auto right = first->x;
  auto bottom = first->y;
  int i = 1;
  for (auto it = first + 1; i < vertex_count; ++it, i++) {
    left = std::min(left, it->x);
    top = std::min(top, it->y);
    right = std::max(right, it->x);
    bottom = std::max(bottom, it->y);
  }
  return Rect::MakeLTRB(left, top, right, bottom);
}

GeometryResult DlVerticesGeometry::GetPositionBuffer(
    const ContentContext& renderer,
    const Entity& entity,
    RenderPass& pass) const {
  int vertex_count = vertices_->vertex_count();
  BufferView vertex_buffer = renderer.GetTransientsBuffer().Emplace(
      vertices_->vertex_data(), vertex_count * sizeof(Point), alignof(Point));

  BufferView index_buffer = {};
  auto index_count =
      performed_normalization_ ? indices_.size() : vertices_->index_count();
  const uint16_t* indices_data =
      performed_normalization_ ? indices_.data() : vertices_->indices();
  if (index_count) {
    index_buffer = renderer.GetTransientsBuffer().Emplace(
        indices_data, index_count * sizeof(uint16_t), alignof(uint16_t));
  }

  return GeometryResult{
      .type = GetPrimitiveType(),
      .vertex_buffer =
          {
              .vertex_buffer = vertex_buffer,
              .index_buffer = index_buffer,
              .vertex_count = index_count > 0 ? index_count : vertex_count,
              .index_type =
                  index_count > 0 ? IndexType::k16bit : IndexType::kNone,
          },
      .transform = entity.GetShaderTransform(pass),
  };
}

GeometryResult DlVerticesGeometry::GetPositionUVColorBuffer(
    Rect texture_coverage,
    Matrix effect_transform,
    const ContentContext& renderer,
    const Entity& entity,
    RenderPass& pass) const {
  using VS = PorterDuffBlendPipeline::VertexShader;

  int vertex_count = vertices_->vertex_count();
  Matrix uv_transform =
      texture_coverage.GetNormalizingTransform() * effect_transform;
  bool has_texture_coordinates = HasTextureCoordinates();
  bool has_colors = HasVertexColors();

  const Point* coordinates = has_texture_coordinates
                                 ? vertices_->texture_coordinate_data()
                                 : vertices_->vertex_data();
  BufferView vertex_buffer = renderer.GetTransientsBuffer().Emplace(
      vertex_count * sizeof(VS::PerVertexData), alignof(VS::PerVertexData),
      [&](uint8_t* data) {
        VS::PerVertexData* vtx_contents =
            reinterpret_cast<VS::PerVertexData*>(data);
        const Point* vertex_points = vertices_->vertex_data();
        for (auto i = 0; i < vertex_count; i++) {
          Point texture_coord = coordinates[i];
          Point uv = uv_transform * texture_coord;
          Color color = has_colors
                            ? skia_conversions::ToColor(vertices_->colors()[i])
                                  .Premultiply()
                            : Color::BlackTransparent();
          VS::PerVertexData vertex_data = {.vertices = vertex_points[i],
                                           .texture_coords = uv,
                                           .color = color};
          vtx_contents[i] = vertex_data;
        }
      });

  BufferView index_buffer = {};
  auto index_count =
      performed_normalization_ ? indices_.size() : vertices_->index_count();
  const uint16_t* indices_data =
      performed_normalization_ ? indices_.data() : vertices_->indices();
  if (index_count) {
    index_buffer = renderer.GetTransientsBuffer().Emplace(
        indices_data, index_count * sizeof(uint16_t), alignof(uint16_t));
  }

  return GeometryResult{
      .type = GetPrimitiveType(),
      .vertex_buffer =
          {
              .vertex_buffer = vertex_buffer,
              .index_buffer = index_buffer,
              .vertex_count = index_count > 0 ? index_count : vertex_count,
              .index_type =
                  index_count > 0 ? IndexType::k16bit : IndexType::kNone,
          },
      .transform = entity.GetShaderTransform(pass),
  };
}

std::optional<Rect> DlVerticesGeometry::GetCoverage(
    const Matrix& transform) const {
  return bounds_.TransformBounds(transform);
}

bool DlVerticesGeometry::MaybePerformIndexNormalization(
    const ContentContext& renderer) {
  if (vertices_->mode() == flutter::DlVertexMode::kTriangleFan &&
      !renderer.GetDeviceCapabilities().SupportsTriangleFan()) {
    indices_ = fromFanIndices(vertices_->vertex_count(),
                              vertices_->index_count(), vertices_->indices());
    return true;
  }
  return false;
}

}  // namespace impeller
