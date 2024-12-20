// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/display_list/dl_atlas_geometry.h"

#include "impeller/core/formats.h"
#include "impeller/core/vertex_buffer.h"
#include "impeller/display_list/skia_conversions.h"
#include "impeller/entity/porter_duff_blend.vert.h"
#include "impeller/entity/texture_fill.vert.h"
#include "impeller/geometry/color.h"
#include "impeller/geometry/point.h"
#include "third_party/skia/include/core/SkPoint.h"

namespace impeller {

DlAtlasGeometry::DlAtlasGeometry(const std::shared_ptr<Texture>& atlas,
                                 const SkRSXform* xform,
                                 const flutter::DlRect* tex,
                                 const flutter::DlColor* colors,
                                 size_t count,
                                 BlendMode mode,
                                 const SamplerDescriptor& sampling,
                                 std::optional<Rect> cull_rect)
    : atlas_(atlas),
      xform_(xform),
      tex_(tex),
      colors_(colors),
      count_(count),
      mode_(mode),
      sampling_(sampling),
      cull_rect_(cull_rect) {}

DlAtlasGeometry::~DlAtlasGeometry() = default;

bool DlAtlasGeometry::ShouldUseBlend() const {
  return colors_ != nullptr && mode_ != BlendMode::kSource;
}

bool DlAtlasGeometry::ShouldSkip() const {
  return atlas_ == nullptr || (ShouldUseBlend() && mode_ == BlendMode::kClear);
}

Rect DlAtlasGeometry::ComputeBoundingBox() const {
  if (cull_rect_.has_value()) {
    return cull_rect_.value();
  }
  Rect bounding_box = {};
  for (size_t i = 0; i < count_; i++) {
    auto matrix = skia_conversions::ToRSXForm(xform_[i]);
    auto sample_rect = tex_[i];
    auto bounds = Rect::MakeSize(sample_rect.GetSize()).TransformBounds(matrix);
    bounding_box = bounds.Union(bounding_box);
  }
  cull_rect_ = bounding_box;
  return bounding_box;
}

std::shared_ptr<Texture> DlAtlasGeometry::GetAtlas() const {
  return atlas_;
}

const SamplerDescriptor& DlAtlasGeometry::GetSamplerDescriptor() const {
  return sampling_;
}

BlendMode DlAtlasGeometry::GetBlendMode() const {
  return mode_;
}

VertexBuffer DlAtlasGeometry::CreateSimpleVertexBuffer(
    HostBuffer& host_buffer) const {
  using VS = TextureFillVertexShader;

  constexpr size_t indices[6] = {0, 1, 2, 1, 2, 3};

  auto buffer_view = host_buffer.Emplace(
      sizeof(VS::PerVertexData) * count_ * 6, alignof(VS::PerVertexData),
      [&](uint8_t* raw_data) {
        VS::PerVertexData* data =
            reinterpret_cast<VS::PerVertexData*>(raw_data);
        int offset = 0;
        auto texture_size = atlas_->GetSize();
        for (auto i = 0u; i < count_; i++) {
          flutter::DlRect sample_rect = tex_[i];
          Matrix matrix = skia_conversions::ToRSXForm(xform_[i]);
          auto points = sample_rect.GetPoints();
          auto transformed_points = Rect::MakeSize(sample_rect.GetSize())
                                        .GetTransformedPoints(matrix);
          for (size_t j = 0; j < 6; j++) {
            data[offset].position = transformed_points[indices[j]];
            data[offset].texture_coords = points[indices[j]] / texture_size;
            offset += 1;
          }
        }
      });

  return VertexBuffer{
      .vertex_buffer = buffer_view,
      .index_buffer = {},
      .vertex_count = count_ * 6,
      .index_type = IndexType::kNone,
  };
}

VertexBuffer DlAtlasGeometry::CreateBlendVertexBuffer(
    HostBuffer& host_buffer) const {
  using VS = PorterDuffBlendVertexShader;

  constexpr size_t indices[6] = {0, 1, 2, 1, 2, 3};

  auto buffer_view = host_buffer.Emplace(
      sizeof(VS::PerVertexData) * count_ * 6, alignof(VS::PerVertexData),
      [&](uint8_t* raw_data) {
        VS::PerVertexData* data =
            reinterpret_cast<VS::PerVertexData*>(raw_data);
        int offset = 0;
        auto texture_size = atlas_->GetSize();
        for (auto i = 0u; i < count_; i++) {
          flutter::DlRect sample_rect = tex_[i];
          Matrix matrix = skia_conversions::ToRSXForm(xform_[i]);
          auto points = sample_rect.GetPoints();
          auto transformed_points = Rect::MakeSize(sample_rect.GetSize())
                                        .GetTransformedPoints(matrix);
          for (size_t j = 0; j < 6; j++) {
            data[offset].vertices = transformed_points[indices[j]];
            data[offset].texture_coords = points[indices[j]] / texture_size;
            data[offset].color =
                skia_conversions::ToColor(colors_[i]).Premultiply();
            offset += 1;
          }
        }
      });

  return VertexBuffer{
      .vertex_buffer = buffer_view,
      .index_buffer = {},
      .vertex_count = count_ * 6,
      .index_type = IndexType::kNone,
  };
}

}  // namespace impeller
