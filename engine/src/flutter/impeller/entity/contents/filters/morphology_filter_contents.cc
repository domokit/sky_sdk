// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/entity/contents/filters/morphology_filter_contents.h"

#include <cmath>

#include "impeller/core/formats.h"
#include "impeller/entity/contents/content_context.h"
#include "impeller/entity/contents/contents.h"
#include "impeller/renderer/render_pass.h"
#include "impeller/renderer/vertex_buffer_builder.h"

namespace impeller {

DirectionalMorphologyFilterContents::DirectionalMorphologyFilterContents() =
    default;

DirectionalMorphologyFilterContents::~DirectionalMorphologyFilterContents() =
    default;

void DirectionalMorphologyFilterContents::SetRadius(Radius radius) {
  radius_ = radius;
}

void DirectionalMorphologyFilterContents::SetDirection(Vector2 direction) {
  direction_ = direction.Normalize();
  if (direction_.IsZero()) {
    direction_ = Vector2(0, 1);
  }
}

void DirectionalMorphologyFilterContents::SetMorphType(MorphType morph_type) {
  morph_type_ = morph_type;
}

std::optional<Entity> DirectionalMorphologyFilterContents::RenderFilter(
    const FilterInput::Vector& inputs,
    const ContentContext& renderer,
    const Entity& entity,
    const Matrix& effect_transform,
    const Rect& coverage,
    const std::optional<Rect>& coverage_hint) const {
  using VS = MorphologyFilterPipeline::VertexShader;
  using FS = MorphologyFilterPipeline::FragmentShader;

  //----------------------------------------------------------------------------
  /// Handle inputs.
  ///

  if (inputs.empty()) {
    return std::nullopt;
  }

  auto input_snapshot = inputs[0]->GetSnapshot("Morphology", renderer, entity);
  if (!input_snapshot.has_value()) {
    return std::nullopt;
  }

  if (radius_.radius < kEhCloseEnough) {
    return Entity::FromSnapshot(input_snapshot.value(), entity.GetBlendMode());
  }

  auto maybe_input_uvs = input_snapshot->GetCoverageUVs(coverage);
  if (!maybe_input_uvs.has_value()) {
    return std::nullopt;
  }
  auto input_uvs = maybe_input_uvs.value();

  //----------------------------------------------------------------------------
  /// Render to texture.
  ///

  ContentContext::SubpassCallback callback = [&](const ContentContext& renderer,
                                                 RenderPass& pass) {
    auto& host_buffer = renderer.GetTransientsBuffer();

    std::array<VS::PerVertexData, 4> vertices = {
        VS::PerVertexData{Point(0, 0), input_uvs[0]},
        VS::PerVertexData{Point(1, 0), input_uvs[1]},
        VS::PerVertexData{Point(0, 1), input_uvs[2]},
        VS::PerVertexData{Point(1, 1), input_uvs[3]},
    };

    VS::FrameInfo frame_info;
    frame_info.mvp = Matrix::MakeOrthographic(ISize(1, 1));
    frame_info.texture_sampler_y_coord_scale =
        input_snapshot->texture->GetYCoordScale();

    auto transform = entity.GetTransform() * effect_transform.Basis();
    auto transformed_radius =
        transform.TransformDirection(direction_ * radius_.radius);
    auto transformed_texture_vertices =
        Rect::MakeSize(input_snapshot->texture->GetSize())
            .GetTransformedPoints(input_snapshot->transform);
    auto transformed_texture_width =
        transformed_texture_vertices[0].GetDistance(
            transformed_texture_vertices[1]);
    auto transformed_texture_height =
        transformed_texture_vertices[0].GetDistance(
            transformed_texture_vertices[2]);

    FS::FragInfo frag_info;
    frag_info.radius = std::round(transformed_radius.GetLength());
    frag_info.morph_type = static_cast<Scalar>(morph_type_);
    frag_info.uv_offset =
        input_snapshot->transform.Invert()
            .TransformDirection(transformed_radius)
            .Normalize() /
        Point(transformed_texture_width, transformed_texture_height);

    pass.SetCommandLabel("Morphology Filter");
    auto options = OptionsFromPass(pass);
    options.primitive_type = PrimitiveType::kTriangleStrip;
    options.blend_mode = BlendMode::kSource;
    pass.SetPipeline(renderer.GetMorphologyFilterPipeline(options));
    pass.SetVertexBuffer(CreateVertexBuffer(vertices, host_buffer));

    auto sampler_descriptor = input_snapshot->sampler_descriptor;
    if (renderer.GetDeviceCapabilities().SupportsDecalSamplerAddressMode()) {
      sampler_descriptor.width_address_mode = SamplerAddressMode::kDecal;
      sampler_descriptor.height_address_mode = SamplerAddressMode::kDecal;
    }

    FS::BindTextureSampler(
        pass, input_snapshot->texture,
        renderer.GetContext()->GetSamplerLibrary()->GetSampler(
            sampler_descriptor));
    VS::BindFrameInfo(pass, host_buffer.EmplaceUniform(frame_info));
    FS::BindFragInfo(pass, host_buffer.EmplaceUniform(frag_info));

    return pass.Draw().ok();
  };
  std::shared_ptr<CommandBuffer> command_buffer =
      renderer.GetContext()->CreateCommandBuffer();
  if (command_buffer == nullptr) {
    return std::nullopt;
  }

  fml::StatusOr<RenderTarget> render_target =
      renderer.MakeSubpass("Directional Morphology Filter",
                           ISize(coverage.GetSize()), command_buffer, callback);
  if (!render_target.ok()) {
    return std::nullopt;
  }

  if (!renderer.GetContext()->EnqueueCommandBuffer(std::move(command_buffer))) {
    return std::nullopt;
  }

  SamplerDescriptor sampler_desc;
  sampler_desc.min_filter = MinMagFilter::kLinear;
  sampler_desc.mag_filter = MinMagFilter::kLinear;

  return Entity::FromSnapshot(
      Snapshot{.texture = render_target.value().GetRenderTargetTexture(),
               .transform = Matrix::MakeTranslation(coverage.GetOrigin()),
               .sampler_descriptor = sampler_desc,
               .opacity = input_snapshot->opacity},
      entity.GetBlendMode());
}

std::optional<Rect> DirectionalMorphologyFilterContents::GetFilterCoverage(
    const FilterInput::Vector& inputs,
    const Entity& entity,
    const Matrix& effect_transform) const {
  if (inputs.empty()) {
    return std::nullopt;
  }

  auto coverage = inputs[0]->GetCoverage(entity);
  if (!coverage.has_value()) {
    return std::nullopt;
  }
  auto transform = inputs[0]->GetTransform(entity) * effect_transform.Basis();
  auto transformed_vector =
      transform.TransformDirection(direction_ * radius_.radius).Abs();

  auto origin = coverage->GetOrigin();
  auto size = Vector2(coverage->GetSize());
  switch (morph_type_) {
    case FilterContents::MorphType::kDilate:
      origin -= transformed_vector;
      size += transformed_vector * 2;
      break;
    case FilterContents::MorphType::kErode:
      origin += transformed_vector;
      size -= transformed_vector * 2;
      break;
  }
  if (size.x < 0 || size.y < 0) {
    return Rect::MakeSize(Size(0, 0));
  }
  return Rect::MakeOriginSize(origin, Size(size.x, size.y));
}

std::optional<Rect>
DirectionalMorphologyFilterContents::GetFilterSourceCoverage(
    const Matrix& effect_transform,
    const Rect& output_limit) const {
  auto transformed_vector =
      effect_transform.TransformDirection(direction_ * radius_.radius).Abs();
  switch (morph_type_) {
    case FilterContents::MorphType::kDilate:
      return output_limit.Expand(-transformed_vector);
    case FilterContents::MorphType::kErode:
      return output_limit.Expand(transformed_vector);
  }
}

}  // namespace impeller
