// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <optional>

#include "impeller/core/formats.h"
#include "impeller/entity/contents/atlas_contents.h"
#include "impeller/entity/contents/content_context.h"
#include "impeller/entity/contents/filters/blend_filter_contents.h"
#include "impeller/entity/entity.h"
#include "impeller/entity/texture_fill.frag.h"
#include "impeller/entity/texture_fill.vert.h"
#include "impeller/geometry/color.h"
#include "impeller/renderer/render_pass.h"

namespace impeller {

AtlasContents::AtlasContents() = default;

AtlasContents::~AtlasContents() = default;

std::optional<Rect> AtlasContents::GetCoverage(const Entity& entity) const {
  if (!geometry_) {
    return std::nullopt;
  }
  return geometry_->ComputeBoundingBox().TransformBounds(entity.GetTransform());
}

void AtlasContents::SetGeometry(AtlasGeometry* geometry) {
  geometry_ = geometry;
}

void AtlasContents::SetAlpha(Scalar alpha) {
  alpha_ = alpha;
}

bool AtlasContents::Render(const ContentContext& renderer,
                           const Entity& entity,
                           RenderPass& pass) const {
  if (geometry_->ShouldSkip() || alpha_ <= 0.0) {
    return true;
  }

  auto dst_sampler_descriptor = geometry_->GetSamplerDescriptor();
  if (renderer.GetDeviceCapabilities().SupportsDecalSamplerAddressMode()) {
    dst_sampler_descriptor.width_address_mode = SamplerAddressMode::kDecal;
    dst_sampler_descriptor.height_address_mode = SamplerAddressMode::kDecal;
  }
  raw_ptr<const Sampler> dst_sampler =
      renderer.GetContext()->GetSamplerLibrary()->GetSampler(
          dst_sampler_descriptor);

  auto& host_buffer = renderer.GetTransientsBuffer();
  if (!geometry_->ShouldUseBlend()) {
    using VS = TextureFillVertexShader;
    using FS = TextureFillFragmentShader;

    auto dst_sampler_descriptor = geometry_->GetSamplerDescriptor();

    raw_ptr<const Sampler> dst_sampler =
        renderer.GetContext()->GetSamplerLibrary()->GetSampler(
            dst_sampler_descriptor);

    auto pipeline_options = OptionsFromPassAndEntity(pass, entity);
    pipeline_options.primitive_type = PrimitiveType::kTriangle;
    pipeline_options.depth_write_enabled =
        pipeline_options.blend_mode == BlendMode::kSource;

    pass.SetPipeline(renderer.GetTexturePipeline(pipeline_options));
    pass.SetVertexBuffer(geometry_->CreateSimpleVertexBuffer(host_buffer));
#ifdef IMPELLER_DEBUG
    pass.SetCommandLabel("DrawAtlas");
#endif  // IMPELLER_DEBUG

    VS::FrameInfo frame_info;
    frame_info.mvp = entity.GetShaderTransform(pass);
    frame_info.texture_sampler_y_coord_scale =
        geometry_->GetAtlas()->GetYCoordScale();

    VS::BindFrameInfo(pass, host_buffer.EmplaceUniform(frame_info));

    FS::FragInfo frag_info;
    frag_info.alpha = alpha_;
    FS::BindFragInfo(pass, host_buffer.EmplaceUniform((frag_info)));
    FS::BindTextureSampler(pass, geometry_->GetAtlas(), dst_sampler);
    return pass.Draw().ok();
  }

  BlendMode blend_mode = geometry_->GetBlendMode();

  if (blend_mode <= BlendMode::kModulate) {
    using VS = PorterDuffBlendPipeline::VertexShader;
    using FS = PorterDuffBlendPipeline::FragmentShader;

#ifdef IMPELLER_DEBUG
    pass.SetCommandLabel("DrawAtlas Blend");
#endif  // IMPELLER_DEBUG
    pass.SetVertexBuffer(geometry_->CreateBlendVertexBuffer(host_buffer));
    pass.SetPipeline(
        renderer.GetPorterDuffBlendPipeline(OptionsFromPass(pass)));

    FS::FragInfo frag_info;
    VS::FrameInfo frame_info;

    FS::BindTextureSamplerDst(pass, geometry_->GetAtlas(), dst_sampler);
    frame_info.texture_sampler_y_coord_scale =
        geometry_->GetAtlas()->GetYCoordScale();

    frag_info.output_alpha = alpha_;
    frag_info.input_alpha = 1.0;

    auto inverted_blend_mode =
        InvertPorterDuffBlend(blend_mode).value_or(BlendMode::kSource);
    auto blend_coefficients =
        kPorterDuffCoefficients[static_cast<int>(inverted_blend_mode)];
    frag_info.src_coeff = blend_coefficients[0];
    frag_info.src_coeff_dst_alpha = blend_coefficients[1];
    frag_info.dst_coeff = blend_coefficients[2];
    frag_info.dst_coeff_src_alpha = blend_coefficients[3];
    frag_info.dst_coeff_src_color = blend_coefficients[4];
    // These values are ignored on platforms that natively support decal.
    frag_info.tmx = static_cast<int>(Entity::TileMode::kDecal);
    frag_info.tmy = static_cast<int>(Entity::TileMode::kDecal);

    FS::BindFragInfo(pass, host_buffer.EmplaceUniform(frag_info));

    frame_info.mvp = entity.GetShaderTransform(pass);

    auto uniform_view = host_buffer.EmplaceUniform(frame_info);
    VS::BindFrameInfo(pass, uniform_view);

    return pass.Draw().ok();
  }

  using VUS = VerticesUberShader::VertexShader;
  using FS = VerticesUberShader::FragmentShader;

#ifdef IMPELLER_DEBUG
  pass.SetCommandLabel("DrawAtlas Advanced Blend");
#endif  // IMPELLER_DEBUG
  pass.SetVertexBuffer(geometry_->CreateBlendVertexBuffer(host_buffer));

  pass.SetPipeline(renderer.GetDrawVerticesUberShader(OptionsFromPass(pass)));
  FS::BindTextureSampler(pass, geometry_->GetAtlas(), dst_sampler);

  VUS::FrameInfo frame_info;
  FS::FragInfo frag_info;

  frame_info.texture_sampler_y_coord_scale =
      geometry_->GetAtlas()->GetYCoordScale();
  frame_info.mvp = entity.GetShaderTransform(pass);

  frag_info.alpha = alpha_;
  frag_info.blend_mode = static_cast<int>(blend_mode);

  // These values are ignored on platforms that natively support decal.
  frag_info.tmx = static_cast<int>(Entity::TileMode::kDecal);
  frag_info.tmy = static_cast<int>(Entity::TileMode::kDecal);

  FS::BindFragInfo(pass, host_buffer.EmplaceUniform(frag_info));
  VUS::BindFrameInfo(pass, host_buffer.EmplaceUniform(frame_info));

  return pass.Draw().ok();
}

}  // namespace impeller
