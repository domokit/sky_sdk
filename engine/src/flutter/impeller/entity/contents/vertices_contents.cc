// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "vertices_contents.h"

#include "fml/logging.h"
#include "impeller/base/validation.h"
#include "impeller/core/formats.h"
#include "impeller/entity/contents/content_context.h"
#include "impeller/entity/contents/contents.h"
#include "impeller/entity/contents/filters/blend_filter_contents.h"
#include "impeller/entity/geometry/geometry.h"
#include "impeller/entity/geometry/vertices_geometry.h"
#include "impeller/geometry/color.h"
#include "impeller/renderer/render_pass.h"

namespace impeller {

namespace {
static std::optional<SamplerAddressMode> TileModeToAddressMode(
    Entity::TileMode tile_mode,
    const Capabilities& capabilities) {
  switch (tile_mode) {
    case Entity::TileMode::kClamp:
      return SamplerAddressMode::kClampToEdge;
      break;
    case Entity::TileMode::kMirror:
      return SamplerAddressMode::kMirror;
      break;
    case Entity::TileMode::kRepeat:
      return SamplerAddressMode::kRepeat;
      break;
    case Entity::TileMode::kDecal:
      if (capabilities.SupportsDecalSamplerAddressMode()) {
        return SamplerAddressMode::kDecal;
      }
      return std::nullopt;
  }
}
}  // namespace

//------------------------------------------------------
// VerticesSimpleBlendContents

VerticesSimpleBlendContents::VerticesSimpleBlendContents() {}

VerticesSimpleBlendContents::~VerticesSimpleBlendContents() {}

void VerticesSimpleBlendContents::SetGeometry(
    std::shared_ptr<VerticesGeometry> geometry) {
  geometry_ = std::move(geometry);
}

void VerticesSimpleBlendContents::SetAlpha(Scalar alpha) {
  alpha_ = alpha;
}

void VerticesSimpleBlendContents::SetBlendMode(BlendMode blend_mode) {
  blend_mode_ = blend_mode;
}

void VerticesSimpleBlendContents::SetTexture(std::shared_ptr<Texture> texture) {
  texture_ = std::move(texture);
}

std::optional<Rect> VerticesSimpleBlendContents::GetCoverage(
    const Entity& entity) const {
  return geometry_->GetCoverage(entity.GetTransform());
}

void VerticesSimpleBlendContents::SetSamplerDescriptor(
    const SamplerDescriptor& descriptor) {
  descriptor_ = descriptor;
}

void VerticesSimpleBlendContents::SetTileMode(Entity::TileMode tile_mode_x,
                                              Entity::TileMode tile_mode_y) {
  tile_mode_x_ = tile_mode_x;
  tile_mode_y_ = tile_mode_y;
}

void VerticesSimpleBlendContents::SetEffectTransform(Matrix transform) {
  inverse_matrix_ = transform.Invert();
}

void VerticesSimpleBlendContents::SetLazyTexture(
    const LazyTexture& lazy_texture) {
  lazy_texture_ = lazy_texture;
}

void VerticesSimpleBlendContents::SetLazyTextureCoverage(Rect rect) {
  lazy_texture_coverage_ = rect;
}

bool VerticesSimpleBlendContents::Render(const ContentContext& renderer,
                                         const Entity& entity,
                                         RenderPass& pass) const {
  FML_DCHECK(texture_ || lazy_texture_ ||
             blend_mode_ == BlendMode::kDestination);
  BlendMode blend_mode = blend_mode_;
  if (!geometry_->HasVertexColors()) {
    blend_mode = BlendMode::kSource;
  }

  std::shared_ptr<Texture> texture;
  if (blend_mode != BlendMode::kDestination) {
    if (!texture_) {
      texture = lazy_texture_(renderer);
    } else {
      texture = texture_;
    }
  } else {
    texture = renderer.GetEmptyTexture();
  }
  if (!texture) {
    VALIDATION_LOG << "Missing texture for VerticesSimpleBlendContents";
    return false;
  }

  auto dst_sampler_descriptor = descriptor_;
  dst_sampler_descriptor.width_address_mode =
      TileModeToAddressMode(tile_mode_x_, renderer.GetDeviceCapabilities())
          .value_or(SamplerAddressMode::kClampToEdge);
  dst_sampler_descriptor.height_address_mode =
      TileModeToAddressMode(tile_mode_y_, renderer.GetDeviceCapabilities())
          .value_or(SamplerAddressMode::kClampToEdge);

  raw_ptr<const Sampler> dst_sampler =
      renderer.GetContext()->GetSamplerLibrary()->GetSampler(
          dst_sampler_descriptor);

  GeometryResult geometry_result = geometry_->GetPositionUVColorBuffer(
      lazy_texture_coverage_.has_value() ? lazy_texture_coverage_.value()
                                         : Rect::MakeSize(texture->GetSize()),
      inverse_matrix_, renderer, entity, pass);
  if (geometry_result.vertex_buffer.vertex_count == 0) {
    return true;
  }
  FML_DCHECK(geometry_result.mode == GeometryResult::Mode::kNormal);

  if (blend_mode <= Entity::kLastPipelineBlendMode) {
    using VS = PorterDuffBlendPipeline::VertexShader;
    using FS = PorterDuffBlendPipeline::FragmentShader;

#ifdef IMPELLER_DEBUG
    pass.SetCommandLabel(SPrintF("DrawVertices Porterduff Blend (%s)",
                                 BlendModeToString(blend_mode)));
#endif  // IMPELLER_DEBUG
    pass.SetVertexBuffer(std::move(geometry_result.vertex_buffer));

    auto options = OptionsFromPassAndEntity(pass, entity);
    options.primitive_type = geometry_result.type;
    pass.SetPipeline(renderer.GetPorterDuffBlendPipeline(options));

    FS::BindTextureSamplerDst(pass, texture, dst_sampler);

    VS::FrameInfo frame_info;
    FS::FragInfo frag_info;

    frame_info.texture_sampler_y_coord_scale = texture->GetYCoordScale();
    frame_info.mvp = geometry_result.transform;

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

    // These values are ignored if the platform supports native decal mode.
    frag_info.tmx = static_cast<int>(tile_mode_x_);
    frag_info.tmy = static_cast<int>(tile_mode_y_);

    auto& host_buffer = renderer.GetTransientsBuffer();
    FS::BindFragInfo(pass, host_buffer.EmplaceUniform(frag_info));
    VS::BindFrameInfo(pass, host_buffer.EmplaceUniform(frame_info));

    return pass.Draw().ok();
  }

  using VS = VerticesUberShader::VertexShader;
  using FS = VerticesUberShader::FragmentShader;

#ifdef IMPELLER_DEBUG
  pass.SetCommandLabel(SPrintF("DrawVertices Advanced Blend (%s)",
                               BlendModeToString(blend_mode)));
#endif  // IMPELLER_DEBUG
  pass.SetVertexBuffer(std::move(geometry_result.vertex_buffer));

  auto options = OptionsFromPassAndEntity(pass, entity);
  options.primitive_type = geometry_result.type;
  pass.SetPipeline(renderer.GetDrawVerticesUberShader(options));

  FS::BindTextureSampler(pass, texture, dst_sampler);

  VS::FrameInfo frame_info;
  FS::FragInfo frag_info;

  frame_info.texture_sampler_y_coord_scale = texture->GetYCoordScale();
  frame_info.mvp = geometry_result.transform;
  frag_info.alpha = alpha_;
  frag_info.blend_mode = static_cast<int>(blend_mode);

  // These values are ignored if the platform supports native decal mode.
  frag_info.tmx = static_cast<int>(tile_mode_x_);
  frag_info.tmy = static_cast<int>(tile_mode_y_);

  auto& host_buffer = renderer.GetTransientsBuffer();
  FS::BindFragInfo(pass, host_buffer.EmplaceUniform(frag_info));
  VS::BindFrameInfo(pass, host_buffer.EmplaceUniform(frame_info));

  return pass.Draw().ok();
}

}  // namespace impeller
