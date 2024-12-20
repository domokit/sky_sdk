// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "radial_gradient_contents.h"

#include "impeller/entity/contents/clip_contents.h"
#include "impeller/entity/contents/content_context.h"
#include "impeller/entity/contents/gradient_generator.h"
#include "impeller/entity/entity.h"
#include "impeller/entity/geometry/geometry.h"
#include "impeller/geometry/gradient.h"
#include "impeller/renderer/render_pass.h"

namespace impeller {

RadialGradientContents::RadialGradientContents() = default;

RadialGradientContents::~RadialGradientContents() = default;

void RadialGradientContents::SetCenterAndRadius(Point center, Scalar radius) {
  center_ = center;
  radius_ = radius;
}

void RadialGradientContents::SetTileMode(Entity::TileMode tile_mode) {
  tile_mode_ = tile_mode;
}

void RadialGradientContents::SetColors(std::vector<Color> colors) {
  colors_ = std::move(colors);
}

void RadialGradientContents::SetStops(std::vector<Scalar> stops) {
  stops_ = std::move(stops);
}

const std::vector<Color>& RadialGradientContents::GetColors() const {
  return colors_;
}

const std::vector<Scalar>& RadialGradientContents::GetStops() const {
  return stops_;
}

bool RadialGradientContents::IsOpaque(const Matrix& transform) const {
  if (GetOpacityFactor() < 1 || tile_mode_ == Entity::TileMode::kDecal) {
    return false;
  }
  for (auto color : colors_) {
    if (!color.IsOpaque()) {
      return false;
    }
  }
  return !AppliesAlphaForStrokeCoverage(transform);
}

#define ARRAY_LEN(a) (sizeof(a) / sizeof(a[0]))
#define UNIFORM_FRAG_INFO(t) \
  t##GradientUniformFillPipeline::FragmentShader::FragInfo
#define UNIFORM_COLOR_SIZE ARRAY_LEN(UNIFORM_FRAG_INFO(Radial)::colors)
#define UNIFORM_STOP_SIZE ARRAY_LEN(UNIFORM_FRAG_INFO(Radial)::stop_pairs)
static_assert(UNIFORM_COLOR_SIZE == kMaxUniformGradientStops);
static_assert(UNIFORM_STOP_SIZE == kMaxUniformGradientStops / 2);

bool RadialGradientContents::Render(const ContentContext& renderer,
                                    const Entity& entity,
                                    RenderPass& pass) const {
  if (renderer.GetDeviceCapabilities().SupportsSSBO()) {
    return RenderSSBO(renderer, entity, pass);
  }
  if (colors_.size() <= kMaxUniformGradientStops &&
      stops_.size() <= kMaxUniformGradientStops) {
    return RenderUniform(renderer, entity, pass);
  }
  return RenderTexture(renderer, entity, pass);
}

bool RadialGradientContents::RenderSSBO(const ContentContext& renderer,
                                        const Entity& entity,
                                        RenderPass& pass) const {
  using VS = RadialGradientSSBOFillPipeline::VertexShader;
  using FS = RadialGradientSSBOFillPipeline::FragmentShader;

  VS::FrameInfo frame_info;
  frame_info.matrix = GetInverseEffectTransform();

  PipelineBuilderCallback pipeline_callback =
      [&renderer](ContentContextOptions options) {
        return renderer.GetRadialGradientSSBOFillPipeline(options);
      };
  return ColorSourceContents::DrawGeometry<VS>(
      renderer, entity, pass, pipeline_callback, frame_info,
      [this, &renderer, &entity](RenderPass& pass) {
        FS::FragInfo frag_info;
        frag_info.center = center_;
        frag_info.radius = radius_;
        frag_info.tile_mode = static_cast<Scalar>(tile_mode_);
        frag_info.decal_border_color = decal_border_color_;
        frag_info.alpha =
            GetOpacityFactor() *
            GetGeometry()->ComputeAlphaCoverage(entity.GetTransform());

        auto& host_buffer = renderer.GetTransientsBuffer();
        auto colors = CreateGradientColors(colors_, stops_);

        frag_info.colors_length = colors.size();
        auto color_buffer =
            host_buffer.Emplace(colors.data(), colors.size() * sizeof(StopData),
                                DefaultUniformAlignment());

        pass.SetCommandLabel("RadialGradientSSBOFill");
        FS::BindFragInfo(
            pass, renderer.GetTransientsBuffer().EmplaceUniform(frag_info));
        FS::BindColorData(pass, color_buffer);

        return true;
      });
}

bool RadialGradientContents::RenderUniform(const ContentContext& renderer,
                                           const Entity& entity,
                                           RenderPass& pass) const {
  using VS = RadialGradientUniformFillPipeline::VertexShader;
  using FS = RadialGradientUniformFillPipeline::FragmentShader;

  VS::FrameInfo frame_info;
  frame_info.matrix = GetInverseEffectTransform();

  PipelineBuilderCallback pipeline_callback =
      [&renderer](ContentContextOptions options) {
        return renderer.GetRadialGradientUniformFillPipeline(options);
      };
  return ColorSourceContents::DrawGeometry<VS>(
      renderer, entity, pass, pipeline_callback, frame_info,
      [this, &renderer, &entity](RenderPass& pass) {
        FS::FragInfo frag_info;
        frag_info.center = center_;
        frag_info.radius = radius_;
        frag_info.tile_mode = static_cast<Scalar>(tile_mode_);
        frag_info.alpha =
            GetOpacityFactor() *
            GetGeometry()->ComputeAlphaCoverage(entity.GetTransform());
        frag_info.colors_length = PopulateUniformGradientColors(
            colors_, stops_, frag_info.colors, frag_info.stop_pairs);
        frag_info.decal_border_color = decal_border_color_;

        pass.SetCommandLabel("RadialGradientUniformFill");

        FS::BindFragInfo(
            pass, renderer.GetTransientsBuffer().EmplaceUniform(frag_info));

        return true;
      });
}

bool RadialGradientContents::RenderTexture(const ContentContext& renderer,
                                           const Entity& entity,
                                           RenderPass& pass) const {
  using VS = RadialGradientFillPipeline::VertexShader;
  using FS = RadialGradientFillPipeline::FragmentShader;

  auto gradient_data = CreateGradientBuffer(colors_, stops_);
  auto gradient_texture =
      CreateGradientTexture(gradient_data, renderer.GetContext());
  if (gradient_texture == nullptr) {
    return false;
  }

  VS::FrameInfo frame_info;
  frame_info.matrix = GetInverseEffectTransform();

  PipelineBuilderCallback pipeline_callback =
      [&renderer](ContentContextOptions options) {
        return renderer.GetRadialGradientFillPipeline(options);
      };
  return ColorSourceContents::DrawGeometry<VS>(
      renderer, entity, pass, pipeline_callback, frame_info,
      [this, &renderer, &gradient_texture, &entity](RenderPass& pass) {
        FS::FragInfo frag_info;
        frag_info.center = center_;
        frag_info.radius = radius_;
        frag_info.tile_mode = static_cast<Scalar>(tile_mode_);
        frag_info.decal_border_color = decal_border_color_;
        frag_info.texture_sampler_y_coord_scale =
            gradient_texture->GetYCoordScale();
        frag_info.alpha =
            GetOpacityFactor() *
            GetGeometry()->ComputeAlphaCoverage(entity.GetTransform());
        frag_info.half_texel =
            Vector2(0.5 / gradient_texture->GetSize().width,
                    0.5 / gradient_texture->GetSize().height);

        SamplerDescriptor sampler_desc;
        sampler_desc.min_filter = MinMagFilter::kLinear;
        sampler_desc.mag_filter = MinMagFilter::kLinear;

        pass.SetCommandLabel("RadialGradientFill");

        FS::BindFragInfo(
            pass, renderer.GetTransientsBuffer().EmplaceUniform(frag_info));
        FS::BindTextureSampler(
            pass, gradient_texture,
            renderer.GetContext()->GetSamplerLibrary()->GetSampler(
                sampler_desc));

        return true;
      });
}

bool RadialGradientContents::ApplyColorFilter(
    const ColorFilterProc& color_filter_proc) {
  for (Color& color : colors_) {
    color = color_filter_proc(color);
  }
  decal_border_color_ = color_filter_proc(decal_border_color_);
  return true;
}

}  // namespace impeller
