// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "sweep_gradient_contents.h"

#include "flutter/fml/logging.h"
#include "impeller/entity/contents/clip_contents.h"
#include "impeller/entity/contents/content_context.h"
#include "impeller/entity/contents/gradient_generator.h"
#include "impeller/entity/entity.h"
#include "impeller/geometry/gradient.h"
#include "impeller/renderer/render_pass.h"

namespace impeller {

SweepGradientContents::SweepGradientContents() = default;

SweepGradientContents::~SweepGradientContents() = default;

void SweepGradientContents::SetCenterAndAngles(Point center,
                                               Degrees start_angle,
                                               Degrees end_angle) {
  center_ = center;
  Scalar t0 = start_angle.degrees / 360;
  Scalar t1 = end_angle.degrees / 360;
  FML_DCHECK(t0 < t1);
  bias_ = -t0;
  scale_ = 1 / (t1 - t0);
}

void SweepGradientContents::SetColors(std::vector<Color> colors) {
  colors_ = std::move(colors);
}

void SweepGradientContents::SetStops(std::vector<Scalar> stops) {
  stops_ = std::move(stops);
}

void SweepGradientContents::SetTileMode(Entity::TileMode tile_mode) {
  tile_mode_ = tile_mode;
}

const std::vector<Color>& SweepGradientContents::GetColors() const {
  return colors_;
}

const std::vector<Scalar>& SweepGradientContents::GetStops() const {
  return stops_;
}

bool SweepGradientContents::IsOpaque(const Matrix& transform) const {
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
#define UNIFORM_COLOR_SIZE ARRAY_LEN(UNIFORM_FRAG_INFO(Sweep)::colors)
#define UNIFORM_STOP_SIZE ARRAY_LEN(UNIFORM_FRAG_INFO(Sweep)::stop_pairs)
static_assert(UNIFORM_COLOR_SIZE == kMaxUniformGradientStops);
static_assert(UNIFORM_STOP_SIZE == kMaxUniformGradientStops / 2);

bool SweepGradientContents::Render(const ContentContext& renderer,
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

bool SweepGradientContents::RenderSSBO(const ContentContext& renderer,
                                       const Entity& entity,
                                       RenderPass& pass) const {
  using VS = SweepGradientSSBOFillPipeline::VertexShader;
  using FS = SweepGradientSSBOFillPipeline::FragmentShader;

  VS::FrameInfo frame_info;
  frame_info.matrix = GetInverseEffectTransform();
  VS::BindFrameInfo(pass,
                    renderer.GetTransientsBuffer().EmplaceUniform(frame_info));

  PipelineBuilderCallback pipeline_callback =
      [&renderer](ContentContextOptions options) {
        return renderer.GetSweepGradientSSBOFillPipeline(options);
      };
  return ColorSourceContents::DrawGeometry<VS>(
      renderer, entity, pass, pipeline_callback, frame_info,
      [this, &renderer, &entity](RenderPass& pass) {
        FS::FragInfo frag_info;
        frag_info.center = center_;
        frag_info.bias = bias_;
        frag_info.scale = scale_;
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

        pass.SetCommandLabel("SweepGradientSSBOFill");

        FS::BindFragInfo(
            pass, renderer.GetTransientsBuffer().EmplaceUniform(frag_info));
        FS::BindColorData(pass, color_buffer);

        return true;
      });
}

bool SweepGradientContents::RenderUniform(const ContentContext& renderer,
                                          const Entity& entity,
                                          RenderPass& pass) const {
  using VS = SweepGradientUniformFillPipeline::VertexShader;
  using FS = SweepGradientUniformFillPipeline::FragmentShader;

  VS::FrameInfo frame_info;
  frame_info.matrix = GetInverseEffectTransform();
  VS::BindFrameInfo(pass,
                    renderer.GetTransientsBuffer().EmplaceUniform(frame_info));

  PipelineBuilderCallback pipeline_callback =
      [&renderer](ContentContextOptions options) {
        return renderer.GetSweepGradientUniformFillPipeline(options);
      };
  return ColorSourceContents::DrawGeometry<VS>(
      renderer, entity, pass, pipeline_callback, frame_info,
      [this, &renderer, &entity](RenderPass& pass) {
        FS::FragInfo frag_info;
        frag_info.center = center_;
        frag_info.bias = bias_;
        frag_info.scale = scale_;
        frag_info.tile_mode = static_cast<Scalar>(tile_mode_);
        frag_info.alpha =
            GetOpacityFactor() *
            GetGeometry()->ComputeAlphaCoverage(entity.GetTransform());
        frag_info.colors_length = PopulateUniformGradientColors(
            colors_, stops_, frag_info.colors, frag_info.stop_pairs);

        frag_info.decal_border_color = decal_border_color_;

        pass.SetCommandLabel("SweepGradientUniformFill");

        FS::BindFragInfo(
            pass, renderer.GetTransientsBuffer().EmplaceUniform(frag_info));

        return true;
      });
}

bool SweepGradientContents::RenderTexture(const ContentContext& renderer,
                                          const Entity& entity,
                                          RenderPass& pass) const {
  using VS = SweepGradientFillPipeline::VertexShader;
  using FS = SweepGradientFillPipeline::FragmentShader;

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
        return renderer.GetSweepGradientFillPipeline(options);
      };
  return ColorSourceContents::DrawGeometry<VS>(
      renderer, entity, pass, pipeline_callback, frame_info,
      [this, &renderer, &gradient_texture, &entity](RenderPass& pass) {
        FS::FragInfo frag_info;
        frag_info.center = center_;
        frag_info.bias = bias_;
        frag_info.scale = scale_;
        frag_info.texture_sampler_y_coord_scale =
            gradient_texture->GetYCoordScale();
        frag_info.tile_mode = static_cast<Scalar>(tile_mode_);
        frag_info.decal_border_color = decal_border_color_;
        frag_info.alpha =
            GetOpacityFactor() *
            GetGeometry()->ComputeAlphaCoverage(entity.GetTransform());
        frag_info.half_texel =
            Vector2(0.5 / gradient_texture->GetSize().width,
                    0.5 / gradient_texture->GetSize().height);

        SamplerDescriptor sampler_desc;
        sampler_desc.min_filter = MinMagFilter::kLinear;
        sampler_desc.mag_filter = MinMagFilter::kLinear;

        pass.SetCommandLabel("SweepGradientFill");

        FS::BindFragInfo(
            pass, renderer.GetTransientsBuffer().EmplaceUniform(frag_info));
        FS::BindTextureSampler(
            pass, gradient_texture,
            renderer.GetContext()->GetSamplerLibrary()->GetSampler(
                sampler_desc));

        return true;
      });
}

bool SweepGradientContents::ApplyColorFilter(
    const ColorFilterProc& color_filter_proc) {
  for (Color& color : colors_) {
    color = color_filter_proc(color);
  }
  decal_border_color_ = color_filter_proc(decal_border_color_);
  return true;
}

}  // namespace impeller
