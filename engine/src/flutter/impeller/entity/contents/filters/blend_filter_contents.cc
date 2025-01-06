// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/entity/contents/filters/blend_filter_contents.h"

#include <array>
#include <memory>
#include <optional>

#include "flutter/fml/logging.h"
#include "impeller/base/strings.h"
#include "impeller/core/formats.h"
#include "impeller/core/sampler_descriptor.h"
#include "impeller/core/texture_descriptor.h"
#include "impeller/core/vertex_buffer.h"
#include "impeller/entity/contents/anonymous_contents.h"
#include "impeller/entity/contents/content_context.h"
#include "impeller/entity/contents/contents.h"
#include "impeller/entity/contents/filters/color_filter_contents.h"
#include "impeller/entity/contents/filters/inputs/filter_input.h"
#include "impeller/entity/contents/solid_color_contents.h"
#include "impeller/entity/entity.h"
#include "impeller/entity/geometry/rect_geometry.h"
#include "impeller/entity/texture_fill.frag.h"
#include "impeller/entity/texture_fill.vert.h"
#include "impeller/geometry/color.h"
#include "impeller/renderer/render_pass.h"
#include "impeller/renderer/snapshot.h"

namespace impeller {

namespace {

#ifdef IMPELLER_DEBUG

#define _IMPELLER_BLEND_MODE_FILTER_NAME_LIST(blend_mode) \
  "Blend Filter " #blend_mode,

static constexpr const char* kBlendModeFilterNames[] = {
    IMPELLER_FOR_EACH_BLEND_MODE(_IMPELLER_BLEND_MODE_FILTER_NAME_LIST)};

const std::string_view BlendModeToFilterString(BlendMode blend_mode) {
  return kBlendModeFilterNames[static_cast<std::underlying_type_t<BlendMode>>(
      blend_mode)];
}
#endif  // IMPELLER_DEBUG

}  // namespace

std::optional<BlendMode> InvertPorterDuffBlend(BlendMode blend_mode) {
  switch (blend_mode) {
    case BlendMode::kClear:
      return BlendMode::kClear;
    case BlendMode::kSource:
      return BlendMode::kDestination;
    case BlendMode::kDestination:
      return BlendMode::kSource;
    case BlendMode::kSourceOver:
      return BlendMode::kDestinationOver;
    case BlendMode::kDestinationOver:
      return BlendMode::kSourceOver;
    case BlendMode::kSourceIn:
      return BlendMode::kDestinationIn;
    case BlendMode::kDestinationIn:
      return BlendMode::kSourceIn;
    case BlendMode::kSourceOut:
      return BlendMode::kDestinationOut;
    case BlendMode::kDestinationOut:
      return BlendMode::kSourceOut;
    case BlendMode::kSourceATop:
      return BlendMode::kDestinationATop;
    case BlendMode::kDestinationATop:
      return BlendMode::kSourceATop;
    case BlendMode::kXor:
      return BlendMode::kXor;
    case BlendMode::kPlus:
      return BlendMode::kPlus;
    case BlendMode::kModulate:
      return BlendMode::kModulate;
    default:
      return std::nullopt;
  }
}

BlendFilterContents::BlendFilterContents() {
  SetBlendMode(BlendMode::kSourceOver);
}

BlendFilterContents::~BlendFilterContents() = default;

using PipelineProc =
    PipelineRef (ContentContext::*)(ContentContextOptions) const;

template <typename TPipeline>
static std::optional<Entity> AdvancedBlend(
    const FilterInput::Vector& inputs,
    const ContentContext& renderer,
    const Entity& entity,
    const Rect& coverage,
    BlendMode blend_mode,
    std::optional<Color> foreground_color,
    ColorFilterContents::AbsorbOpacity absorb_opacity,
    PipelineProc pipeline_proc,
    std::optional<Scalar> alpha) {
  using VS = typename TPipeline::VertexShader;
  using FS = typename TPipeline::FragmentShader;

  //----------------------------------------------------------------------------
  /// Handle inputs.
  ///

  const size_t total_inputs =
      inputs.size() + (foreground_color.has_value() ? 1 : 0);
  if (total_inputs < 2) {
    return std::nullopt;
  }

  auto dst_snapshot =
      inputs[0]->GetSnapshot("AdvancedBlend(Dst)", renderer, entity);
  if (!dst_snapshot.has_value()) {
    return std::nullopt;
  }
  auto maybe_dst_uvs = dst_snapshot->GetCoverageUVs(coverage);
  if (!maybe_dst_uvs.has_value()) {
    return std::nullopt;
  }
  auto dst_uvs = maybe_dst_uvs.value();

  std::optional<Snapshot> src_snapshot;
  std::array<Point, 4> src_uvs;
  if (!foreground_color.has_value()) {
    src_snapshot =
        inputs[1]->GetSnapshot("AdvancedBlend(Src)", renderer, entity);
    if (!src_snapshot.has_value()) {
      if (!dst_snapshot.has_value()) {
        return std::nullopt;
      }
      return Entity::FromSnapshot(dst_snapshot.value(), entity.GetBlendMode());
    }
    auto maybe_src_uvs = src_snapshot->GetCoverageUVs(coverage);
    if (!maybe_src_uvs.has_value()) {
      if (!dst_snapshot.has_value()) {
        return std::nullopt;
      }
      return Entity::FromSnapshot(dst_snapshot.value(), entity.GetBlendMode());
    }
    src_uvs = maybe_src_uvs.value();
  }

  Rect subpass_coverage = coverage;
  if (entity.GetContents()) {
    auto coverage_hint = entity.GetContents()->GetCoverageHint();

    if (coverage_hint.has_value()) {
      auto maybe_subpass_coverage =
          subpass_coverage.Intersection(*coverage_hint);
      if (!maybe_subpass_coverage.has_value()) {
        return std::nullopt;  // Nothing to render.
      }

      subpass_coverage = *maybe_subpass_coverage;
    }
  }

  //----------------------------------------------------------------------------
  /// Render to texture.
  ///

  ContentContext::SubpassCallback callback = [&](const ContentContext& renderer,
                                                 RenderPass& pass) {
    auto& host_buffer = renderer.GetTransientsBuffer();

    auto size = pass.GetRenderTargetSize();

    std::array<typename VS::PerVertexData, 4> vertices = {
        typename VS::PerVertexData{Point(0, 0), dst_uvs[0], src_uvs[0]},
        typename VS::PerVertexData{Point(size.width, 0), dst_uvs[1],
                                   src_uvs[1]},
        typename VS::PerVertexData{Point(0, size.height), dst_uvs[2],
                                   src_uvs[2]},
        typename VS::PerVertexData{Point(size.width, size.height), dst_uvs[3],
                                   src_uvs[3]},
    };
    auto vtx_buffer =
        CreateVertexBuffer(vertices, renderer.GetTransientsBuffer());

    auto options = OptionsFromPass(pass);
    options.primitive_type = PrimitiveType::kTriangleStrip;
    options.blend_mode = BlendMode::kSource;
    PipelineRef pipeline = std::invoke(pipeline_proc, renderer, options);

#ifdef IMPELLER_DEBUG
    pass.SetCommandLabel(BlendModeToFilterString(blend_mode));
#endif  // IMPELLER_DEBUG
    pass.SetVertexBuffer(std::move(vtx_buffer));
    pass.SetPipeline(pipeline);

    typename FS::BlendInfo blend_info;
    typename VS::FrameInfo frame_info;

    auto dst_sampler_descriptor = dst_snapshot->sampler_descriptor;
    if (renderer.GetDeviceCapabilities().SupportsDecalSamplerAddressMode()) {
      dst_sampler_descriptor.width_address_mode = SamplerAddressMode::kDecal;
      dst_sampler_descriptor.height_address_mode = SamplerAddressMode::kDecal;
    }
    raw_ptr<const Sampler> dst_sampler =
        renderer.GetContext()->GetSamplerLibrary()->GetSampler(
            dst_sampler_descriptor);
    FS::BindTextureSamplerDst(pass, dst_snapshot->texture, dst_sampler);
    frame_info.dst_y_coord_scale = dst_snapshot->texture->GetYCoordScale();
    blend_info.dst_input_alpha =
        absorb_opacity == ColorFilterContents::AbsorbOpacity::kYes
            ? dst_snapshot->opacity
            : 1.0;

    if (foreground_color.has_value()) {
      blend_info.color_factor = 1;
      blend_info.color = foreground_color.value();
      // This texture will not be sampled from due to the color factor. But
      // this is present so that validation doesn't trip on a missing
      // binding.
      FS::BindTextureSamplerSrc(pass, dst_snapshot->texture, dst_sampler);
    } else {
      auto src_sampler_descriptor = src_snapshot->sampler_descriptor;
      if (renderer.GetDeviceCapabilities().SupportsDecalSamplerAddressMode()) {
        src_sampler_descriptor.width_address_mode = SamplerAddressMode::kDecal;
        src_sampler_descriptor.height_address_mode = SamplerAddressMode::kDecal;
      }
      raw_ptr<const Sampler> src_sampler =
          renderer.GetContext()->GetSamplerLibrary()->GetSampler(
              src_sampler_descriptor);
      blend_info.color_factor = 0;
      blend_info.src_input_alpha = src_snapshot->opacity;
      FS::BindTextureSamplerSrc(pass, src_snapshot->texture, src_sampler);
      frame_info.src_y_coord_scale = src_snapshot->texture->GetYCoordScale();
    }
    auto blend_uniform = host_buffer.EmplaceUniform(blend_info);
    FS::BindBlendInfo(pass, blend_uniform);

    frame_info.mvp = pass.GetOrthographicTransform() *
                     Matrix::MakeTranslation(coverage.GetOrigin() -
                                             subpass_coverage.GetOrigin());

    auto uniform_view = host_buffer.EmplaceUniform(frame_info);
    VS::BindFrameInfo(pass, uniform_view);

    return pass.Draw().ok();
  };

  std::shared_ptr<CommandBuffer> command_buffer =
      renderer.GetContext()->CreateCommandBuffer();
  if (!command_buffer) {
    return std::nullopt;
  }
  fml::StatusOr<RenderTarget> render_target = renderer.MakeSubpass(
      "Advanced Blend Filter", ISize(subpass_coverage.GetSize()),
      command_buffer, callback);
  if (!render_target.ok()) {
    return std::nullopt;
  }
  if (!renderer.GetContext()->EnqueueCommandBuffer(std::move(command_buffer))) {
    return std::nullopt;
  }

  return Entity::FromSnapshot(
      Snapshot{
          .texture = render_target.value().GetRenderTargetTexture(),
          .transform = Matrix::MakeTranslation(subpass_coverage.GetOrigin()),
          // Since we absorbed the transform of the inputs and used the
          // respective snapshot sampling modes when blending, pass on
          // the default NN clamp sampler.
          .sampler_descriptor = {},
          .opacity = (absorb_opacity == ColorFilterContents::AbsorbOpacity::kYes
                          ? 1.0f
                          : dst_snapshot->opacity) *
                     alpha.value_or(1.0)},
      entity.GetBlendMode());
}

std::optional<Entity> BlendFilterContents::CreateForegroundAdvancedBlend(
    const std::shared_ptr<FilterInput>& input,
    const ContentContext& renderer,
    const Entity& entity,
    const Rect& coverage,
    Color foreground_color,
    BlendMode blend_mode,
    std::optional<Scalar> alpha,
    ColorFilterContents::AbsorbOpacity absorb_opacity) const {
  auto dst_snapshot =
      input->GetSnapshot("ForegroundAdvancedBlend", renderer, entity);
  if (!dst_snapshot.has_value()) {
    return std::nullopt;
  }

  RenderProc render_proc = [foreground_color, dst_snapshot, blend_mode, alpha,
                            absorb_opacity](const ContentContext& renderer,
                                            const Entity& entity,
                                            RenderPass& pass) -> bool {
    using VS = BlendScreenPipeline::VertexShader;
    using FS = BlendScreenPipeline::FragmentShader;

    auto& host_buffer = renderer.GetTransientsBuffer();
    auto size = dst_snapshot->texture->GetSize();

    std::array<VS::PerVertexData, 4> vertices = {
        VS::PerVertexData{{0, 0}, {0, 0}, {0, 0}},
        VS::PerVertexData{Point(size.width, 0), {1, 0}, {1, 0}},
        VS::PerVertexData{Point(0, size.height), {0, 1}, {0, 1}},
        VS::PerVertexData{Point(size.width, size.height), {1, 1}, {1, 1}},
    };
    auto vtx_buffer =
        CreateVertexBuffer(vertices, renderer.GetTransientsBuffer());

#ifdef IMPELLER_DEBUG
    pass.SetCommandLabel(BlendModeToFilterString(blend_mode));
#endif  // IMPELLER_DEBUG
    pass.SetVertexBuffer(std::move(vtx_buffer));
    auto options = OptionsFromPassAndEntity(pass, entity);
    options.primitive_type = PrimitiveType::kTriangleStrip;

    switch (blend_mode) {
      case BlendMode::kScreen:
        pass.SetPipeline(renderer.GetBlendScreenPipeline(options));
        break;
      case BlendMode::kOverlay:
        pass.SetPipeline(renderer.GetBlendOverlayPipeline(options));
        break;
      case BlendMode::kDarken:
        pass.SetPipeline(renderer.GetBlendDarkenPipeline(options));
        break;
      case BlendMode::kLighten:
        pass.SetPipeline(renderer.GetBlendLightenPipeline(options));
        break;
      case BlendMode::kColorDodge:
        pass.SetPipeline(renderer.GetBlendColorDodgePipeline(options));
        break;
      case BlendMode::kColorBurn:
        pass.SetPipeline(renderer.GetBlendColorBurnPipeline(options));
        break;
      case BlendMode::kHardLight:
        pass.SetPipeline(renderer.GetBlendHardLightPipeline(options));
        break;
      case BlendMode::kSoftLight:
        pass.SetPipeline(renderer.GetBlendSoftLightPipeline(options));
        break;
      case BlendMode::kDifference:
        pass.SetPipeline(renderer.GetBlendDifferencePipeline(options));
        break;
      case BlendMode::kExclusion:
        pass.SetPipeline(renderer.GetBlendExclusionPipeline(options));
        break;
      case BlendMode::kMultiply:
        pass.SetPipeline(renderer.GetBlendMultiplyPipeline(options));
        break;
      case BlendMode::kHue:
        pass.SetPipeline(renderer.GetBlendHuePipeline(options));
        break;
      case BlendMode::kSaturation:
        pass.SetPipeline(renderer.GetBlendSaturationPipeline(options));
        break;
      case BlendMode::kColor:
        pass.SetPipeline(renderer.GetBlendColorPipeline(options));
        break;
      case BlendMode::kLuminosity:
        pass.SetPipeline(renderer.GetBlendLuminosityPipeline(options));
        break;
      default:
        return false;
    }

    FS::BlendInfo blend_info;
    VS::FrameInfo frame_info;

    auto dst_sampler_descriptor = dst_snapshot->sampler_descriptor;
    if (renderer.GetDeviceCapabilities().SupportsDecalSamplerAddressMode()) {
      dst_sampler_descriptor.width_address_mode = SamplerAddressMode::kDecal;
      dst_sampler_descriptor.height_address_mode = SamplerAddressMode::kDecal;
    }
    raw_ptr<const Sampler> dst_sampler =
        renderer.GetContext()->GetSamplerLibrary()->GetSampler(
            dst_sampler_descriptor);
    FS::BindTextureSamplerDst(pass, dst_snapshot->texture, dst_sampler);
    frame_info.dst_y_coord_scale = dst_snapshot->texture->GetYCoordScale();

    frame_info.mvp = Entity::GetShaderTransform(
        entity.GetShaderClipDepth(), pass,
        entity.GetTransform() * dst_snapshot->transform);

    blend_info.dst_input_alpha =
        absorb_opacity == ColorFilterContents::AbsorbOpacity::kYes
            ? dst_snapshot->opacity * alpha.value_or(1.0)
            : 1.0;

    blend_info.color_factor = 1;
    blend_info.color = foreground_color;
    // This texture will not be sampled from due to the color factor. But
    // this is present so that validation doesn't trip on a missing
    // binding.
    FS::BindTextureSamplerSrc(pass, dst_snapshot->texture, dst_sampler);

    auto blend_uniform = host_buffer.EmplaceUniform(blend_info);
    FS::BindBlendInfo(pass, blend_uniform);

    auto uniform_view = host_buffer.EmplaceUniform(frame_info);
    VS::BindFrameInfo(pass, uniform_view);

    return pass.Draw().ok();
  };
  CoverageProc coverage_proc =
      [coverage](const Entity& entity) -> std::optional<Rect> {
    return coverage.TransformBounds(entity.GetTransform());
  };

  auto contents = AnonymousContents::Make(render_proc, coverage_proc);

  Entity sub_entity;
  sub_entity.SetContents(std::move(contents));
  sub_entity.SetBlendMode(entity.GetBlendMode());

  return sub_entity;
}

std::optional<Entity> BlendFilterContents::CreateForegroundPorterDuffBlend(
    const std::shared_ptr<FilterInput>& input,
    const ContentContext& renderer,
    const Entity& entity,
    const Rect& coverage,
    Color foreground_color,
    BlendMode blend_mode,
    std::optional<Scalar> alpha,
    ColorFilterContents::AbsorbOpacity absorb_opacity) const {
  if (blend_mode == BlendMode::kClear) {
    return std::nullopt;
  }

  auto dst_snapshot =
      input->GetSnapshot("ForegroundPorterDuffBlend", renderer, entity);
  if (!dst_snapshot.has_value()) {
    return std::nullopt;
  }

  if (blend_mode == BlendMode::kDestination) {
    return Entity::FromSnapshot(dst_snapshot.value(), entity.GetBlendMode());
  }

  RenderProc render_proc = [foreground_color, dst_snapshot, blend_mode,
                            absorb_opacity, alpha](
                               const ContentContext& renderer,
                               const Entity& entity, RenderPass& pass) -> bool {
    using VS = PorterDuffBlendPipeline::VertexShader;
    using FS = PorterDuffBlendPipeline::FragmentShader;

    auto& host_buffer = renderer.GetTransientsBuffer();
    auto size = dst_snapshot->texture->GetSize();
    auto color = foreground_color.Premultiply();

    std::array<VS::PerVertexData, 4> vertices = {
        VS::PerVertexData{{0, 0}, {0, 0}, color},
        VS::PerVertexData{Point(size.width, 0), {1, 0}, color},
        VS::PerVertexData{Point(0, size.height), {0, 1}, color},
        VS::PerVertexData{Point(size.width, size.height), {1, 1}, color},
    };
    auto vtx_buffer =
        CreateVertexBuffer(vertices, renderer.GetTransientsBuffer());

#ifdef IMPELLER_DEBUG
    pass.SetCommandLabel(BlendModeToFilterString(blend_mode));
#endif  // IMPELLER_DEBUG
    pass.SetVertexBuffer(std::move(vtx_buffer));
    auto options = OptionsFromPassAndEntity(pass, entity);
    options.primitive_type = PrimitiveType::kTriangleStrip;
    pass.SetPipeline(renderer.GetPorterDuffBlendPipeline(options));

    FS::FragInfo frag_info;
    VS::FrameInfo frame_info;

    frame_info.mvp = Entity::GetShaderTransform(
        entity.GetShaderClipDepth(), pass,
        entity.GetTransform() * dst_snapshot->transform);

    auto dst_sampler_descriptor = dst_snapshot->sampler_descriptor;
    if (renderer.GetDeviceCapabilities().SupportsDecalSamplerAddressMode()) {
      dst_sampler_descriptor.width_address_mode = SamplerAddressMode::kDecal;
      dst_sampler_descriptor.height_address_mode = SamplerAddressMode::kDecal;
    }
    raw_ptr<const Sampler> dst_sampler =
        renderer.GetContext()->GetSamplerLibrary()->GetSampler(
            dst_sampler_descriptor);
    FS::BindTextureSamplerDst(pass, dst_snapshot->texture, dst_sampler);
    frame_info.texture_sampler_y_coord_scale =
        dst_snapshot->texture->GetYCoordScale();

    frag_info.input_alpha =
        absorb_opacity == ColorFilterContents::AbsorbOpacity::kYes
            ? dst_snapshot->opacity * alpha.value_or(1.0)
            : 1.0;
    frag_info.output_alpha = 1.0;

    auto blend_coefficients =
        kPorterDuffCoefficients[static_cast<int>(blend_mode)];
    frag_info.src_coeff = blend_coefficients[0];
    frag_info.src_coeff_dst_alpha = blend_coefficients[1];
    frag_info.dst_coeff = blend_coefficients[2];
    frag_info.dst_coeff_src_alpha = blend_coefficients[3];
    frag_info.dst_coeff_src_color = blend_coefficients[4];

    FS::BindFragInfo(pass, host_buffer.EmplaceUniform(frag_info));
    VS::BindFrameInfo(pass, host_buffer.EmplaceUniform(frame_info));

    return pass.Draw().ok();
  };

  CoverageProc coverage_proc =
      [coverage](const Entity& entity) -> std::optional<Rect> {
    return coverage.TransformBounds(entity.GetTransform());
  };

  auto contents = AnonymousContents::Make(render_proc, coverage_proc);

  Entity sub_entity;
  sub_entity.SetContents(std::move(contents));
  sub_entity.SetBlendMode(entity.GetBlendMode());

  return sub_entity;
}

static std::optional<Entity> PipelineBlend(
    const FilterInput::Vector& inputs,
    const ContentContext& renderer,
    const Entity& entity,
    const Rect& coverage,
    BlendMode blend_mode,
    std::optional<Color> foreground_color,
    ColorFilterContents::AbsorbOpacity absorb_opacity,
    std::optional<Scalar> alpha) {
  using VS = TexturePipeline::VertexShader;
  using FS = TexturePipeline::FragmentShader;

  auto dst_snapshot =
      inputs[0]->GetSnapshot("PipelineBlend(Dst)", renderer, entity);
  if (!dst_snapshot.has_value()) {
    return std::nullopt;  // Nothing to render.
  }

  Rect subpass_coverage = coverage;
  if (entity.GetContents()) {
    auto coverage_hint = entity.GetContents()->GetCoverageHint();

    if (coverage_hint.has_value()) {
      auto maybe_subpass_coverage =
          subpass_coverage.Intersection(*coverage_hint);
      if (!maybe_subpass_coverage.has_value()) {
        return std::nullopt;  // Nothing to render.
      }

      subpass_coverage = *maybe_subpass_coverage;
    }
  }

  ContentContext::SubpassCallback callback = [&](const ContentContext& renderer,
                                                 RenderPass& pass) {
    auto& host_buffer = renderer.GetTransientsBuffer();

#ifdef IMPELLER_DEBUG
    pass.SetCommandLabel(BlendModeToFilterString(blend_mode));
#endif  // IMPELLER_DEBUG
    auto options = OptionsFromPass(pass);
    options.primitive_type = PrimitiveType::kTriangleStrip;

    auto add_blend_command = [&](std::optional<Snapshot> input) {
      if (!input.has_value()) {
        return false;
      }
      auto input_coverage = input->GetCoverage();
      if (!input_coverage.has_value()) {
        return false;
      }

      raw_ptr<const Sampler> sampler =
          renderer.GetContext()->GetSamplerLibrary()->GetSampler(
              input->sampler_descriptor);
      FS::BindTextureSampler(pass, input->texture, sampler);

      auto size = input->texture->GetSize();
      std::array<VS::PerVertexData, 4> vertices = {
          VS::PerVertexData{Point(0, 0), Point(0, 0)},
          VS::PerVertexData{Point(size.width, 0), Point(1, 0)},
          VS::PerVertexData{Point(0, size.height), Point(0, 1)},
          VS::PerVertexData{Point(size.width, size.height), Point(1, 1)},
      };
      pass.SetVertexBuffer(
          CreateVertexBuffer(vertices, renderer.GetTransientsBuffer()));

      VS::FrameInfo frame_info;
      frame_info.mvp = pass.GetOrthographicTransform() *
                       Matrix::MakeTranslation(-subpass_coverage.GetOrigin()) *
                       input->transform;
      frame_info.texture_sampler_y_coord_scale =
          input->texture->GetYCoordScale();

      FS::FragInfo frag_info;
      frag_info.alpha =
          absorb_opacity == ColorFilterContents::AbsorbOpacity::kYes
              ? input->opacity
              : 1.0;
      FS::BindFragInfo(pass, host_buffer.EmplaceUniform(frag_info));
      VS::BindFrameInfo(pass, host_buffer.EmplaceUniform(frame_info));

      return pass.Draw().ok();
    };

    // Draw the first texture using kSource.
    options.blend_mode = BlendMode::kSource;
    pass.SetPipeline(renderer.GetTexturePipeline(options));
    if (!add_blend_command(dst_snapshot)) {
      return true;
    }

    // Write subsequent textures using the selected blend mode.

    if (inputs.size() >= 2) {
      options.blend_mode = blend_mode;
      pass.SetPipeline(renderer.GetTexturePipeline(options));

      for (auto texture_i = inputs.begin() + 1; texture_i < inputs.end();
           texture_i++) {
        auto src_input = texture_i->get()->GetSnapshot("PipelineBlend(Src)",
                                                       renderer, entity);
        if (!add_blend_command(src_input)) {
          return true;
        }
      }
    }

    // If a foreground color is set, blend it in.

    if (foreground_color.has_value()) {
      auto contents = std::make_shared<SolidColorContents>();
      RectGeometry geom(Rect::MakeSize(pass.GetRenderTargetSize()));
      contents->SetGeometry(&geom);
      contents->SetColor(foreground_color.value());

      Entity foreground_entity;
      foreground_entity.SetBlendMode(blend_mode);
      foreground_entity.SetContents(contents);
      if (!foreground_entity.Render(renderer, pass)) {
        return false;
      }
    }

    return true;
  };

  std::shared_ptr<CommandBuffer> command_buffer =
      renderer.GetContext()->CreateCommandBuffer();
  if (!command_buffer) {
    return std::nullopt;
  }

  fml::StatusOr<RenderTarget> render_target = renderer.MakeSubpass(
      "Pipeline Blend Filter", ISize(subpass_coverage.GetSize()),
      command_buffer, callback);

  if (!render_target.ok()) {
    return std::nullopt;
  }
  if (!renderer.GetContext()->EnqueueCommandBuffer(std::move(command_buffer))) {
    return std::nullopt;
  }

  return Entity::FromSnapshot(
      Snapshot{
          .texture = render_target.value().GetRenderTargetTexture(),
          .transform = Matrix::MakeTranslation(subpass_coverage.GetOrigin()),
          // Since we absorbed the transform of the inputs and used the
          // respective snapshot sampling modes when blending, pass on
          // the default NN clamp sampler.
          .sampler_descriptor = {},
          .opacity = (absorb_opacity == ColorFilterContents::AbsorbOpacity::kYes
                          ? 1.0f
                          : dst_snapshot->opacity) *
                     alpha.value_or(1.0)},
      entity.GetBlendMode());
}

std::optional<Entity> BlendFilterContents::CreateFramebufferAdvancedBlend(
    const FilterInput::Vector& inputs,
    const ContentContext& renderer,
    const Entity& entity,
    const Rect& coverage,
    std::optional<Color> foreground_color,
    BlendMode blend_mode,
    std::optional<Scalar> alpha,
    ColorFilterContents::AbsorbOpacity absorb_opacity) const {
  // This works with either 2 contents or 1 contents and a foreground color.
  FML_DCHECK(inputs.size() == 2u ||
             (inputs.size() == 1u && foreground_color.has_value()));

  auto dst_snapshot =
      inputs[0]->GetSnapshot("ForegroundAdvancedBlend", renderer, entity);
  if (!dst_snapshot.has_value()) {
    return std::nullopt;
  }

  std::shared_ptr<Texture> foreground_texture;

  ContentContext::SubpassCallback subpass_callback = [&](const ContentContext&
                                                             renderer,
                                                         RenderPass& pass) {
    // First, we create a new render pass and populate it with the contents
    // of the  first (dst) input.
    HostBuffer& host_buffer = renderer.GetTransientsBuffer();

    {
      using FS = TextureFillFragmentShader;
      using VS = TextureFillVertexShader;

      pass.SetCommandLabel("Framebuffer Advanced Blend");
      auto pipeline_options = OptionsFromPass(pass);
      pipeline_options.primitive_type = PrimitiveType::kTriangleStrip;
      pass.SetPipeline(renderer.GetTexturePipeline(pipeline_options));

      VS::FrameInfo frame_info;
      frame_info.mvp = Matrix::MakeOrthographic(ISize(1, 1));
      frame_info.texture_sampler_y_coord_scale = 1.0;

      FS::FragInfo frag_info;
      frag_info.alpha = 1.0;

      std::array<VS::PerVertexData, 4> vertices = {
          VS::PerVertexData{{0, 0}, {0, 0}},
          VS::PerVertexData{Point(1, 0), {1, 0}},
          VS::PerVertexData{Point(0, 1), {0, 1}},
          VS::PerVertexData{Point(1, 1), {1, 1}},
      };
      pass.SetVertexBuffer(
          CreateVertexBuffer(vertices, renderer.GetTransientsBuffer()));

      VS::BindFrameInfo(pass, host_buffer.EmplaceUniform(frame_info));
      FS::BindFragInfo(pass, host_buffer.EmplaceUniform(frag_info));
      FS::BindTextureSampler(
          pass, dst_snapshot->texture,
          renderer.GetContext()->GetSamplerLibrary()->GetSampler({}));

      if (!pass.Draw().ok()) {
        return false;
      }
    }

    {
      using VS = FramebufferBlendScreenPipeline::VertexShader;
      using FS = FramebufferBlendScreenPipeline::FragmentShader;

      // Next, we render the second contents to a snapshot, or create a 1x1
      // texture for the foreground color.
      std::shared_ptr<Texture> src_texture;
      if (foreground_color.has_value()) {
        src_texture = foreground_texture;
      } else {
        auto src_snapshot =
            inputs[0]->GetSnapshot("ForegroundAdvancedBlend", renderer, entity);
        if (!src_snapshot.has_value()) {
          return false;
        }
        // This doesn't really handle the case where the transforms are wildly
        // different, but we only need to support blending two contents together
        // in limited circumstances (mask blur).
        src_texture = src_snapshot->texture;
      }

      std::array<VS::PerVertexData, 4> vertices = {
          VS::PerVertexData{Point(0, 0), Point(0, 0)},
          VS::PerVertexData{Point(1, 0), Point(1, 0)},
          VS::PerVertexData{Point(0, 1), Point(0, 1)},
          VS::PerVertexData{Point(1, 1), Point(1, 1)},
      };

      auto options = OptionsFromPass(pass);
      options.blend_mode = BlendMode::kSource;
      options.primitive_type = PrimitiveType::kTriangleStrip;

      pass.SetCommandLabel("Framebuffer Advanced Blend Filter");
      pass.SetVertexBuffer(
          CreateVertexBuffer(vertices, renderer.GetTransientsBuffer()));

      switch (blend_mode) {
        case BlendMode::kScreen:
          pass.SetPipeline(renderer.GetFramebufferBlendScreenPipeline(options));
          break;
        case BlendMode::kOverlay:
          pass.SetPipeline(
              renderer.GetFramebufferBlendOverlayPipeline(options));
          break;
        case BlendMode::kDarken:
          pass.SetPipeline(renderer.GetFramebufferBlendDarkenPipeline(options));
          break;
        case BlendMode::kLighten:
          pass.SetPipeline(
              renderer.GetFramebufferBlendLightenPipeline(options));
          break;
        case BlendMode::kColorDodge:
          pass.SetPipeline(
              renderer.GetFramebufferBlendColorDodgePipeline(options));
          break;
        case BlendMode::kColorBurn:
          pass.SetPipeline(
              renderer.GetFramebufferBlendColorBurnPipeline(options));
          break;
        case BlendMode::kHardLight:
          pass.SetPipeline(
              renderer.GetFramebufferBlendHardLightPipeline(options));
          break;
        case BlendMode::kSoftLight:
          pass.SetPipeline(
              renderer.GetFramebufferBlendSoftLightPipeline(options));
          break;
        case BlendMode::kDifference:
          pass.SetPipeline(
              renderer.GetFramebufferBlendDifferencePipeline(options));
          break;
        case BlendMode::kExclusion:
          pass.SetPipeline(
              renderer.GetFramebufferBlendExclusionPipeline(options));
          break;
        case BlendMode::kMultiply:
          pass.SetPipeline(
              renderer.GetFramebufferBlendMultiplyPipeline(options));
          break;
        case BlendMode::kHue:
          pass.SetPipeline(renderer.GetFramebufferBlendHuePipeline(options));
          break;
        case BlendMode::kSaturation:
          pass.SetPipeline(
              renderer.GetFramebufferBlendSaturationPipeline(options));
          break;
        case BlendMode::kColor:
          pass.SetPipeline(renderer.GetFramebufferBlendColorPipeline(options));
          break;
        case BlendMode::kLuminosity:
          pass.SetPipeline(
              renderer.GetFramebufferBlendLuminosityPipeline(options));
          break;
        default:
          return false;
      }

      VS::FrameInfo frame_info;
      FS::FragInfo frag_info;

      auto src_sampler_descriptor = SamplerDescriptor{};
      if (renderer.GetDeviceCapabilities().SupportsDecalSamplerAddressMode()) {
        src_sampler_descriptor.width_address_mode = SamplerAddressMode::kDecal;
        src_sampler_descriptor.height_address_mode = SamplerAddressMode::kDecal;
      }
      raw_ptr<const Sampler> src_sampler =
          renderer.GetContext()->GetSamplerLibrary()->GetSampler(
              src_sampler_descriptor);
      FS::BindTextureSamplerSrc(pass, src_texture, src_sampler);

      frame_info.mvp = Matrix::MakeOrthographic(ISize(1, 1));
      frame_info.src_y_coord_scale = src_texture->GetYCoordScale();
      VS::BindFrameInfo(pass, host_buffer.EmplaceUniform(frame_info));

      frag_info.src_input_alpha = 1.0;
      frag_info.dst_input_alpha =
          absorb_opacity == ColorFilterContents::AbsorbOpacity::kYes
              ? dst_snapshot->opacity
              : 1.0;
      FS::BindFragInfo(pass, host_buffer.EmplaceUniform(frag_info));

      return pass.Draw().ok();
    }
  };

  std::shared_ptr<CommandBuffer> cmd_buffer =
      renderer.GetContext()->CreateCommandBuffer();

  // Generate a 1x1 texture to implement foreground color blending.
  if (foreground_color.has_value()) {
    TextureDescriptor desc;
    desc.size = {1, 1};
    desc.format = PixelFormat::kR8G8B8A8UNormInt;
    desc.storage_mode = StorageMode::kDevicePrivate;
    foreground_texture =
        renderer.GetContext()->GetResourceAllocator()->CreateTexture(desc);
    if (!foreground_texture) {
      return std::nullopt;
    }
    auto blit_pass = cmd_buffer->CreateBlitPass();
    auto buffer_view = renderer.GetTransientsBuffer().Emplace(
        foreground_color->Premultiply().ToR8G8B8A8(), /*alignment=*/4);

    blit_pass->AddCopy(std::move(buffer_view), foreground_texture);
    if (!blit_pass->EncodeCommands()) {
      return std::nullopt;
    }
  }

  auto render_target =
      renderer.MakeSubpass("FramebufferBlend", dst_snapshot->texture->GetSize(),
                           cmd_buffer, subpass_callback);

  if (!render_target.ok()) {
    return std::nullopt;
  }
  if (!renderer.GetContext()->EnqueueCommandBuffer(std::move(cmd_buffer))) {
    return std::nullopt;
  }

  return Entity::FromSnapshot(
      Snapshot{
          .texture = render_target.value().GetRenderTargetTexture(),
          .transform = dst_snapshot->transform,
          // Since we absorbed the transform of the inputs and used the
          // respective snapshot sampling modes when blending, pass on
          // the default NN clamp sampler.
          .sampler_descriptor = {},
          .opacity = (absorb_opacity == ColorFilterContents::AbsorbOpacity::kYes
                          ? 1.0f
                          : dst_snapshot->opacity) *
                     alpha.value_or(1.0)},
      entity.GetBlendMode());
}

#define BLEND_CASE(mode)                                                      \
  case BlendMode::k##mode:                                                    \
    advanced_blend_proc_ =                                                    \
        [](const FilterInput::Vector& inputs, const ContentContext& renderer, \
           const Entity& entity, const Rect& coverage, BlendMode blend_mode,  \
           std::optional<Color> fg_color,                                     \
           ColorFilterContents::AbsorbOpacity absorb_opacity,                 \
           std::optional<Scalar> alpha) {                                     \
          PipelineProc p = &ContentContext::GetBlend##mode##Pipeline;         \
          return AdvancedBlend<Blend##mode##Pipeline>(                        \
              inputs, renderer, entity, coverage, blend_mode, fg_color,       \
              absorb_opacity, p, alpha);                                      \
        };                                                                    \
    break;

void BlendFilterContents::SetBlendMode(BlendMode blend_mode) {
  if (blend_mode > Entity::kLastAdvancedBlendMode) {
    VALIDATION_LOG << "Invalid blend mode " << static_cast<int>(blend_mode)
                   << " assigned to BlendFilterContents.";
  }

  blend_mode_ = blend_mode;

  if (blend_mode > Entity::kLastPipelineBlendMode) {
    switch (blend_mode) {
      BLEND_CASE(Screen)
      BLEND_CASE(Overlay)
      BLEND_CASE(Darken)
      BLEND_CASE(Lighten)
      BLEND_CASE(ColorDodge)
      BLEND_CASE(ColorBurn)
      BLEND_CASE(HardLight)
      BLEND_CASE(SoftLight)
      BLEND_CASE(Difference)
      BLEND_CASE(Exclusion)
      BLEND_CASE(Multiply)
      BLEND_CASE(Hue)
      BLEND_CASE(Saturation)
      BLEND_CASE(Color)
      BLEND_CASE(Luminosity)
      default:
        FML_UNREACHABLE();
    }
  }
}

void BlendFilterContents::SetForegroundColor(std::optional<Color> color) {
  foreground_color_ = color;
}

std::optional<Entity> BlendFilterContents::RenderFilter(
    const FilterInput::Vector& inputs,
    const ContentContext& renderer,
    const Entity& entity,
    const Matrix& effect_transform,
    const Rect& coverage,
    const std::optional<Rect>& coverage_hint) const {
  if (inputs.empty()) {
    return std::nullopt;
  }

  if (inputs.size() == 1 && !foreground_color_.has_value()) {
    // Nothing to blend.
    return PipelineBlend(inputs, renderer, entity, coverage, BlendMode::kSource,
                         std::nullopt, GetAbsorbOpacity(), GetAlpha());
  }

  if (blend_mode_ <= Entity::kLastPipelineBlendMode) {
    if (inputs.size() == 1 && foreground_color_.has_value() &&
        GetAbsorbOpacity() == ColorFilterContents::AbsorbOpacity::kYes) {
      return CreateForegroundPorterDuffBlend(
          inputs[0], renderer, entity, coverage, foreground_color_.value(),
          blend_mode_, GetAlpha(), GetAbsorbOpacity());
    }
    return PipelineBlend(inputs, renderer, entity, coverage, blend_mode_,
                         foreground_color_, GetAbsorbOpacity(), GetAlpha());
  }

  if (blend_mode_ <= Entity::kLastAdvancedBlendMode) {
    if (renderer.GetDeviceCapabilities().SupportsFramebufferFetch()) {
      return CreateFramebufferAdvancedBlend(inputs, renderer, entity, coverage,
                                            foreground_color_, blend_mode_,
                                            GetAlpha(), GetAbsorbOpacity());
    }
    if (inputs.size() == 1 && foreground_color_.has_value() &&
        GetAbsorbOpacity() == ColorFilterContents::AbsorbOpacity::kYes) {
      return CreateForegroundAdvancedBlend(
          inputs[0], renderer, entity, coverage, foreground_color_.value(),
          blend_mode_, GetAlpha(), GetAbsorbOpacity());
    }
    return advanced_blend_proc_(inputs, renderer, entity, coverage, blend_mode_,
                                foreground_color_, GetAbsorbOpacity(),
                                GetAlpha());
  }

  FML_UNREACHABLE();
}

}  // namespace impeller
