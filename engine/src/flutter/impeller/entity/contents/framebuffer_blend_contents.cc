// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "framebuffer_blend_contents.h"

#include "impeller/entity/contents/content_context.h"
#include "impeller/renderer/render_pass.h"

namespace impeller {

FramebufferBlendContents::FramebufferBlendContents() = default;

FramebufferBlendContents::~FramebufferBlendContents() = default;

void FramebufferBlendContents::SetBlendMode(BlendMode blend_mode) {
  blend_mode_ = blend_mode;
}

void FramebufferBlendContents::SetChildContents(
    std::shared_ptr<Contents> child_contents) {
  child_contents_ = std::move(child_contents);
}

// |Contents|
std::optional<Rect> FramebufferBlendContents::GetCoverage(
    const Entity& entity) const {
  return child_contents_->GetCoverage(entity);
}

bool FramebufferBlendContents::Render(const ContentContext& renderer,
                                      const Entity& entity,
                                      RenderPass& pass) const {
  if (!renderer.GetDeviceCapabilities().SupportsFramebufferFetch()) {
    return false;
  }

  using VS = FramebufferBlendScreenPipeline::VertexShader;
  using FS = FramebufferBlendScreenPipeline::FragmentShader;

  auto& host_buffer = renderer.GetTransientsBuffer();

  auto src_snapshot = child_contents_->RenderToSnapshot(
      renderer,                                    // renderer
      entity,                                      // entity
      Rect::MakeSize(pass.GetRenderTargetSize()),  // coverage_limit
      std::nullopt,                                // sampler_descriptor
      true,                                        // msaa_enabled
      /*mip_count=*/1,
      "FramebufferBlendContents Snapshot");  // label

  if (!src_snapshot.has_value()) {
    return true;
  }

  auto size = src_snapshot->texture->GetSize();

  std::array<VS::PerVertexData, 4> vertices = {
      VS::PerVertexData{Point(0, 0), Point(0, 0)},
      VS::PerVertexData{Point(size.width, 0), Point(1, 0)},
      VS::PerVertexData{Point(0, size.height), Point(0, 1)},
      VS::PerVertexData{Point(size.width, size.height), Point(1, 1)},
  };

  auto options = OptionsFromPass(pass);
  options.blend_mode = BlendMode::kSource;
  options.primitive_type = PrimitiveType::kTriangleStrip;

  pass.SetCommandLabel("Framebuffer Advanced Blend Filter");
  pass.SetVertexBuffer(
      CreateVertexBuffer(vertices, renderer.GetTransientsBuffer()));

  switch (blend_mode_) {
    case BlendMode::kScreen:
      pass.SetPipeline(renderer.GetFramebufferBlendScreenPipeline(options));
      break;
    case BlendMode::kOverlay:
      pass.SetPipeline(renderer.GetFramebufferBlendOverlayPipeline(options));
      break;
    case BlendMode::kDarken:
      pass.SetPipeline(renderer.GetFramebufferBlendDarkenPipeline(options));
      break;
    case BlendMode::kLighten:
      pass.SetPipeline(renderer.GetFramebufferBlendLightenPipeline(options));
      break;
    case BlendMode::kColorDodge:
      pass.SetPipeline(renderer.GetFramebufferBlendColorDodgePipeline(options));
      break;
    case BlendMode::kColorBurn:
      pass.SetPipeline(renderer.GetFramebufferBlendColorBurnPipeline(options));
      break;
    case BlendMode::kHardLight:
      pass.SetPipeline(renderer.GetFramebufferBlendHardLightPipeline(options));
      break;
    case BlendMode::kSoftLight:
      pass.SetPipeline(renderer.GetFramebufferBlendSoftLightPipeline(options));
      break;
    case BlendMode::kDifference:
      pass.SetPipeline(renderer.GetFramebufferBlendDifferencePipeline(options));
      break;
    case BlendMode::kExclusion:
      pass.SetPipeline(renderer.GetFramebufferBlendExclusionPipeline(options));
      break;
    case BlendMode::kMultiply:
      pass.SetPipeline(renderer.GetFramebufferBlendMultiplyPipeline(options));
      break;
    case BlendMode::kHue:
      pass.SetPipeline(renderer.GetFramebufferBlendHuePipeline(options));
      break;
    case BlendMode::kSaturation:
      pass.SetPipeline(renderer.GetFramebufferBlendSaturationPipeline(options));
      break;
    case BlendMode::kColor:
      pass.SetPipeline(renderer.GetFramebufferBlendColorPipeline(options));
      break;
    case BlendMode::kLuminosity:
      pass.SetPipeline(renderer.GetFramebufferBlendLuminosityPipeline(options));
      break;
    default:
      return false;
  }

  VS::FrameInfo frame_info;
  FS::FragInfo frag_info;

  auto src_sampler_descriptor = src_snapshot->sampler_descriptor;
  if (renderer.GetDeviceCapabilities().SupportsDecalSamplerAddressMode()) {
    src_sampler_descriptor.width_address_mode = SamplerAddressMode::kDecal;
    src_sampler_descriptor.height_address_mode = SamplerAddressMode::kDecal;
  }
  raw_ptr<const Sampler> src_sampler =
      renderer.GetContext()->GetSamplerLibrary()->GetSampler(
          src_sampler_descriptor);
  FS::BindTextureSamplerSrc(pass, src_snapshot->texture, src_sampler);

  frame_info.mvp = Entity::GetShaderTransform(entity.GetShaderClipDepth(), pass,
                                              src_snapshot->transform);
  frame_info.src_y_coord_scale = src_snapshot->texture->GetYCoordScale();
  VS::BindFrameInfo(pass, host_buffer.EmplaceUniform(frame_info));

  frag_info.src_input_alpha = src_snapshot->opacity;
  frag_info.dst_input_alpha = 1.0;
  FS::BindFragInfo(pass, host_buffer.EmplaceUniform(frag_info));

  return pass.Draw().ok();
}

}  // namespace impeller
