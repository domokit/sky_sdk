// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/entity/contents/content_context.h"

#include <memory>
#include <utility>

#include "fml/trace_event.h"
#include "impeller/base/validation.h"
#include "impeller/core/formats.h"
#include "impeller/core/texture_descriptor.h"
#include "impeller/entity/contents/framebuffer_blend_contents.h"
#include "impeller/entity/entity.h"
#include "impeller/entity/render_target_cache.h"
#include "impeller/renderer/command_buffer.h"
#include "impeller/renderer/pipeline_descriptor.h"
#include "impeller/renderer/pipeline_library.h"
#include "impeller/renderer/render_target.h"
#include "impeller/renderer/texture_mipmap.h"
#include "impeller/tessellator/tessellator.h"
#include "impeller/typographer/typographer_context.h"

namespace impeller {

void ContentContextOptions::ApplyToPipelineDescriptor(
    PipelineDescriptor& desc) const {
  auto pipeline_blend = blend_mode;
  if (blend_mode > Entity::kLastPipelineBlendMode) {
    VALIDATION_LOG << "Cannot use blend mode " << static_cast<int>(blend_mode)
                   << " as a pipeline blend.";
    pipeline_blend = BlendMode::kSourceOver;
  }

  desc.SetSampleCount(sample_count);

  ColorAttachmentDescriptor color0 = *desc.GetColorAttachmentDescriptor(0u);
  color0.format = color_attachment_pixel_format;
  color0.alpha_blend_op = BlendOperation::kAdd;
  color0.color_blend_op = BlendOperation::kAdd;
  color0.write_mask = ColorWriteMaskBits::kAll;

  switch (pipeline_blend) {
    case BlendMode::kClear:
      if (is_for_rrect_blur_clear) {
        color0.alpha_blend_op = BlendOperation::kReverseSubtract;
        color0.color_blend_op = BlendOperation::kReverseSubtract;
        color0.dst_alpha_blend_factor = BlendFactor::kOne;
        color0.dst_color_blend_factor = BlendFactor::kOne;
        color0.src_alpha_blend_factor = BlendFactor::kDestinationColor;
        color0.src_color_blend_factor = BlendFactor::kDestinationColor;
      } else {
        color0.dst_alpha_blend_factor = BlendFactor::kZero;
        color0.dst_color_blend_factor = BlendFactor::kZero;
        color0.src_alpha_blend_factor = BlendFactor::kZero;
        color0.src_color_blend_factor = BlendFactor::kZero;
      }
      break;
    case BlendMode::kSource:
      color0.blending_enabled = false;
      color0.dst_alpha_blend_factor = BlendFactor::kZero;
      color0.dst_color_blend_factor = BlendFactor::kZero;
      color0.src_alpha_blend_factor = BlendFactor::kOne;
      color0.src_color_blend_factor = BlendFactor::kOne;
      break;
    case BlendMode::kDestination:
      color0.dst_alpha_blend_factor = BlendFactor::kOne;
      color0.dst_color_blend_factor = BlendFactor::kOne;
      color0.src_alpha_blend_factor = BlendFactor::kZero;
      color0.src_color_blend_factor = BlendFactor::kZero;
      color0.write_mask = ColorWriteMaskBits::kNone;
      break;
    case BlendMode::kSourceOver:
      color0.dst_alpha_blend_factor = BlendFactor::kOneMinusSourceAlpha;
      color0.dst_color_blend_factor = BlendFactor::kOneMinusSourceAlpha;
      color0.src_alpha_blend_factor = BlendFactor::kOne;
      color0.src_color_blend_factor = BlendFactor::kOne;
      break;
    case BlendMode::kDestinationOver:
      color0.dst_alpha_blend_factor = BlendFactor::kOne;
      color0.dst_color_blend_factor = BlendFactor::kOne;
      color0.src_alpha_blend_factor = BlendFactor::kOneMinusDestinationAlpha;
      color0.src_color_blend_factor = BlendFactor::kOneMinusDestinationAlpha;
      break;
    case BlendMode::kSourceIn:
      color0.dst_alpha_blend_factor = BlendFactor::kZero;
      color0.dst_color_blend_factor = BlendFactor::kZero;
      color0.src_alpha_blend_factor = BlendFactor::kDestinationAlpha;
      color0.src_color_blend_factor = BlendFactor::kDestinationAlpha;
      break;
    case BlendMode::kDestinationIn:
      color0.dst_alpha_blend_factor = BlendFactor::kSourceAlpha;
      color0.dst_color_blend_factor = BlendFactor::kSourceAlpha;
      color0.src_alpha_blend_factor = BlendFactor::kZero;
      color0.src_color_blend_factor = BlendFactor::kZero;
      break;
    case BlendMode::kSourceOut:
      color0.dst_alpha_blend_factor = BlendFactor::kZero;
      color0.dst_color_blend_factor = BlendFactor::kZero;
      color0.src_alpha_blend_factor = BlendFactor::kOneMinusDestinationAlpha;
      color0.src_color_blend_factor = BlendFactor::kOneMinusDestinationAlpha;
      break;
    case BlendMode::kDestinationOut:
      color0.dst_alpha_blend_factor = BlendFactor::kOneMinusSourceAlpha;
      color0.dst_color_blend_factor = BlendFactor::kOneMinusSourceAlpha;
      color0.src_alpha_blend_factor = BlendFactor::kZero;
      color0.src_color_blend_factor = BlendFactor::kZero;
      break;
    case BlendMode::kSourceATop:
      color0.dst_alpha_blend_factor = BlendFactor::kOneMinusSourceAlpha;
      color0.dst_color_blend_factor = BlendFactor::kOneMinusSourceAlpha;
      color0.src_alpha_blend_factor = BlendFactor::kDestinationAlpha;
      color0.src_color_blend_factor = BlendFactor::kDestinationAlpha;
      break;
    case BlendMode::kDestinationATop:
      color0.dst_alpha_blend_factor = BlendFactor::kSourceAlpha;
      color0.dst_color_blend_factor = BlendFactor::kSourceAlpha;
      color0.src_alpha_blend_factor = BlendFactor::kOneMinusDestinationAlpha;
      color0.src_color_blend_factor = BlendFactor::kOneMinusDestinationAlpha;
      break;
    case BlendMode::kXor:
      color0.dst_alpha_blend_factor = BlendFactor::kOneMinusSourceAlpha;
      color0.dst_color_blend_factor = BlendFactor::kOneMinusSourceAlpha;
      color0.src_alpha_blend_factor = BlendFactor::kOneMinusDestinationAlpha;
      color0.src_color_blend_factor = BlendFactor::kOneMinusDestinationAlpha;
      break;
    case BlendMode::kPlus:
      color0.dst_alpha_blend_factor = BlendFactor::kOne;
      color0.dst_color_blend_factor = BlendFactor::kOne;
      color0.src_alpha_blend_factor = BlendFactor::kOne;
      color0.src_color_blend_factor = BlendFactor::kOne;
      break;
    case BlendMode::kModulate:
      color0.dst_alpha_blend_factor = BlendFactor::kSourceAlpha;
      color0.dst_color_blend_factor = BlendFactor::kSourceColor;
      color0.src_alpha_blend_factor = BlendFactor::kZero;
      color0.src_color_blend_factor = BlendFactor::kZero;
      break;
    default:
      FML_UNREACHABLE();
  }
  desc.SetColorAttachmentDescriptor(0u, color0);

  if (!has_depth_stencil_attachments) {
    desc.ClearDepthAttachment();
    desc.ClearStencilAttachments();
  }

  auto maybe_stencil = desc.GetFrontStencilAttachmentDescriptor();
  auto maybe_depth = desc.GetDepthStencilAttachmentDescriptor();
  FML_DCHECK(has_depth_stencil_attachments == maybe_depth.has_value())
      << "Depth attachment doesn't match expected pipeline state. "
         "has_depth_stencil_attachments="
      << has_depth_stencil_attachments;
  FML_DCHECK(has_depth_stencil_attachments == maybe_stencil.has_value())
      << "Stencil attachment doesn't match expected pipeline state. "
         "has_depth_stencil_attachments="
      << has_depth_stencil_attachments;
  if (maybe_stencil.has_value()) {
    StencilAttachmentDescriptor front_stencil = maybe_stencil.value();
    StencilAttachmentDescriptor back_stencil = front_stencil;

    switch (stencil_mode) {
      case StencilMode::kIgnore:
        front_stencil.stencil_compare = CompareFunction::kAlways;
        front_stencil.depth_stencil_pass = StencilOperation::kKeep;
        desc.SetStencilAttachmentDescriptors(front_stencil);
        break;
      case StencilMode::kStencilNonZeroFill:
        // The stencil ref should be 0 on commands that use this mode.
        front_stencil.stencil_compare = CompareFunction::kAlways;
        front_stencil.depth_stencil_pass = StencilOperation::kIncrementWrap;
        back_stencil.stencil_compare = CompareFunction::kAlways;
        back_stencil.depth_stencil_pass = StencilOperation::kDecrementWrap;
        desc.SetStencilAttachmentDescriptors(front_stencil, back_stencil);
        break;
      case StencilMode::kStencilEvenOddFill:
        // The stencil ref should be 0 on commands that use this mode.
        front_stencil.stencil_compare = CompareFunction::kEqual;
        front_stencil.depth_stencil_pass = StencilOperation::kIncrementWrap;
        front_stencil.stencil_failure = StencilOperation::kDecrementWrap;
        desc.SetStencilAttachmentDescriptors(front_stencil);
        break;
      case StencilMode::kCoverCompare:
        // The stencil ref should be 0 on commands that use this mode.
        front_stencil.stencil_compare = CompareFunction::kNotEqual;
        front_stencil.depth_stencil_pass =
            StencilOperation::kSetToReferenceValue;
        desc.SetStencilAttachmentDescriptors(front_stencil);
        break;
      case StencilMode::kCoverCompareInverted:
        // The stencil ref should be 0 on commands that use this mode.
        front_stencil.stencil_compare = CompareFunction::kEqual;
        front_stencil.stencil_failure = StencilOperation::kSetToReferenceValue;
        desc.SetStencilAttachmentDescriptors(front_stencil);
        break;
      case StencilMode::kOverdrawPreventionIncrement:
        front_stencil.stencil_compare = CompareFunction::kEqual;
        front_stencil.depth_stencil_pass = StencilOperation::kIncrementClamp;
        desc.SetStencilAttachmentDescriptors(front_stencil);
        break;
      case StencilMode::kOverdrawPreventionRestore:
        front_stencil.stencil_compare = CompareFunction::kLess;
        front_stencil.depth_stencil_pass =
            StencilOperation::kSetToReferenceValue;
        desc.SetStencilAttachmentDescriptors(front_stencil);
        break;
    }
  }
  if (maybe_depth.has_value()) {
    DepthAttachmentDescriptor depth = maybe_depth.value();
    depth.depth_write_enabled = depth_write_enabled;
    depth.depth_compare = depth_compare;
    desc.SetDepthStencilAttachmentDescriptor(depth);
  }

  desc.SetPrimitiveType(primitive_type);

  desc.SetPolygonMode(wireframe ? PolygonMode::kLine : PolygonMode::kFill);
}

template <typename PipelineT>
static std::unique_ptr<PipelineT> CreateDefaultPipeline(
    const Context& context) {
  auto desc = PipelineT::Builder::MakeDefaultPipelineDescriptor(context);
  if (!desc.has_value()) {
    return nullptr;
  }
  // Apply default ContentContextOptions to the descriptor.
  const auto default_color_format =
      context.GetCapabilities()->GetDefaultColorFormat();
  ContentContextOptions{.sample_count = SampleCount::kCount4,
                        .primitive_type = PrimitiveType::kTriangleStrip,
                        .color_attachment_pixel_format = default_color_format}
      .ApplyToPipelineDescriptor(*desc);
  return std::make_unique<PipelineT>(context, desc);
}

ContentContext::ContentContext(
    std::shared_ptr<Context> context,
    std::shared_ptr<TypographerContext> typographer_context,
    std::shared_ptr<RenderTargetAllocator> render_target_allocator)
    : context_(std::move(context)),
      lazy_glyph_atlas_(
          std::make_shared<LazyGlyphAtlas>(std::move(typographer_context))),
      tessellator_(std::make_shared<Tessellator>()),
      render_target_cache_(render_target_allocator == nullptr
                               ? std::make_shared<RenderTargetCache>(
                                     context_->GetResourceAllocator())
                               : std::move(render_target_allocator)),
      host_buffer_(HostBuffer::Create(context_->GetResourceAllocator(),
                                      context_->GetIdleWaiter())) {
  if (!context_ || !context_->IsValid()) {
    return;
  }

  {
    TextureDescriptor desc;
    desc.storage_mode = StorageMode::kDevicePrivate;
    desc.format = PixelFormat::kR8G8B8A8UNormInt;
    desc.size = ISize{1, 1};
    empty_texture_ = GetContext()->GetResourceAllocator()->CreateTexture(desc);
    auto data = Color::BlackTransparent().ToR8G8B8A8();
    auto cmd_buffer = GetContext()->CreateCommandBuffer();
    auto blit_pass = cmd_buffer->CreateBlitPass();
    auto& host_buffer = GetTransientsBuffer();
    auto buffer_view = host_buffer.Emplace(data);
    blit_pass->AddCopy(buffer_view, empty_texture_);

    if (!blit_pass->EncodeCommands() || !GetContext()
                                             ->GetCommandQueue()
                                             ->Submit({std::move(cmd_buffer)})
                                             .ok()) {
      VALIDATION_LOG << "Failed to create empty texture.";
    }
  }

  auto options = ContentContextOptions{
      .sample_count = SampleCount::kCount4,
      .color_attachment_pixel_format =
          context_->GetCapabilities()->GetDefaultColorFormat()};
  auto options_trianglestrip = ContentContextOptions{
      .sample_count = SampleCount::kCount4,
      .primitive_type = PrimitiveType::kTriangleStrip,
      .color_attachment_pixel_format =
          context_->GetCapabilities()->GetDefaultColorFormat()};
  const auto supports_decal = static_cast<Scalar>(
      context_->GetCapabilities()->SupportsDecalSamplerAddressMode());

  // Futures for the following pipelines may block in case the first frame is
  // rendered without the pipelines being ready. Put pipelines that are more
  // likely to be used first.
  {
    glyph_atlas_pipelines_.CreateDefault(
        *context_, options,
        {static_cast<Scalar>(
            GetContext()->GetCapabilities()->GetDefaultGlyphAtlasFormat() ==
            PixelFormat::kA8UNormInt)});
    solid_fill_pipelines_.CreateDefault(*context_, options);
    texture_pipelines_.CreateDefault(*context_, options);
    fast_gradient_pipelines_.CreateDefault(*context_, options);

    if (context_->GetCapabilities()->SupportsSSBO()) {
      linear_gradient_ssbo_fill_pipelines_.CreateDefault(*context_, options);
      radial_gradient_ssbo_fill_pipelines_.CreateDefault(*context_, options);
      conical_gradient_ssbo_fill_pipelines_.CreateDefault(*context_, options);
      sweep_gradient_ssbo_fill_pipelines_.CreateDefault(*context_, options);
    } else {
      linear_gradient_uniform_fill_pipelines_.CreateDefault(*context_, options);
      radial_gradient_uniform_fill_pipelines_.CreateDefault(*context_, options);
      conical_gradient_uniform_fill_pipelines_.CreateDefault(*context_,
                                                             options);
      sweep_gradient_uniform_fill_pipelines_.CreateDefault(*context_, options);

      linear_gradient_fill_pipelines_.CreateDefault(*context_, options);
      radial_gradient_fill_pipelines_.CreateDefault(*context_, options);
      conical_gradient_fill_pipelines_.CreateDefault(*context_, options);
      sweep_gradient_fill_pipelines_.CreateDefault(*context_, options);
    }

    /// Setup default clip pipeline.
    auto clip_pipeline_descriptor =
        ClipPipeline::Builder::MakeDefaultPipelineDescriptor(*context_);
    if (!clip_pipeline_descriptor.has_value()) {
      return;
    }
    ContentContextOptions{
        .sample_count = SampleCount::kCount4,
        .color_attachment_pixel_format =
            context_->GetCapabilities()->GetDefaultColorFormat()}
        .ApplyToPipelineDescriptor(*clip_pipeline_descriptor);
    // Disable write to all color attachments.
    auto clip_color_attachments =
        clip_pipeline_descriptor->GetColorAttachmentDescriptors();
    for (auto& color_attachment : clip_color_attachments) {
      color_attachment.second.write_mask = ColorWriteMaskBits::kNone;
    }
    clip_pipeline_descriptor->SetColorAttachmentDescriptors(
        std::move(clip_color_attachments));
    clip_pipelines_.SetDefault(
        options,
        std::make_unique<ClipPipeline>(*context_, clip_pipeline_descriptor));
    texture_downsample_pipelines_.CreateDefault(*context_,
                                                options_trianglestrip);
    rrect_blur_pipelines_.CreateDefault(*context_, options_trianglestrip);
    texture_strict_src_pipelines_.CreateDefault(*context_, options);
    tiled_texture_pipelines_.CreateDefault(*context_, options,
                                           {supports_decal});
    gaussian_blur_pipelines_.CreateDefault(*context_, options_trianglestrip,
                                           {supports_decal});
    border_mask_blur_pipelines_.CreateDefault(*context_, options_trianglestrip);
    color_matrix_color_filter_pipelines_.CreateDefault(*context_,
                                                       options_trianglestrip);
    porter_duff_blend_pipelines_.CreateDefault(*context_, options_trianglestrip,
                                               {supports_decal});
    vertices_uber_shader_.CreateDefault(*context_, options, {supports_decal});
  }

  if (context_->GetCapabilities()->SupportsFramebufferFetch()) {
    framebuffer_blend_color_pipelines_.CreateDefault(
        *context_, options_trianglestrip,
        {static_cast<Scalar>(BlendSelectValues::kColor), supports_decal});
    framebuffer_blend_colorburn_pipelines_.CreateDefault(
        *context_, options_trianglestrip,
        {static_cast<Scalar>(BlendSelectValues::kColorBurn), supports_decal});
    framebuffer_blend_colordodge_pipelines_.CreateDefault(
        *context_, options_trianglestrip,
        {static_cast<Scalar>(BlendSelectValues::kColorDodge), supports_decal});
    framebuffer_blend_darken_pipelines_.CreateDefault(
        *context_, options_trianglestrip,
        {static_cast<Scalar>(BlendSelectValues::kDarken), supports_decal});
    framebuffer_blend_difference_pipelines_.CreateDefault(
        *context_, options_trianglestrip,
        {static_cast<Scalar>(BlendSelectValues::kDifference), supports_decal});
    framebuffer_blend_exclusion_pipelines_.CreateDefault(
        *context_, options_trianglestrip,
        {static_cast<Scalar>(BlendSelectValues::kExclusion), supports_decal});
    framebuffer_blend_hardlight_pipelines_.CreateDefault(
        *context_, options_trianglestrip,
        {static_cast<Scalar>(BlendSelectValues::kHardLight), supports_decal});
    framebuffer_blend_hue_pipelines_.CreateDefault(
        *context_, options_trianglestrip,
        {static_cast<Scalar>(BlendSelectValues::kHue), supports_decal});
    framebuffer_blend_lighten_pipelines_.CreateDefault(
        *context_, options_trianglestrip,
        {static_cast<Scalar>(BlendSelectValues::kLighten), supports_decal});
    framebuffer_blend_luminosity_pipelines_.CreateDefault(
        *context_, options_trianglestrip,
        {static_cast<Scalar>(BlendSelectValues::kLuminosity), supports_decal});
    framebuffer_blend_multiply_pipelines_.CreateDefault(
        *context_, options_trianglestrip,
        {static_cast<Scalar>(BlendSelectValues::kMultiply), supports_decal});
    framebuffer_blend_overlay_pipelines_.CreateDefault(
        *context_, options_trianglestrip,
        {static_cast<Scalar>(BlendSelectValues::kOverlay), supports_decal});
    framebuffer_blend_saturation_pipelines_.CreateDefault(
        *context_, options_trianglestrip,
        {static_cast<Scalar>(BlendSelectValues::kSaturation), supports_decal});
    framebuffer_blend_screen_pipelines_.CreateDefault(
        *context_, options_trianglestrip,
        {static_cast<Scalar>(BlendSelectValues::kScreen), supports_decal});
    framebuffer_blend_softlight_pipelines_.CreateDefault(
        *context_, options_trianglestrip,
        {static_cast<Scalar>(BlendSelectValues::kSoftLight), supports_decal});
  } else {
    blend_color_pipelines_.CreateDefault(
        *context_, options_trianglestrip,
        {static_cast<Scalar>(BlendSelectValues::kColor), supports_decal});
    blend_colorburn_pipelines_.CreateDefault(
        *context_, options_trianglestrip,
        {static_cast<Scalar>(BlendSelectValues::kColorBurn), supports_decal});
    blend_colordodge_pipelines_.CreateDefault(
        *context_, options_trianglestrip,
        {static_cast<Scalar>(BlendSelectValues::kColorDodge), supports_decal});
    blend_darken_pipelines_.CreateDefault(
        *context_, options_trianglestrip,
        {static_cast<Scalar>(BlendSelectValues::kDarken), supports_decal});
    blend_difference_pipelines_.CreateDefault(
        *context_, options_trianglestrip,
        {static_cast<Scalar>(BlendSelectValues::kDifference), supports_decal});
    blend_exclusion_pipelines_.CreateDefault(
        *context_, options_trianglestrip,
        {static_cast<Scalar>(BlendSelectValues::kExclusion), supports_decal});
    blend_hardlight_pipelines_.CreateDefault(
        *context_, options_trianglestrip,
        {static_cast<Scalar>(BlendSelectValues::kHardLight), supports_decal});
    blend_hue_pipelines_.CreateDefault(
        *context_, options_trianglestrip,
        {static_cast<Scalar>(BlendSelectValues::kHue), supports_decal});
    blend_lighten_pipelines_.CreateDefault(
        *context_, options_trianglestrip,
        {static_cast<Scalar>(BlendSelectValues::kLighten), supports_decal});
    blend_luminosity_pipelines_.CreateDefault(
        *context_, options_trianglestrip,
        {static_cast<Scalar>(BlendSelectValues::kLuminosity), supports_decal});
    blend_multiply_pipelines_.CreateDefault(
        *context_, options_trianglestrip,
        {static_cast<Scalar>(BlendSelectValues::kMultiply), supports_decal});
    blend_overlay_pipelines_.CreateDefault(
        *context_, options_trianglestrip,
        {static_cast<Scalar>(BlendSelectValues::kOverlay), supports_decal});
    blend_saturation_pipelines_.CreateDefault(
        *context_, options_trianglestrip,
        {static_cast<Scalar>(BlendSelectValues::kSaturation), supports_decal});
    blend_screen_pipelines_.CreateDefault(
        *context_, options_trianglestrip,
        {static_cast<Scalar>(BlendSelectValues::kScreen), supports_decal});
    blend_softlight_pipelines_.CreateDefault(
        *context_, options_trianglestrip,
        {static_cast<Scalar>(BlendSelectValues::kSoftLight), supports_decal});
  }

  morphology_filter_pipelines_.CreateDefault(*context_, options_trianglestrip,
                                             {supports_decal});
  linear_to_srgb_filter_pipelines_.CreateDefault(*context_,
                                                 options_trianglestrip);
  srgb_to_linear_filter_pipelines_.CreateDefault(*context_,
                                                 options_trianglestrip);
  yuv_to_rgb_filter_pipelines_.CreateDefault(*context_, options_trianglestrip);

#if defined(IMPELLER_ENABLE_OPENGLES)
  if (GetContext()->GetBackendType() == Context::BackendType::kOpenGLES) {
#if !defined(FML_OS_MACOSX)
    // GLES only shader that is unsupported on macOS.
    tiled_texture_external_pipelines_.CreateDefault(*context_, options);
    tiled_texture_uv_external_pipelines_.CreateDefault(*context_, options);
#endif  // !defined(FML_OS_MACOSX)
    texture_downsample_gles_pipelines_.CreateDefault(*context_,
                                                     options_trianglestrip);
  }
#endif  // IMPELLER_ENABLE_OPENGLES

  is_valid_ = true;
  InitializeCommonlyUsedShadersIfNeeded();
}

ContentContext::~ContentContext() = default;

bool ContentContext::IsValid() const {
  return is_valid_;
}

std::shared_ptr<Texture> ContentContext::GetEmptyTexture() const {
  return empty_texture_;
}

fml::StatusOr<RenderTarget> ContentContext::MakeSubpass(
    std::string_view label,
    ISize texture_size,
    const std::shared_ptr<CommandBuffer>& command_buffer,
    const SubpassCallback& subpass_callback,
    bool msaa_enabled,
    bool depth_stencil_enabled,
    int32_t mip_count) const {
  const std::shared_ptr<Context>& context = GetContext();
  RenderTarget subpass_target;

  std::optional<RenderTarget::AttachmentConfig> depth_stencil_config =
      depth_stencil_enabled ? RenderTarget::kDefaultStencilAttachmentConfig
                            : std::optional<RenderTarget::AttachmentConfig>();

  if (context->GetCapabilities()->SupportsOffscreenMSAA() && msaa_enabled) {
    subpass_target = GetRenderTargetCache()->CreateOffscreenMSAA(
        *context, texture_size,
        /*mip_count=*/mip_count, label,
        RenderTarget::kDefaultColorAttachmentConfigMSAA, depth_stencil_config);
  } else {
    subpass_target = GetRenderTargetCache()->CreateOffscreen(
        *context, texture_size,
        /*mip_count=*/mip_count, label,
        RenderTarget::kDefaultColorAttachmentConfig, depth_stencil_config);
  }
  return MakeSubpass(label, subpass_target, command_buffer, subpass_callback);
}

fml::StatusOr<RenderTarget> ContentContext::MakeSubpass(
    std::string_view label,
    const RenderTarget& subpass_target,
    const std::shared_ptr<CommandBuffer>& command_buffer,
    const SubpassCallback& subpass_callback) const {
  const std::shared_ptr<Context>& context = GetContext();

  auto subpass_texture = subpass_target.GetRenderTargetTexture();
  if (!subpass_texture) {
    return fml::Status(fml::StatusCode::kUnknown, "");
  }

  auto sub_renderpass = command_buffer->CreateRenderPass(subpass_target);
  if (!sub_renderpass) {
    return fml::Status(fml::StatusCode::kUnknown, "");
  }
  sub_renderpass->SetLabel(label);

  if (!subpass_callback(*this, *sub_renderpass)) {
    return fml::Status(fml::StatusCode::kUnknown, "");
  }

  if (!sub_renderpass->EncodeCommands()) {
    return fml::Status(fml::StatusCode::kUnknown, "");
  }

  const std::shared_ptr<Texture>& target_texture =
      subpass_target.GetRenderTargetTexture();
  if (target_texture->GetMipCount() > 1) {
    fml::Status mipmap_status =
        AddMipmapGeneration(command_buffer, context, target_texture);
    if (!mipmap_status.ok()) {
      return mipmap_status;
    }
  }

  return subpass_target;
}

Tessellator& ContentContext::GetTessellator() const {
  return *tessellator_;
}

std::shared_ptr<Context> ContentContext::GetContext() const {
  return context_;
}

const Capabilities& ContentContext::GetDeviceCapabilities() const {
  return *context_->GetCapabilities();
}

void ContentContext::SetWireframe(bool wireframe) {
  wireframe_ = wireframe;
}

PipelineRef ContentContext::GetCachedRuntimeEffectPipeline(
    const std::string& unique_entrypoint_name,
    const ContentContextOptions& options,
    const std::function<std::shared_ptr<Pipeline<PipelineDescriptor>>()>&
        create_callback) const {
  RuntimeEffectPipelineKey key{unique_entrypoint_name, options};
  auto it = runtime_effect_pipelines_.find(key);
  if (it == runtime_effect_pipelines_.end()) {
    it = runtime_effect_pipelines_.insert(it, {key, create_callback()});
  }
  return raw_ptr(it->second);
}

void ContentContext::ClearCachedRuntimeEffectPipeline(
    const std::string& unique_entrypoint_name) const {
  for (auto it = runtime_effect_pipelines_.begin();
       it != runtime_effect_pipelines_.end();) {
    if (it->first.unique_entrypoint_name == unique_entrypoint_name) {
      it = runtime_effect_pipelines_.erase(it);
    } else {
      it++;
    }
  }
}

void ContentContext::InitializeCommonlyUsedShadersIfNeeded() const {
  TRACE_EVENT0("flutter", "InitializeCommonlyUsedShadersIfNeeded");
  GetContext()->InitializeCommonlyUsedShadersIfNeeded();
}

}  // namespace impeller
