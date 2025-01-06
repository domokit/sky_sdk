// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_ENTITY_CONTENTS_CONTENT_CONTEXT_H_
#define FLUTTER_IMPELLER_ENTITY_CONTENTS_CONTENT_CONTEXT_H_

#include <initializer_list>
#include <memory>
#include <optional>
#include <unordered_map>

#include "flutter/fml/logging.h"
#include "flutter/fml/status_or.h"
#include "impeller/base/validation.h"
#include "impeller/core/formats.h"
#include "impeller/core/host_buffer.h"
#include "impeller/renderer/capabilities.h"
#include "impeller/renderer/command_buffer.h"
#include "impeller/renderer/pipeline.h"
#include "impeller/renderer/pipeline_descriptor.h"
#include "impeller/renderer/render_target.h"
#include "impeller/typographer/lazy_glyph_atlas.h"
#include "impeller/typographer/typographer_context.h"

#include "impeller/entity/border_mask_blur.frag.h"
#include "impeller/entity/clip.frag.h"
#include "impeller/entity/clip.vert.h"
#include "impeller/entity/color_matrix_color_filter.frag.h"
#include "impeller/entity/conical_gradient_fill.frag.h"
#include "impeller/entity/fast_gradient.frag.h"
#include "impeller/entity/fast_gradient.vert.h"
#include "impeller/entity/filter_position.vert.h"
#include "impeller/entity/filter_position_uv.vert.h"
#include "impeller/entity/gaussian.frag.h"
#include "impeller/entity/glyph_atlas.frag.h"
#include "impeller/entity/glyph_atlas.vert.h"
#include "impeller/entity/gradient_fill.vert.h"
#include "impeller/entity/linear_gradient_fill.frag.h"
#include "impeller/entity/linear_to_srgb_filter.frag.h"
#include "impeller/entity/morphology_filter.frag.h"
#include "impeller/entity/porter_duff_blend.frag.h"
#include "impeller/entity/porter_duff_blend.vert.h"
#include "impeller/entity/radial_gradient_fill.frag.h"
#include "impeller/entity/rrect_blur.frag.h"
#include "impeller/entity/rrect_blur.vert.h"
#include "impeller/entity/solid_fill.frag.h"
#include "impeller/entity/solid_fill.vert.h"
#include "impeller/entity/srgb_to_linear_filter.frag.h"
#include "impeller/entity/sweep_gradient_fill.frag.h"
#include "impeller/entity/texture_downsample.frag.h"
#include "impeller/entity/texture_fill.frag.h"
#include "impeller/entity/texture_fill.vert.h"
#include "impeller/entity/texture_fill_strict_src.frag.h"
#include "impeller/entity/texture_uv_fill.vert.h"
#include "impeller/entity/tiled_texture_fill.frag.h"
#include "impeller/entity/yuv_to_rgb_filter.frag.h"

#include "impeller/entity/conical_gradient_uniform_fill.frag.h"
#include "impeller/entity/linear_gradient_uniform_fill.frag.h"
#include "impeller/entity/radial_gradient_uniform_fill.frag.h"
#include "impeller/entity/sweep_gradient_uniform_fill.frag.h"

#include "impeller/entity/conical_gradient_ssbo_fill.frag.h"
#include "impeller/entity/linear_gradient_ssbo_fill.frag.h"
#include "impeller/entity/radial_gradient_ssbo_fill.frag.h"
#include "impeller/entity/sweep_gradient_ssbo_fill.frag.h"

#include "impeller/entity/advanced_blend.frag.h"
#include "impeller/entity/advanced_blend.vert.h"

#include "impeller/entity/framebuffer_blend.frag.h"
#include "impeller/entity/framebuffer_blend.vert.h"

#include "impeller/entity/vertices_uber.frag.h"

#ifdef IMPELLER_ENABLE_OPENGLES
#include "impeller/entity/texture_downsample_gles.frag.h"
#include "impeller/entity/tiled_texture_fill_external.frag.h"
#endif  // IMPELLER_ENABLE_OPENGLES

namespace impeller {

using FastGradientPipeline =
    RenderPipelineHandle<FastGradientVertexShader, FastGradientFragmentShader>;
using LinearGradientFillPipeline =
    RenderPipelineHandle<GradientFillVertexShader,
                         LinearGradientFillFragmentShader>;
using SolidFillPipeline =
    RenderPipelineHandle<SolidFillVertexShader, SolidFillFragmentShader>;
using RadialGradientFillPipeline =
    RenderPipelineHandle<GradientFillVertexShader,
                         RadialGradientFillFragmentShader>;
using ConicalGradientFillPipeline =
    RenderPipelineHandle<GradientFillVertexShader,
                         ConicalGradientFillFragmentShader>;
using SweepGradientFillPipeline =
    RenderPipelineHandle<GradientFillVertexShader,
                         SweepGradientFillFragmentShader>;
using LinearGradientUniformFillPipeline =
    RenderPipelineHandle<GradientFillVertexShader,
                         LinearGradientUniformFillFragmentShader>;
using ConicalGradientUniformFillPipeline =
    RenderPipelineHandle<GradientFillVertexShader,
                         ConicalGradientUniformFillFragmentShader>;
using RadialGradientUniformFillPipeline =
    RenderPipelineHandle<GradientFillVertexShader,
                         RadialGradientUniformFillFragmentShader>;
using SweepGradientUniformFillPipeline =
    RenderPipelineHandle<GradientFillVertexShader,
                         SweepGradientUniformFillFragmentShader>;
using LinearGradientSSBOFillPipeline =
    RenderPipelineHandle<GradientFillVertexShader,
                         LinearGradientSsboFillFragmentShader>;
using ConicalGradientSSBOFillPipeline =
    RenderPipelineHandle<GradientFillVertexShader,
                         ConicalGradientSsboFillFragmentShader>;
using RadialGradientSSBOFillPipeline =
    RenderPipelineHandle<GradientFillVertexShader,
                         RadialGradientSsboFillFragmentShader>;
using SweepGradientSSBOFillPipeline =
    RenderPipelineHandle<GradientFillVertexShader,
                         SweepGradientSsboFillFragmentShader>;
using RRectBlurPipeline =
    RenderPipelineHandle<RrectBlurVertexShader, RrectBlurFragmentShader>;
using TexturePipeline =
    RenderPipelineHandle<TextureFillVertexShader, TextureFillFragmentShader>;
using TextureDownsamplePipeline =
    RenderPipelineHandle<TextureFillVertexShader,
                         TextureDownsampleFragmentShader>;
using TextureStrictSrcPipeline =
    RenderPipelineHandle<TextureFillVertexShader,
                         TextureFillStrictSrcFragmentShader>;
using TiledTexturePipeline =
    RenderPipelineHandle<TextureUvFillVertexShader,
                         TiledTextureFillFragmentShader>;
using GaussianBlurPipeline =
    RenderPipelineHandle<FilterPositionUvVertexShader, GaussianFragmentShader>;
using BorderMaskBlurPipeline =
    RenderPipelineHandle<FilterPositionUvVertexShader,
                         BorderMaskBlurFragmentShader>;
using MorphologyFilterPipeline =
    RenderPipelineHandle<FilterPositionUvVertexShader,
                         MorphologyFilterFragmentShader>;
using ColorMatrixColorFilterPipeline =
    RenderPipelineHandle<FilterPositionVertexShader,
                         ColorMatrixColorFilterFragmentShader>;
using LinearToSrgbFilterPipeline =
    RenderPipelineHandle<FilterPositionVertexShader,
                         LinearToSrgbFilterFragmentShader>;
using SrgbToLinearFilterPipeline =
    RenderPipelineHandle<FilterPositionVertexShader,
                         SrgbToLinearFilterFragmentShader>;
using YUVToRGBFilterPipeline =
    RenderPipelineHandle<FilterPositionVertexShader,
                         YuvToRgbFilterFragmentShader>;

using GlyphAtlasPipeline =
    RenderPipelineHandle<GlyphAtlasVertexShader, GlyphAtlasFragmentShader>;

using PorterDuffBlendPipeline =
    RenderPipelineHandle<PorterDuffBlendVertexShader,
                         PorterDuffBlendFragmentShader>;
using ClipPipeline = RenderPipelineHandle<ClipVertexShader, ClipFragmentShader>;

// Advanced blends
using BlendColorPipeline = RenderPipelineHandle<AdvancedBlendVertexShader,
                                                AdvancedBlendFragmentShader>;
using BlendColorBurnPipeline =
    RenderPipelineHandle<AdvancedBlendVertexShader,
                         AdvancedBlendFragmentShader>;
using BlendColorDodgePipeline =
    RenderPipelineHandle<AdvancedBlendVertexShader,
                         AdvancedBlendFragmentShader>;
using BlendDarkenPipeline = RenderPipelineHandle<AdvancedBlendVertexShader,
                                                 AdvancedBlendFragmentShader>;
using BlendDifferencePipeline =
    RenderPipelineHandle<AdvancedBlendVertexShader,
                         AdvancedBlendFragmentShader>;
using BlendExclusionPipeline =
    RenderPipelineHandle<AdvancedBlendVertexShader,
                         AdvancedBlendFragmentShader>;
using BlendHardLightPipeline =
    RenderPipelineHandle<AdvancedBlendVertexShader,
                         AdvancedBlendFragmentShader>;
using BlendHuePipeline = RenderPipelineHandle<AdvancedBlendVertexShader,
                                              AdvancedBlendFragmentShader>;
using BlendLightenPipeline = RenderPipelineHandle<AdvancedBlendVertexShader,
                                                  AdvancedBlendFragmentShader>;
using BlendLuminosityPipeline =
    RenderPipelineHandle<AdvancedBlendVertexShader,
                         AdvancedBlendFragmentShader>;
using BlendMultiplyPipeline = RenderPipelineHandle<AdvancedBlendVertexShader,
                                                   AdvancedBlendFragmentShader>;
using BlendOverlayPipeline = RenderPipelineHandle<AdvancedBlendVertexShader,
                                                  AdvancedBlendFragmentShader>;
using BlendSaturationPipeline =
    RenderPipelineHandle<AdvancedBlendVertexShader,
                         AdvancedBlendFragmentShader>;
using BlendScreenPipeline = RenderPipelineHandle<AdvancedBlendVertexShader,
                                                 AdvancedBlendFragmentShader>;
using BlendSoftLightPipeline =
    RenderPipelineHandle<AdvancedBlendVertexShader,
                         AdvancedBlendFragmentShader>;
// Framebuffer Advanced Blends
using FramebufferBlendColorPipeline =
    RenderPipelineHandle<FramebufferBlendVertexShader,
                         FramebufferBlendFragmentShader>;
using FramebufferBlendColorBurnPipeline =
    RenderPipelineHandle<FramebufferBlendVertexShader,
                         FramebufferBlendFragmentShader>;
using FramebufferBlendColorDodgePipeline =
    RenderPipelineHandle<FramebufferBlendVertexShader,
                         FramebufferBlendFragmentShader>;
using FramebufferBlendDarkenPipeline =
    RenderPipelineHandle<FramebufferBlendVertexShader,
                         FramebufferBlendFragmentShader>;
using FramebufferBlendDifferencePipeline =
    RenderPipelineHandle<FramebufferBlendVertexShader,
                         FramebufferBlendFragmentShader>;
using FramebufferBlendExclusionPipeline =
    RenderPipelineHandle<FramebufferBlendVertexShader,
                         FramebufferBlendFragmentShader>;
using FramebufferBlendHardLightPipeline =
    RenderPipelineHandle<FramebufferBlendVertexShader,
                         FramebufferBlendFragmentShader>;
using FramebufferBlendHuePipeline =
    RenderPipelineHandle<FramebufferBlendVertexShader,
                         FramebufferBlendFragmentShader>;
using FramebufferBlendLightenPipeline =
    RenderPipelineHandle<FramebufferBlendVertexShader,
                         FramebufferBlendFragmentShader>;
using FramebufferBlendLuminosityPipeline =
    RenderPipelineHandle<FramebufferBlendVertexShader,
                         FramebufferBlendFragmentShader>;
using FramebufferBlendMultiplyPipeline =
    RenderPipelineHandle<FramebufferBlendVertexShader,
                         FramebufferBlendFragmentShader>;
using FramebufferBlendOverlayPipeline =
    RenderPipelineHandle<FramebufferBlendVertexShader,
                         FramebufferBlendFragmentShader>;
using FramebufferBlendSaturationPipeline =
    RenderPipelineHandle<FramebufferBlendVertexShader,
                         FramebufferBlendFragmentShader>;
using FramebufferBlendScreenPipeline =
    RenderPipelineHandle<FramebufferBlendVertexShader,
                         FramebufferBlendFragmentShader>;
using FramebufferBlendSoftLightPipeline =
    RenderPipelineHandle<FramebufferBlendVertexShader,
                         FramebufferBlendFragmentShader>;

/// Draw Vertices/Atlas Uber Shader
using VerticesUberShader = RenderPipelineHandle<PorterDuffBlendVertexShader,
                                                VerticesUberFragmentShader>;

#ifdef IMPELLER_ENABLE_OPENGLES
using TiledTextureExternalPipeline =
    RenderPipelineHandle<TextureFillVertexShader,
                         TiledTextureFillExternalFragmentShader>;
using TiledTextureUvExternalPipeline =
    RenderPipelineHandle<TextureUvFillVertexShader,
                         TiledTextureFillExternalFragmentShader>;
using TextureDownsampleGlesPipeline =
    RenderPipelineHandle<TextureFillVertexShader,
                         TextureDownsampleGlesFragmentShader>;
#endif  // IMPELLER_ENABLE_OPENGLES

/// Pipeline state configuration.
///
/// Each unique combination of these options requires a different pipeline state
/// object to be built. This struct is used as a key for the per-pipeline
/// variant cache.
///
/// When adding fields to this key, reliant features should take care to limit
/// the combinatorical explosion of variations. A sufficiently complicated
/// Flutter application may easily require building hundreds of PSOs in total,
/// but they shouldn't require e.g. 10s of thousands.
struct ContentContextOptions {
  enum class StencilMode : uint8_t {
    /// Turn the stencil test off. Used when drawing without stencil-then-cover
    /// or overdraw prevention.
    kIgnore,

    // Operations used for stencil-then-cover.

    /// Draw the stencil for the NonZero fill path rule.
    ///
    /// The stencil ref should always be 0 on commands using this mode.
    kStencilNonZeroFill,
    /// Draw the stencil for the EvenOdd fill path rule.
    ///
    /// The stencil ref should always be 0 on commands using this mode.
    kStencilEvenOddFill,
    /// Used for draw calls which fill in the stenciled area. Intended to be
    /// used after `kStencilNonZeroFill` or `kStencilEvenOddFill` is used to set
    /// up the stencil buffer. Also cleans up the stencil buffer by resetting
    /// everything to zero.
    ///
    /// The stencil ref should always be 0 on commands using this mode.
    kCoverCompare,
    /// The opposite of `kCoverCompare`. Used for draw calls which fill in the
    /// non-stenciled area (intersection clips). Intended to be used after
    /// `kStencilNonZeroFill` or `kStencilEvenOddFill` is used to set up the
    /// stencil buffer. Also cleans up the stencil buffer by resetting
    /// everything to zero.
    ///
    /// The stencil ref should always be 0 on commands using this mode.
    kCoverCompareInverted,

    // Operations used for the "overdraw prevention" mechanism. This is used for
    // drawing strokes.

    /// For each fragment, increment the stencil value if it's currently zero.
    /// Discard fragments when the value is non-zero. This prevents
    /// self-overlapping strokes from drawing over themselves.
    ///
    /// Note that this is done for rendering correctness, not performance. If a
    /// stroke is drawn with a backdrop-reliant blend and self-intersects, then
    /// the intersected geometry will render incorrectly when overdrawn because
    /// we don't adjust the geometry prevent self-intersection.
    ///
    /// The stencil ref should always be 0 on commands using this mode.
    kOverdrawPreventionIncrement,
    /// Reset the stencil to a new maximum value specified by the ref (currently
    /// always 0).
    ///
    /// The stencil ref should always be 0 on commands using this mode.
    kOverdrawPreventionRestore,
  };

  SampleCount sample_count = SampleCount::kCount1;
  BlendMode blend_mode = BlendMode::kSourceOver;
  CompareFunction depth_compare = CompareFunction::kAlways;
  StencilMode stencil_mode = ContentContextOptions::StencilMode::kIgnore;
  PrimitiveType primitive_type = PrimitiveType::kTriangle;
  PixelFormat color_attachment_pixel_format = PixelFormat::kUnknown;
  bool has_depth_stencil_attachments = true;
  bool depth_write_enabled = false;
  bool wireframe = false;
  bool is_for_rrect_blur_clear = false;

  constexpr uint64_t ToKey() const {
    static_assert(sizeof(sample_count) == 1);
    static_assert(sizeof(blend_mode) == 1);
    static_assert(sizeof(sample_count) == 1);
    static_assert(sizeof(depth_compare) == 1);
    static_assert(sizeof(stencil_mode) == 1);
    static_assert(sizeof(primitive_type) == 1);
    static_assert(sizeof(color_attachment_pixel_format) == 1);

    return (is_for_rrect_blur_clear ? 1llu : 0llu) << 0 |
           (wireframe ? 1llu : 0llu) << 1 |
           (has_depth_stencil_attachments ? 1llu : 0llu) << 2 |
           (depth_write_enabled ? 1llu : 0llu) << 3 |
           // enums
           static_cast<uint64_t>(color_attachment_pixel_format) << 8 |
           static_cast<uint64_t>(primitive_type) << 16 |
           static_cast<uint64_t>(stencil_mode) << 24 |
           static_cast<uint64_t>(depth_compare) << 32 |
           static_cast<uint64_t>(blend_mode) << 40 |
           static_cast<uint64_t>(sample_count) << 48;
  }

  void ApplyToPipelineDescriptor(PipelineDescriptor& desc) const;
};

class Tessellator;
class RenderTargetCache;

class ContentContext {
 public:
  explicit ContentContext(
      std::shared_ptr<Context> context,
      std::shared_ptr<TypographerContext> typographer_context,
      std::shared_ptr<RenderTargetAllocator> render_target_allocator = nullptr);

  ~ContentContext();

  bool IsValid() const;

  Tessellator& GetTessellator() const;

  PipelineRef GetFastGradientPipeline(ContentContextOptions opts) const {
    return GetPipeline(fast_gradient_pipelines_, opts);
  }

  PipelineRef GetLinearGradientFillPipeline(ContentContextOptions opts) const {
    return GetPipeline(linear_gradient_fill_pipelines_, opts);
  }

  PipelineRef GetLinearGradientUniformFillPipeline(
      ContentContextOptions opts) const {
    return GetPipeline(linear_gradient_uniform_fill_pipelines_, opts);
  }

  PipelineRef GetRadialGradientUniformFillPipeline(
      ContentContextOptions opts) const {
    return GetPipeline(radial_gradient_uniform_fill_pipelines_, opts);
  }

  PipelineRef GetConicalGradientUniformFillPipeline(
      ContentContextOptions opts) const {
    return GetPipeline(conical_gradient_uniform_fill_pipelines_, opts);
  }

  PipelineRef GetSweepGradientUniformFillPipeline(
      ContentContextOptions opts) const {
    return GetPipeline(sweep_gradient_uniform_fill_pipelines_, opts);
  }

  PipelineRef GetLinearGradientSSBOFillPipeline(
      ContentContextOptions opts) const {
    FML_DCHECK(GetDeviceCapabilities().SupportsSSBO());
    return GetPipeline(linear_gradient_ssbo_fill_pipelines_, opts);
  }

  PipelineRef GetRadialGradientSSBOFillPipeline(
      ContentContextOptions opts) const {
    FML_DCHECK(GetDeviceCapabilities().SupportsSSBO());
    return GetPipeline(radial_gradient_ssbo_fill_pipelines_, opts);
  }

  PipelineRef GetConicalGradientSSBOFillPipeline(
      ContentContextOptions opts) const {
    FML_DCHECK(GetDeviceCapabilities().SupportsSSBO());
    return GetPipeline(conical_gradient_ssbo_fill_pipelines_, opts);
  }

  PipelineRef GetSweepGradientSSBOFillPipeline(
      ContentContextOptions opts) const {
    FML_DCHECK(GetDeviceCapabilities().SupportsSSBO());
    return GetPipeline(sweep_gradient_ssbo_fill_pipelines_, opts);
  }

  PipelineRef GetRadialGradientFillPipeline(ContentContextOptions opts) const {
    return GetPipeline(radial_gradient_fill_pipelines_, opts);
  }

  PipelineRef GetConicalGradientFillPipeline(ContentContextOptions opts) const {
    return GetPipeline(conical_gradient_fill_pipelines_, opts);
  }

  PipelineRef GetRRectBlurPipeline(ContentContextOptions opts) const {
    return GetPipeline(rrect_blur_pipelines_, opts);
  }

  PipelineRef GetSweepGradientFillPipeline(ContentContextOptions opts) const {
    return GetPipeline(sweep_gradient_fill_pipelines_, opts);
  }

  PipelineRef GetSolidFillPipeline(ContentContextOptions opts) const {
    return GetPipeline(solid_fill_pipelines_, opts);
  }

  PipelineRef GetTexturePipeline(ContentContextOptions opts) const {
    return GetPipeline(texture_pipelines_, opts);
  }

  PipelineRef GetTextureStrictSrcPipeline(ContentContextOptions opts) const {
    return GetPipeline(texture_strict_src_pipelines_, opts);
  }

#ifdef IMPELLER_ENABLE_OPENGLES
  PipelineRef GetDownsampleTextureGlesPipeline(
      ContentContextOptions opts) const {
    return GetPipeline(texture_downsample_gles_pipelines_, opts);
  }

  PipelineRef GetTiledTextureExternalPipeline(
      ContentContextOptions opts) const {
    FML_DCHECK(GetContext()->GetBackendType() ==
               Context::BackendType::kOpenGLES);
    return GetPipeline(tiled_texture_external_pipelines_, opts);
  }

  PipelineRef GetTiledTextureUvExternalPipeline(
      ContentContextOptions opts) const {
    FML_DCHECK(GetContext()->GetBackendType() ==
               Context::BackendType::kOpenGLES);
    return GetPipeline(tiled_texture_uv_external_pipelines_, opts);
  }
#endif  // IMPELLER_ENABLE_OPENGLES

  PipelineRef GetTiledTexturePipeline(ContentContextOptions opts) const {
    return GetPipeline(tiled_texture_pipelines_, opts);
  }

  PipelineRef GetGaussianBlurPipeline(ContentContextOptions opts) const {
    return GetPipeline(gaussian_blur_pipelines_, opts);
  }

  PipelineRef GetBorderMaskBlurPipeline(ContentContextOptions opts) const {
    return GetPipeline(border_mask_blur_pipelines_, opts);
  }

  PipelineRef GetMorphologyFilterPipeline(ContentContextOptions opts) const {
    return GetPipeline(morphology_filter_pipelines_, opts);
  }

  PipelineRef GetColorMatrixColorFilterPipeline(
      ContentContextOptions opts) const {
    return GetPipeline(color_matrix_color_filter_pipelines_, opts);
  }

  PipelineRef GetLinearToSrgbFilterPipeline(ContentContextOptions opts) const {
    return GetPipeline(linear_to_srgb_filter_pipelines_, opts);
  }

  PipelineRef GetSrgbToLinearFilterPipeline(ContentContextOptions opts) const {
    return GetPipeline(srgb_to_linear_filter_pipelines_, opts);
  }

  PipelineRef GetClipPipeline(ContentContextOptions opts) const {
    return GetPipeline(clip_pipelines_, opts);
  }

  PipelineRef GetGlyphAtlasPipeline(ContentContextOptions opts) const {
    return GetPipeline(glyph_atlas_pipelines_, opts);
  }

  PipelineRef GetYUVToRGBFilterPipeline(ContentContextOptions opts) const {
    return GetPipeline(yuv_to_rgb_filter_pipelines_, opts);
  }

  PipelineRef GetPorterDuffBlendPipeline(ContentContextOptions opts) const {
    return GetPipeline(porter_duff_blend_pipelines_, opts);
  }

  // Advanced blends.

  PipelineRef GetBlendColorPipeline(ContentContextOptions opts) const {
    return GetPipeline(blend_color_pipelines_, opts);
  }

  PipelineRef GetBlendColorBurnPipeline(ContentContextOptions opts) const {
    return GetPipeline(blend_colorburn_pipelines_, opts);
  }

  PipelineRef GetBlendColorDodgePipeline(ContentContextOptions opts) const {
    return GetPipeline(blend_colordodge_pipelines_, opts);
  }

  PipelineRef GetBlendDarkenPipeline(ContentContextOptions opts) const {
    return GetPipeline(blend_darken_pipelines_, opts);
  }

  PipelineRef GetBlendDifferencePipeline(ContentContextOptions opts) const {
    return GetPipeline(blend_difference_pipelines_, opts);
  }

  PipelineRef GetBlendExclusionPipeline(ContentContextOptions opts) const {
    return GetPipeline(blend_exclusion_pipelines_, opts);
  }

  PipelineRef GetBlendHardLightPipeline(ContentContextOptions opts) const {
    return GetPipeline(blend_hardlight_pipelines_, opts);
  }

  PipelineRef GetBlendHuePipeline(ContentContextOptions opts) const {
    return GetPipeline(blend_hue_pipelines_, opts);
  }

  PipelineRef GetBlendLightenPipeline(ContentContextOptions opts) const {
    return GetPipeline(blend_lighten_pipelines_, opts);
  }

  PipelineRef GetBlendLuminosityPipeline(ContentContextOptions opts) const {
    return GetPipeline(blend_luminosity_pipelines_, opts);
  }

  PipelineRef GetBlendMultiplyPipeline(ContentContextOptions opts) const {
    return GetPipeline(blend_multiply_pipelines_, opts);
  }

  PipelineRef GetBlendOverlayPipeline(ContentContextOptions opts) const {
    return GetPipeline(blend_overlay_pipelines_, opts);
  }

  PipelineRef GetBlendSaturationPipeline(ContentContextOptions opts) const {
    return GetPipeline(blend_saturation_pipelines_, opts);
  }

  PipelineRef GetBlendScreenPipeline(ContentContextOptions opts) const {
    return GetPipeline(blend_screen_pipelines_, opts);
  }

  PipelineRef GetBlendSoftLightPipeline(ContentContextOptions opts) const {
    return GetPipeline(blend_softlight_pipelines_, opts);
  }

  PipelineRef GetDownsamplePipeline(ContentContextOptions opts) const {
    return GetPipeline(texture_downsample_pipelines_, opts);
  }

  // Framebuffer Advanced Blends
  PipelineRef GetFramebufferBlendColorPipeline(
      ContentContextOptions opts) const {
    FML_DCHECK(GetDeviceCapabilities().SupportsFramebufferFetch());
    return GetPipeline(framebuffer_blend_color_pipelines_, opts);
  }

  PipelineRef GetFramebufferBlendColorBurnPipeline(
      ContentContextOptions opts) const {
    FML_DCHECK(GetDeviceCapabilities().SupportsFramebufferFetch());
    return GetPipeline(framebuffer_blend_colorburn_pipelines_, opts);
  }

  PipelineRef GetFramebufferBlendColorDodgePipeline(
      ContentContextOptions opts) const {
    FML_DCHECK(GetDeviceCapabilities().SupportsFramebufferFetch());
    return GetPipeline(framebuffer_blend_colordodge_pipelines_, opts);
  }

  PipelineRef GetFramebufferBlendDarkenPipeline(
      ContentContextOptions opts) const {
    FML_DCHECK(GetDeviceCapabilities().SupportsFramebufferFetch());
    return GetPipeline(framebuffer_blend_darken_pipelines_, opts);
  }

  PipelineRef GetFramebufferBlendDifferencePipeline(
      ContentContextOptions opts) const {
    FML_DCHECK(GetDeviceCapabilities().SupportsFramebufferFetch());
    return GetPipeline(framebuffer_blend_difference_pipelines_, opts);
  }

  PipelineRef GetFramebufferBlendExclusionPipeline(
      ContentContextOptions opts) const {
    FML_DCHECK(GetDeviceCapabilities().SupportsFramebufferFetch());
    return GetPipeline(framebuffer_blend_exclusion_pipelines_, opts);
  }

  PipelineRef GetFramebufferBlendHardLightPipeline(
      ContentContextOptions opts) const {
    FML_DCHECK(GetDeviceCapabilities().SupportsFramebufferFetch());
    return GetPipeline(framebuffer_blend_hardlight_pipelines_, opts);
  }

  PipelineRef GetFramebufferBlendHuePipeline(ContentContextOptions opts) const {
    FML_DCHECK(GetDeviceCapabilities().SupportsFramebufferFetch());
    return GetPipeline(framebuffer_blend_hue_pipelines_, opts);
  }

  PipelineRef GetFramebufferBlendLightenPipeline(
      ContentContextOptions opts) const {
    FML_DCHECK(GetDeviceCapabilities().SupportsFramebufferFetch());
    return GetPipeline(framebuffer_blend_lighten_pipelines_, opts);
  }

  PipelineRef GetFramebufferBlendLuminosityPipeline(
      ContentContextOptions opts) const {
    FML_DCHECK(GetDeviceCapabilities().SupportsFramebufferFetch());
    return GetPipeline(framebuffer_blend_luminosity_pipelines_, opts);
  }

  PipelineRef GetFramebufferBlendMultiplyPipeline(
      ContentContextOptions opts) const {
    FML_DCHECK(GetDeviceCapabilities().SupportsFramebufferFetch());
    return GetPipeline(framebuffer_blend_multiply_pipelines_, opts);
  }

  PipelineRef GetFramebufferBlendOverlayPipeline(
      ContentContextOptions opts) const {
    FML_DCHECK(GetDeviceCapabilities().SupportsFramebufferFetch());
    return GetPipeline(framebuffer_blend_overlay_pipelines_, opts);
  }

  PipelineRef GetFramebufferBlendSaturationPipeline(
      ContentContextOptions opts) const {
    FML_DCHECK(GetDeviceCapabilities().SupportsFramebufferFetch());
    return GetPipeline(framebuffer_blend_saturation_pipelines_, opts);
  }

  PipelineRef GetFramebufferBlendScreenPipeline(
      ContentContextOptions opts) const {
    FML_DCHECK(GetDeviceCapabilities().SupportsFramebufferFetch());
    return GetPipeline(framebuffer_blend_screen_pipelines_, opts);
  }

  PipelineRef GetFramebufferBlendSoftLightPipeline(
      ContentContextOptions opts) const {
    FML_DCHECK(GetDeviceCapabilities().SupportsFramebufferFetch());
    return GetPipeline(framebuffer_blend_softlight_pipelines_, opts);
  }

  PipelineRef GetDrawVerticesUberShader(ContentContextOptions opts) const {
    return GetPipeline(vertices_uber_shader_, opts);
  }

  // An empty 1x1 texture for binding drawVertices/drawAtlas or other cases
  // that don't always have a texture (due to blending).
  std::shared_ptr<Texture> GetEmptyTexture() const;

  std::shared_ptr<Context> GetContext() const;

  const Capabilities& GetDeviceCapabilities() const;

  void SetWireframe(bool wireframe);

  using SubpassCallback =
      std::function<bool(const ContentContext&, RenderPass&)>;

  /// @brief  Creates a new texture of size `texture_size` and calls
  ///         `subpass_callback` with a `RenderPass` for drawing to the texture.
  fml::StatusOr<RenderTarget> MakeSubpass(
      std::string_view label,
      ISize texture_size,
      const std::shared_ptr<CommandBuffer>& command_buffer,
      const SubpassCallback& subpass_callback,
      bool msaa_enabled = true,
      bool depth_stencil_enabled = false,
      int32_t mip_count = 1) const;

  /// Makes a subpass that will render to `subpass_target`.
  fml::StatusOr<RenderTarget> MakeSubpass(
      std::string_view label,
      const RenderTarget& subpass_target,
      const std::shared_ptr<CommandBuffer>& command_buffer,
      const SubpassCallback& subpass_callback) const;

  const std::shared_ptr<LazyGlyphAtlas>& GetLazyGlyphAtlas() const {
    return lazy_glyph_atlas_;
  }

  const std::shared_ptr<RenderTargetAllocator>& GetRenderTargetCache() const {
    return render_target_cache_;
  }

  /// RuntimeEffect pipelines must be obtained via this method to avoid
  /// re-creating them every frame.
  ///
  /// The unique_entrypoint_name comes from RuntimeEffect::GetEntrypoint.
  /// Impellerc generates a unique entrypoint name for runtime effect shaders
  /// based on the input file name and shader stage.
  ///
  /// The create_callback is synchronously invoked exactly once if a cached
  /// pipeline is not found.
  PipelineRef GetCachedRuntimeEffectPipeline(
      const std::string& unique_entrypoint_name,
      const ContentContextOptions& options,
      const std::function<std::shared_ptr<Pipeline<PipelineDescriptor>>()>&
          create_callback) const;

  /// Used by hot reload/hot restart to clear a cached pipeline from
  /// GetCachedRuntimeEffectPipeline.
  void ClearCachedRuntimeEffectPipeline(
      const std::string& unique_entrypoint_name) const;

  /// @brief Retrieve the currnent host buffer for transient storage.
  ///
  /// This is only safe to use from the raster threads. Other threads should
  /// allocate their own device buffers.
  HostBuffer& GetTransientsBuffer() const { return *host_buffer_; }

 private:
  std::shared_ptr<Context> context_;
  std::shared_ptr<LazyGlyphAtlas> lazy_glyph_atlas_;

  /// Run backend specific additional setup and create common shader variants.
  ///
  /// This bootstrap is intended to improve the performance of several
  /// first frame benchmarks that are tracked in the flutter device lab.
  /// The workload includes initializing commonly used but not default
  /// shader variants, as well as forcing driver initialization.
  void InitializeCommonlyUsedShadersIfNeeded() const;

  struct RuntimeEffectPipelineKey {
    std::string unique_entrypoint_name;
    ContentContextOptions options;

    struct Hash {
      std::size_t operator()(const RuntimeEffectPipelineKey& key) const {
        return fml::HashCombine(key.unique_entrypoint_name,
                                key.options.ToKey());
      }
    };

    struct Equal {
      constexpr bool operator()(const RuntimeEffectPipelineKey& lhs,
                                const RuntimeEffectPipelineKey& rhs) const {
        return lhs.unique_entrypoint_name == rhs.unique_entrypoint_name &&
               lhs.options.ToKey() == rhs.options.ToKey();
      }
    };
  };

  mutable std::unordered_map<RuntimeEffectPipelineKey,
                             std::shared_ptr<Pipeline<PipelineDescriptor>>,
                             RuntimeEffectPipelineKey::Hash,
                             RuntimeEffectPipelineKey::Equal>
      runtime_effect_pipelines_;

  /// Holds multiple Pipelines associated with the same PipelineHandle types.
  ///
  /// For example, it may have multiple
  /// RenderPipelineHandle<SolidFillVertexShader, SolidFillFragmentShader>
  /// instances for different blend modes. From them you can access the
  /// Pipeline.
  ///
  /// See also:
  ///  - impeller::ContentContextOptions - options from which variants are
  ///    created.
  ///  - impeller::Pipeline::CreateVariant
  ///  - impeller::RenderPipelineHandle<> - The type of objects this typically
  ///    contains.
  template <class PipelineHandleT>
  class Variants {
   public:
    Variants() = default;

    void Set(const ContentContextOptions& options,
             std::unique_ptr<PipelineHandleT> pipeline) {
      uint64_t p_key = options.ToKey();
      for (const auto& [key, pipeline] : pipelines_) {
        if (key == p_key) {
          return;
        }
      }
      pipelines_.push_back(std::make_pair(p_key, std::move(pipeline)));
    }

    void SetDefault(const ContentContextOptions& options,
                    std::unique_ptr<PipelineHandleT> pipeline) {
      default_options_ = options;
      Set(options, std::move(pipeline));
    }

    void CreateDefault(const Context& context,
                       const ContentContextOptions& options,
                       const std::initializer_list<Scalar>& constants = {}) {
      auto desc = PipelineHandleT::Builder::MakeDefaultPipelineDescriptor(
          context, constants);
      if (!desc.has_value()) {
        VALIDATION_LOG << "Failed to create default pipeline.";
        return;
      }
      options.ApplyToPipelineDescriptor(*desc);
      SetDefault(options, std::make_unique<PipelineHandleT>(context, desc));
    }

    PipelineHandleT* Get(const ContentContextOptions& options) const {
      uint64_t p_key = options.ToKey();
      for (const auto& [key, pipeline] : pipelines_) {
        if (key == p_key) {
          return pipeline.get();
        }
      }
      return nullptr;
    }

    PipelineHandleT* GetDefault() const {
      if (!default_options_.has_value()) {
        return nullptr;
      }
      return Get(default_options_.value());
    }

    size_t GetPipelineCount() const { return pipelines_.size(); }

   private:
    std::optional<ContentContextOptions> default_options_;
    std::vector<std::pair<uint64_t, std::unique_ptr<PipelineHandleT>>>
        pipelines_;

    Variants(const Variants&) = delete;

    Variants& operator=(const Variants&) = delete;
  };

  // These are mutable because while the prototypes are created eagerly, any
  // variants requested from that are lazily created and cached in the variants
  // map.

  mutable Variants<SolidFillPipeline> solid_fill_pipelines_;
  mutable Variants<FastGradientPipeline> fast_gradient_pipelines_;
  mutable Variants<LinearGradientFillPipeline> linear_gradient_fill_pipelines_;
  mutable Variants<RadialGradientFillPipeline> radial_gradient_fill_pipelines_;
  mutable Variants<ConicalGradientFillPipeline>
      conical_gradient_fill_pipelines_;
  mutable Variants<SweepGradientFillPipeline> sweep_gradient_fill_pipelines_;
  mutable Variants<LinearGradientUniformFillPipeline>
      linear_gradient_uniform_fill_pipelines_;
  mutable Variants<RadialGradientUniformFillPipeline>
      radial_gradient_uniform_fill_pipelines_;
  mutable Variants<ConicalGradientUniformFillPipeline>
      conical_gradient_uniform_fill_pipelines_;
  mutable Variants<SweepGradientUniformFillPipeline>
      sweep_gradient_uniform_fill_pipelines_;
  mutable Variants<LinearGradientSSBOFillPipeline>
      linear_gradient_ssbo_fill_pipelines_;
  mutable Variants<RadialGradientSSBOFillPipeline>
      radial_gradient_ssbo_fill_pipelines_;
  mutable Variants<ConicalGradientSSBOFillPipeline>
      conical_gradient_ssbo_fill_pipelines_;
  mutable Variants<SweepGradientSSBOFillPipeline>
      sweep_gradient_ssbo_fill_pipelines_;
  mutable Variants<RRectBlurPipeline> rrect_blur_pipelines_;
  mutable Variants<TexturePipeline> texture_pipelines_;
  mutable Variants<TextureDownsamplePipeline> texture_downsample_pipelines_;
  mutable Variants<TextureStrictSrcPipeline> texture_strict_src_pipelines_;
#ifdef IMPELLER_ENABLE_OPENGLES
  mutable Variants<TiledTextureExternalPipeline>
      tiled_texture_external_pipelines_;
  mutable Variants<TextureDownsampleGlesPipeline>
      texture_downsample_gles_pipelines_;
  mutable Variants<TiledTextureUvExternalPipeline>
      tiled_texture_uv_external_pipelines_;
#endif  // IMPELLER_ENABLE_OPENGLES
  mutable Variants<TiledTexturePipeline> tiled_texture_pipelines_;
  mutable Variants<GaussianBlurPipeline> gaussian_blur_pipelines_;
  mutable Variants<BorderMaskBlurPipeline> border_mask_blur_pipelines_;
  mutable Variants<MorphologyFilterPipeline> morphology_filter_pipelines_;
  mutable Variants<ColorMatrixColorFilterPipeline>
      color_matrix_color_filter_pipelines_;
  mutable Variants<LinearToSrgbFilterPipeline> linear_to_srgb_filter_pipelines_;
  mutable Variants<SrgbToLinearFilterPipeline> srgb_to_linear_filter_pipelines_;
  mutable Variants<ClipPipeline> clip_pipelines_;
  mutable Variants<GlyphAtlasPipeline> glyph_atlas_pipelines_;
  mutable Variants<YUVToRGBFilterPipeline> yuv_to_rgb_filter_pipelines_;
  mutable Variants<PorterDuffBlendPipeline> porter_duff_blend_pipelines_;
  // Advanced blends.
  mutable Variants<BlendColorPipeline> blend_color_pipelines_;
  mutable Variants<BlendColorBurnPipeline> blend_colorburn_pipelines_;
  mutable Variants<BlendColorDodgePipeline> blend_colordodge_pipelines_;
  mutable Variants<BlendDarkenPipeline> blend_darken_pipelines_;
  mutable Variants<BlendDifferencePipeline> blend_difference_pipelines_;
  mutable Variants<BlendExclusionPipeline> blend_exclusion_pipelines_;
  mutable Variants<BlendHardLightPipeline> blend_hardlight_pipelines_;
  mutable Variants<BlendHuePipeline> blend_hue_pipelines_;
  mutable Variants<BlendLightenPipeline> blend_lighten_pipelines_;
  mutable Variants<BlendLuminosityPipeline> blend_luminosity_pipelines_;
  mutable Variants<BlendMultiplyPipeline> blend_multiply_pipelines_;
  mutable Variants<BlendOverlayPipeline> blend_overlay_pipelines_;
  mutable Variants<BlendSaturationPipeline> blend_saturation_pipelines_;
  mutable Variants<BlendScreenPipeline> blend_screen_pipelines_;
  mutable Variants<BlendSoftLightPipeline> blend_softlight_pipelines_;
  // Framebuffer Advanced blends.
  mutable Variants<FramebufferBlendColorPipeline>
      framebuffer_blend_color_pipelines_;
  mutable Variants<FramebufferBlendColorBurnPipeline>
      framebuffer_blend_colorburn_pipelines_;
  mutable Variants<FramebufferBlendColorDodgePipeline>
      framebuffer_blend_colordodge_pipelines_;
  mutable Variants<FramebufferBlendDarkenPipeline>
      framebuffer_blend_darken_pipelines_;
  mutable Variants<FramebufferBlendDifferencePipeline>
      framebuffer_blend_difference_pipelines_;
  mutable Variants<FramebufferBlendExclusionPipeline>
      framebuffer_blend_exclusion_pipelines_;
  mutable Variants<FramebufferBlendHardLightPipeline>
      framebuffer_blend_hardlight_pipelines_;
  mutable Variants<FramebufferBlendHuePipeline>
      framebuffer_blend_hue_pipelines_;
  mutable Variants<FramebufferBlendLightenPipeline>
      framebuffer_blend_lighten_pipelines_;
  mutable Variants<FramebufferBlendLuminosityPipeline>
      framebuffer_blend_luminosity_pipelines_;
  mutable Variants<FramebufferBlendMultiplyPipeline>
      framebuffer_blend_multiply_pipelines_;
  mutable Variants<FramebufferBlendOverlayPipeline>
      framebuffer_blend_overlay_pipelines_;
  mutable Variants<FramebufferBlendSaturationPipeline>
      framebuffer_blend_saturation_pipelines_;
  mutable Variants<FramebufferBlendScreenPipeline>
      framebuffer_blend_screen_pipelines_;
  mutable Variants<FramebufferBlendSoftLightPipeline>
      framebuffer_blend_softlight_pipelines_;
  mutable Variants<VerticesUberShader> vertices_uber_shader_;

  template <class TypedPipeline>
  PipelineRef GetPipeline(Variants<TypedPipeline>& container,
                          ContentContextOptions opts) const {
    TypedPipeline* pipeline = CreateIfNeeded(container, opts);
    if (!pipeline) {
      return raw_ptr<Pipeline<PipelineDescriptor>>();
    }
    return raw_ptr(pipeline->WaitAndGet());
  }

  template <class RenderPipelineHandleT>
  RenderPipelineHandleT* CreateIfNeeded(
      Variants<RenderPipelineHandleT>& container,
      ContentContextOptions opts) const {
    if (!IsValid()) {
      return nullptr;
    }

    if (wireframe_) {
      opts.wireframe = true;
    }

    if (RenderPipelineHandleT* found = container.Get(opts)) {
      return found;
    }

    RenderPipelineHandleT* default_handle = container.GetDefault();

    // The default must always be initialized in the constructor.
    FML_CHECK(default_handle != nullptr);

    const std::shared_ptr<Pipeline<PipelineDescriptor>>& pipeline =
        default_handle->WaitAndGet();
    if (!pipeline) {
      return nullptr;
    }

    auto variant_future = pipeline->CreateVariant(
        /*async=*/false, [&opts, variants_count = container.GetPipelineCount()](
                             PipelineDescriptor& desc) {
          opts.ApplyToPipelineDescriptor(desc);
          desc.SetLabel(
              SPrintF("%s V#%zu", desc.GetLabel().data(), variants_count));
        });
    std::unique_ptr<RenderPipelineHandleT> variant =
        std::make_unique<RenderPipelineHandleT>(std::move(variant_future));
    container.Set(opts, std::move(variant));
    return container.Get(opts);
  }

  bool is_valid_ = false;
  std::shared_ptr<Tessellator> tessellator_;
  std::shared_ptr<RenderTargetAllocator> render_target_cache_;
  std::shared_ptr<HostBuffer> host_buffer_;
  std::shared_ptr<Texture> empty_texture_;
  bool wireframe_ = false;

  ContentContext(const ContentContext&) = delete;

  ContentContext& operator=(const ContentContext&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_ENTITY_CONTENTS_CONTENT_CONTEXT_H_
