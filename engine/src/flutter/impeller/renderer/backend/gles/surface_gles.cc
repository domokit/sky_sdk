// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/backend/gles/surface_gles.h"

#include "flutter/fml/trace_event.h"
#include "impeller/base/config.h"
#include "impeller/renderer/backend/gles/context_gles.h"
#include "impeller/renderer/backend/gles/texture_gles.h"

namespace impeller {

std::unique_ptr<Surface> SurfaceGLES::WrapFBO(
    const std::shared_ptr<Context>& context,
    SwapCallback swap_callback,
    GLuint fbo,
    PixelFormat color_format,
    ISize fbo_size) {
  TRACE_EVENT0("impeller", "SurfaceGLES::WrapOnScreenFBO");

  if (context == nullptr || !context->IsValid() || !swap_callback) {
    return nullptr;
  }

  const auto& gl_context = ContextGLES::Cast(*context);

  TextureDescriptor color0_tex;
  color0_tex.type = TextureType::kTexture2D;
  color0_tex.format = color_format;
  color0_tex.size = fbo_size;
  color0_tex.usage = TextureUsage::kRenderTarget;
  color0_tex.sample_count = SampleCount::kCount1;
  color0_tex.storage_mode = StorageMode::kDevicePrivate;

  ColorAttachment color0;
  color0.texture =
      TextureGLES::WrapFBO(gl_context.GetReactor(), color0_tex, fbo);
  color0.clear_color = Color::DarkSlateGray();
  color0.load_action = LoadAction::kClear;
  color0.store_action = StoreAction::kStore;

  TextureDescriptor depth_stencil_texture_desc;
  depth_stencil_texture_desc.type = TextureType::kTexture2D;
  depth_stencil_texture_desc.format = color_format;
  depth_stencil_texture_desc.size = fbo_size;
  depth_stencil_texture_desc.usage = TextureUsage::kRenderTarget;
  depth_stencil_texture_desc.sample_count = SampleCount::kCount1;

  auto depth_stencil_tex =
      TextureGLES::CreatePlaceholder(gl_context.GetReactor(),    //
                                     depth_stencil_texture_desc  //
      );

  DepthAttachment depth0;
  depth0.clear_depth = 0;
  depth0.texture = depth_stencil_tex;
  depth0.load_action = LoadAction::kClear;
  depth0.store_action = StoreAction::kDontCare;

  StencilAttachment stencil0;
  stencil0.clear_stencil = 0;
  stencil0.texture = depth_stencil_tex;
  stencil0.load_action = LoadAction::kClear;
  stencil0.store_action = StoreAction::kDontCare;

  RenderTarget render_target_desc;

  render_target_desc.SetColorAttachment(color0, 0u);
  render_target_desc.SetDepthAttachment(depth0);
  render_target_desc.SetStencilAttachment(stencil0);

#ifdef IMPELLER_DEBUG
  gl_context.GetGPUTracer()->RecordRasterThread();
#endif  // IMPELLER_DEBUG

  return std::unique_ptr<SurfaceGLES>(
      new SurfaceGLES(std::move(swap_callback), render_target_desc));
}

SurfaceGLES::SurfaceGLES(SwapCallback swap_callback,
                         const RenderTarget& target_desc)
    : Surface(target_desc), swap_callback_(std::move(swap_callback)) {}

// |Surface|
SurfaceGLES::~SurfaceGLES() = default;

// |Surface|
bool SurfaceGLES::Present() const {
  return swap_callback_ ? swap_callback_() : false;
}

}  // namespace impeller
