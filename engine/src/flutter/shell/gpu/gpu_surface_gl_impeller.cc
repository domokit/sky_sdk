// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/gpu/gpu_surface_gl_impeller.h"

#include "flow/surface_frame.h"
#include "flutter/fml/make_copyable.h"
#include "impeller/display_list/dl_dispatcher.h"
#include "impeller/renderer/backend/gles/surface_gles.h"
#include "impeller/typographer/backends/skia/typographer_context_skia.h"

namespace flutter {

GPUSurfaceGLImpeller::GPUSurfaceGLImpeller(
    GPUSurfaceGLDelegate* delegate,
    std::shared_ptr<impeller::Context> context,
    bool render_to_surface)
    : weak_factory_(this) {
  if (delegate == nullptr) {
    return;
  }

  if (!context || !context->IsValid()) {
    return;
  }

  auto aiks_context = std::make_shared<impeller::AiksContext>(
      context, impeller::TypographerContextSkia::Make());

  if (!aiks_context->IsValid()) {
    return;
  }

  delegate_ = delegate;
  impeller_context_ = std::move(context);
  render_to_surface_ = render_to_surface;
  aiks_context_ = std::move(aiks_context);
  is_valid_ = true;
}

// |Surface|
GPUSurfaceGLImpeller::~GPUSurfaceGLImpeller() = default;

// |Surface|
bool GPUSurfaceGLImpeller::IsValid() {
  return is_valid_;
}

// |Surface|
std::unique_ptr<SurfaceFrame> GPUSurfaceGLImpeller::AcquireFrame(
    const SkISize& size) {
  if (!IsValid()) {
    FML_LOG(ERROR) << "OpenGL surface was invalid.";
    return nullptr;
  }

  auto swap_callback = [weak = weak_factory_.GetWeakPtr(),
                        delegate = delegate_]() -> bool {
    if (weak) {
      GLPresentInfo present_info = {
          .fbo_id = 0u,
          .frame_damage = std::nullopt,
          // TODO (https://github.com/flutter/flutter/issues/105597): wire-up
          // presentation time to impeller backend.
          .presentation_time = std::nullopt,
          .buffer_damage = std::nullopt,
      };
      delegate->GLContextPresent(present_info);
    }
    return true;
  };

  auto context_switch = delegate_->GLContextMakeCurrent();
  if (!context_switch->GetResult()) {
    FML_LOG(ERROR)
        << "Could not make the context current to acquire the frame.";
    return nullptr;
  }

  if (!render_to_surface_) {
    return std::make_unique<SurfaceFrame>(
        nullptr, SurfaceFrame::FramebufferInfo{.supports_readback = true},
        [](const SurfaceFrame& surface_frame, DlCanvas* canvas) {
          return true;
        },
        [](const SurfaceFrame& surface_frame) { return true; }, size);
  }

  GLFrameInfo frame_info = {static_cast<uint32_t>(size.width()),
                            static_cast<uint32_t>(size.height())};
  const GLFBOInfo fbo_info = delegate_->GLContextFBO(frame_info);
  auto surface = impeller::SurfaceGLES::WrapFBO(
      impeller_context_,                            // context
      swap_callback,                                // swap_callback
      fbo_info.fbo_id,                              // fbo
      impeller::PixelFormat::kR8G8B8A8UNormInt,     // color_format
      impeller::ISize{size.width(), size.height()}  // fbo_size
  );

  impeller::RenderTarget render_target = surface->GetRenderTarget();

  SurfaceFrame::EncodeCallback encode_calback =
      [aiks_context = aiks_context_,  //
       render_target](SurfaceFrame& surface_frame,
                      DlCanvas* canvas) mutable -> bool {
    if (!aiks_context) {
      return false;
    }

    auto display_list = surface_frame.BuildDisplayList();
    if (!display_list) {
      FML_LOG(ERROR) << "Could not build display list for surface frame.";
      return false;
    }

    auto cull_rect = render_target.GetRenderTargetSize();
    SkIRect sk_cull_rect = SkIRect::MakeWH(cull_rect.width, cull_rect.height);
    return impeller::RenderToOnscreen(aiks_context->GetContentContext(),  //
                                      render_target,                      //
                                      display_list,                       //
                                      sk_cull_rect,                       //
                                      /*reset_host_buffer=*/true          //
    );
    return true;
  };

  return std::make_unique<SurfaceFrame>(
      nullptr,                                // surface
      delegate_->GLContextFramebufferInfo(),  // framebuffer info
      encode_calback,                         // encode callback
      fml::MakeCopyable([surface = std::move(surface)](const SurfaceFrame&) {
        return surface->Present();
      }),                         // submit callback
      size,                       // frame size
      std::move(context_switch),  // context result
      true                        // display list fallback
  );
}

// |Surface|
SkMatrix GPUSurfaceGLImpeller::GetRootTransformation() const {
  // This backend does not currently support root surface transformations. Just
  // return identity.
  return {};
}

// |Surface|
GrDirectContext* GPUSurfaceGLImpeller::GetContext() {
  // Impeller != Skia.
  return nullptr;
}

// |Surface|
std::unique_ptr<GLContextResult>
GPUSurfaceGLImpeller::MakeRenderContextCurrent() {
  return delegate_->GLContextMakeCurrent();
}

// |Surface|
bool GPUSurfaceGLImpeller::ClearRenderContext() {
  return delegate_->GLContextClearCurrent();
}

bool GPUSurfaceGLImpeller::AllowsDrawingWhenGpuDisabled() const {
  return delegate_->AllowsDrawingWhenGpuDisabled();
}

// |Surface|
bool GPUSurfaceGLImpeller::EnableRasterCache() const {
  return false;
}

// |Surface|
std::shared_ptr<impeller::AiksContext> GPUSurfaceGLImpeller::GetAiksContext()
    const {
  return aiks_context_;
}

}  // namespace flutter
