// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/gpu/gpu_surface_gl_skia.h"

#include "flutter/common/graphics/persistent_cache.h"
#include "flutter/fml/base32.h"
#include "flutter/fml/logging.h"
#include "flutter/fml/trace_event.h"
#include "flutter/shell/common/context_options.h"
#include "flutter/shell/gpu/gpu_surface_gl_delegate.h"
#include "third_party/skia/include/core/SkAlphaType.h"
#include "third_party/skia/include/core/SkColorFilter.h"
#include "third_party/skia/include/core/SkColorSpace.h"
#include "third_party/skia/include/core/SkColorType.h"
#include "third_party/skia/include/core/SkSurface.h"
#include "third_party/skia/include/gpu/ganesh/GrBackendSurface.h"
#include "third_party/skia/include/gpu/ganesh/GrContextOptions.h"
#include "third_party/skia/include/gpu/ganesh/SkSurfaceGanesh.h"
#include "third_party/skia/include/gpu/ganesh/gl/GrGLBackendSurface.h"
#include "third_party/skia/include/gpu/ganesh/gl/GrGLDirectContext.h"
#include "third_party/skia/include/gpu/ganesh/gl/GrGLTypes.h"

// These are common defines present on all OpenGL headers. However, we don't
// want to perform GL header resolution on each platform we support. So just
// define these upfront. It is unlikely we will need more. But, if we do, we can
// add the same here.
#define GPU_GL_RGBA8 0x8058
#define GPU_GL_RGBA4 0x8056
#define GPU_GL_RGB565 0x8D62

namespace flutter {

// Default maximum number of bytes of GPU memory of budgeted resources in the
// cache.
// The shell will dynamically increase or decrease this cache based on the
// viewport size, unless a user has specifically requested a size on the Skia
// system channel.
static const size_t kGrCacheMaxByteSize = 24 * (1 << 20);

sk_sp<GrDirectContext> GPUSurfaceGLSkia::MakeGLContext(
    GPUSurfaceGLDelegate* delegate) {
  auto context_switch = delegate->GLContextMakeCurrent();
  if (!context_switch->GetResult()) {
    FML_LOG(ERROR)
        << "Could not make the context current to set up the Gr context.";
    return nullptr;
  }

  const auto options =
      MakeDefaultContextOptions(ContextType::kRender, GrBackendApi::kOpenGL);

  auto context = GrDirectContexts::MakeGL(delegate->GetGLInterface(), options);

  if (!context) {
    FML_LOG(ERROR) << "Failed to set up Skia Gr context.";
    return nullptr;
  }

  context->setResourceCacheLimit(kGrCacheMaxByteSize);

  PersistentCache::GetCacheForProcess()->PrecompileKnownSkSLs(context.get());

  return context;
}

GPUSurfaceGLSkia::GPUSurfaceGLSkia(GPUSurfaceGLDelegate* delegate,
                                   bool render_to_surface)
    : GPUSurfaceGLSkia(MakeGLContext(delegate), delegate, render_to_surface) {
  context_owner_ = true;
}

GPUSurfaceGLSkia::GPUSurfaceGLSkia(const sk_sp<GrDirectContext>& gr_context,
                                   GPUSurfaceGLDelegate* delegate,
                                   bool render_to_surface)
    : delegate_(delegate),
      context_(gr_context),

      render_to_surface_(render_to_surface),
      weak_factory_(this) {
  auto context_switch = delegate_->GLContextMakeCurrent();
  if (!context_switch->GetResult()) {
    FML_LOG(ERROR)
        << "Could not make the context current to set up the Gr context.";
    return;
  }

  delegate_->GLContextClearCurrent();

  valid_ = gr_context != nullptr;
}

GPUSurfaceGLSkia::~GPUSurfaceGLSkia() {
  if (!valid_) {
    return;
  }
  auto context_switch = delegate_->GLContextMakeCurrent();
  if (!context_switch->GetResult()) {
    FML_LOG(ERROR) << "Could not make the context current to destroy the "
                      "GrDirectContext resources.";
    return;
  }

  onscreen_surface_ = nullptr;
  fbo_id_ = 0;
  if (context_owner_) {
    context_->releaseResourcesAndAbandonContext();
  }
  context_ = nullptr;

  delegate_->GLContextClearCurrent();
}

// |Surface|
bool GPUSurfaceGLSkia::IsValid() {
  return valid_;
}

static SkColorType FirstSupportedColorType(GrDirectContext* context,
                                           GrGLenum* format) {
#define RETURN_IF_RENDERABLE(x, y)                 \
  if (context->colorTypeSupportedAsSurface((x))) { \
    *format = (y);                                 \
    return (x);                                    \
  }
  RETURN_IF_RENDERABLE(kRGBA_8888_SkColorType, GPU_GL_RGBA8);
  RETURN_IF_RENDERABLE(kARGB_4444_SkColorType, GPU_GL_RGBA4);
  RETURN_IF_RENDERABLE(kRGB_565_SkColorType, GPU_GL_RGB565);
  return kUnknown_SkColorType;
}

static sk_sp<SkSurface> WrapOnscreenSurface(GrDirectContext* context,
                                            const SkISize& size,
                                            intptr_t fbo) {
  GrGLenum format = kUnknown_SkColorType;
  const SkColorType color_type = FirstSupportedColorType(context, &format);

  GrGLFramebufferInfo framebuffer_info = {};
  framebuffer_info.fFBOID = static_cast<GrGLuint>(fbo);
  framebuffer_info.fFormat = format;

  auto render_target =
      GrBackendRenderTargets::MakeGL(size.width(),     // width
                                     size.height(),    // height
                                     0,                // sample count
                                     0,                // stencil bits
                                     framebuffer_info  // framebuffer info
      );

  sk_sp<SkColorSpace> colorspace = SkColorSpace::MakeSRGB();
  SkSurfaceProps surface_props(0, kUnknown_SkPixelGeometry);

  return SkSurfaces::WrapBackendRenderTarget(
      context,                                       // Gr context
      render_target,                                 // render target
      GrSurfaceOrigin::kBottomLeft_GrSurfaceOrigin,  // origin
      color_type,                                    // color type
      colorspace,                                    // colorspace
      &surface_props                                 // surface properties
  );
}

bool GPUSurfaceGLSkia::CreateOrUpdateSurfaces(const SkISize& size) {
  if (onscreen_surface_ != nullptr &&
      size == SkISize::Make(onscreen_surface_->width(),
                            onscreen_surface_->height())) {
    // Surface size appears unchanged. So bail.
    return true;
  }

  // We need to do some updates.
  TRACE_EVENT0("flutter", "UpdateSurfacesSize");

  // Either way, we need to get rid of previous surface.
  onscreen_surface_ = nullptr;
  fbo_id_ = 0;

  if (size.isEmpty()) {
    FML_LOG(ERROR) << "Cannot create surfaces of empty size.";
    return false;
  }

  sk_sp<SkSurface> onscreen_surface;

  GLFrameInfo frame_info = {static_cast<uint32_t>(size.width()),
                            static_cast<uint32_t>(size.height())};
  const GLFBOInfo fbo_info = delegate_->GLContextFBO(frame_info);
  onscreen_surface = WrapOnscreenSurface(context_.get(),  // GL context
                                         size,            // root surface size
                                         fbo_info.fbo_id  // window FBO ID
  );

  if (onscreen_surface == nullptr) {
    // If the onscreen surface could not be wrapped. There is absolutely no
    // point in moving forward.
    FML_LOG(ERROR) << "Could not wrap onscreen surface.";
    return false;
  }

  onscreen_surface_ = std::move(onscreen_surface);
  fbo_id_ = fbo_info.fbo_id;
  existing_damage_ = fbo_info.existing_damage;

  return true;
}

// |Surface|
SkMatrix GPUSurfaceGLSkia::GetRootTransformation() const {
  return delegate_->GLContextSurfaceTransformation();
}

// |Surface|
std::unique_ptr<SurfaceFrame> GPUSurfaceGLSkia::AcquireFrame(
    const SkISize& size) {
  if (delegate_ == nullptr) {
    return nullptr;
  }
  auto context_switch = delegate_->GLContextMakeCurrent();
  if (!context_switch->GetResult()) {
    FML_LOG(ERROR)
        << "Could not make the context current to acquire the frame.";
    return nullptr;
  }

  SurfaceFrame::FramebufferInfo framebuffer_info;

  // TODO(38466): Refactor GPU surface APIs take into account the fact that an
  // external view embedder may want to render to the root surface.
  if (!render_to_surface_) {
    framebuffer_info.supports_readback = true;
    return std::make_unique<SurfaceFrame>(
        nullptr, framebuffer_info,
        [](const SurfaceFrame& surface_frame, DlCanvas* canvas) {
          return true;
        },
        [](const SurfaceFrame& surface_frame) { return true; }, size);
  }

  const auto root_surface_transformation = GetRootTransformation();

  sk_sp<SkSurface> surface =
      AcquireRenderSurface(size, root_surface_transformation);

  if (surface == nullptr) {
    return nullptr;
  }

  surface->getCanvas()->setMatrix(root_surface_transformation);

  SurfaceFrame::EncodeCallback encode_callback =
      [weak = weak_factory_.GetWeakPtr()](const SurfaceFrame& surface_frame,
                                          DlCanvas* canvas) {
        TRACE_EVENT0("flutter", "GrDirectContext::flushAndSubmit");
        if (weak) {
          weak->context_->flushAndSubmit();
          return true;
        }
        return false;
      };
  SurfaceFrame::SubmitCallback submit_callback =
      [weak = weak_factory_.GetWeakPtr()](const SurfaceFrame& surface_frame) {
        return weak ? weak->PresentSurface(surface_frame) : false;
      };

  framebuffer_info = delegate_->GLContextFramebufferInfo();
  if (!framebuffer_info.existing_damage.has_value()) {
    framebuffer_info.existing_damage = existing_damage_;
  }
  return std::make_unique<SurfaceFrame>(surface, framebuffer_info,
                                        encode_callback, submit_callback, size,
                                        std::move(context_switch));
}

bool GPUSurfaceGLSkia::PresentSurface(const SurfaceFrame& frame) {
  if (delegate_ == nullptr || context_ == nullptr) {
    return false;
  }

  delegate_->GLContextSetDamageRegion(frame.submit_info().buffer_damage);

  GLPresentInfo present_info = {
      .fbo_id = fbo_id_,
      .frame_damage = frame.submit_info().frame_damage,
      .presentation_time = frame.submit_info().presentation_time,
      .buffer_damage = frame.submit_info().buffer_damage,
  };
  if (!delegate_->GLContextPresent(present_info)) {
    return false;
  }

  if (delegate_->GLContextFBOResetAfterPresent()) {
    auto current_size =
        SkISize::Make(onscreen_surface_->width(), onscreen_surface_->height());

    GLFrameInfo frame_info = {static_cast<uint32_t>(current_size.width()),
                              static_cast<uint32_t>(current_size.height())};

    // The FBO has changed, ask the delegate for the new FBO and do a surface
    // re-wrap.
    const GLFBOInfo fbo_info = delegate_->GLContextFBO(frame_info);
    auto new_onscreen_surface =
        WrapOnscreenSurface(context_.get(),  // GL context
                            current_size,    // root surface size
                            fbo_info.fbo_id  // window FBO ID
        );

    if (!new_onscreen_surface) {
      return false;
    }

    onscreen_surface_ = std::move(new_onscreen_surface);
    fbo_id_ = fbo_info.fbo_id;
    existing_damage_ = fbo_info.existing_damage;
  }

  return true;
}

sk_sp<SkSurface> GPUSurfaceGLSkia::AcquireRenderSurface(
    const SkISize& untransformed_size,
    const SkMatrix& root_surface_transformation) {
  const auto transformed_rect = root_surface_transformation.mapRect(
      SkRect::MakeWH(untransformed_size.width(), untransformed_size.height()));

  const auto transformed_size =
      SkISize::Make(transformed_rect.width(), transformed_rect.height());

  if (!CreateOrUpdateSurfaces(transformed_size)) {
    return nullptr;
  }

  return onscreen_surface_;
}

// |Surface|
GrDirectContext* GPUSurfaceGLSkia::GetContext() {
  return context_.get();
}

// |Surface|
std::unique_ptr<GLContextResult> GPUSurfaceGLSkia::MakeRenderContextCurrent() {
  return delegate_->GLContextMakeCurrent();
}

// |Surface|
bool GPUSurfaceGLSkia::ClearRenderContext() {
  return delegate_->GLContextClearCurrent();
}

// |Surface|
bool GPUSurfaceGLSkia::AllowsDrawingWhenGpuDisabled() const {
  return delegate_->AllowsDrawingWhenGpuDisabled();
}

}  // namespace flutter
