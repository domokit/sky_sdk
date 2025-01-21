// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/gpu/gpu_surface_metal_impeller.h"

#import <Metal/Metal.h>
#import <QuartzCore/QuartzCore.h>
#include "flow/surface.h"
#include "flow/surface_frame.h"
#include "impeller/display_list/aiks_context.h"
#include "impeller/renderer/backend/metal/swapchain_transients_mtl.h"

#include "flutter/common/settings.h"
#include "flutter/fml/make_copyable.h"
#include "flutter/fml/mapping.h"
#include "flutter/fml/trace_event.h"
#include "impeller/display_list/dl_dispatcher.h"
#include "impeller/renderer/backend/metal/surface_mtl.h"
#include "impeller/typographer/backends/skia/typographer_context_skia.h"

static_assert(__has_feature(objc_arc), "ARC must be enabled.");

namespace flutter {

GPUSurfaceMetalImpeller::GPUSurfaceMetalImpeller(
    GPUSurfaceMetalDelegate* delegate,
    const std::shared_ptr<impeller::AiksContext>& context,
    bool render_to_surface)
    : delegate_(delegate),
      render_target_type_(delegate->GetRenderTargetType()),
      aiks_context_(context),
      render_to_surface_(render_to_surface) {
  // If this preference is explicitly set, we allow for disabling partial repaint.
  NSNumber* disablePartialRepaint =
      [[NSBundle mainBundle] objectForInfoDictionaryKey:@"FLTDisablePartialRepaint"];
  if (disablePartialRepaint != nil) {
    disable_partial_repaint_ = disablePartialRepaint.boolValue;
  }
  if (aiks_context_) {
    swapchain_transients_ = std::make_shared<impeller::SwapchainTransientsMTL>(
        aiks_context_->GetContext()->GetResourceAllocator());
  }
}

GPUSurfaceMetalImpeller::~GPUSurfaceMetalImpeller() = default;

// |Surface|
bool GPUSurfaceMetalImpeller::IsValid() {
  return !!aiks_context_ && aiks_context_->IsValid();
}

// |Surface|
std::unique_ptr<SurfaceFrame> GPUSurfaceMetalImpeller::AcquireFrame(const SkISize& frame_size) {
  TRACE_EVENT0("impeller", "GPUSurfaceMetalImpeller::AcquireFrame");

  if (!IsValid()) {
    FML_LOG(ERROR) << "Metal surface was invalid.";
    return nullptr;
  }

  if (frame_size.isEmpty()) {
    FML_LOG(ERROR) << "Metal surface was asked for an empty frame.";
    return nullptr;
  }

  if (!render_to_surface_) {
    return std::make_unique<SurfaceFrame>(
        nullptr, SurfaceFrame::FramebufferInfo(),
        [](const SurfaceFrame& surface_frame, DlCanvas* canvas) { return true; },
        [](const SurfaceFrame& surface_frame) { return true; }, frame_size);
  }

  switch (render_target_type_) {
    case MTLRenderTargetType::kCAMetalLayer:
      return AcquireFrameFromCAMetalLayer(frame_size);
    case MTLRenderTargetType::kMTLTexture:
      return AcquireFrameFromMTLTexture(frame_size);
    default:
      FML_CHECK(false) << "Unknown MTLRenderTargetType type.";
  }

  return nullptr;
}

std::unique_ptr<SurfaceFrame> GPUSurfaceMetalImpeller::AcquireFrameFromCAMetalLayer(
    const SkISize& frame_size) {
  CAMetalLayer* layer = (__bridge CAMetalLayer*)delegate_->GetCAMetalLayer(frame_size);
  if (!layer) {
    FML_LOG(ERROR) << "Invalid CAMetalLayer given by the embedder.";
    return nullptr;
  }

  id<CAMetalDrawable> drawable =
      impeller::SurfaceMTL::GetMetalDrawableAndValidate(aiks_context_->GetContext(), layer);
  if (!drawable) {
    return nullptr;
  }
  if (Settings::kSurfaceDataAccessible) {
    last_texture_ = drawable.texture;
  }

#ifdef IMPELLER_DEBUG
  impeller::ContextMTL::Cast(*aiks_context_->GetContext()).GetCaptureManager()->StartCapture();
#endif  // IMPELLER_DEBUG

  __weak id<MTLTexture> weak_last_texture = last_texture_;
  __weak CAMetalLayer* weak_layer = layer;
  SurfaceFrame::EncodeCallback encode_callback =
      fml::MakeCopyable([damage = damage_,
                         disable_partial_repaint = disable_partial_repaint_,  //
                         aiks_context = aiks_context_,                        //
                         drawable,                                            //
                         weak_last_texture,                                   //
                         weak_layer,                                          //
                         swapchain_transients = swapchain_transients_         //
  ](SurfaceFrame& surface_frame, DlCanvas* canvas) mutable -> bool {
        id<MTLTexture> strong_last_texture = weak_last_texture;
        CAMetalLayer* strong_layer = weak_layer;
        if (!strong_last_texture || !strong_layer) {
          return false;
        }
        strong_layer.presentsWithTransaction = surface_frame.submit_info().present_with_transaction;
        if (!aiks_context) {
          return false;
        }

        auto display_list = surface_frame.BuildDisplayList();
        if (!display_list) {
          FML_LOG(ERROR) << "Could not build display list for surface frame.";
          return false;
        }

        if (!disable_partial_repaint && damage) {
          void* texture = (__bridge void*)strong_last_texture;
          for (auto& entry : *damage) {
            if (entry.first != texture) {
              // Accumulate damage for other framebuffers
              if (surface_frame.submit_info().frame_damage) {
                entry.second.join(*surface_frame.submit_info().frame_damage);
              }
            }
          }
          // Reset accumulated damage for current framebuffer
          (*damage)[texture] = SkIRect::MakeEmpty();
        }

        std::optional<impeller::IRect> clip_rect;
        if (surface_frame.submit_info().buffer_damage.has_value()) {
          auto buffer_damage = surface_frame.submit_info().buffer_damage;
          clip_rect = impeller::IRect::MakeXYWH(buffer_damage->x(), buffer_damage->y(),
                                                buffer_damage->width(), buffer_damage->height());
        }

        auto surface = impeller::SurfaceMTL::MakeFromMetalLayerDrawable(
            aiks_context->GetContext(), drawable, swapchain_transients, clip_rect);

        // The surface may be null if we failed to allocate the onscreen render target
        // due to running out of memory.
        if (!surface) {
          return false;
        }
        surface->PresentWithTransaction(surface_frame.submit_info().present_with_transaction);

        if (clip_rect && clip_rect->IsEmpty()) {
          if (!surface->PreparePresent()) {
            return false;
          }
          surface_frame.set_user_data(std::move(surface));
          return true;
        }

        impeller::IRect cull_rect = surface->coverage();
        SkIRect sk_cull_rect = SkIRect::MakeWH(cull_rect.GetWidth(), cull_rect.GetHeight());
        surface->SetFrameBoundary(surface_frame.submit_info().frame_boundary);

        const bool reset_host_buffer = surface_frame.submit_info().frame_boundary;
        auto render_result = impeller::RenderToTarget(aiks_context->GetContentContext(),       //
                                                      surface->GetRenderTarget(),              //
                                                      display_list,                            //
                                                      sk_cull_rect,                            //
                                                      /*reset_host_buffer=*/reset_host_buffer  //
        );
        if (!render_result) {
          return false;
        }

        if (!surface->PreparePresent()) {
          return false;
        }
        surface_frame.set_user_data(std::move(surface));
        return true;
      });

  SurfaceFrame::FramebufferInfo framebuffer_info;
  framebuffer_info.supports_readback = true;

  if (!disable_partial_repaint_) {
    // Provide accumulated damage to rasterizer (area in current framebuffer that lags behind
    // front buffer)
    void* texture = (__bridge void*)drawable.texture;
    auto i = damage_->find(texture);
    if (i != damage_->end()) {
      framebuffer_info.existing_damage = i->second;
    }
    framebuffer_info.supports_partial_repaint = true;
  }

  return std::make_unique<SurfaceFrame>(
      nullptr,           // surface
      framebuffer_info,  // framebuffer info
      encode_callback,   // submit callback
      [](SurfaceFrame& surface_frame) { return surface_frame.take_user_data()->Present(); },
      frame_size,  // frame size
      nullptr,     // context result
      true         // display list fallback
  );
}

std::unique_ptr<SurfaceFrame> GPUSurfaceMetalImpeller::AcquireFrameFromMTLTexture(
    const SkISize& frame_size) {
  GPUMTLTextureInfo texture_info = delegate_->GetMTLTexture(frame_size);
  id<MTLTexture> mtl_texture = (__bridge id<MTLTexture>)texture_info.texture;
  if (!mtl_texture) {
    FML_LOG(ERROR) << "Invalid MTLTexture given by the embedder.";
    return nullptr;
  }

  if (Settings::kSurfaceDataAccessible) {
    last_texture_ = mtl_texture;
  }

#ifdef IMPELLER_DEBUG
  impeller::ContextMTL::Cast(*aiks_context_->GetContext()).GetCaptureManager()->StartCapture();
#endif  // IMPELLER_DEBUG

  __weak id<MTLTexture> weak_texture = mtl_texture;
  SurfaceFrame::EncodeCallback encode_callback =
      fml::MakeCopyable([disable_partial_repaint = disable_partial_repaint_,  //
                         damage = damage_,
                         aiks_context = aiks_context_,                 //
                         weak_texture,                                 //
                         swapchain_transients = swapchain_transients_  //
  ](SurfaceFrame& surface_frame, DlCanvas* canvas) mutable -> bool {
        id<MTLTexture> strong_texture = weak_texture;
        if (!strong_texture) {
          return false;
        }
        if (!aiks_context) {
          return false;
        }
        auto display_list = surface_frame.BuildDisplayList();
        if (!display_list) {
          FML_LOG(ERROR) << "Could not build display list for surface frame.";
          return false;
        }

        if (!disable_partial_repaint && damage) {
          void* texture_ptr = (__bridge void*)strong_texture;
          for (auto& entry : *damage) {
            if (entry.first != texture_ptr) {
              // Accumulate damage for other framebuffers
              if (surface_frame.submit_info().frame_damage) {
                entry.second.join(*surface_frame.submit_info().frame_damage);
              }
            }
          }
          // Reset accumulated damage for current framebuffer
          (*damage)[texture_ptr] = SkIRect::MakeEmpty();
        }

        std::optional<impeller::IRect> clip_rect;
        if (surface_frame.submit_info().buffer_damage.has_value()) {
          auto buffer_damage = surface_frame.submit_info().buffer_damage;
          clip_rect = impeller::IRect::MakeXYWH(buffer_damage->x(), buffer_damage->y(),
                                                buffer_damage->width(), buffer_damage->height());
        }

        auto surface = impeller::SurfaceMTL::MakeFromTexture(
            aiks_context->GetContext(), strong_texture, swapchain_transients, clip_rect);

        surface->PresentWithTransaction(surface_frame.submit_info().present_with_transaction);

        if (clip_rect && clip_rect->IsEmpty()) {
          if (!surface->PreparePresent()) {
            return false;
          }
          return surface->Present();
        }

        impeller::IRect cull_rect = surface->coverage();
        SkIRect sk_cull_rect = SkIRect::MakeWH(cull_rect.GetWidth(), cull_rect.GetHeight());
        auto render_result = impeller::RenderToTarget(aiks_context->GetContentContext(),  //
                                                      surface->GetRenderTarget(),         //
                                                      display_list,                       //
                                                      sk_cull_rect,                       //
                                                      /*reset_host_buffer=*/true          //
        );
        if (!render_result) {
          FML_LOG(ERROR) << "Failed to render Impeller frame";
          return false;
        }
        if (!surface->PreparePresent()) {
          return false;
        }
        return surface->PreparePresent();
      });

  SurfaceFrame::SubmitCallback submit_callback =
      [texture_info, delegate = delegate_](const SurfaceFrame& surface_frame) {
        return delegate->PresentTexture(texture_info);
      };

  SurfaceFrame::FramebufferInfo framebuffer_info;
  framebuffer_info.supports_readback = true;

  if (!disable_partial_repaint_) {
    // Provide accumulated damage to rasterizer (area in current framebuffer that lags behind
    // front buffer)
    void* texture = (__bridge void*)mtl_texture;
    auto i = damage_->find(texture);
    if (i != damage_->end()) {
      framebuffer_info.existing_damage = i->second;
    }
    framebuffer_info.supports_partial_repaint = true;
  }

  return std::make_unique<SurfaceFrame>(nullptr,           // surface
                                        framebuffer_info,  // framebuffer info
                                        encode_callback,
                                        submit_callback,  // submit callback
                                        frame_size,       // frame size
                                        nullptr,          // context result
                                        true              // display list fallback
  );
}

// |Surface|
SkMatrix GPUSurfaceMetalImpeller::GetRootTransformation() const {
  // This backend does not currently support root surface transformations. Just
  // return identity.
  return {};
}

// |Surface|
GrDirectContext* GPUSurfaceMetalImpeller::GetContext() {
  return nullptr;
}

// |Surface|
std::unique_ptr<GLContextResult> GPUSurfaceMetalImpeller::MakeRenderContextCurrent() {
  // This backend has no such concept.
  return std::make_unique<GLContextDefaultResult>(true);
}

bool GPUSurfaceMetalImpeller::AllowsDrawingWhenGpuDisabled() const {
  return delegate_->AllowsDrawingWhenGpuDisabled();
}

// |Surface|
bool GPUSurfaceMetalImpeller::EnableRasterCache() const {
  return false;
}

// |Surface|
std::shared_ptr<impeller::AiksContext> GPUSurfaceMetalImpeller::GetAiksContext() const {
  return aiks_context_;
}

Surface::SurfaceData GPUSurfaceMetalImpeller::GetSurfaceData() const {
  if (!(last_texture_ && [last_texture_ conformsToProtocol:@protocol(MTLTexture)])) {
    return {};
  }
  id<MTLTexture> texture = last_texture_;
  int bytesPerPixel = 0;
  std::string pixel_format;
  switch (texture.pixelFormat) {
    case MTLPixelFormatBGR10_XR:
      bytesPerPixel = 4;
      pixel_format = "MTLPixelFormatBGR10_XR";
      break;
    case MTLPixelFormatBGRA10_XR:
      bytesPerPixel = 8;
      pixel_format = "MTLPixelFormatBGRA10_XR";
      break;
    case MTLPixelFormatBGRA8Unorm:
      bytesPerPixel = 4;
      pixel_format = "MTLPixelFormatBGRA8Unorm";
      break;
    case MTLPixelFormatRGBA16Float:
      bytesPerPixel = 8;
      pixel_format = "MTLPixelFormatRGBA16Float";
      break;
    default:
      return {};
  }

  // Zero initialized so that errors are easier to find at the cost of
  // performance.
  sk_sp<SkData> result =
      SkData::MakeZeroInitialized(texture.width * texture.height * bytesPerPixel);
  [texture getBytes:result->writable_data()
        bytesPerRow:texture.width * bytesPerPixel
         fromRegion:MTLRegionMake2D(0, 0, texture.width, texture.height)
        mipmapLevel:0];
  return {
      .pixel_format = pixel_format,
      .data = result,
  };
}

}  // namespace flutter
