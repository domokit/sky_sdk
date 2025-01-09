// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/backend/metal/surface_mtl.h"

#include "flutter/fml/trace_event.h"
#include "flutter/impeller/renderer/command_buffer.h"
#include "impeller/base/validation.h"
#include "impeller/core/texture_descriptor.h"
#include "impeller/renderer/backend/metal/context_mtl.h"
#include "impeller/renderer/backend/metal/formats_mtl.h"
#include "impeller/renderer/backend/metal/texture_mtl.h"
#include "impeller/renderer/render_target.h"

static_assert(__has_feature(objc_arc), "ARC must be enabled.");

@protocol FlutterMetalDrawable <MTLDrawable>
- (void)flutterPrepareForPresent:(nonnull id<MTLCommandBuffer>)commandBuffer;
@end

namespace impeller {

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunguarded-availability-new"

id<CAMetalDrawable> SurfaceMTL::GetMetalDrawableAndValidate(
    const std::shared_ptr<Context>& context,
    CAMetalLayer* layer) {
  TRACE_EVENT0("impeller", "SurfaceMTL::WrapCurrentMetalLayerDrawable");

  if (context == nullptr || !context->IsValid() || layer == nil) {
    return nullptr;
  }

  id<CAMetalDrawable> current_drawable = nil;
  {
    TRACE_EVENT0("impeller", "WaitForNextDrawable");
    current_drawable = [layer nextDrawable];
  }

  if (!current_drawable) {
    VALIDATION_LOG << "Could not acquire current drawable.";
    return nullptr;
  }
  return current_drawable;
}

static std::optional<RenderTarget> WrapTextureWithRenderTarget(
    Allocator& allocator,
    id<MTLTexture> texture,
    bool requires_blit,
    std::optional<IRect> clip_rect) {
  // compositor_context.cc will offset the rendering by the clip origin. Here we
  // shrink to the size of the clip. This has the same effect as clipping the
  // rendering but also creates smaller intermediate passes.
  ISize root_size;
  if (requires_blit) {
    if (!clip_rect.has_value()) {
      VALIDATION_LOG << "Missing clip rectangle.";
      return std::nullopt;
    }
    root_size = ISize(clip_rect->GetWidth(), clip_rect->GetHeight());
  } else {
    root_size = {static_cast<ISize::Type>(texture.width),
                 static_cast<ISize::Type>(texture.height)};
  }

  TextureDescriptor resolve_tex_desc;
  resolve_tex_desc.format = FromMTLPixelFormat(texture.pixelFormat);
  resolve_tex_desc.size = root_size;
  resolve_tex_desc.usage =
      TextureUsage::kRenderTarget | TextureUsage::kShaderRead;
  resolve_tex_desc.sample_count = SampleCount::kCount1;
  resolve_tex_desc.storage_mode = StorageMode::kDevicePrivate;

  if (resolve_tex_desc.format == PixelFormat::kUnknown) {
    VALIDATION_LOG << "Unknown drawable color format.";
    return std::nullopt;
  }

  // Create color resolve texture.
  std::shared_ptr<Texture> resolve_tex;
  if (requires_blit) {
    resolve_tex_desc.compression_type = CompressionType::kLossy;
    resolve_tex = allocator.CreateTexture(resolve_tex_desc);
  } else {
    resolve_tex = TextureMTL::Create(resolve_tex_desc, texture);
  }

  if (!resolve_tex) {
    VALIDATION_LOG << "Could not wrap resolve texture.";
    return std::nullopt;
  }
  resolve_tex->SetLabel("ImpellerOnscreenResolve");

  TextureDescriptor msaa_tex_desc;
  msaa_tex_desc.storage_mode = StorageMode::kDeviceTransient;
  msaa_tex_desc.type = TextureType::kTexture2DMultisample;
  msaa_tex_desc.sample_count = SampleCount::kCount4;
  msaa_tex_desc.format = resolve_tex->GetTextureDescriptor().format;
  msaa_tex_desc.size = resolve_tex->GetSize();
  msaa_tex_desc.usage = TextureUsage::kRenderTarget;

  auto msaa_tex = allocator.CreateTexture(msaa_tex_desc);
  if (!msaa_tex) {
    VALIDATION_LOG << "Could not allocate MSAA color texture.";
    return std::nullopt;
  }
  msaa_tex->SetLabel("ImpellerOnscreenColorMSAA");

  ColorAttachment color0;
  color0.texture = msaa_tex;
  color0.clear_color = Color::DarkSlateGray();
  color0.load_action = LoadAction::kClear;
  color0.store_action = StoreAction::kMultisampleResolve;
  color0.resolve_texture = std::move(resolve_tex);

  auto render_target_desc = std::make_optional<RenderTarget>();
  render_target_desc->SetColorAttachment(color0, 0u);

  return render_target_desc;
}

std::unique_ptr<SurfaceMTL> SurfaceMTL::MakeFromMetalLayerDrawable(
    const std::shared_ptr<Context>& context,
    id<CAMetalDrawable> drawable,
    std::optional<IRect> clip_rect) {
  return SurfaceMTL::MakeFromTexture(context, drawable.texture, clip_rect,
                                     drawable);
}

std::unique_ptr<SurfaceMTL> SurfaceMTL::MakeFromTexture(
    const std::shared_ptr<Context>& context,
    id<MTLTexture> texture,
    std::optional<IRect> clip_rect,
    id<CAMetalDrawable> drawable) {
  bool partial_repaint_blit_required = ShouldPerformPartialRepaint(clip_rect);

  // The returned render target is the texture that Impeller will render the
  // root pass to. If partial repaint is in use, this may be a new texture which
  // is smaller than the given MTLTexture.
  auto render_target =
      WrapTextureWithRenderTarget(*context->GetResourceAllocator(), texture,
                                  partial_repaint_blit_required, clip_rect);
  if (!render_target) {
    return nullptr;
  }

  // If partial repainting, set a "source" texture. The presence of a source
  // texture and clip rect instructs the surface to blit this texture to the
  // destination texture.
  auto source_texture = partial_repaint_blit_required
                            ? render_target->GetRenderTargetTexture()
                            : nullptr;

  // The final "destination" texture is the texture that will be presented. In
  // this case, it's always the given drawable.
  std::shared_ptr<Texture> destination_texture;
  if (partial_repaint_blit_required) {
    // If blitting for partial repaint, we need to wrap the drawable. Simply
    // reuse the texture descriptor that was already formed for the new render
    // target, but override the size with the drawable's size.
    auto destination_descriptor =
        render_target->GetRenderTargetTexture()->GetTextureDescriptor();
    destination_descriptor.size = {static_cast<ISize::Type>(texture.width),
                                   static_cast<ISize::Type>(texture.height)};
    destination_texture = TextureMTL::Wrapper(destination_descriptor, texture);
  } else {
    // When not partial repaint blit is needed, the render target texture _is_
    // the drawable texture.
    destination_texture = render_target->GetRenderTargetTexture();
  }

  return std::unique_ptr<SurfaceMTL>(new SurfaceMTL(
      context,                                  // context
      *render_target,                           // target
      render_target->GetRenderTargetTexture(),  // resolve_texture
      drawable,                                 // drawable
      source_texture,                           // source_texture
      destination_texture,                      // destination_texture
      partial_repaint_blit_required,            // requires_blit
      clip_rect                                 // clip_rect
      ));
}

SurfaceMTL::SurfaceMTL(const std::weak_ptr<Context>& context,
                       const RenderTarget& target,
                       std::shared_ptr<Texture> resolve_texture,
                       id<CAMetalDrawable> drawable,
                       std::shared_ptr<Texture> source_texture,
                       std::shared_ptr<Texture> destination_texture,
                       bool requires_blit,
                       std::optional<IRect> clip_rect)
    : Surface(target),
      context_(context),
      resolve_texture_(std::move(resolve_texture)),
      drawable_(drawable),
      source_texture_(std::move(source_texture)),
      destination_texture_(std::move(destination_texture)),
      requires_blit_(requires_blit),
      clip_rect_(clip_rect) {}

// |Surface|
SurfaceMTL::~SurfaceMTL() = default;

bool SurfaceMTL::ShouldPerformPartialRepaint(std::optional<IRect> damage_rect) {
  // compositor_context.cc will conditionally disable partial repaint if the
  // damage region is large. If that happened, then a nullopt damage rect
  // will be provided here.
  if (!damage_rect.has_value()) {
    return false;
  }
  // If the damage rect is 0 in at least one dimension, partial repaint isn't
  // performed as we skip right to present.
  if (damage_rect->IsEmpty()) {
    return false;
  }
  return true;
}

// |Surface|
IRect SurfaceMTL::coverage() const {
  return IRect::MakeSize(resolve_texture_->GetSize());
}

bool SurfaceMTL::PreparePresent() const {
  auto context = context_.lock();
  if (!context) {
    return false;
  }

#ifdef IMPELLER_DEBUG
  context->GetResourceAllocator()->DebugTraceMemoryStatistics();
  if (frame_boundary_) {
    ContextMTL::Cast(context.get())->GetCaptureManager()->FinishCapture();
  }
#endif  // IMPELLER_DEBUG

  if (requires_blit_) {
    if (!(source_texture_ && destination_texture_)) {
      return false;
    }

    auto blit_command_buffer = context->CreateCommandBuffer();
    if (!blit_command_buffer) {
      return false;
    }
    auto blit_pass = blit_command_buffer->CreateBlitPass();
    if (!clip_rect_.has_value()) {
      VALIDATION_LOG << "Missing clip rectangle.";
      return false;
    }
    blit_pass->AddCopy(source_texture_, destination_texture_, std::nullopt,
                       clip_rect_->GetOrigin());
    blit_pass->EncodeCommands();
    if (!context->GetCommandQueue()->Submit({blit_command_buffer}).ok()) {
      return false;
    }
  }
#ifdef IMPELLER_DEBUG
  ContextMTL::Cast(context.get())->GetGPUTracer()->MarkFrameEnd();
#endif  // IMPELLER_DEBUG
  prepared_ = true;
  return true;
}

// |Surface|
bool SurfaceMTL::Present() const {
  if (!prepared_) {
    PreparePresent();
  }
  auto context = context_.lock();
  if (!context) {
    return false;
  }

  if (drawable_) {
    id<MTLCommandBuffer> command_buffer =
        ContextMTL::Cast(context.get())
            ->CreateMTLCommandBuffer("Present Waiter Command Buffer");

    id<CAMetalDrawable> metal_drawable =
        reinterpret_cast<id<CAMetalDrawable>>(drawable_);
    if ([metal_drawable conformsToProtocol:@protocol(FlutterMetalDrawable)]) {
      [(id<FlutterMetalDrawable>)metal_drawable
          flutterPrepareForPresent:command_buffer];
    }

    // Intel iOS simulators do not seem to give backpressure on Metal drawable
    // aquisition, which can result in Impeller running head of the GPU
    // workload by dozens of frames. Slow this process down by blocking
    // on submit until the last command buffer is at least scheduled.
#if defined(FML_OS_IOS_SIMULATOR) && defined(FML_ARCH_CPU_X86_64)
    constexpr bool alwaysWaitForScheduling = true;
#else
    constexpr bool alwaysWaitForScheduling = false;
#endif  // defined(FML_OS_IOS_SIMULATOR) && defined(FML_ARCH_CPU_X86_64)

    // If the threads have been merged, or there is a pending frame capture,
    // then block on cmd buffer scheduling to ensure that the
    // transaction/capture work correctly.
    if (present_with_transaction_ || [[NSThread currentThread] isMainThread] ||
        [[MTLCaptureManager sharedCaptureManager] isCapturing] ||
        alwaysWaitForScheduling) {
      TRACE_EVENT0("flutter", "waitUntilScheduled");
      [command_buffer commit];
#if defined(FML_OS_IOS_SIMULATOR) && defined(FML_ARCH_CPU_X86_64)
      [command_buffer waitUntilCompleted];
#else
      [command_buffer waitUntilScheduled];
#endif  // defined(FML_OS_IOS_SIMULATOR) && defined(FML_ARCH_CPU_X86_64)
      [drawable_ present];
    } else {
      // The drawable may come from a FlutterMetalLayer, so it can't be
      // presented through the command buffer.
      id<CAMetalDrawable> drawable = drawable_;
      [command_buffer addScheduledHandler:^(id<MTLCommandBuffer> buffer) {
        [drawable present];
      }];
      [command_buffer commit];
    }
  }

  return true;
}
#pragma GCC diagnostic pop

}  // namespace impeller
