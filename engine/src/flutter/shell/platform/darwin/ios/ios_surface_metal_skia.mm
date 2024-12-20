// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#if !SLIMPELLER

#import "flutter/shell/platform/darwin/ios/ios_surface_metal_skia.h"

#include "flutter/shell/gpu/gpu_surface_metal_delegate.h"
#include "flutter/shell/gpu/gpu_surface_metal_skia.h"
#include "flutter/shell/platform/darwin/ios/ios_context_metal_skia.h"

FLUTTER_ASSERT_ARC

@protocol FlutterMetalDrawable <MTLDrawable>
- (void)flutterPrepareForPresent:(nonnull id<MTLCommandBuffer>)commandBuffer;
@end

namespace flutter {

IOSSurfaceMetalSkia::IOSSurfaceMetalSkia(CAMetalLayer* layer, std::shared_ptr<IOSContext> context)
    : IOSSurface(std::move(context)),
      GPUSurfaceMetalDelegate(MTLRenderTargetType::kCAMetalLayer),
      layer_(layer) {
  is_valid_ = layer_;
  IOSContextMetalSkia* metal_context = static_cast<IOSContextMetalSkia*>(GetContext().get());
  FlutterDarwinContextMetalSkia* darwin_context = metal_context->GetDarwinContext();
  command_queue_ = darwin_context.commandQueue;
  device_ = darwin_context.device;
}

// |IOSSurface|
IOSSurfaceMetalSkia::~IOSSurfaceMetalSkia() = default;

// |IOSSurface|
bool IOSSurfaceMetalSkia::IsValid() const {
  return is_valid_;
}

// |IOSSurface|
void IOSSurfaceMetalSkia::UpdateStorageSizeIfNecessary() {
  // Nothing to do.
}

// |IOSSurface|
std::unique_ptr<Surface> IOSSurfaceMetalSkia::CreateGPUSurface(GrDirectContext* context) {
  FML_DCHECK(context);
  return std::make_unique<GPUSurfaceMetalSkia>(this,               // delegate
                                               sk_ref_sp(context)  // context
  );
}

// |GPUSurfaceMetalDelegate|
GPUCAMetalLayerHandle IOSSurfaceMetalSkia::GetCAMetalLayer(const SkISize& frame_info) const {
  layer_.device = device_;

  layer_.pixelFormat = MTLPixelFormatBGRA8Unorm;
  // Flutter needs to read from the color attachment in cases where there are effects such as
  // backdrop filters. Flutter plugins that create platform views may also read from the layer.
  layer_.framebufferOnly = NO;

  const auto drawable_size = CGSizeMake(frame_info.width(), frame_info.height());
  if (!CGSizeEqualToSize(drawable_size, layer_.drawableSize)) {
    layer_.drawableSize = drawable_size;
  }

  // When there are platform views in the scene, the drawable needs to be presented in the same
  // transaction as the one created for platform views. When the drawable are being presented from
  // the raster thread, there is no such transaction.
  layer_.presentsWithTransaction = [[NSThread currentThread] isMainThread];

  return (__bridge GPUCAMetalLayerHandle)layer_;
}

// |GPUSurfaceMetalDelegate|
bool IOSSurfaceMetalSkia::PreparePresent(GrMTLHandle drawable) const {
  id<MTLCommandBuffer> command_buffer = [command_queue_ commandBuffer];
  id<CAMetalDrawable> metal_drawable = (__bridge id<CAMetalDrawable>)drawable;
  if ([metal_drawable conformsToProtocol:@protocol(FlutterMetalDrawable)]) {
    [(id<FlutterMetalDrawable>)metal_drawable flutterPrepareForPresent:command_buffer];
  }
  [command_buffer commit];
  [command_buffer waitUntilScheduled];
  return true;
}

// |GPUSurfaceMetalDelegate|
bool IOSSurfaceMetalSkia::PresentDrawable(GrMTLHandle drawable) const {
  if (drawable == nullptr) {
    FML_DLOG(ERROR) << "Could not acquire next Metal drawable from the SkSurface.";
    return false;
  }

  id<CAMetalDrawable> metal_drawable = (__bridge id<CAMetalDrawable>)drawable;
  [metal_drawable present];
  return true;
}

// |GPUSurfaceMetalDelegate|
GPUMTLTextureInfo IOSSurfaceMetalSkia::GetMTLTexture(const SkISize& frame_info) const {
  FML_CHECK(false) << "render to texture not supported on ios";
  return {.texture_id = -1, .texture = nullptr};
}

// |GPUSurfaceMetalDelegate|
bool IOSSurfaceMetalSkia::PresentTexture(GPUMTLTextureInfo texture) const {
  FML_CHECK(false) << "render to texture not supported on ios";
  return false;
}

// |GPUSurfaceMetalDelegate|
bool IOSSurfaceMetalSkia::AllowsDrawingWhenGpuDisabled() const {
  return false;
}

}  // namespace flutter

#endif  //  !SLIMPELLER
