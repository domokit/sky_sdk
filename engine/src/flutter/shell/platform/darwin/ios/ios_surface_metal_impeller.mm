// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/ios/ios_surface_metal_impeller.h"

#include "flutter/impeller/renderer/backend/metal/formats_mtl.h"
#include "flutter/impeller/renderer/context.h"
#include "flutter/shell/gpu/gpu_surface_metal_impeller.h"
#include "impeller/display_list/aiks_context.h"
#include "impeller/typographer/backends/skia/typographer_context_skia.h"
#include "impeller/typographer/typographer_context.h"

FLUTTER_ASSERT_ARC

namespace flutter {

IOSSurfaceMetalImpeller::IOSSurfaceMetalImpeller(CAMetalLayer* layer,
                                                 const std::shared_ptr<IOSContext>& context)
    : IOSSurface(context),
      GPUSurfaceMetalDelegate(MTLRenderTargetType::kCAMetalLayer),
      layer_(layer),
      impeller_context_(context ? context->GetImpellerContext() : nullptr),
      aiks_context_(context ? context->GetAiksContext() : nullptr) {
  if (!impeller_context_ || !aiks_context_) {
    return;
  }
  is_valid_ = true;
}

// |IOSSurface|
IOSSurfaceMetalImpeller::~IOSSurfaceMetalImpeller() = default;

// |IOSSurface|
bool IOSSurfaceMetalImpeller::IsValid() const {
  return is_valid_;
}

// |IOSSurface|
void IOSSurfaceMetalImpeller::UpdateStorageSizeIfNecessary() {
  // Nothing to do.
}

// |IOSSurface|
std::unique_ptr<Surface> IOSSurfaceMetalImpeller::CreateGPUSurface(GrDirectContext*) {
  impeller_context_->UpdateOffscreenLayerPixelFormat(
      impeller::FromMTLPixelFormat(layer_.pixelFormat));
  return std::make_unique<GPUSurfaceMetalImpeller>(this,          //
                                                   aiks_context_  //
  );
}

// |GPUSurfaceMetalDelegate|
GPUCAMetalLayerHandle IOSSurfaceMetalImpeller::GetCAMetalLayer(const SkISize& frame_info) const {
  const auto drawable_size = CGSizeMake(frame_info.width(), frame_info.height());
  if (!CGSizeEqualToSize(drawable_size, layer_.drawableSize)) {
    layer_.drawableSize = drawable_size;
  }

  // Flutter needs to read from the color attachment in cases where there are effects such as
  // backdrop filters. Flutter plugins that create platform views may also read from the layer.
  layer_.framebufferOnly = NO;

  return (__bridge GPUCAMetalLayerHandle)layer_;
}

// |GPUSurfaceMetalDelegate|
bool IOSSurfaceMetalImpeller::PresentDrawable(GrMTLHandle drawable) const {
  FML_DCHECK(false);
  return false;
}

// |GPUSurfaceMetalDelegate|
GPUMTLTextureInfo IOSSurfaceMetalImpeller::GetMTLTexture(const SkISize& frame_info) const {
  FML_CHECK(false);
  return GPUMTLTextureInfo{
      .texture_id = -1,   //
      .texture = nullptr  //
  };
}

// |GPUSurfaceMetalDelegate|
bool IOSSurfaceMetalImpeller::PresentTexture(GPUMTLTextureInfo texture) const {
  FML_CHECK(false);
  return false;
}

// |GPUSurfaceMetalDelegate|
bool IOSSurfaceMetalImpeller::AllowsDrawingWhenGpuDisabled() const {
  return false;
}

}  // namespace flutter
