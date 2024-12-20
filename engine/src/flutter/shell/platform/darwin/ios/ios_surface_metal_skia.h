// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_IOS_IOS_SURFACE_METAL_SKIA_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_IOS_IOS_SURFACE_METAL_SKIA_H_

#if !SLIMPELLER

#include "flutter/fml/macros.h"
#include "flutter/shell/gpu/gpu_surface_metal_delegate.h"
#import "flutter/shell/platform/darwin/ios/ios_surface.h"
#include "third_party/skia/include/gpu/ganesh/mtl/GrMtlTypes.h"

@class CAMetalLayer;

namespace flutter {

class SK_API_AVAILABLE_CA_METAL_LAYER IOSSurfaceMetalSkia final : public IOSSurface,
                                                                  public GPUSurfaceMetalDelegate {
 public:
  IOSSurfaceMetalSkia(CAMetalLayer* layer, std::shared_ptr<IOSContext> context);

  // |IOSSurface|
  ~IOSSurfaceMetalSkia();

 private:
  CAMetalLayer* layer_;
  id<MTLDevice> device_;
  id<MTLCommandQueue> command_queue_;
  bool is_valid_ = false;

  // |IOSSurface|
  bool IsValid() const override;

  // |IOSSurface|
  void UpdateStorageSizeIfNecessary() override;

  // |IOSSurface|
  std::unique_ptr<Surface> CreateGPUSurface(GrDirectContext* gr_context) override;

  // |GPUSurfaceMetalDelegate|
  GPUCAMetalLayerHandle GetCAMetalLayer(const SkISize& frame_info) const override
      __attribute__((cf_audited_transfer));

  // |GPUSurfaceMetalDelegate|
  bool PresentDrawable(GrMTLHandle drawable) const override __attribute__((cf_audited_transfer));

  // |GPUSurfaceMetalDelegate|
  bool PreparePresent(GrMTLHandle drawable) const override;

  // |GPUSurfaceMetalDelegate|
  GPUMTLTextureInfo GetMTLTexture(const SkISize& frame_info) const override
      __attribute__((cf_audited_transfer));

  // |GPUSurfaceMetalDelegate|
  bool PresentTexture(GPUMTLTextureInfo texture) const override
      __attribute__((cf_audited_transfer));

  // |GPUSurfaceMetalDelegate|
  bool AllowsDrawingWhenGpuDisabled() const override;

  FML_DISALLOW_COPY_AND_ASSIGN(IOSSurfaceMetalSkia);
};

}  // namespace flutter

#endif  //  !SLIMPELLER

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_IOS_IOS_SURFACE_METAL_SKIA_H_
