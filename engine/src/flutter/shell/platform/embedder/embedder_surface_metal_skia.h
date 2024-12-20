// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_EMBEDDER_EMBEDDER_SURFACE_METAL_SKIA_H_
#define FLUTTER_SHELL_PLATFORM_EMBEDDER_EMBEDDER_SURFACE_METAL_SKIA_H_

#if !SLIMPELLER

#include "flutter/fml/macros.h"
#include "flutter/shell/gpu/gpu_surface_metal_delegate.h"
#include "flutter/shell/gpu/gpu_surface_metal_skia.h"
#include "flutter/shell/platform/embedder/embedder_external_view_embedder.h"
#include "flutter/shell/platform/embedder/embedder_surface.h"

#include "third_party/skia/include/core/SkSurface.h"

namespace flutter {

// TODO(148235): This class is Skia specific and there is another on
// specifically for Impeller called EmbedderSurfaceMetalImpeller. Rename this to
// EmbedderSurfaceMetalSkia to avoid confusion.
class EmbedderSurfaceMetalSkia final : public EmbedderSurface,
                                       public GPUSurfaceMetalDelegate {
 public:
  struct MetalDispatchTable {
    std::function<bool(GPUMTLTextureInfo texture)> present;  // required
    std::function<GPUMTLTextureInfo(const SkISize& frame_size)>
        get_texture;  // required
  };

  EmbedderSurfaceMetalSkia(
      GPUMTLDeviceHandle device,
      GPUMTLCommandQueueHandle command_queue,
      MetalDispatchTable dispatch_table,
      std::shared_ptr<EmbedderExternalViewEmbedder> external_view_embedder);

  ~EmbedderSurfaceMetalSkia() override;

 private:
  bool valid_ = false;
  MetalDispatchTable metal_dispatch_table_;
  std::shared_ptr<EmbedderExternalViewEmbedder> external_view_embedder_;
  sk_sp<SkSurface> surface_;
  sk_sp<GrDirectContext> main_context_;
  sk_sp<GrDirectContext> resource_context_;

  // |EmbedderSurface|
  bool IsValid() const override;

  // |EmbedderSurface|
  std::unique_ptr<Surface> CreateGPUSurface() override;

  // |EmbedderSurface|
  sk_sp<GrDirectContext> CreateResourceContext() const override;

  // |GPUSurfaceMetalDelegate|
  GPUCAMetalLayerHandle GetCAMetalLayer(
      const SkISize& frame_size) const override;

  // |GPUSurfaceMetalDelegate|
  bool PresentDrawable(GrMTLHandle drawable) const override;

  // |GPUSurfaceMetalDelegate|
  GPUMTLTextureInfo GetMTLTexture(const SkISize& frame_size) const override;

  // |GPUSurfaceMetalDelegate|
  bool PresentTexture(GPUMTLTextureInfo texture) const override;

  FML_DISALLOW_COPY_AND_ASSIGN(EmbedderSurfaceMetalSkia);
};

}  // namespace flutter

#endif  //  !SLIMPELLER

#endif  // FLUTTER_SHELL_PLATFORM_EMBEDDER_EMBEDDER_SURFACE_METAL_SKIA_H_
