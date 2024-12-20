// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_EMBEDDER_EMBEDDER_SURFACE_METAL_IMPELLER_H_
#define FLUTTER_SHELL_PLATFORM_EMBEDDER_EMBEDDER_SURFACE_METAL_IMPELLER_H_

#include <memory>
#include "flutter/fml/macros.h"
#include "flutter/shell/gpu/gpu_surface_metal_delegate.h"
#include "flutter/shell/gpu/gpu_surface_metal_skia.h"
#include "flutter/shell/platform/embedder/embedder_external_view_embedder.h"
#include "flutter/shell/platform/embedder/embedder_surface.h"
#include "fml/concurrent_message_loop.h"
#include "impeller/display_list/aiks_context.h"

namespace impeller {
class Context;
}

namespace flutter {

class EmbedderSurfaceMetalImpeller final : public EmbedderSurface,
                                           public GPUSurfaceMetalDelegate {
 public:
  struct MetalDispatchTable {
    std::function<bool(GPUMTLTextureInfo texture)> present;  // required
    std::function<GPUMTLTextureInfo(const SkISize& frame_size)>
        get_texture;  // required
  };

  EmbedderSurfaceMetalImpeller(
      GPUMTLDeviceHandle device,
      GPUMTLCommandQueueHandle command_queue,
      MetalDispatchTable dispatch_table,
      std::shared_ptr<EmbedderExternalViewEmbedder> external_view_embedder);

  ~EmbedderSurfaceMetalImpeller() override;

 private:
  bool valid_ = false;
  MetalDispatchTable metal_dispatch_table_;
  std::shared_ptr<EmbedderExternalViewEmbedder> external_view_embedder_;
  std::shared_ptr<impeller::Context> context_;
  std::shared_ptr<impeller::AiksContext> aiks_context_;

  // |EmbedderSurface|
  bool IsValid() const override;

  // |EmbedderSurface|
  std::unique_ptr<Surface> CreateGPUSurface() override;

  // |GPUSurfaceMetalDelegate|
  GPUCAMetalLayerHandle GetCAMetalLayer(
      const SkISize& frame_size) const override;

  // |GPUSurfaceMetalDelegate|
  bool PresentDrawable(GrMTLHandle drawable) const override;

  // |GPUSurfaceMetalDelegate|
  GPUMTLTextureInfo GetMTLTexture(const SkISize& frame_size) const override;

  // |GPUSurfaceMetalDelegate|
  bool PresentTexture(GPUMTLTextureInfo texture) const override;

  // |EmbedderSurface|
  std::shared_ptr<impeller::Context> CreateImpellerContext() const override;

  FML_DISALLOW_COPY_AND_ASSIGN(EmbedderSurfaceMetalImpeller);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_EMBEDDER_EMBEDDER_SURFACE_METAL_IMPELLER_H_
