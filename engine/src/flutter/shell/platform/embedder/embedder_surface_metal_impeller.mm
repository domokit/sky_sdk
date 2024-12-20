// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <memory>
#include <utility>

#include "flutter/shell/platform/embedder/embedder_surface_metal_impeller.h"

#include "flutter/fml/logging.h"
#include "flutter/fml/synchronization/sync_switch.h"
#include "flutter/shell/gpu/gpu_surface_metal_delegate.h"
#include "flutter/shell/gpu/gpu_surface_metal_impeller.h"
#import "flutter/shell/platform/darwin/graphics/FlutterDarwinContextMetalImpeller.h"
#include "impeller/display_list/aiks_context.h"
#include "impeller/entity/mtl/entity_shaders.h"
#include "impeller/entity/mtl/framebuffer_blend_shaders.h"
#include "impeller/entity/mtl/modern_shaders.h"
#include "impeller/renderer/backend/metal/context_mtl.h"
#include "impeller/typographer/backends/skia/typographer_context_skia.h"
#include "impeller/typographer/typographer_context.h"

FLUTTER_ASSERT_ARC

namespace flutter {

EmbedderSurfaceMetalImpeller::EmbedderSurfaceMetalImpeller(
    GPUMTLDeviceHandle device,
    GPUMTLCommandQueueHandle command_queue,
    MetalDispatchTable metal_dispatch_table,
    std::shared_ptr<EmbedderExternalViewEmbedder> external_view_embedder)
    : GPUSurfaceMetalDelegate(MTLRenderTargetType::kMTLTexture),
      metal_dispatch_table_(std::move(metal_dispatch_table)),
      external_view_embedder_(std::move(external_view_embedder)) {
  std::vector<std::shared_ptr<fml::Mapping>> shader_mappings = {
      std::make_shared<fml::NonOwnedMapping>(impeller_entity_shaders_data,
                                             impeller_entity_shaders_length),
      std::make_shared<fml::NonOwnedMapping>(impeller_modern_shaders_data,
                                             impeller_modern_shaders_length),
      std::make_shared<fml::NonOwnedMapping>(impeller_framebuffer_blend_shaders_data,
                                             impeller_framebuffer_blend_shaders_length),
  };
  context_ = impeller::ContextMTL::Create(
      (__bridge id<MTLDevice>)device,               // device
      (__bridge id<MTLCommandQueue>)command_queue,  // command_queue
      shader_mappings,                              // shader_libraries_data
      std::make_shared<fml::SyncSwitch>(false),     // is_gpu_disabled_sync_switch
      "Impeller Library"                            // library_label
  );
  FML_LOG(IMPORTANT) << "Using the Impeller rendering backend (Metal).";

  valid_ = !!context_;
}

EmbedderSurfaceMetalImpeller::~EmbedderSurfaceMetalImpeller() = default;

bool EmbedderSurfaceMetalImpeller::IsValid() const {
  return valid_;
}

std::unique_ptr<Surface> EmbedderSurfaceMetalImpeller::CreateGPUSurface()
    IMPELLER_CA_METAL_LAYER_AVAILABLE {
  if (!IsValid()) {
    return nullptr;
  }
  if (!aiks_context_) {
    aiks_context_ =
        std::make_shared<impeller::AiksContext>(context_, impeller::TypographerContextSkia::Make());
  }

  const bool render_to_surface = !external_view_embedder_;
  auto surface = std::make_unique<GPUSurfaceMetalImpeller>(this, aiks_context_, render_to_surface);

  if (!surface->IsValid()) {
    return nullptr;
  }

  return surface;
}

std::shared_ptr<impeller::Context> EmbedderSurfaceMetalImpeller::CreateImpellerContext() const {
  return context_;
}

GPUCAMetalLayerHandle EmbedderSurfaceMetalImpeller::GetCAMetalLayer(
    const SkISize& frame_info) const {
  FML_CHECK(false) << "Only rendering to MTLTexture is supported.";
  return nullptr;
}

bool EmbedderSurfaceMetalImpeller::PresentDrawable(GrMTLHandle drawable) const {
  FML_CHECK(false) << "Only rendering to MTLTexture is supported.";
  return false;
}

GPUMTLTextureInfo EmbedderSurfaceMetalImpeller::GetMTLTexture(const SkISize& frame_info) const {
  return metal_dispatch_table_.get_texture(frame_info);
}

bool EmbedderSurfaceMetalImpeller::PresentTexture(GPUMTLTextureInfo texture) const {
  return metal_dispatch_table_.present(texture);
}

}  // namespace flutter
