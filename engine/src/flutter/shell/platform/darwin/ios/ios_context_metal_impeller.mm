// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/ios/ios_context_metal_impeller.h"

#include "flutter/impeller/entity/mtl/entity_shaders.h"
#import "flutter/shell/platform/darwin/ios/ios_external_texture_metal.h"
#include "impeller/display_list/aiks_context.h"
#include "impeller/typographer/backends/skia/typographer_context_skia.h"

FLUTTER_ASSERT_ARC

namespace flutter {

IOSContextMetalImpeller::IOSContextMetalImpeller(
    const std::shared_ptr<const fml::SyncSwitch>& is_gpu_disabled_sync_switch)
    : darwin_context_metal_impeller_(
          [[FlutterDarwinContextMetalImpeller alloc] init:is_gpu_disabled_sync_switch]) {
  if (darwin_context_metal_impeller_.context) {
    aiks_context_ = std::make_shared<impeller::AiksContext>(
        darwin_context_metal_impeller_.context, impeller::TypographerContextSkia::Make());
  }
}

IOSContextMetalImpeller::~IOSContextMetalImpeller() = default;

IOSRenderingBackend IOSContextMetalImpeller::GetBackend() const {
  return IOSRenderingBackend::kImpeller;
}

sk_sp<GrDirectContext> IOSContextMetalImpeller::GetMainContext() const {
  return nullptr;
}

sk_sp<GrDirectContext> IOSContextMetalImpeller::GetResourceContext() const {
  return nullptr;
}

// |IOSContext|
sk_sp<GrDirectContext> IOSContextMetalImpeller::CreateResourceContext() {
  return nullptr;
}

// |IOSContext|
std::shared_ptr<impeller::Context> IOSContextMetalImpeller::GetImpellerContext() const {
  return darwin_context_metal_impeller_.context;
}

// |IOSContext|
std::shared_ptr<impeller::AiksContext> IOSContextMetalImpeller::GetAiksContext() const {
  return aiks_context_;
}

// |IOSContext|
std::unique_ptr<GLContextResult> IOSContextMetalImpeller::MakeCurrent() {
  // This only makes sense for contexts that need to be bound to a specific thread.
  return std::make_unique<GLContextDefaultResult>(true);
}

// |IOSContext|
std::unique_ptr<Texture> IOSContextMetalImpeller::CreateExternalTexture(
    int64_t texture_id,
    NSObject<FlutterTexture>* texture) {
  return std::make_unique<IOSExternalTextureMetal>([darwin_context_metal_impeller_
      createExternalTextureWithIdentifier:texture_id
                                  texture:texture]);
}

}  // namespace flutter
