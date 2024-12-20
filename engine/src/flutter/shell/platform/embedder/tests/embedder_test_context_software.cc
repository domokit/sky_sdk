// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/embedder/tests/embedder_test_context_software.h"

#include <utility>

#include "flutter/fml/make_copyable.h"
#include "flutter/fml/paths.h"
#include "flutter/runtime/dart_vm.h"
#include "flutter/shell/platform/embedder/tests/embedder_assertions.h"
#include "flutter/shell/platform/embedder/tests/embedder_test_compositor_software.h"
#include "flutter/testing/testing.h"
#include "third_party/dart/runtime/bin/elf_loader.h"
#include "third_party/skia/include/core/SkBitmap.h"
#include "third_party/skia/include/core/SkSurface.h"

namespace flutter::testing {

EmbedderTestContextSoftware::EmbedderTestContextSoftware(
    std::string assets_path)
    : EmbedderTestContext(std::move(assets_path)) {
  renderer_config_.type = FlutterRendererType::kSoftware;
  renderer_config_.software = {
      .struct_size = sizeof(FlutterSoftwareRendererConfig),
      .surface_present_callback =
          [](void* context, const void* allocation, size_t row_bytes,
             size_t height) {
            auto image_info = SkImageInfo::MakeN32Premul(
                SkISize::Make(row_bytes / 4, height));
            SkBitmap bitmap;
            if (!bitmap.installPixels(image_info, const_cast<void*>(allocation),
                                      row_bytes)) {
              FML_LOG(ERROR) << "Could not copy pixels for the software "
                                "composition from the engine.";
              return false;
            }
            bitmap.setImmutable();
            return reinterpret_cast<EmbedderTestContextSoftware*>(context)
                ->Present(SkImages::RasterFromBitmap(bitmap));
          },
  };
}

EmbedderTestContextSoftware::~EmbedderTestContextSoftware() = default;

EmbedderTestContextType EmbedderTestContextSoftware::GetContextType() const {
  return EmbedderTestContextType::kSoftwareContext;
}

void EmbedderTestContextSoftware::SetSurface(SkISize surface_size) {
  surface_size_ = surface_size;
}

void EmbedderTestContextSoftware::SetupCompositor() {
  FML_CHECK(!compositor_) << "Already set up a compositor in this context.";
  compositor_ = std::make_unique<EmbedderTestCompositorSoftware>(surface_size_);
}

size_t EmbedderTestContextSoftware::GetSurfacePresentCount() const {
  return software_surface_present_count_;
}

bool EmbedderTestContextSoftware::Present(const sk_sp<SkImage>& image) {
  software_surface_present_count_++;
  FireRootSurfacePresentCallbackIfPresent([image] { return image; });
  return true;
}

}  // namespace flutter::testing
