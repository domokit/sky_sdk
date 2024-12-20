// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_EMBEDDER_EMBEDDER_SURFACE_SOFTWARE_H_
#define FLUTTER_SHELL_PLATFORM_EMBEDDER_EMBEDDER_SURFACE_SOFTWARE_H_

#include "flutter/fml/macros.h"
#include "flutter/shell/gpu/gpu_surface_software.h"
#include "flutter/shell/platform/embedder/embedder_external_view_embedder.h"
#include "flutter/shell/platform/embedder/embedder_surface.h"

#include "third_party/skia/include/core/SkSurface.h"

namespace flutter {

class EmbedderSurfaceSoftware final : public EmbedderSurface,
                                      public GPUSurfaceSoftwareDelegate {
 public:
  struct SoftwareDispatchTable {
    std::function<bool(const void* allocation, size_t row_bytes, size_t height)>
        software_present_backing_store;  // required
  };

  EmbedderSurfaceSoftware(
      SoftwareDispatchTable software_dispatch_table,
      std::shared_ptr<EmbedderExternalViewEmbedder> external_view_embedder);

  ~EmbedderSurfaceSoftware() override;

 private:
  bool valid_ = false;
  SoftwareDispatchTable software_dispatch_table_;
  sk_sp<SkSurface> sk_surface_;
  std::shared_ptr<EmbedderExternalViewEmbedder> external_view_embedder_;

  // |EmbedderSurface|
  bool IsValid() const override;

  // |EmbedderSurface|
  std::unique_ptr<Surface> CreateGPUSurface() override;

  // |EmbedderSurface|
  sk_sp<GrDirectContext> CreateResourceContext() const override;

  // |GPUSurfaceSoftwareDelegate|
  sk_sp<SkSurface> AcquireBackingStore(const SkISize& size) override;

  // |GPUSurfaceSoftwareDelegate|
  bool PresentBackingStore(sk_sp<SkSurface> backing_store) override;

  FML_DISALLOW_COPY_AND_ASSIGN(EmbedderSurfaceSoftware);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_EMBEDDER_EMBEDDER_SURFACE_SOFTWARE_H_
