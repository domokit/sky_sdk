// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/display_list/testing/dl_test_surface_provider.h"

#include "third_party/skia/include/core/SkCanvas.h"
#include "third_party/skia/include/core/SkData.h"
#include "third_party/skia/include/core/SkImage.h"
#include "third_party/skia/include/core/SkSurface.h"
#include "third_party/skia/include/encode/SkPngEncoder.h"
#include "third_party/skia/include/gpu/ganesh/GrDirectContext.h"

namespace flutter::testing {

std::string DlSurfaceProvider::BackendName(BackendType type) {
  switch (type) {
    case kSoftwareBackend:
      return "Software";
    case kOpenGlBackend:
      return "OpenGL";
    case kMetalBackend:
      return "Metal";
  }
}

std::unique_ptr<DlSurfaceProvider> DlSurfaceProvider::Create(
    BackendType backend_type) {
  switch (backend_type) {
    case kSoftwareBackend:
      return CreateSoftware();
    case kOpenGlBackend:
      return CreateOpenGL();
    case kMetalBackend:
      return CreateMetal();
  }
}

bool DlSurfaceProvider::Snapshot(std::string& filename) const {
#ifdef BENCHMARKS_NO_SNAPSHOT
  return false;
#else
  auto image = GetPrimarySurface()->sk_surface()->makeImageSnapshot();
  if (!image) {
    return false;
  }
  auto raster = image->makeRasterImage();
  if (!raster) {
    return false;
  }
  auto data = SkPngEncoder::Encode(nullptr, raster.get(), {});
  if (!data) {
    return false;
  }
  fml::NonOwnedMapping mapping(static_cast<const uint8_t*>(data->data()),
                               data->size());
  return WriteAtomically(OpenFixturesDirectory(), filename.c_str(), mapping);
#endif
}

#ifndef ENABLE_SOFTWARE_BENCHMARKS
std::unique_ptr<DlSurfaceProvider> DlSurfaceProvider::CreateSoftware() {
  return nullptr;
}
#endif
#ifndef ENABLE_OPENGL_BENCHMARKS
std::unique_ptr<DlSurfaceProvider> DlSurfaceProvider::CreateOpenGL() {
  return nullptr;
}
#endif
#ifndef ENABLE_METAL_BENCHMARKS
std::unique_ptr<DlSurfaceProvider> DlSurfaceProvider::CreateMetal() {
  return nullptr;
}
#endif

void DlSurfaceInstance::FlushSubmitCpuSync() {
  auto surface = sk_surface();
  if (!surface) {
    return;
  }
  if (GrDirectContext* dContext =
          GrAsDirectContext(surface->recordingContext())) {
    dContext->flushAndSubmit(surface.get(), GrSyncCpu::kYes);
  }
}

}  // namespace flutter::testing
