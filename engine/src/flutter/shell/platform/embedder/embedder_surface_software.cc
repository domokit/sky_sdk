// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/embedder/embedder_surface_software.h"

#include <utility>

#include "flutter/fml/trace_event.h"

#include "third_party/skia/include/core/SkColorSpace.h"
#include "third_party/skia/include/core/SkImageInfo.h"
#include "third_party/skia/include/core/SkSurface.h"
#include "third_party/skia/include/gpu/ganesh/GrDirectContext.h"

namespace flutter {

EmbedderSurfaceSoftware::EmbedderSurfaceSoftware(
    SoftwareDispatchTable software_dispatch_table,
    std::shared_ptr<EmbedderExternalViewEmbedder> external_view_embedder)
    : software_dispatch_table_(std::move(software_dispatch_table)),
      external_view_embedder_(std::move(external_view_embedder)) {
  if (!software_dispatch_table_.software_present_backing_store) {
    return;
  }
  valid_ = true;
}

EmbedderSurfaceSoftware::~EmbedderSurfaceSoftware() = default;

// |EmbedderSurface|
bool EmbedderSurfaceSoftware::IsValid() const {
  return valid_;
}

// |EmbedderSurface|
std::unique_ptr<Surface> EmbedderSurfaceSoftware::CreateGPUSurface() {
  if (!IsValid()) {
    return nullptr;
  }
  const bool render_to_surface = !external_view_embedder_;
  auto surface = std::make_unique<GPUSurfaceSoftware>(this, render_to_surface);

  if (!surface->IsValid()) {
    return nullptr;
  }

  return surface;
}

// |EmbedderSurface|
sk_sp<GrDirectContext> EmbedderSurfaceSoftware::CreateResourceContext() const {
  return nullptr;
}

// |GPUSurfaceSoftwareDelegate|
sk_sp<SkSurface> EmbedderSurfaceSoftware::AcquireBackingStore(
    const SkISize& size) {
  TRACE_EVENT0("flutter", "EmbedderSurfaceSoftware::AcquireBackingStore");
  if (!IsValid()) {
    FML_LOG(ERROR)
        << "Could not acquire backing store for the software surface.";
    return nullptr;
  }

  if (sk_surface_ != nullptr &&
      SkISize::Make(sk_surface_->width(), sk_surface_->height()) == size) {
    // The old and new surface sizes are the same. Nothing to do here.
    return sk_surface_;
  }

  SkImageInfo info = SkImageInfo::MakeN32(
      size.fWidth, size.fHeight, kPremul_SkAlphaType, SkColorSpace::MakeSRGB());
  sk_surface_ = SkSurfaces::Raster(info, nullptr);

  if (sk_surface_ == nullptr) {
    FML_LOG(ERROR) << "Could not create backing store for software rendering.";
    return nullptr;
  }

  return sk_surface_;
}

// |GPUSurfaceSoftwareDelegate|
bool EmbedderSurfaceSoftware::PresentBackingStore(
    sk_sp<SkSurface> backing_store) {
  if (!IsValid()) {
    FML_LOG(ERROR) << "Tried to present an invalid software surface.";
    return false;
  }

  SkPixmap pixmap;
  if (!backing_store->peekPixels(&pixmap)) {
    FML_LOG(ERROR) << "Could not peek the pixels of the backing store.";
    return false;
  }

  // Some basic sanity checking.
  uint64_t expected_pixmap_data_size = pixmap.width() * pixmap.height() * 4;

  const size_t pixmap_size = pixmap.computeByteSize();

  if (expected_pixmap_data_size != pixmap_size) {
    FML_LOG(ERROR) << "Software backing store had unexpected size.";
    return false;
  }

  return software_dispatch_table_.software_present_backing_store(
      pixmap.addr(),      //
      pixmap.rowBytes(),  //
      pixmap.height()     //
  );
}

}  // namespace flutter
