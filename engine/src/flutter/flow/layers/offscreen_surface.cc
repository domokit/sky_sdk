// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#if !SLIMPELLER

#include "flutter/flow/layers/offscreen_surface.h"

#include "third_party/skia/include/core/SkColorSpace.h"
#include "third_party/skia/include/core/SkData.h"
#include "third_party/skia/include/core/SkImage.h"
#include "third_party/skia/include/core/SkImageInfo.h"
#include "third_party/skia/include/core/SkPixmap.h"
#include "third_party/skia/include/encode/SkPngEncoder.h"
#include "third_party/skia/include/gpu/ganesh/GrDirectContext.h"
#include "third_party/skia/include/gpu/ganesh/SkSurfaceGanesh.h"

namespace flutter {

static sk_sp<SkSurface> CreateSnapshotSurface(GrDirectContext* surface_context,
                                              const DlISize& size) {
  const auto image_info = SkImageInfo::MakeN32Premul(size.width, size.height,
                                                     SkColorSpace::MakeSRGB());
  if (surface_context) {
    // There is a rendering surface that may contain textures that are going to
    // be referenced in the layer tree about to be drawn.
    return SkSurfaces::RenderTarget(surface_context, skgpu::Budgeted::kNo,
                                    image_info);
  }

  // There is no rendering surface, assume no GPU textures are present and
  // create a raster surface.
  return SkSurfaces::Raster(image_info);
}

/// Returns a buffer containing a snapshot of the surface.
///
/// If compressed is true the data is encoded as PNG.
static sk_sp<SkData> GetRasterData(const sk_sp<SkSurface>& offscreen_surface,
                                   bool compressed) {
  // Prepare an image from the surface, this image may potentially be on th GPU.
  auto potentially_gpu_snapshot = offscreen_surface->makeImageSnapshot();
  if (!potentially_gpu_snapshot) {
    FML_LOG(ERROR) << "Screenshot: unable to make image screenshot";
    return nullptr;
  }

  // Copy the GPU image snapshot into CPU memory.
  // TODO (https://github.com/flutter/flutter/issues/13498)
  auto cpu_snapshot = potentially_gpu_snapshot->makeRasterImage();
  if (!cpu_snapshot) {
    FML_LOG(ERROR) << "Screenshot: unable to make raster image";
    return nullptr;
  }

  // If the caller want the pixels to be compressed, there is a Skia utility to
  // compress to PNG. Use that.
  if (compressed) {
    return SkPngEncoder::Encode(nullptr, cpu_snapshot.get(), {});
  }

  // Copy it into a bitmap and return the same.
  SkPixmap pixmap;
  if (!cpu_snapshot->peekPixels(&pixmap)) {
    FML_LOG(ERROR) << "Screenshot: unable to obtain bitmap pixels";
    return nullptr;
  }
  return SkData::MakeWithCopy(pixmap.addr32(), pixmap.computeByteSize());
}

OffscreenSurface::OffscreenSurface(GrDirectContext* surface_context,
                                   const DlISize& size) {
  offscreen_surface_ = CreateSnapshotSurface(surface_context, size);
  if (offscreen_surface_) {
    adapter_.set_canvas(offscreen_surface_->getCanvas());
  }
}

sk_sp<SkData> OffscreenSurface::GetRasterData(bool compressed) const {
  return flutter::GetRasterData(offscreen_surface_, compressed);
}

DlCanvas* OffscreenSurface::GetCanvas() {
  return &adapter_;
}

bool OffscreenSurface::IsValid() const {
  return offscreen_surface_ != nullptr;
}

}  // namespace flutter

#endif  //  !SLIMPELLER
