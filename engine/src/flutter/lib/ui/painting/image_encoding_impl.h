// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_UI_PAINTING_IMAGE_ENCODING_IMPL_H_
#define FLUTTER_LIB_UI_PAINTING_IMAGE_ENCODING_IMPL_H_

#include "flutter/lib/ui/ui_dart_state.h"
#include "third_party/skia/include/core/SkCanvas.h"
#include "third_party/skia/include/core/SkImage.h"
#include "third_party/skia/include/core/SkSurface.h"
#include "third_party/skia/include/gpu/ganesh/GrDirectContext.h"
#include "third_party/skia/include/gpu/ganesh/SkSurfaceGanesh.h"

namespace flutter {

template <typename SyncSwitch>
sk_sp<SkImage> ConvertToRasterUsingResourceContext(
    const sk_sp<SkImage>& image,
    const fml::WeakPtr<GrDirectContext>& resource_context,
    const std::shared_ptr<const SyncSwitch>& is_gpu_disabled_sync_switch) {
  sk_sp<SkSurface> surface;
  SkImageInfo surface_info = SkImageInfo::MakeN32Premul(image->dimensions());

  is_gpu_disabled_sync_switch->Execute(
      typename SyncSwitch::Handlers()
          .SetIfTrue([&surface, &surface_info] {
            surface = SkSurfaces::Raster(surface_info);
          })
          .SetIfFalse([&surface, &surface_info, resource_context] {
            if (resource_context) {
              surface = SkSurfaces::RenderTarget(
                  resource_context.get(), skgpu::Budgeted::kNo, surface_info);
            } else {
              surface = SkSurfaces::Raster(surface_info);
            }
          }));

  if (surface == nullptr || surface->getCanvas() == nullptr) {
    FML_LOG(ERROR) << "Could not create a surface to copy the texture into.";
    return nullptr;
  }

  surface->getCanvas()->drawImage(image, 0, 0);
  if (resource_context) {
    resource_context->flushAndSubmit();
  }

  auto snapshot = surface->makeImageSnapshot();

  if (snapshot == nullptr) {
    FML_LOG(ERROR) << "Could not snapshot image to encode.";
    return nullptr;
  }

  return snapshot->makeRasterImage();
}

}  // namespace flutter

#endif  // FLUTTER_LIB_UI_PAINTING_IMAGE_ENCODING_IMPL_H_
