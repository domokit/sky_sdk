// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/display_list/testing/dl_test_surface_software.h"

#include "third_party/skia/include/core/SkCanvas.h"
#include "third_party/skia/include/core/SkSurface.h"

namespace flutter {
namespace testing {

using PixelFormat = DlSurfaceProvider::PixelFormat;

bool DlSoftwareSurfaceProvider::InitializeSurface(size_t width,
                                                  size_t height,
                                                  PixelFormat format) {
  primary_ = MakeOffscreenSurface(width, height, format);
  return primary_ != nullptr;
}

std::shared_ptr<DlSurfaceInstance>
DlSoftwareSurfaceProvider::MakeOffscreenSurface(size_t width,
                                                size_t height,
                                                PixelFormat format) const {
  auto surface = SkSurfaces::Raster(MakeInfo(format, width, height));
  surface->getCanvas()->clear(SK_ColorTRANSPARENT);
  return std::make_shared<DlSurfaceInstanceBase>(surface);
}

}  // namespace testing
}  // namespace flutter
