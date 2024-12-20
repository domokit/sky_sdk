// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/display_list/testing/dl_test_surface_gl.h"

#include "third_party/skia/include/core/SkCanvas.h"
#include "third_party/skia/include/core/SkSurface.h"
#include "third_party/skia/include/gpu/ganesh/SkSurfaceGanesh.h"

namespace flutter {
namespace testing {

using PixelFormat = DlSurfaceProvider::PixelFormat;

bool DlOpenGLSurfaceProvider::InitializeSurface(size_t width,
                                                size_t height,
                                                PixelFormat format) {
  gl_surface_ = std::make_unique<TestGLSurface>(SkISize::Make(width, height));
  gl_surface_->MakeCurrent();

  primary_ = MakeOffscreenSurface(width, height, format);
  return true;
}

std::shared_ptr<DlSurfaceInstance> DlOpenGLSurfaceProvider::GetPrimarySurface()
    const {
  if (!gl_surface_->MakeCurrent()) {
    return nullptr;
  }
  return primary_;
}

std::shared_ptr<DlSurfaceInstance>
DlOpenGLSurfaceProvider::MakeOffscreenSurface(size_t width,
                                              size_t height,
                                              PixelFormat format) const {
  auto offscreen_surface = SkSurfaces::RenderTarget(
      (GrRecordingContext*)gl_surface_->GetGrContext().get(),
      skgpu::Budgeted::kNo, MakeInfo(format, width, height), 1,
      kTopLeft_GrSurfaceOrigin, nullptr, false);

  offscreen_surface->getCanvas()->clear(SK_ColorTRANSPARENT);
  return std::make_shared<DlSurfaceInstanceBase>(offscreen_surface);
}

}  // namespace testing
}  // namespace flutter
