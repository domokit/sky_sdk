// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/gpu/gpu_surface_software.h"

#include <memory>

#include "flow/surface_frame.h"
#include "flutter/fml/logging.h"

#include "third_party/skia/include/core/SkSurface.h"

namespace flutter {

GPUSurfaceSoftware::GPUSurfaceSoftware(GPUSurfaceSoftwareDelegate* delegate,
                                       bool render_to_surface)
    : delegate_(delegate),
      render_to_surface_(render_to_surface),
      weak_factory_(this) {}

GPUSurfaceSoftware::~GPUSurfaceSoftware() = default;

// |Surface|
bool GPUSurfaceSoftware::IsValid() {
  return delegate_ != nullptr;
}

// |Surface|
std::unique_ptr<SurfaceFrame> GPUSurfaceSoftware::AcquireFrame(
    const SkISize& logical_size) {
  SurfaceFrame::FramebufferInfo framebuffer_info;
  framebuffer_info.supports_readback = true;

  // TODO(38466): Refactor GPU surface APIs take into account the fact that an
  // external view embedder may want to render to the root surface.
  if (!render_to_surface_) {
    return std::make_unique<SurfaceFrame>(
        nullptr, framebuffer_info,
        [](const SurfaceFrame& surface_frame, DlCanvas* canvas) {
          return true;
        },
        [](const SurfaceFrame& surface_frame) { return true; }, logical_size);
  }

  if (!IsValid()) {
    return nullptr;
  }

  const auto size = SkISize::Make(logical_size.width(), logical_size.height());

  sk_sp<SkSurface> backing_store = delegate_->AcquireBackingStore(size);

  if (backing_store == nullptr) {
    return nullptr;
  }

  if (size != SkISize::Make(backing_store->width(), backing_store->height())) {
    return nullptr;
  }

  // If the surface has been scaled, we need to apply the inverse scaling to the
  // underlying canvas so that coordinates are mapped to the same spot
  // irrespective of surface scaling.
  SkCanvas* canvas = backing_store->getCanvas();
  canvas->resetMatrix();

  SurfaceFrame::EncodeCallback encode_callback =
      [self = weak_factory_.GetWeakPtr()](const SurfaceFrame& surface_frame,
                                          DlCanvas* canvas) -> bool {
    // If the surface itself went away, there is nothing more to do.
    if (!self || !self->IsValid() || canvas == nullptr) {
      return false;
    }

    canvas->Flush();
    return true;
  };
  SurfaceFrame::SubmitCallback submit_callback =
      [self = weak_factory_.GetWeakPtr()](const SurfaceFrame& surface_frame) {
        // If the surface itself went away, there is nothing more to do.
        if (!self || !self->IsValid()) {
          return false;
        }
        return self->delegate_->PresentBackingStore(
            surface_frame.SkiaSurface());
      };

  return std::make_unique<SurfaceFrame>(backing_store, framebuffer_info,
                                        encode_callback, submit_callback,
                                        logical_size);
}

// |Surface|
SkMatrix GPUSurfaceSoftware::GetRootTransformation() const {
  // This backend does not currently support root surface transformations. Just
  // return identity.
  SkMatrix matrix;
  matrix.reset();
  return matrix;
}

// |Surface|
GrDirectContext* GPUSurfaceSoftware::GetContext() {
  // There is no GrContext associated with a software surface.
  return nullptr;
}

}  // namespace flutter
