// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/gpu/gpu_surface_noop.h"

#import <Metal/Metal.h>
#import <QuartzCore/QuartzCore.h>

#include "flow/surface.h"
#include "flow/surface_frame.h"
#include "flutter/common/settings.h"
#include "flutter/fml/mapping.h"
#include "flutter/fml/trace_event.h"

static_assert(__has_feature(objc_arc), "ARC must be enabled.");

namespace flutter {

GPUSurfaceNoop::GPUSurfaceNoop() = default;

GPUSurfaceNoop::~GPUSurfaceNoop() = default;

// |Surface|
bool GPUSurfaceNoop::IsValid() {
  return true;
}

Surface::SurfaceData GPUSurfaceNoop::GetSurfaceData() const {
  return Surface::SurfaceData{};
}

// |Surface|
std::unique_ptr<SurfaceFrame> GPUSurfaceNoop::AcquireFrame(const SkISize& frame_size) {
  auto callback = [](const SurfaceFrame&, DlCanvas*) { return true; };
  auto submit_callback = [](const SurfaceFrame&) { return true; };
  SurfaceFrame::FramebufferInfo framebuffer_info;

  return std::make_unique<SurfaceFrame>(
      /*surface=*/nullptr,
      /*framebuffer_info=*/framebuffer_info,
      /*encode_callback=*/callback,
      /*submit_callback=*/submit_callback,
      /*frame_size=*/frame_size,
      /*context_result=*/nullptr,
      /*display_list_fallback=*/true);
}

std::unique_ptr<SurfaceFrame> GPUSurfaceNoop::AcquireFrameFromMTLTexture(
    const SkISize& frame_size) {
  auto callback = [](const SurfaceFrame&, DlCanvas*) { return true; };
  auto submit_callback = [](const SurfaceFrame&) { return true; };
  SurfaceFrame::FramebufferInfo framebuffer_info;

  return std::make_unique<SurfaceFrame>(
      /*surface=*/nullptr,
      /*framebuffer_info=*/framebuffer_info,
      /*encode_callback=*/callback,
      /*submit_callback=*/submit_callback,
      /*frame_size=*/frame_size,
      /*context_result=*/nullptr,
      /*display_list_fallback=*/true);
}

// |Surface|
SkMatrix GPUSurfaceNoop::GetRootTransformation() const {
  // This backend does not currently support root surface transformations. Just
  // return identity.
  return {};
}

// |Surface|
GrDirectContext* GPUSurfaceNoop::GetContext() {
  return nullptr;
}

// |Surface|
std::unique_ptr<GLContextResult> GPUSurfaceNoop::MakeRenderContextCurrent() {
  // This backend has no such concept.
  return std::make_unique<GLContextDefaultResult>(true);
}

bool GPUSurfaceNoop::AllowsDrawingWhenGpuDisabled() const {
  return true;
}

// |Surface|
bool GPUSurfaceNoop::EnableRasterCache() const {
  return false;
}

// |Surface|
std::shared_ptr<impeller::AiksContext> GPUSurfaceNoop::GetAiksContext() const {
  return nullptr;
}

}  // namespace flutter
