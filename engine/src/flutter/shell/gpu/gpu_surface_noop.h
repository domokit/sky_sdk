// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_GPU_GPU_SURFACE_NOOP_H_
#define FLUTTER_SHELL_GPU_GPU_SURFACE_NOOP_H_

#include <Metal/Metal.h>

#include "flutter/flow/surface.h"

namespace flutter {

/// @brief A rendering surface that accepts rendering intent but does not render
///        anything.
///
/// This is useful for running on platforms that need an engine instance and
/// don't have the required drivers.
class GPUSurfaceNoop : public Surface {
 public:
  explicit GPUSurfaceNoop();

  // |Surface|
  ~GPUSurfaceNoop();

  // |Surface|
  bool IsValid() override;

  // |Surface|
  Surface::SurfaceData GetSurfaceData() const override;

 private:
  // |Surface|
  std::unique_ptr<SurfaceFrame> AcquireFrame(
      const SkISize& frame_size) override;

  std::unique_ptr<SurfaceFrame> AcquireFrameFromCAMetalLayer(
      const SkISize& frame_size);

  std::unique_ptr<SurfaceFrame> AcquireFrameFromMTLTexture(
      const SkISize& frame_size);

  // |Surface|
  SkMatrix GetRootTransformation() const override;

  // |Surface|
  GrDirectContext* GetContext() override;

  // |Surface|
  std::unique_ptr<GLContextResult> MakeRenderContextCurrent() override;

  // |Surface|
  bool AllowsDrawingWhenGpuDisabled() const override;

  // |Surface|
  bool EnableRasterCache() const override;

  // |Surface|
  std::shared_ptr<impeller::AiksContext> GetAiksContext() const override;

  GPUSurfaceNoop(const GPUSurfaceNoop&) = delete;

  GPUSurfaceNoop& operator=(const GPUSurfaceNoop&) = delete;
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_GPU_GPU_SURFACE_NOOP_H_
