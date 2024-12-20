// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BACKEND_GLES_SURFACE_GLES_H_
#define FLUTTER_IMPELLER_RENDERER_BACKEND_GLES_SURFACE_GLES_H_

#include <functional>
#include <memory>

#include "impeller/renderer/backend/gles/gles.h"
#include "impeller/renderer/context.h"
#include "impeller/renderer/surface.h"

namespace impeller {

class SurfaceGLES final : public Surface {
 public:
  using SwapCallback = std::function<bool(void)>;

  static std::unique_ptr<Surface> WrapFBO(
      const std::shared_ptr<Context>& context,
      SwapCallback swap_callback,
      GLuint fbo,
      PixelFormat color_format,
      ISize fbo_size);

  // |Surface|
  ~SurfaceGLES() override;

 private:
  SwapCallback swap_callback_;

  SurfaceGLES(SwapCallback swap_callback, const RenderTarget& target_desc);

  // |Surface|
  bool Present() const override;

  SurfaceGLES(const SurfaceGLES&) = delete;

  SurfaceGLES& operator=(const SurfaceGLES&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_BACKEND_GLES_SURFACE_GLES_H_
