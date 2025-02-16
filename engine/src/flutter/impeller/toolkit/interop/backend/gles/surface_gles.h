// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_TOOLKIT_INTEROP_BACKEND_GLES_SURFACE_GLES_H_
#define FLUTTER_IMPELLER_TOOLKIT_INTEROP_BACKEND_GLES_SURFACE_GLES_H_

#include "impeller/toolkit/interop/surface.h"

namespace impeller::interop {

class SurfaceGLES final : public Surface {
 public:
  SurfaceGLES(Context& context,
              uint64_t fbo,
              PixelFormat color_format,
              ISize size);

  SurfaceGLES(Context& context, std::shared_ptr<impeller::Surface> surface);

  // |Surface|
  ~SurfaceGLES();

  SurfaceGLES(const SurfaceGLES&) = delete;

  SurfaceGLES& operator=(const SurfaceGLES&) = delete;
};

}  // namespace impeller::interop

#endif  // FLUTTER_IMPELLER_TOOLKIT_INTEROP_BACKEND_GLES_SURFACE_GLES_H_
