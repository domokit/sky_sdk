// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/toolkit/interop/backend/gles/surface_gles.h"

#include "impeller/renderer/backend/gles/surface_gles.h"

namespace impeller::interop {

SurfaceGLES::SurfaceGLES(Context& context,
                         uint64_t fbo,
                         PixelFormat color_format,
                         ISize size)
    : SurfaceGLES(context,
                  impeller::SurfaceGLES::WrapFBO(
                      context.GetContext(),
                      []() { return true; },
                      fbo,
                      color_format,
                      size)) {}

SurfaceGLES::SurfaceGLES(Context& context,
                         std::shared_ptr<impeller::Surface> surface)
    : Surface(context, std::move(surface)) {}

SurfaceGLES::~SurfaceGLES() = default;

}  // namespace impeller::interop
