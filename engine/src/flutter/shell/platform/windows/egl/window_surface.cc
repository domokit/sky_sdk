// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/windows/egl/window_surface.h"

#include "flutter/fml/logging.h"
#include "flutter/shell/platform/windows/egl/egl.h"

namespace flutter {
namespace egl {

WindowSurface::WindowSurface(EGLDisplay display,
                             EGLContext context,
                             EGLSurface surface,
                             size_t width,
                             size_t height)
    : Surface(display, context, surface), width_(width), height_(height) {}

bool WindowSurface::SetVSyncEnabled(bool enabled) {
  FML_DCHECK(IsCurrent());

  if (::eglSwapInterval(display_, enabled ? 1 : 0) != EGL_TRUE) {
    WINDOWS_LOG_EGL_ERROR;
    return false;
  }

  vsync_enabled_ = enabled;
  return true;
}

size_t WindowSurface::width() const {
  return width_;
}

size_t WindowSurface::height() const {
  return height_;
}

bool WindowSurface::vsync_enabled() const {
  return vsync_enabled_;
}

}  // namespace egl
}  // namespace flutter
