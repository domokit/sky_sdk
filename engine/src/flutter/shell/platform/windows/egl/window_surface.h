// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_WINDOWS_EGL_WINDOW_SURFACE_H_
#define FLUTTER_SHELL_PLATFORM_WINDOWS_EGL_WINDOW_SURFACE_H_

#include <EGL/egl.h>

#include "flutter/fml/macros.h"
#include "flutter/shell/platform/windows/egl/surface.h"

namespace flutter {
namespace egl {

// An EGL surface used to render a Flutter view to a win32 HWND.
//
// This enables automatic error logging and mocking.
class WindowSurface : public Surface {
 public:
  WindowSurface(EGLDisplay display,
                EGLContext context,
                EGLSurface surface,
                size_t width,
                size_t height);

  // If enabled, makes the surface's buffer swaps block until the v-blank.
  //
  // If disabled, allows one thread to swap multiple buffers per v-blank
  // but can result in screen tearing if the system compositor is disabled.
  //
  // The surface must be current before calling this.
  virtual bool SetVSyncEnabled(bool enabled);

  // Get the surface's width in physical pixels.
  virtual size_t width() const;

  // Get the surface's height in physical pixels.
  virtual size_t height() const;

  // Get whether the surface's buffer swap blocks until the v-blank.
  virtual bool vsync_enabled() const;

 private:
  size_t width_ = 0;
  size_t height_ = 0;
  bool vsync_enabled_ = true;

  FML_DISALLOW_COPY_AND_ASSIGN(WindowSurface);
};

}  // namespace egl
}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_WINDOWS_EGL_WINDOW_SURFACE_H_
