// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_TOOLKIT_EGL_SURFACE_H_
#define FLUTTER_IMPELLER_TOOLKIT_EGL_SURFACE_H_

#include "impeller/toolkit/egl/egl.h"

namespace impeller {
namespace egl {

//------------------------------------------------------------------------------
/// @brief      An instance of an EGL surface. There is no ability to create
///             surfaces directly. Instead, one must be created using a Display
///             connection.
///
class Surface {
 public:
  ~Surface();

  //----------------------------------------------------------------------------
  /// @return     True if this is a valid surface.
  ///
  bool IsValid() const;

  //----------------------------------------------------------------------------
  /// @return     Get the handle to the underlying surface.
  ///
  const EGLSurface& GetHandle() const;

  //----------------------------------------------------------------------------
  /// @brief      Present the surface. For an offscreen pixel buffer surface,
  ///             this is a no-op.
  ///
  /// @return     True if the surface could be presented.
  ///
  bool Present() const;

 private:
  friend class Display;

  EGLDisplay display_ = EGL_NO_DISPLAY;
  EGLSurface surface_ = EGL_NO_SURFACE;

  Surface(EGLDisplay display, EGLSurface surface);

  Surface(const Surface&) = delete;

  Surface& operator=(const Surface&) = delete;
};

}  // namespace egl
}  // namespace impeller

#endif  // FLUTTER_IMPELLER_TOOLKIT_EGL_SURFACE_H_
