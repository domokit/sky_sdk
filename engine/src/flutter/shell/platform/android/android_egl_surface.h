// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_ANDROID_ANDROID_EGL_SURFACE_H_
#define FLUTTER_SHELL_PLATFORM_ANDROID_ANDROID_EGL_SURFACE_H_

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <KHR/khrplatform.h>
#include <optional>

#include "flutter/fml/macros.h"
#include "flutter/fml/time/time_point.h"
#include "flutter/shell/platform/android/android_environment_gl.h"
#include "third_party/skia/include/core/SkRect.h"

namespace flutter {

//------------------------------------------------------------------------------
/// Holds an `EGLSurface` reference.
///
///
/// This can be used in conjunction to unique_ptr to provide better guarantees
/// about the lifespan of the `EGLSurface` object.
///
class AndroidEGLSurfaceDamage;

/// Result of calling MakeCurrent on AndroidEGLSurface.
enum class AndroidEGLSurfaceMakeCurrentStatus {
  /// Success, the egl context for the surface was already current.
  kSuccessAlreadyCurrent,
  /// Success, the egl context for the surface made current.
  kSuccessMadeCurrent,
  /// Failed to make the egl context for the surface current.
  kFailure,
};

void LogLastEGLError();

class AndroidEGLSurface {
 public:
  AndroidEGLSurface(EGLSurface surface, EGLDisplay display, EGLContext context);

  ~AndroidEGLSurface();

  //----------------------------------------------------------------------------
  /// @return     Whether the current `EGLSurface` reference is valid. That is,
  /// if
  ///             the surface doesn't point to `EGL_NO_SURFACE`.
  ///
  bool IsValid() const;

  //----------------------------------------------------------------------------
  /// @brief      Binds the EGLContext context to the current rendering thread
  ///             and to the draw and read surface.
  ///
  /// @return     Whether the surface was made current.
  ///
  AndroidEGLSurfaceMakeCurrentStatus MakeCurrent() const;

  //----------------------------------------------------------------------------
  ///
  /// @return     Whether target surface supports partial repaint.
  ///
  bool SupportsPartialRepaint() const;

  //----------------------------------------------------------------------------
  /// @brief      This is the minimal area that needs to be repainted to get
  ///             correct result.
  ///
  /// With double or triple buffering this buffer content may lag behind
  /// current front buffer and the rect accounts for accumulated damage.
  ///
  /// @return     The area of current surface where it is behind front buffer.
  ///
  std::optional<SkIRect> InitialDamage();

  //----------------------------------------------------------------------------
  /// @brief      Sets the damage region for current surface. Corresponds to
  //              eglSetDamageRegionKHR
  void SetDamageRegion(const std::optional<SkIRect>& buffer_damage);

  //----------------------------------------------------------------------------
  /// @brief      Sets the presentation time for the current surface. This
  //              corresponds to calling eglPresentationTimeAndroid when
  //              available.
  bool SetPresentationTime(const fml::TimePoint& presentation_time);

  //----------------------------------------------------------------------------
  /// @brief      This only applies to on-screen surfaces such as those created
  ///             by `AndroidContextGL::CreateOnscreenSurface`.
  ///
  /// @return     Whether the EGL surface color buffer was swapped.
  ///
  bool SwapBuffers(const std::optional<SkIRect>& surface_damage);

  //----------------------------------------------------------------------------
  /// @return     The size of an `EGLSurface`.
  ///
  SkISize GetSize() const;

 private:
  /// Returns true if the EGLContext held is current for the display and surface
  bool IsContextCurrent() const;

  const EGLSurface surface_;
  const EGLDisplay display_;
  const EGLContext context_;
  std::unique_ptr<AndroidEGLSurfaceDamage> damage_;
  PFNEGLPRESENTATIONTIMEANDROIDPROC presentation_time_proc_ = nullptr;
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_ANDROID_ANDROID_EGL_SURFACE_H_
