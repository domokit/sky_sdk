// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/android/android_egl_surface.h"

#include <EGL/eglext.h>
#include <sys/system_properties.h>

#include <array>
#include <list>

#include "flutter/fml/trace_event.h"

namespace flutter {

void LogLastEGLError() {
  struct EGLNameErrorPair {
    const char* name;
    EGLint code;
  };

#define _EGL_ERROR_DESC(a) \
  { #a, a }

  const EGLNameErrorPair pairs[] = {
      _EGL_ERROR_DESC(EGL_SUCCESS),
      _EGL_ERROR_DESC(EGL_NOT_INITIALIZED),
      _EGL_ERROR_DESC(EGL_BAD_ACCESS),
      _EGL_ERROR_DESC(EGL_BAD_ALLOC),
      _EGL_ERROR_DESC(EGL_BAD_ATTRIBUTE),
      _EGL_ERROR_DESC(EGL_BAD_CONTEXT),
      _EGL_ERROR_DESC(EGL_BAD_CONFIG),
      _EGL_ERROR_DESC(EGL_BAD_CURRENT_SURFACE),
      _EGL_ERROR_DESC(EGL_BAD_DISPLAY),
      _EGL_ERROR_DESC(EGL_BAD_SURFACE),
      _EGL_ERROR_DESC(EGL_BAD_MATCH),
      _EGL_ERROR_DESC(EGL_BAD_PARAMETER),
      _EGL_ERROR_DESC(EGL_BAD_NATIVE_PIXMAP),
      _EGL_ERROR_DESC(EGL_BAD_NATIVE_WINDOW),
      _EGL_ERROR_DESC(EGL_CONTEXT_LOST),
  };

#undef _EGL_ERROR_DESC

  const auto count = sizeof(pairs) / sizeof(EGLNameErrorPair);

  EGLint last_error = eglGetError();

  for (size_t i = 0; i < count; i++) {
    if (last_error == pairs[i].code) {
      FML_LOG(ERROR) << "EGL Error: " << pairs[i].name << " (" << pairs[i].code
                     << ")";
      return;
    }
  }

  FML_LOG(ERROR) << "Unknown EGL Error";
}

class AndroidEGLSurfaceDamage {
 public:
  void init(EGLDisplay display, EGLContext context) {}

  void SetDamageRegion(EGLDisplay display,
                       EGLSurface surface,
                       const std::optional<SkIRect>& region) {}

  /// This was disabled after discussion in
  /// https://github.com/flutter/flutter/issues/123353
  bool SupportsPartialRepaint() const { return false; }

  std::optional<SkIRect> InitialDamage(EGLDisplay display, EGLSurface surface) {
    return std::nullopt;
  }

  bool SwapBuffersWithDamage(EGLDisplay display,
                             EGLSurface surface,
                             const std::optional<SkIRect>& damage) {
    return eglSwapBuffers(display, surface);
  }
};

AndroidEGLSurface::AndroidEGLSurface(EGLSurface surface,
                                     EGLDisplay display,
                                     EGLContext context)
    : surface_(surface),
      display_(display),
      context_(context),
      damage_(std::make_unique<AndroidEGLSurfaceDamage>()) {
  damage_->init(display_, context);
}

AndroidEGLSurface::~AndroidEGLSurface() {
  [[maybe_unused]] auto result = eglDestroySurface(display_, surface_);
  FML_DCHECK(result == EGL_TRUE);
}

bool AndroidEGLSurface::IsValid() const {
  return surface_ != EGL_NO_SURFACE;
}

bool AndroidEGLSurface::IsContextCurrent() const {
  EGLContext current_egl_context = eglGetCurrentContext();
  if (context_ != current_egl_context) {
    return false;
  }

  EGLDisplay current_egl_display = eglGetCurrentDisplay();
  if (display_ != current_egl_display) {
    return false;
  }

  EGLSurface draw_surface = eglGetCurrentSurface(EGL_DRAW);
  if (draw_surface != surface_) {
    return false;
  }

  EGLSurface read_surface = eglGetCurrentSurface(EGL_READ);
  if (read_surface != surface_) {
    return false;
  }

  return true;
}

AndroidEGLSurfaceMakeCurrentStatus AndroidEGLSurface::MakeCurrent() const {
  if (IsContextCurrent()) {
    return AndroidEGLSurfaceMakeCurrentStatus::kSuccessAlreadyCurrent;
  }
  if (eglMakeCurrent(display_, surface_, surface_, context_) != EGL_TRUE) {
    FML_LOG(ERROR) << "Could not make the context current";
    LogLastEGLError();
    return AndroidEGLSurfaceMakeCurrentStatus::kFailure;
  }
  return AndroidEGLSurfaceMakeCurrentStatus::kSuccessMadeCurrent;
}

void AndroidEGLSurface::SetDamageRegion(
    const std::optional<SkIRect>& buffer_damage) {
  damage_->SetDamageRegion(display_, surface_, buffer_damage);
}

bool AndroidEGLSurface::SetPresentationTime(
    const fml::TimePoint& presentation_time) {
  if (presentation_time_proc_) {
    const auto time_ns = presentation_time.ToEpochDelta().ToNanoseconds();
    return presentation_time_proc_(display_, surface_, time_ns);
  } else {
    return false;
  }
}

bool AndroidEGLSurface::SwapBuffers(
    const std::optional<SkIRect>& surface_damage) {
  TRACE_EVENT0("flutter", "AndroidContextGL::SwapBuffers");
  return damage_->SwapBuffersWithDamage(display_, surface_, surface_damage);
}

bool AndroidEGLSurface::SupportsPartialRepaint() const {
  return damage_->SupportsPartialRepaint();
}

std::optional<SkIRect> AndroidEGLSurface::InitialDamage() {
  return damage_->InitialDamage(display_, surface_);
}

SkISize AndroidEGLSurface::GetSize() const {
  EGLint width = 0;
  EGLint height = 0;

  if (!eglQuerySurface(display_, surface_, EGL_WIDTH, &width) ||
      !eglQuerySurface(display_, surface_, EGL_HEIGHT, &height)) {
    FML_LOG(ERROR) << "Unable to query EGL surface size";
    LogLastEGLError();
    return SkISize::Make(0, 0);
  }
  return SkISize::Make(width, height);
}

}  // namespace flutter
