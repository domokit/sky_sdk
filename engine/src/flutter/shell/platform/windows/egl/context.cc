// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/windows/egl/context.h"

#include "flutter/shell/platform/windows/egl/egl.h"

namespace flutter {
namespace egl {

Context::Context(EGLDisplay display, EGLContext context)
    : display_(display), context_(context) {}

Context::~Context() {
  if (display_ == EGL_NO_DISPLAY && context_ == EGL_NO_CONTEXT) {
    return;
  }

  if (::eglDestroyContext(display_, context_) != EGL_TRUE) {
    WINDOWS_LOG_EGL_ERROR;
  }
}

bool Context::IsCurrent() const {
  return ::eglGetCurrentContext() == context_;
}

bool Context::MakeCurrent() const {
  const auto result =
      ::eglMakeCurrent(display_, EGL_NO_SURFACE, EGL_NO_SURFACE, context_);
  if (result != EGL_TRUE) {
    WINDOWS_LOG_EGL_ERROR;
    return false;
  }

  return true;
}

bool Context::ClearCurrent() const {
  const auto result = ::eglMakeCurrent(display_, EGL_NO_SURFACE, EGL_NO_SURFACE,
                                       EGL_NO_CONTEXT);
  if (result != EGL_TRUE) {
    WINDOWS_LOG_EGL_ERROR;
    return false;
  }

  return true;
}

const EGLContext& Context::GetHandle() const {
  return context_;
}

}  // namespace egl
}  // namespace flutter
