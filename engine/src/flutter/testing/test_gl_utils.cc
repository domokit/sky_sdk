// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/testing/test_gl_utils.h"

#include <EGL/egl.h>

#include <sstream>

namespace flutter::testing {

std::string GetEGLError() {
  std::stringstream stream;

  auto error = ::eglGetError();

  stream << "EGL Result: '";

  switch (error) {
    case EGL_SUCCESS:
      stream << "EGL_SUCCESS";
      break;
    case EGL_NOT_INITIALIZED:
      stream << "EGL_NOT_INITIALIZED";
      break;
    case EGL_BAD_ACCESS:
      stream << "EGL_BAD_ACCESS";
      break;
    case EGL_BAD_ALLOC:
      stream << "EGL_BAD_ALLOC";
      break;
    case EGL_BAD_ATTRIBUTE:
      stream << "EGL_BAD_ATTRIBUTE";
      break;
    case EGL_BAD_CONTEXT:
      stream << "EGL_BAD_CONTEXT";
      break;
    case EGL_BAD_CONFIG:
      stream << "EGL_BAD_CONFIG";
      break;
    case EGL_BAD_CURRENT_SURFACE:
      stream << "EGL_BAD_CURRENT_SURFACE";
      break;
    case EGL_BAD_DISPLAY:
      stream << "EGL_BAD_DISPLAY";
      break;
    case EGL_BAD_SURFACE:
      stream << "EGL_BAD_SURFACE";
      break;
    case EGL_BAD_MATCH:
      stream << "EGL_BAD_MATCH";
      break;
    case EGL_BAD_PARAMETER:
      stream << "EGL_BAD_PARAMETER";
      break;
    case EGL_BAD_NATIVE_PIXMAP:
      stream << "EGL_BAD_NATIVE_PIXMAP";
      break;
    case EGL_BAD_NATIVE_WINDOW:
      stream << "EGL_BAD_NATIVE_WINDOW";
      break;
    case EGL_CONTEXT_LOST:
      stream << "EGL_CONTEXT_LOST";
      break;
    default:
      stream << "Unknown";
  }

  stream << "' (0x" << std::hex << error << std::dec << ").";
  return stream.str();
}

}  // namespace flutter::testing
