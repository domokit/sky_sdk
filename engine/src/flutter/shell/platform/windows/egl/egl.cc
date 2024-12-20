// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/windows/egl/egl.h"

#include <EGL/egl.h>

#include "flutter/fml/logging.h"

namespace flutter {
namespace egl {

namespace {

const char* EGLErrorToString(EGLint error) {
  switch (error) {
    case EGL_SUCCESS:
      return "Success";
    case EGL_NOT_INITIALIZED:
      return "Not Initialized";
    case EGL_BAD_ACCESS:
      return "Bad Access";
    case EGL_BAD_ALLOC:
      return "Bad Alloc";
    case EGL_BAD_ATTRIBUTE:
      return "Bad Attribute";
    case EGL_BAD_CONTEXT:
      return "Bad Context";
    case EGL_BAD_CONFIG:
      return "Bad Config";
    case EGL_BAD_CURRENT_SURFACE:
      return "Bad Current Surface";
    case EGL_BAD_DISPLAY:
      return "Bad Display";
    case EGL_BAD_SURFACE:
      return "Bad Surface";
    case EGL_BAD_MATCH:
      return "Bad Match";
    case EGL_BAD_PARAMETER:
      return "Bad Parameter";
    case EGL_BAD_NATIVE_PIXMAP:
      return "Bad Native Pixmap";
    case EGL_BAD_NATIVE_WINDOW:
      return "Bad Native Window";
    case EGL_CONTEXT_LOST:
      return "Context Lost";
  }
  FML_UNREACHABLE();
  return "Unknown";
}

}  // namespace

void LogEGLError(std::string_view message) {
  const EGLint error = ::eglGetError();
  return FML_LOG(ERROR) << "EGL Error: " << EGLErrorToString(error) << " ("
                        << error << ") " << message;
}

void LogEGLError(std::string_view file, int line) {
  std::stringstream stream;
  stream << "in " << file << ":" << line;
  LogEGLError(stream.str());
}

}  // namespace egl
}  // namespace flutter
