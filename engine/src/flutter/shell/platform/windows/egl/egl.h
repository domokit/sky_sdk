// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_WINDOWS_EGL_EGL_H_
#define FLUTTER_SHELL_PLATFORM_WINDOWS_EGL_EGL_H_

#include <string_view>

namespace flutter {
namespace egl {

/// Log the last EGL error with an error message.
void LogEGLError(std::string_view message);

/// Log the last EGL error.
void LogEGLError(std::string_view file, int line);

#define WINDOWS_LOG_EGL_ERROR LogEGLError(__FILE__, __LINE__);

}  // namespace egl
}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_WINDOWS_EGL_EGL_H_
