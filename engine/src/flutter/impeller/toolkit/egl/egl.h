// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_TOOLKIT_EGL_EGL_H_
#define FLUTTER_IMPELLER_TOOLKIT_EGL_EGL_H_

#include <EGL/egl.h>
#define EGL_EGLEXT_PROTOTYPES
#include <EGL/eglext.h>

#include <functional>

namespace impeller {
namespace egl {

//------------------------------------------------------------------------------
/// @brief      Creates a proc address resolver that resolves function pointers
///             to EGL and OpenGL (ES) procs.
///
/// @return     The resolver if one can be created.
///
std::function<void*(const char*)> CreateProcAddressResolver();

#define IMPELLER_LOG_EGL_ERROR LogEGLError(__FILE__, __LINE__);

void LogEGLError(const char* file, int line);

}  // namespace egl
}  // namespace impeller

#endif  // FLUTTER_IMPELLER_TOOLKIT_EGL_EGL_H_
