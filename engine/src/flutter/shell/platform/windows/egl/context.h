// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_WINDOWS_EGL_CONTEXT_H_
#define FLUTTER_SHELL_PLATFORM_WINDOWS_EGL_CONTEXT_H_

#include <EGL/egl.h>

#include "flutter/fml/macros.h"

namespace flutter {
namespace egl {

// An EGL context to interact with OpenGL.
//
// This enables automatic error logging and mocking.
//
// Flutter Windows uses this to create render and resource contexts.
class Context {
 public:
  Context(EGLDisplay display, EGLContext context);
  ~Context();

  // Check if this context is currently bound to the thread.
  virtual bool IsCurrent() const;

  // Bind the context to the thread without any read or draw surfaces.
  //
  // Returns true on success.
  virtual bool MakeCurrent() const;

  // Unbind any context and surfaces from the thread.
  //
  // Returns true on success.
  virtual bool ClearCurrent() const;

  // Get the raw EGL context.
  virtual const EGLContext& GetHandle() const;

 private:
  EGLDisplay display_ = EGL_NO_DISPLAY;
  EGLContext context_ = EGL_NO_CONTEXT;

  FML_DISALLOW_COPY_AND_ASSIGN(Context);
};

}  // namespace egl
}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_WINDOWS_EGL_CONTEXT_H_
