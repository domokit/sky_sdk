// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_TESTING_TEST_GL_CONTEXT_H_
#define FLUTTER_TESTING_TEST_GL_CONTEXT_H_

namespace flutter::testing {

struct TestEGLContext {
  explicit TestEGLContext();

  ~TestEGLContext();

  using EGLDisplay = void*;
  using EGLContext = void*;
  using EGLConfig = void*;

  EGLDisplay display;
  EGLContext onscreen_context;
  EGLContext offscreen_context;

  // EGLConfig is technically a property of the surfaces, no the context,
  // but it's not that well separated in EGL (e.g. when
  // EGL_KHR_no_config_context is not supported), so we just store it here.
  EGLConfig config;
};

}  // namespace flutter::testing

#endif  // FLUTTER_TESTING_TEST_GL_CONTEXT_H_
