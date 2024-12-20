// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/android/surface/android_surface_mock.h"

namespace flutter {

std::unique_ptr<GLContextResult> AndroidSurfaceMock::GLContextMakeCurrent() {
  return std::make_unique<GLContextDefaultResult>(/*static_result=*/true);
}

bool AndroidSurfaceMock::GLContextClearCurrent() {
  return true;
}

bool AndroidSurfaceMock::GLContextPresent(const GLPresentInfo& present_info) {
  return true;
}

GLFBOInfo AndroidSurfaceMock::GLContextFBO(GLFrameInfo frame_info) const {
  return GLFBOInfo{
      .fbo_id = 0,
  };
}

}  // namespace flutter
