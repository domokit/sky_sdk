// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_WINDOWS_TESTING_EGL_MOCK_WINDOW_SURFACE_H_
#define FLUTTER_SHELL_PLATFORM_WINDOWS_TESTING_EGL_MOCK_WINDOW_SURFACE_H_

#include "flutter/fml/macros.h"
#include "flutter/shell/platform/windows/egl/window_surface.h"
#include "gmock/gmock.h"

namespace flutter {
namespace testing {
namespace egl {

/// Mock for the |WindowSurface| base class.
class MockWindowSurface : public flutter::egl::WindowSurface {
 public:
  MockWindowSurface()
      : WindowSurface(EGL_NO_DISPLAY, EGL_NO_CONTEXT, EGL_NO_SURFACE, 0, 0) {}

  MOCK_METHOD(bool, IsValid, (), (const, override));
  MOCK_METHOD(bool, Destroy, (), (override));
  MOCK_METHOD(bool, MakeCurrent, (), (const, override));
  MOCK_METHOD(bool, SwapBuffers, (), (const, override));
  MOCK_METHOD(bool, SetVSyncEnabled, (bool), (override));

 private:
  FML_DISALLOW_COPY_AND_ASSIGN(MockWindowSurface);
};

}  // namespace egl
}  // namespace testing
}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_WINDOWS_TESTING_EGL_MOCK_WINDOW_SURFACE_H_
