// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_WINDOWS_TESTING_EGL_MOCK_MANAGER_H_
#define FLUTTER_SHELL_PLATFORM_WINDOWS_TESTING_EGL_MOCK_MANAGER_H_

#include "flutter/fml/macros.h"
#include "flutter/shell/platform/windows/egl/manager.h"
#include "gmock/gmock.h"

namespace flutter {
namespace testing {
namespace egl {

/// Mock for the |Manager| base class.
class MockManager : public flutter::egl::Manager {
 public:
  MockManager() : Manager(flutter::egl::GpuPreference::NoPreference) {}

  MOCK_METHOD(std::unique_ptr<flutter::egl::WindowSurface>,
              CreateWindowSurface,
              (HWND, size_t, size_t),
              (override));

  MOCK_METHOD(flutter::egl::Context*, render_context, (), (const, override));
  MOCK_METHOD(flutter::egl::Context*, resource_context, (), (const, override));

 private:
  FML_DISALLOW_COPY_AND_ASSIGN(MockManager);
};

}  // namespace egl
}  // namespace testing
}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_WINDOWS_TESTING_EGL_MOCK_MANAGER_H_
