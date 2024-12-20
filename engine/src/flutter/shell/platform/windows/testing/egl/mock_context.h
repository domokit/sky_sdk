// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_WINDOWS_TESTING_EGL_MOCK_CONTEXT_H_
#define FLUTTER_SHELL_PLATFORM_WINDOWS_TESTING_EGL_MOCK_CONTEXT_H_

#include "flutter/fml/macros.h"
#include "flutter/shell/platform/windows/egl/context.h"
#include "gmock/gmock.h"

namespace flutter {
namespace testing {
namespace egl {

/// Mock for the |Context| base class.
class MockContext : public flutter::egl::Context {
 public:
  MockContext() : Context(EGL_NO_DISPLAY, EGL_NO_CONTEXT) {}

  MOCK_METHOD(bool, MakeCurrent, (), (const, override));
  MOCK_METHOD(bool, ClearCurrent, (), (const, override));

 private:
  FML_DISALLOW_COPY_AND_ASSIGN(MockContext);
};

}  // namespace egl
}  // namespace testing
}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_WINDOWS_TESTING_EGL_MOCK_CONTEXT_H_
