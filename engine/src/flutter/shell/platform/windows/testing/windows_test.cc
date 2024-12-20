// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/windows/testing/windows_test.h"

#include <string>

#include "flutter/shell/platform/windows/testing/windows_test_context.h"
#include "flutter/testing/testing.h"

namespace flutter {
namespace testing {

WindowsTest::WindowsTest() : context_(GetFixturesDirectory()) {}

std::string WindowsTest::GetFixturesDirectory() const {
  return GetFixturesPath();
}

WindowsTestContext& WindowsTest::GetContext() {
  return context_;
}

}  // namespace testing
}  // namespace flutter
