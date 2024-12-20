// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_WINDOWS_TESTING_WINDOWS_TEST_H_
#define FLUTTER_SHELL_PLATFORM_WINDOWS_TESTING_WINDOWS_TEST_H_

#include <string>

#include "flutter/fml/macros.h"
#include "flutter/shell/platform/windows/testing/windows_test_context.h"
#include "flutter/testing/thread_test.h"

namespace flutter {
namespace testing {

/// A GoogleTest test fixture for Windows tests.
///
/// Supports looking up the test fixture data defined in the GN `test_fixtures`
/// associated with the unit test executable target. This typically includes
/// the kernel bytecode `kernel_blob.bin` compiled from the Dart file specified
/// in the test fixture's `dart_main` property, as well as any other data files
/// used in tests, such as image files used in a screenshot golden test.
///
/// This test class can be used in GoogleTest tests using the standard
/// `TEST_F(WindowsTest, TestName)` macro.
class WindowsTest : public ThreadTest {
 public:
  WindowsTest();

  // Returns the path to test fixture data such as kernel bytecode or images
  // used by the C++ side of the test.
  std::string GetFixturesDirectory() const;

  // Returns the test context associated with this fixture.
  WindowsTestContext& GetContext();

 private:
  WindowsTestContext context_;

  FML_DISALLOW_COPY_AND_ASSIGN(WindowsTest);
};

}  // namespace testing
}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_WINDOWS_TESTING_WINDOWS_TEST_H_
