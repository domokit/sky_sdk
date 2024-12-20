// Copyright 2022 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "dart_runner/dart_test_component_controller.h"

#include "gtest/gtest.h"

namespace dart_runner::testing {
namespace {

std::string GetCurrentTestName() {
  return ::testing::UnitTest::GetInstance()->current_test_info()->name();
}

}  // namespace

// TODO(naudzghebre): Add unit tests for the dart_test_runner.
TEST(SuiteImplTest, EQUALITY) {
  EXPECT_EQ(1, 1) << GetCurrentTestName();
}

}  // namespace dart_runner::testing
