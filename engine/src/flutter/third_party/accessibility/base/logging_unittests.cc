// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "gtest/gtest.h"
#include "logging.h"

namespace base {
namespace testing {

int UnreachableScopeWithoutReturnDoesNotMakeCompilerMad() {
  KillProcess();
  // return 0; <--- Missing but compiler is fine.
}

int UnreachableScopeWithMacroWithoutReturnDoesNotMakeCompilerMad() {
  BASE_UNREACHABLE();
  // return 0; <--- Missing but compiler is fine.
}

TEST(LoggingTest, UnreachableKillProcess) {
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  ASSERT_DEATH(KillProcess(), "");
}

TEST(LoggingTest, UnreachableKillProcessWithMacro) {
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  ASSERT_DEATH({ BASE_UNREACHABLE(); }, "");
}

}  // namespace testing
}  // namespace base
