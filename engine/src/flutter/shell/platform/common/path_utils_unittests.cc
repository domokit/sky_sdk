// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/common/path_utils.h"

#include "gtest/gtest.h"

namespace flutter {

// Tests that GetExecutableDirectory returns a valid, absolute path.
TEST(PathUtilsTest, ExecutableDirector) {
  std::filesystem::path exe_directory = GetExecutableDirectory();
#if defined(__linux__) || defined(_WIN32)
  EXPECT_EQ(exe_directory.empty(), false);
  EXPECT_EQ(exe_directory.is_absolute(), true);
#else
  // On platforms where it's not implemented, it should indicate that
  // by returning an empty path.
  EXPECT_EQ(exe_directory.empty(), true);
#endif
}

}  // namespace flutter
