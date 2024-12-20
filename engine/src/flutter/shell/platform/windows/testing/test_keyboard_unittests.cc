// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <windows.h>

#include "flutter/shell/platform/windows/testing/test_keyboard.h"
#include "gtest/gtest.h"

namespace flutter {
namespace testing {

TEST(TestKeyboard, CloneString) {
  const char* str1 = "123";
  char* cloned_str1 = clone_string(str1);
  EXPECT_STREQ(str1, cloned_str1);
  EXPECT_NE(str1, cloned_str1);
  delete[] cloned_str1;

  EXPECT_EQ(clone_string(nullptr), nullptr);
};

TEST(TestKeyboard, CreateKeyEventLparam) {
  EXPECT_EQ(CreateKeyEventLparam(0x1, true, true), 0xC1010001);

  EXPECT_EQ(CreateKeyEventLparam(0x05, false, false, 0, 1, 0), 0x20050000);
};

}  // namespace testing
}  // namespace flutter
