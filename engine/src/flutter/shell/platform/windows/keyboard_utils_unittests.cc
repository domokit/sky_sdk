// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.
#include "flutter/shell/platform/windows/keyboard_utils.h"

#include "gtest/gtest.h"

namespace flutter {
namespace testing {

TEST(KeyboardWin32CommonTest, EncodeUtf16) {
  std::u16string result;

  result = EncodeUtf16(0x24);
  EXPECT_EQ(result.size(), 1);
  EXPECT_EQ(result[0], 0x24);

  result = EncodeUtf16(0x20AC);
  EXPECT_EQ(result.size(), 1);
  EXPECT_EQ(result[0], 0x20AC);

  result = EncodeUtf16(0x10437);
  EXPECT_EQ(result.size(), 2);
  EXPECT_EQ(result[0], 0xD801);
  EXPECT_EQ(result[1], 0xDC37);

  result = EncodeUtf16(0x24B62);
  EXPECT_EQ(result.size(), 2);
  EXPECT_EQ(result[0], 0xD852);
  EXPECT_EQ(result[1], 0xDF62);
}

}  // namespace testing
}  // namespace flutter
