// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/fml/hex_codec.h"

#include <iostream>

#include "gtest/gtest.h"

TEST(HexCodecTest, CanEncode) {
  {
    auto result = fml::HexEncode("hello");
    ASSERT_EQ(result, "68656c6c6f");
  }

  {
    auto result = fml::HexEncode("");
    ASSERT_EQ(result, "");
  }

  {
    auto result = fml::HexEncode("1");
    ASSERT_EQ(result, "31");
  }

  {
    auto result = fml::HexEncode(std::string_view("\xFF\xFE\x00\x01", 4));
    ASSERT_EQ(result, "fffe0001");
  }
}
