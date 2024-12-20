// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <cmath>

#include "flutter/fml/math.h"
#include "gtest/gtest.h"

namespace flutter {
namespace testing {

TEST(MathTest, Constants) {
  // Don't use the constants in cmath as those aren't portable.
  EXPECT_FLOAT_EQ(std::log2(math::kE), math::kLog2E);
  EXPECT_FLOAT_EQ(std::log10(math::kE), math::kLog10E);
  EXPECT_FLOAT_EQ(std::log(2.0f), math::kLogE2);
  EXPECT_FLOAT_EQ(math::kPi / 2.0f, math::kPiOver2);
  EXPECT_FLOAT_EQ(math::kPi / 4.0f, math::kPiOver4);
  EXPECT_FLOAT_EQ(1.0f / math::kPi, math::k1OverPi);
  EXPECT_FLOAT_EQ(2.0f / math::kPi, math::k2OverPi);
  EXPECT_FLOAT_EQ(std::sqrt(2.0f), math::kSqrt2);
  EXPECT_FLOAT_EQ(1.0f / std::sqrt(2.0f), math::k1OverSqrt2);
}

}  // namespace testing
}  // namespace flutter
