// Copyright (c) 2011 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "base/win/display.h"

#include "gtest/gtest.h"

namespace base {
namespace win {

TEST(Display, ScaleFactorToFloat) {
  EXPECT_EQ(ScaleFactorToFloat(SCALE_100_PERCENT), 1.00f);
  EXPECT_EQ(ScaleFactorToFloat(SCALE_120_PERCENT), 1.20f);
  EXPECT_EQ(ScaleFactorToFloat(SCALE_125_PERCENT), 1.25f);
  EXPECT_EQ(ScaleFactorToFloat(SCALE_140_PERCENT), 1.40f);
  EXPECT_EQ(ScaleFactorToFloat(SCALE_150_PERCENT), 1.50f);
  EXPECT_EQ(ScaleFactorToFloat(SCALE_160_PERCENT), 1.60f);
  EXPECT_EQ(ScaleFactorToFloat(SCALE_175_PERCENT), 1.75f);
  EXPECT_EQ(ScaleFactorToFloat(SCALE_180_PERCENT), 1.80f);
  EXPECT_EQ(ScaleFactorToFloat(SCALE_200_PERCENT), 2.00f);
  EXPECT_EQ(ScaleFactorToFloat(SCALE_225_PERCENT), 2.25f);
  EXPECT_EQ(ScaleFactorToFloat(SCALE_250_PERCENT), 2.50f);
  EXPECT_EQ(ScaleFactorToFloat(SCALE_350_PERCENT), 3.50f);
  EXPECT_EQ(ScaleFactorToFloat(SCALE_400_PERCENT), 4.00f);
  EXPECT_EQ(ScaleFactorToFloat(SCALE_450_PERCENT), 4.50f);
  EXPECT_EQ(ScaleFactorToFloat(SCALE_500_PERCENT), 5.00f);
  EXPECT_EQ(ScaleFactorToFloat(DEVICE_SCALE_FACTOR_INVALID), 1.0f);
}

}  // namespace win
}  // namespace base
