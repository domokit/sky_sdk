// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "KeyCodeMap_Internal.h"

#import "flutter/testing/testing.h"
#include "third_party/googletest/googletest/include/gtest/gtest.h"

namespace flutter {

bool operator==(const LayoutGoal& a, const LayoutGoal& b) {
  return a.keyCode == b.keyCode && a.keyChar == b.keyChar && a.mandatory == b.mandatory;
}

namespace testing {

// Spot check some expected values so that we know that some classes of key
// aren't excluded.
TEST(KeyMappingTest, HasExpectedValues) {
  // Has Space
  EXPECT_NE(std::find(kLayoutGoals.begin(), kLayoutGoals.end(), LayoutGoal{0x31, 0x20, false}),
            kLayoutGoals.end());
  // Has Digit0
  EXPECT_NE(std::find(kLayoutGoals.begin(), kLayoutGoals.end(), LayoutGoal{0x1d, 0x30, true}),
            kLayoutGoals.end());
  // Has KeyA
  EXPECT_NE(std::find(kLayoutGoals.begin(), kLayoutGoals.end(), LayoutGoal{0x00, 0x61, true}),
            kLayoutGoals.end());
  // Has Equal
  EXPECT_NE(std::find(kLayoutGoals.begin(), kLayoutGoals.end(), LayoutGoal{0x18, 0x3d, false}),
            kLayoutGoals.end());
  // Has IntlBackslash
  EXPECT_NE(
      std::find(kLayoutGoals.begin(), kLayoutGoals.end(), LayoutGoal{0x0a, 0x200000020, false}),
      kLayoutGoals.end());
}
}  // namespace testing
}  // namespace flutter
