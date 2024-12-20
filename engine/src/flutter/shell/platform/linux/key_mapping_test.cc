// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "key_mapping.h"
#include <gmodule.h>
#include <algorithm>
#include <vector>

#include "gtest/gtest.h"

bool operator==(const LayoutGoal& a, const LayoutGoal& b) {
  return a.keycode == b.keycode && a.logical_key == b.logical_key &&
         a.mandatory == b.mandatory;
}

// Spot check some expected values so that we know that some classes of key
// aren't excluded.
TEST(KeyMappingTest, HasExpectedValues) {
  // Has Space
  EXPECT_NE(std::find(layout_goals.begin(), layout_goals.end(),
                      LayoutGoal{0x41, 0x20, false}),
            layout_goals.end());
  // Has Digit0
  EXPECT_NE(std::find(layout_goals.begin(), layout_goals.end(),
                      LayoutGoal{0x13, 0x30, true}),
            layout_goals.end());
  // Has KeyA
  EXPECT_NE(std::find(layout_goals.begin(), layout_goals.end(),
                      LayoutGoal{0x26, 0x61, true}),
            layout_goals.end());
  // Has Equal
  EXPECT_NE(std::find(layout_goals.begin(), layout_goals.end(),
                      LayoutGoal{0x15, 0x3d, false}),
            layout_goals.end());
  // Has IntlBackslash
  EXPECT_NE(std::find(layout_goals.begin(), layout_goals.end(),
                      LayoutGoal{0x5e, 0x200000020, false}),
            layout_goals.end());
}
