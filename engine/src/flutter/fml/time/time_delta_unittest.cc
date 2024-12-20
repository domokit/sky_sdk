// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/fml/time/time_delta.h"

#include "gtest/gtest.h"

namespace fml {
namespace {

TEST(TimeDelta, Control) {
  EXPECT_LT(TimeDelta::Min(), TimeDelta::Zero());
  EXPECT_GT(TimeDelta::Max(), TimeDelta::Zero());

  EXPECT_GT(TimeDelta::Zero(), TimeDelta::FromMilliseconds(-100));
  EXPECT_LT(TimeDelta::Zero(), TimeDelta::FromMilliseconds(100));

  EXPECT_EQ(TimeDelta::FromMilliseconds(1000), TimeDelta::FromSeconds(1));
}

}  // namespace
}  // namespace fml
