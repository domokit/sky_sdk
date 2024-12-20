// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <math.h>

#include "fml/logging.h"
#include "gtest/gtest.h"

#include "flutter/impeller/geometry/trig.h"

namespace impeller {
namespace testing {

TEST(TrigTest, TrigAngles) {
  {
    Trig trig(Degrees(0.0));
    EXPECT_EQ(trig.cos, 1.0);
    EXPECT_EQ(trig.sin, 0.0);
  }

  {
    Trig trig(Radians(0.0));
    EXPECT_EQ(trig.cos, 1.0);
    EXPECT_EQ(trig.sin, 0.0);
  }

  {
    Trig trig(Degrees(30.0));
    EXPECT_NEAR(trig.cos, sqrt(0.75), kEhCloseEnough);
    EXPECT_NEAR(trig.sin, 0.5, kEhCloseEnough);
  }

  {
    Trig trig(Radians(kPi / 6.0));
    EXPECT_NEAR(trig.cos, sqrt(0.75), kEhCloseEnough);
    EXPECT_NEAR(trig.sin, 0.5, kEhCloseEnough);
  }

  {
    Trig trig(Degrees(60.0));
    EXPECT_NEAR(trig.cos, 0.5, kEhCloseEnough);
    EXPECT_NEAR(trig.sin, sqrt(0.75), kEhCloseEnough);
  }

  {
    Trig trig(Radians(kPi / 3.0));
    EXPECT_NEAR(trig.cos, 0.5, kEhCloseEnough);
    EXPECT_NEAR(trig.sin, sqrt(0.75), kEhCloseEnough);
  }

  {
    Trig trig(Degrees(90.0));
    EXPECT_NEAR(trig.cos, 0.0, kEhCloseEnough);
    EXPECT_NEAR(trig.sin, 1.0, kEhCloseEnough);
  }

  {
    Trig trig(Radians(kPi / 2.0));
    EXPECT_NEAR(trig.cos, 0.0, kEhCloseEnough);
    EXPECT_NEAR(trig.sin, 1.0, kEhCloseEnough);
  }
}

TEST(TrigTest, MultiplyByScalarRadius) {
  for (int i = 0; i <= 360; i++) {
    for (int i = 1; i <= 10; i++) {
      Scalar radius = i * 5.0f;
      EXPECT_EQ(Trig(Degrees(i)) * radius,
                Point(radius * std::cos(i * kPi / 180),
                      radius * std::sin(i * kPi / 180)))
          << "at " << i << " degrees and radius " << radius;
    }
  }
}

}  // namespace testing
}  // namespace impeller
