// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <thread>

#include "flutter/fml/time/chrono_timestamp_provider.h"
#include "flutter/fml/time/time_delta.h"
#include "gtest/gtest.h"

namespace fml {
namespace {

TEST(Time, Now) {
  auto start = ChronoTicksSinceEpoch();
  for (int i = 0; i < 3; ++i) {
    auto now = ChronoTicksSinceEpoch();
    EXPECT_GE(now, start);
    std::this_thread::yield();
  }
}

TEST(Time, IntConversions) {
  // Integer conversions should all truncate, not round.
  TimeDelta delta = TimeDelta::FromNanoseconds(102304506708ll);
  EXPECT_EQ(102304506708ll, delta.ToNanoseconds());
  EXPECT_EQ(102304506ll, delta.ToMicroseconds());
  EXPECT_EQ(102304ll, delta.ToMilliseconds());
  EXPECT_EQ(102ll, delta.ToSeconds());
}

TEST(Time, FloatConversions) {
  // Float conversions should remain close to the original value.
  TimeDelta delta = TimeDelta::FromNanoseconds(102304506708ll);
  EXPECT_FLOAT_EQ(102304506708.0, delta.ToNanosecondsF());
  EXPECT_FLOAT_EQ(102304506.708, delta.ToMicrosecondsF());
  EXPECT_FLOAT_EQ(102304.506708, delta.ToMillisecondsF());
  EXPECT_FLOAT_EQ(102.304506708, delta.ToSecondsF());
}

TEST(Time, TimespecConversions) {
  struct timespec ts;
  ts.tv_sec = 5;
  ts.tv_nsec = 7;
  TimeDelta from_timespec = TimeDelta::FromTimespec(ts);
  EXPECT_EQ(5, from_timespec.ToSeconds());
  EXPECT_EQ(5 * 1000000000ll + 7, from_timespec.ToNanoseconds());
  struct timespec to_timespec = from_timespec.ToTimespec();
  EXPECT_EQ(ts.tv_sec, to_timespec.tv_sec);
  EXPECT_EQ(ts.tv_nsec, to_timespec.tv_nsec);
}

}  // namespace
}  // namespace fml
