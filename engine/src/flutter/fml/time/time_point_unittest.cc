// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/fml/time/chrono_timestamp_provider.h"

#include "flutter/runtime/dart_timestamp_provider.h"

#include <thread>

#include "gtest/gtest.h"

namespace fml {
namespace {

TEST(TimePoint, Control) {
  EXPECT_LT(TimePoint::Min(), ChronoTicksSinceEpoch());
  EXPECT_GT(TimePoint::Max(), ChronoTicksSinceEpoch());
}

TEST(TimePoint, DartClockIsMonotonic) {
  using namespace std::chrono_literals;
  const auto t1 = flutter::DartTimelineTicksSinceEpoch();
  std::this_thread::sleep_for(1us);
  const auto t2 = flutter::DartTimelineTicksSinceEpoch();
  std::this_thread::sleep_for(1us);
  const auto t3 = flutter::DartTimelineTicksSinceEpoch();
  EXPECT_LT(TimePoint::Min(), t1);
  EXPECT_LE(t1, t2);
  EXPECT_LE(t2, t3);
  EXPECT_LT(t3, TimePoint::Max());
}

}  // namespace
}  // namespace fml
