// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/fml/time/chrono_timestamp_provider.h"

#include <chrono>

namespace fml {

ChronoTimestampProvider::ChronoTimestampProvider() = default;

ChronoTimestampProvider::~ChronoTimestampProvider() = default;

fml::TimePoint ChronoTimestampProvider::Now() {
  const auto chrono_time_point = std::chrono::steady_clock::now();
  const auto ticks_since_epoch = chrono_time_point.time_since_epoch().count();
  return fml::TimePoint::FromTicks(ticks_since_epoch);
}

fml::TimePoint ChronoTicksSinceEpoch() {
  return ChronoTimestampProvider::Instance().Now();
}

}  // namespace fml
