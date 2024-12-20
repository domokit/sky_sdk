// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "dart_timestamp_provider.h"

#include "dart_tools_api.h"

namespace flutter {

DartTimestampProvider::DartTimestampProvider() = default;

DartTimestampProvider::~DartTimestampProvider() = default;

int64_t DartTimestampProvider::ConvertToNanos(int64_t ticks,
                                              int64_t frequency) {
  int64_t nano_seconds = (ticks / frequency) * kNanosPerSecond;
  int64_t leftover_ticks = ticks % frequency;
  int64_t leftover_nanos = (leftover_ticks * kNanosPerSecond) / frequency;
  return nano_seconds + leftover_nanos;
}

fml::TimePoint DartTimestampProvider::Now() {
  const int64_t ticks = Dart_TimelineGetTicks();
  const int64_t frequency = Dart_TimelineGetTicksFrequency();
  // optimization for the most common case.
  if (frequency != kNanosPerSecond) {
    return fml::TimePoint::FromTicks(ConvertToNanos(ticks, frequency));
  } else {
    return fml::TimePoint::FromTicks(ticks);
  }
}

fml::TimePoint DartTimelineTicksSinceEpoch() {
  return DartTimestampProvider::Instance().Now();
}

}  // namespace flutter
