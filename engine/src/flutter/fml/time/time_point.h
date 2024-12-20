// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FML_TIME_TIME_POINT_H_
#define FLUTTER_FML_TIME_TIME_POINT_H_

#include <cstdint>
#include <functional>
#include <iosfwd>

#include "flutter/fml/time/time_delta.h"

namespace fml {

// A TimePoint represents a point in time represented as an integer number of
// nanoseconds elapsed since an arbitrary point in the past.
//
// WARNING: This class should not be serialized across reboots, or across
// devices: the reference point is only stable for a given device between
// reboots.
class TimePoint {
 public:
  using ClockSource = TimePoint (*)();

  // Default TimePoint with internal value 0 (epoch).
  constexpr TimePoint() = default;

  static void SetClockSource(ClockSource source);

  static TimePoint Now();

  static TimePoint CurrentWallTime();

  static constexpr TimePoint Min() {
    return TimePoint(std::numeric_limits<int64_t>::min());
  }

  static constexpr TimePoint Max() {
    return TimePoint(std::numeric_limits<int64_t>::max());
  }

  static constexpr TimePoint FromEpochDelta(TimeDelta ticks) {
    return TimePoint(ticks.ToNanoseconds());
  }

  // Expects ticks in nanos.
  static constexpr TimePoint FromTicks(int64_t ticks) {
    return TimePoint(ticks);
  }

  TimeDelta ToEpochDelta() const { return TimeDelta::FromNanoseconds(ticks_); }

  // Compute the difference between two time points.
  TimeDelta operator-(TimePoint other) const {
    return TimeDelta::FromNanoseconds(ticks_ - other.ticks_);
  }

  TimePoint operator+(TimeDelta duration) const {
    return TimePoint(ticks_ + duration.ToNanoseconds());
  }
  TimePoint operator-(TimeDelta duration) const {
    return TimePoint(ticks_ - duration.ToNanoseconds());
  }

  bool operator==(TimePoint other) const { return ticks_ == other.ticks_; }
  bool operator!=(TimePoint other) const { return ticks_ != other.ticks_; }
  bool operator<(TimePoint other) const { return ticks_ < other.ticks_; }
  bool operator<=(TimePoint other) const { return ticks_ <= other.ticks_; }
  bool operator>(TimePoint other) const { return ticks_ > other.ticks_; }
  bool operator>=(TimePoint other) const { return ticks_ >= other.ticks_; }

 private:
  explicit constexpr TimePoint(int64_t ticks) : ticks_(ticks) {}

  int64_t ticks_ = 0;
};

}  // namespace fml

#endif  // FLUTTER_FML_TIME_TIME_POINT_H_
