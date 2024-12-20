// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FML_TIME_TIMESTAMP_PROVIDER_H_
#define FLUTTER_FML_TIME_TIMESTAMP_PROVIDER_H_

#include <cstdint>

#include "flutter/fml/time/time_point.h"

namespace fml {

/// Pluggable provider of monotonic timestamps. Invocations of `Now` must return
/// unique values. Any two consecutive invocations must be ordered.
class TimestampProvider {
 public:
  virtual ~TimestampProvider(){};

  // Returns the number of ticks elapsed by a monotonic clock since epoch.
  virtual fml::TimePoint Now() = 0;
};

}  // namespace fml

#endif  // FLUTTER_FML_TIME_TIMESTAMP_PROVIDER_H_
