// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FML_TIME_CHRONO_TIMESTAMP_PROVIDER_H_
#define FLUTTER_FML_TIME_CHRONO_TIMESTAMP_PROVIDER_H_

#include "flutter/fml/time/timestamp_provider.h"

#include "flutter/fml/macros.h"
#include "flutter/fml/time/time_point.h"

namespace fml {

/// TimestampProvider implementation that is backed by std::chrono::steady_clock
/// meant to be used only in tests for `fml`. Other components needing the
/// current time ticks since epoch should instantiate their own time stamp
/// provider backed by Dart clock.
class ChronoTimestampProvider : TimestampProvider {
 public:
  static ChronoTimestampProvider& Instance() {
    static ChronoTimestampProvider instance;
    return instance;
  }

  ~ChronoTimestampProvider() override;

  fml::TimePoint Now() override;

 private:
  ChronoTimestampProvider();

  FML_DISALLOW_COPY_AND_ASSIGN(ChronoTimestampProvider);
};

fml::TimePoint ChronoTicksSinceEpoch();

}  // namespace fml

#endif  // FLUTTER_FML_TIME_CHRONO_TIMESTAMP_PROVIDER_H_
