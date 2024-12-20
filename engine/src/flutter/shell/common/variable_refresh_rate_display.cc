// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/common/variable_refresh_rate_display.h"
#include "flutter/fml/logging.h"

static double GetInitialRefreshRate(
    const std::weak_ptr<flutter::VariableRefreshRateReporter>&
        refresh_rate_reporter) {
  if (auto reporter = refresh_rate_reporter.lock()) {
    return reporter->GetRefreshRate();
  }
  return 0;
}

namespace flutter {

VariableRefreshRateDisplay::VariableRefreshRateDisplay(
    DisplayId display_id,
    const std::weak_ptr<VariableRefreshRateReporter>& refresh_rate_reporter,
    double width,
    double height,
    double device_pixel_ratio)
    : Display(display_id,
              GetInitialRefreshRate(refresh_rate_reporter),
              width,
              height,
              device_pixel_ratio),
      refresh_rate_reporter_(refresh_rate_reporter) {}

double VariableRefreshRateDisplay::GetRefreshRate() const {
  return GetInitialRefreshRate(refresh_rate_reporter_);
}

}  // namespace flutter
