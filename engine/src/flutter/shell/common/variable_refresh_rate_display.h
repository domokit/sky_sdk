// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_COMMON_VARIABLE_REFRESH_RATE_DISPLAY_H_
#define FLUTTER_SHELL_COMMON_VARIABLE_REFRESH_RATE_DISPLAY_H_

#include <optional>

#include "display.h"
#include "flutter/fml/macros.h"
#include "variable_refresh_rate_reporter.h"

namespace flutter {

/// A Display where the refresh rate can change over time.
class VariableRefreshRateDisplay : public Display {
 public:
  explicit VariableRefreshRateDisplay(
      DisplayId display_id,
      const std::weak_ptr<VariableRefreshRateReporter>& refresh_rate_reporter,
      double width,
      double height,
      double device_pixel_ratio);
  ~VariableRefreshRateDisplay() = default;

  // |Display|
  double GetRefreshRate() const override;

 private:
  const std::weak_ptr<VariableRefreshRateReporter> refresh_rate_reporter_;

  FML_DISALLOW_COPY_AND_ASSIGN(VariableRefreshRateDisplay);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_COMMON_VARIABLE_REFRESH_RATE_DISPLAY_H_
