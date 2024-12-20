// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_COMMON_VARIABLE_REFRESH_RATE_REPORTER_H_
#define FLUTTER_SHELL_COMMON_VARIABLE_REFRESH_RATE_REPORTER_H_

#include <functional>
#include <memory>
#include <mutex>
#include <unordered_map>

namespace flutter {

/// Abstract class that reprents a platform specific mechanism to report current
/// refresh rates.
class VariableRefreshRateReporter {
 public:
  VariableRefreshRateReporter() = default;

  virtual double GetRefreshRate() const = 0;

  FML_DISALLOW_COPY_AND_ASSIGN(VariableRefreshRateReporter);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_COMMON_VARIABLE_REFRESH_RATE_REPORTER_H_
