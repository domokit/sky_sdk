// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_PROFILER_METRICS_IOS_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_PROFILER_METRICS_IOS_H_

#include <mach/mach.h>

#include <cassert>
#include <optional>

#include "flutter/fml/logging.h"
#include "flutter/shell/profiling/sampling_profiler.h"

namespace flutter {

/**
 * @brief Utility class that gathers profiling metrics used by
 * `flutter::SamplingProfiler`.
 *
 * @see flutter::SamplingProfiler
 */
class ProfilerMetricsIOS {
 public:
  ProfilerMetricsIOS() = default;

  ProfileSample GenerateSample();

 private:
  std::optional<CpuUsageInfo> CpuUsage();

  std::optional<MemoryUsageInfo> MemoryUsage();

  FML_DISALLOW_COPY_AND_ASSIGN(ProfilerMetricsIOS);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_PROFILER_METRICS_IOS_H_
