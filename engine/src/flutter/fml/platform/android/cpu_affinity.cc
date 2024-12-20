// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/fml/platform/android/cpu_affinity.h"

#include <pthread.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <unistd.h>
#include <mutex>
#include <optional>
#include <thread>
#include "flutter/fml/logging.h"

namespace fml {

/// The CPUSpeedTracker is initialized once the first time a thread affinity is
/// requested.
std::once_flag gCPUTrackerFlag;
static CPUSpeedTracker* gCPUTracker;

// For each CPU index provided, attempts to open the file
// /sys/devices/system/cpu/cpu$NUM/cpufreq/cpuinfo_max_freq and parse a number
// containing the CPU frequency.
void InitCPUInfo(size_t cpu_count) {
  std::vector<CpuIndexAndSpeed> cpu_speeds;

  for (auto i = 0u; i < cpu_count; i++) {
    auto path = "/sys/devices/system/cpu/cpu" + std::to_string(i) +
                "/cpufreq/cpuinfo_max_freq";
    auto speed = ReadIntFromFile(path);
    if (speed.has_value()) {
      cpu_speeds.push_back({.index = i, .speed = speed.value()});
    }
  }
  gCPUTracker = new CPUSpeedTracker(cpu_speeds);
}

bool SetUpCPUTracker() {
  // Populate CPU Info if uninitialized.
  auto count = std::thread::hardware_concurrency();
  std::call_once(gCPUTrackerFlag, [count]() { InitCPUInfo(count); });
  if (gCPUTracker == nullptr || !gCPUTracker->IsValid()) {
    return false;
  }
  return true;
}

std::optional<size_t> AndroidEfficiencyCoreCount() {
  if (!SetUpCPUTracker()) {
    return true;
  }
  auto result = gCPUTracker->GetIndices(CpuAffinity::kEfficiency).size();
  FML_DCHECK(result > 0);
  return result;
}

bool AndroidRequestAffinity(CpuAffinity affinity) {
  if (!SetUpCPUTracker()) {
    return true;
  }

  cpu_set_t set;
  CPU_ZERO(&set);
  for (const auto index : gCPUTracker->GetIndices(affinity)) {
    CPU_SET(index, &set);
  }
  return sched_setaffinity(gettid(), sizeof(set), &set) == 0;
}

}  // namespace fml
