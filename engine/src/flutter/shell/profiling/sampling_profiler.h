// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PROFILING_SAMPLING_PROFILER_H_
#define FLUTTER_SHELL_PROFILING_SAMPLING_PROFILER_H_

#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "flutter/fml/synchronization/count_down_latch.h"
#include "flutter/fml/task_runner.h"
#include "flutter/fml/trace_event.h"

namespace flutter {

/**
 * @brief CPU usage stats. `num_threads` is the number of threads owned by the
 * process. It is to be noted that this is not per shell, there can be multiple
 * shells within the process. `total_cpu_usage` is the percentage (between [0,
 * 100]) cpu usage of the application. This is across all the cores, for example
 * an application using 100% of all the core will report `total_cpu_usage` as
 * `100`, if it has 100% across 2 cores and 0% across the other cores, embedder
 * must report `total_cpu_usage` as `50`.
 */
struct CpuUsageInfo {
  uint32_t num_threads;
  double total_cpu_usage;
};

/**
 * @brief Memory usage stats. `dirty_memory_usage` is the memory usage (in
 * MB) such that the app uses its physical memory for dirty memory. Dirty memory
 * is the memory data that cannot be paged to disk. `owned_shared_memory_usage`
 * is the memory usage (in MB) such that the app uses its physical memory for
 * shared memory, including loaded frameworks and executables. On iOS, it's
 * `physical memory - dirty memory`.
 */
struct MemoryUsageInfo {
  double dirty_memory_usage;
  double owned_shared_memory_usage;
};

/**
 * @brief Polled information related to the usage of the GPU.
 */
struct GpuUsageInfo {
  double percent_usage;
};

/**
 * @brief Container for the metrics we collect during each run of `Sampler`.
 * This currently holds `CpuUsageInfo` and `MemoryUsageInfo` but the intent
 * is to expand it to other metrics.
 *
 * @see flutter::Sampler
 */
struct ProfileSample {
  std::optional<CpuUsageInfo> cpu_usage;
  std::optional<MemoryUsageInfo> memory_usage;
  std::optional<GpuUsageInfo> gpu_usage;
};

/**
 * @brief Sampler is run during `SamplingProfiler::SampleRepeatedly`. Each
 * platform should implement its version of a `Sampler` if they decide to
 * participate in gathering profiling metrics.
 *
 * @see flutter::SamplingProfiler::SampleRepeatedly
 */
using Sampler = std::function<ProfileSample(void)>;

/**
 * @brief a Sampling Profiler that runs peridically and calls the `Sampler`
 * which servers as a value function to gather various profiling metrics as
 * represented by `ProfileSample`. These profiling metrics are then posted to
 * the Dart VM Service timeline.
 *
 */
class SamplingProfiler {
 public:
  /**
   * @brief Construct a new Sampling Profiler object
   *
   * @param thread_label Dart VM Service prefix to be set for the profiling task
   * runner.
   * @param profiler_task_runner the task runner to service sampling requests.
   * @param sampler the value function to collect the profiling metrics.
   * @param num_samples_per_sec number of times you wish to run the sampler per
   * second.
   *
   * @see fml::TaskRunner
   */
  SamplingProfiler(const char* thread_label,
                   fml::RefPtr<fml::TaskRunner> profiler_task_runner,
                   Sampler sampler,
                   int num_samples_per_sec);

  ~SamplingProfiler();

  /**
   * @brief Starts the SamplingProfiler by triggering `SampleRepeatedly`.
   *
   */
  void Start();

  void Stop();

 private:
  const std::string thread_label_;
  const fml::RefPtr<fml::TaskRunner> profiler_task_runner_;
  const Sampler sampler_;
  const uint32_t num_samples_per_sec_;
  bool is_running_ = false;
  std::atomic<fml::AutoResetWaitableEvent*> shutdown_latch_ = nullptr;

  void SampleRepeatedly(fml::TimeDelta task_delay) const;

  /**
   * @brief This doesn't update the underlying OS thread name for the thread
   * backing `profiler_task_runner_`. Instead, this is just additional metadata
   * for the VM Service to show the thread name of the isolate.
   *
   */
  void UpdateDartVMServiceThreadName() const;

  FML_DISALLOW_COPY_AND_ASSIGN(SamplingProfiler);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PROFILING_SAMPLING_PROFILER_H_
