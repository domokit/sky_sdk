// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/profiling/sampling_profiler.h"

#include <utility>

namespace flutter {

SamplingProfiler::SamplingProfiler(
    const char* thread_label,
    fml::RefPtr<fml::TaskRunner> profiler_task_runner,
    Sampler sampler,
    int num_samples_per_sec)
    : thread_label_(thread_label),
      profiler_task_runner_(std::move(profiler_task_runner)),
      sampler_(std::move(sampler)),
      num_samples_per_sec_(num_samples_per_sec) {}

SamplingProfiler::~SamplingProfiler() {
  if (is_running_) {
    Stop();
  }
}

void SamplingProfiler::Start() {
  if (!profiler_task_runner_) {
    return;
  }
  FML_CHECK(num_samples_per_sec_ > 0)
      << "number of samples must be a positive integer, got: "
      << num_samples_per_sec_;
  double delay_between_samples = 1.0 / num_samples_per_sec_;
  auto task_delay = fml::TimeDelta::FromSecondsF(delay_between_samples);
  UpdateDartVMServiceThreadName();
  is_running_ = true;
  SampleRepeatedly(task_delay);
}

void SamplingProfiler::Stop() {
  FML_DCHECK(is_running_);
  auto latch = std::make_unique<fml::AutoResetWaitableEvent>();
  shutdown_latch_.store(latch.get());
  latch->Wait();
  shutdown_latch_.store(nullptr);
  is_running_ = false;
}

void SamplingProfiler::SampleRepeatedly(fml::TimeDelta task_delay) const {
  profiler_task_runner_->PostDelayedTask(
      [profiler = this, task_delay = task_delay, sampler = sampler_,
       &shutdown_latch = shutdown_latch_]() {
        // TODO(kaushikiska): consider buffering these every n seconds to
        // avoid spamming the trace buffer.
        const ProfileSample usage = sampler();
        if (usage.cpu_usage) {
          const auto& cpu_usage = usage.cpu_usage;
          std::string total_cpu_usage =
              std::to_string(cpu_usage->total_cpu_usage);
          std::string num_threads = std::to_string(cpu_usage->num_threads);
          TRACE_EVENT_INSTANT2("flutter::profiling", "CpuUsage",
                               "total_cpu_usage", total_cpu_usage.c_str(),
                               "num_threads", num_threads.c_str());
        }
        if (usage.memory_usage) {
          std::string dirty_memory_usage =
              std::to_string(usage.memory_usage->dirty_memory_usage);
          std::string owned_shared_memory_usage =
              std::to_string(usage.memory_usage->owned_shared_memory_usage);
          TRACE_EVENT_INSTANT2("flutter::profiling", "MemoryUsage",
                               "dirty_memory_usage", dirty_memory_usage.c_str(),
                               "owned_shared_memory_usage",
                               owned_shared_memory_usage.c_str());
        }
        if (usage.gpu_usage) {
          std::string gpu_usage =
              std::to_string(usage.gpu_usage->percent_usage);
          TRACE_EVENT_INSTANT1("flutter::profiling", "GpuUsage", "gpu_usage",
                               gpu_usage.c_str());
        }
        if (shutdown_latch.load()) {
          shutdown_latch.load()->Signal();
        } else {
          profiler->SampleRepeatedly(task_delay);
        }
      },
      task_delay);
}

void SamplingProfiler::UpdateDartVMServiceThreadName() const {
  FML_CHECK(profiler_task_runner_);

  profiler_task_runner_->PostTask(
      [label = thread_label_ + std::string{".profiler"}]() {
        Dart_SetThreadName(label.c_str());
      });
}

}  // namespace flutter
