// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_COMMON_THREAD_HOST_H_
#define FLUTTER_SHELL_COMMON_THREAD_HOST_H_

#include <memory>
#include <optional>
#include <string>

#include "flutter/fml/macros.h"
#include "flutter/fml/thread.h"

namespace flutter {

using ThreadConfig = fml::Thread::ThreadConfig;
using ThreadConfigSetter = fml::Thread::ThreadConfigSetter;

/// The collection of all the threads used by the engine.
struct ThreadHost {
  enum Type {
    kPlatform = 1 << 0,
    kUi = 1 << 1,
    kRaster = 1 << 2,
    kIo = 1 << 3,
    kProfiler = 1 << 4,
  };

  /// The collection of all the thread configures, and we create custom thread
  /// configure in engine to info the thread.
  struct ThreadHostConfig {
    explicit ThreadHostConfig(
        const ThreadConfigSetter& setter = fml::Thread::SetCurrentThreadName)
        : type_mask(0), config_setter(setter) {}

    ThreadHostConfig(
        const std::string& name_prefix,
        uint64_t mask,
        const ThreadConfigSetter& setter = fml::Thread::SetCurrentThreadName)
        : type_mask(mask), name_prefix(name_prefix), config_setter(setter) {}

    explicit ThreadHostConfig(
        uint64_t mask,
        const ThreadConfigSetter& setter = fml::Thread::SetCurrentThreadName)
        : ThreadHostConfig("", mask, setter) {}

    /// Check if need to create thread.
    bool isThreadNeeded(Type type) const { return type_mask & type; }

    /// Use the prefix and thread type to generator a thread name.
    static std::string MakeThreadName(Type type, const std::string& prefix);

    /// Specified the UI Thread Config, meanwhile set the mask.
    void SetUIConfig(const ThreadConfig&);

    /// Specified the Platform Thread Config, meanwhile set the mask.
    void SetPlatformConfig(const ThreadConfig&);

    /// Specified the IO Thread Config, meanwhile set the mask.
    void SetRasterConfig(const ThreadConfig&);

    /// Specified the IO Thread Config, meanwhile set the mask.
    void SetIOConfig(const ThreadConfig&);

    /// Specified the ProfilerThread  Config, meanwhile set the mask.
    void SetProfilerConfig(const ThreadConfig&);

    uint64_t type_mask;

    std::string name_prefix = "";

    const ThreadConfigSetter config_setter;

    std::optional<ThreadConfig> platform_config;
    std::optional<ThreadConfig> ui_config;
    std::optional<ThreadConfig> raster_config;
    std::optional<ThreadConfig> io_config;
    std::optional<ThreadConfig> profiler_config;
  };

  std::string name_prefix;
  std::unique_ptr<fml::Thread> platform_thread;
  std::unique_ptr<fml::Thread> ui_thread;
  std::unique_ptr<fml::Thread> raster_thread;
  std::unique_ptr<fml::Thread> io_thread;
  std::unique_ptr<fml::Thread> profiler_thread;

  ThreadHost();

  ThreadHost(ThreadHost&&);

  ThreadHost& operator=(ThreadHost&&) = default;

  ThreadHost(const std::string& name_prefix, uint64_t mask);

  explicit ThreadHost(const ThreadHostConfig& host_config);

  ~ThreadHost();

 private:
  std::unique_ptr<fml::Thread> CreateThread(
      Type type,
      std::optional<ThreadConfig> thread_config,
      const ThreadHostConfig& host_config) const;
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_COMMON_THREAD_HOST_H_
