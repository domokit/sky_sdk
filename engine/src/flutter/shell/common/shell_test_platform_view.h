// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_COMMON_SHELL_TEST_PLATFORM_VIEW_H_
#define FLUTTER_SHELL_COMMON_SHELL_TEST_PLATFORM_VIEW_H_

#include <exception>

#include "flutter/shell/common/platform_view.h"
#include "flutter/shell/common/shell_test_external_view_embedder.h"
#include "flutter/shell/common/vsync_waiters_test.h"

namespace flutter::testing {

class ShellTestPlatformView : public PlatformView {
 public:
  enum class BackendType {
    kGLBackend,
    kVulkanBackend,
    kMetalBackend,
  };

  static BackendType DefaultBackendType() {
#if defined(SHELL_ENABLE_GL)
    return BackendType::kGLBackend;
#elif defined(SHELL_ENABLE_METAL)
    return BackendType::kMetalBackend;
#elif defined(SHELL_ENABLE_VULKAN)
    return BackendType::kVulkanBackend;
#else
    FML_LOG(FATAL) << "No backend is enabled in this build.";
    std::terminate();
#endif
  }

  static std::unique_ptr<ShellTestPlatformView> Create(
      BackendType backend,
      PlatformView::Delegate& delegate,
      const TaskRunners& task_runners,
      const std::shared_ptr<ShellTestVsyncClock>& vsync_clock,
      const CreateVsyncWaiter& create_vsync_waiter,
      const std::shared_ptr<ShellTestExternalViewEmbedder>&
          shell_test_external_view_embedder,
      const std::shared_ptr<const fml::SyncSwitch>&
          is_gpu_disabled_sync_switch);

  virtual void SimulateVSync() = 0;

 protected:
  ShellTestPlatformView(PlatformView::Delegate& delegate,
                        const TaskRunners& task_runners)
      : PlatformView(delegate, task_runners) {}

  static std::unique_ptr<ShellTestPlatformView> CreateGL(
      PlatformView::Delegate& delegate,
      const TaskRunners& task_runners,
      const std::shared_ptr<ShellTestVsyncClock>& vsync_clock,
      const CreateVsyncWaiter& create_vsync_waiter,
      const std::shared_ptr<ShellTestExternalViewEmbedder>&
          shell_test_external_view_embedder,
      const std::shared_ptr<const fml::SyncSwitch>&
          is_gpu_disabled_sync_switch);
  static std::unique_ptr<ShellTestPlatformView> CreateMetal(
      PlatformView::Delegate& delegate,
      const TaskRunners& task_runners,
      const std::shared_ptr<ShellTestVsyncClock>& vsync_clock,
      const CreateVsyncWaiter& create_vsync_waiter,
      const std::shared_ptr<ShellTestExternalViewEmbedder>&
          shell_test_external_view_embedder,
      const std::shared_ptr<const fml::SyncSwitch>&
          is_gpu_disabled_sync_switch);
  static std::unique_ptr<ShellTestPlatformView> CreateVulkan(
      PlatformView::Delegate& delegate,
      const TaskRunners& task_runners,
      const std::shared_ptr<ShellTestVsyncClock>& vsync_clock,
      const CreateVsyncWaiter& create_vsync_waiter,
      const std::shared_ptr<ShellTestExternalViewEmbedder>&
          shell_test_external_view_embedder,
      const std::shared_ptr<const fml::SyncSwitch>&
          is_gpu_disabled_sync_switch);

  FML_DISALLOW_COPY_AND_ASSIGN(ShellTestPlatformView);
};

// Create a ShellTestPlatformView from configuration struct.
class ShellTestPlatformViewBuilder {
 public:
  struct Config {
    bool simulate_vsync = false;
    std::shared_ptr<ShellTestExternalViewEmbedder>
        shell_test_external_view_embedder = nullptr;
    ShellTestPlatformView::BackendType rendering_backend =
        ShellTestPlatformView::DefaultBackendType();
  };

  explicit ShellTestPlatformViewBuilder(Config config);
  ~ShellTestPlatformViewBuilder() = default;

  // Override operator () to make this class assignable to std::function.
  std::unique_ptr<PlatformView> operator()(Shell& shell);

 private:
  Config config_;
};

}  // namespace flutter::testing

#endif  // FLUTTER_SHELL_COMMON_SHELL_TEST_PLATFORM_VIEW_H_
