// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_COMMON_SHELL_TEST_PLATFORM_VIEW_VULKAN_H_
#define FLUTTER_SHELL_COMMON_SHELL_TEST_PLATFORM_VIEW_VULKAN_H_

#include "flutter/shell/common/shell_test_external_view_embedder.h"
#include "flutter/shell/common/shell_test_platform_view.h"
#include "flutter/shell/gpu/gpu_surface_vulkan_delegate.h"
#include "flutter/vulkan/vulkan_application.h"
#include "flutter/vulkan/vulkan_device.h"
#include "flutter/vulkan/vulkan_skia_proc_table.h"
#include "third_party/skia/include/gpu/vk/VulkanBackendContext.h"
#include "third_party/skia/include/gpu/vk/VulkanMemoryAllocator.h"
#include "third_party/skia/include/gpu/vk/VulkanTypes.h"

namespace flutter::testing {

class ShellTestPlatformViewVulkan : public ShellTestPlatformView {
 public:
  ShellTestPlatformViewVulkan(PlatformView::Delegate& delegate,
                              const TaskRunners& task_runners,
                              std::shared_ptr<ShellTestVsyncClock> vsync_clock,
                              CreateVsyncWaiter create_vsync_waiter,
                              std::shared_ptr<ShellTestExternalViewEmbedder>
                                  shell_test_external_view_embedder);

  ~ShellTestPlatformViewVulkan() override;

  void SimulateVSync() override;

 private:
  class OffScreenSurface : public flutter::Surface {
   public:
    OffScreenSurface(fml::RefPtr<vulkan::VulkanProcTable> vk,
                     std::shared_ptr<ShellTestExternalViewEmbedder>
                         shell_test_external_view_embedder);

    ~OffScreenSurface() override;

    // |Surface|
    bool IsValid() override;

    // |Surface|
    std::unique_ptr<SurfaceFrame> AcquireFrame(const SkISize& size) override;

    // |Surface|
    SkMatrix GetRootTransformation() const override;

    // |Surface|
    GrDirectContext* GetContext() override;

   private:
    bool valid_ = false;
    fml::RefPtr<vulkan::VulkanProcTable> vk_;
    std::shared_ptr<ShellTestExternalViewEmbedder>
        shell_test_external_view_embedder_;
    std::unique_ptr<vulkan::VulkanApplication> application_;
    std::unique_ptr<vulkan::VulkanDevice> logical_device_;
    sk_sp<skgpu::VulkanMemoryAllocator> memory_allocator_;
    sk_sp<GrDirectContext> context_;

    bool CreateSkiaGrContext();
    bool CreateSkiaBackendContext(skgpu::VulkanBackendContext*,
                                  VkPhysicalDeviceFeatures*);

    FML_DISALLOW_COPY_AND_ASSIGN(OffScreenSurface);
  };

  CreateVsyncWaiter create_vsync_waiter_;

  std::shared_ptr<ShellTestVsyncClock> vsync_clock_;

  fml::RefPtr<vulkan::VulkanProcTable> proc_table_;

  std::shared_ptr<ShellTestExternalViewEmbedder>
      shell_test_external_view_embedder_;

  // |PlatformView|
  std::unique_ptr<Surface> CreateRenderingSurface() override;

  // |PlatformView|
  std::shared_ptr<ExternalViewEmbedder> CreateExternalViewEmbedder() override;

  // |PlatformView|
  std::unique_ptr<VsyncWaiter> CreateVSyncWaiter() override;

  // |PlatformView|
  PointerDataDispatcherMaker GetDispatcherMaker() override;

  FML_DISALLOW_COPY_AND_ASSIGN(ShellTestPlatformViewVulkan);
};

}  // namespace flutter::testing

#endif  // FLUTTER_SHELL_COMMON_SHELL_TEST_PLATFORM_VIEW_VULKAN_H_
