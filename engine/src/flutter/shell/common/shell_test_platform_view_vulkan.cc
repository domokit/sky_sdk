// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/common/shell_test_platform_view_vulkan.h"

#include <utility>

#include "flutter/common/graphics/persistent_cache.h"
#include "flutter/flutter_vma/flutter_skia_vma.h"
#include "flutter/shell/common/context_options.h"
#include "flutter/vulkan/vulkan_skia_proc_table.h"
#include "flutter/vulkan/vulkan_utilities.h"

#include "third_party/skia/include/core/SkSurface.h"
#include "third_party/skia/include/gpu/ganesh/SkSurfaceGanesh.h"
#include "third_party/skia/include/gpu/ganesh/vk/GrVkDirectContext.h"
#include "third_party/skia/include/gpu/vk/VulkanExtensions.h"

#if OS_FUCHSIA
#define VULKAN_SO_PATH "libvulkan.so"
#else
#include "flutter/vulkan/swiftshader_path.h"
#endif

namespace flutter::testing {

std::unique_ptr<ShellTestPlatformView> ShellTestPlatformView::CreateVulkan(
    PlatformView::Delegate& delegate,
    const TaskRunners& task_runners,
    const std::shared_ptr<ShellTestVsyncClock>& vsync_clock,
    const CreateVsyncWaiter& create_vsync_waiter,
    const std::shared_ptr<ShellTestExternalViewEmbedder>&
        shell_test_external_view_embedder,
    const std::shared_ptr<const fml::SyncSwitch>& is_gpu_disabled_sync_switch) {
  return std::make_unique<ShellTestPlatformViewVulkan>(
      delegate, task_runners, vsync_clock, create_vsync_waiter,
      shell_test_external_view_embedder);
}

ShellTestPlatformViewVulkan::ShellTestPlatformViewVulkan(
    PlatformView::Delegate& delegate,
    const TaskRunners& task_runners,
    std::shared_ptr<ShellTestVsyncClock> vsync_clock,
    CreateVsyncWaiter create_vsync_waiter,
    std::shared_ptr<ShellTestExternalViewEmbedder>
        shell_test_external_view_embedder)
    : ShellTestPlatformView(delegate, task_runners),
      create_vsync_waiter_(std::move(create_vsync_waiter)),
      vsync_clock_(std::move(vsync_clock)),
      proc_table_(fml::MakeRefCounted<vulkan::VulkanProcTable>(VULKAN_SO_PATH)),
      shell_test_external_view_embedder_(
          std::move(shell_test_external_view_embedder)) {}

ShellTestPlatformViewVulkan::~ShellTestPlatformViewVulkan() = default;

std::unique_ptr<VsyncWaiter> ShellTestPlatformViewVulkan::CreateVSyncWaiter() {
  return create_vsync_waiter_();
}

void ShellTestPlatformViewVulkan::SimulateVSync() {
  vsync_clock_->SimulateVSync();
}

// |PlatformView|
std::unique_ptr<Surface> ShellTestPlatformViewVulkan::CreateRenderingSurface() {
  return std::make_unique<OffScreenSurface>(proc_table_,
                                            shell_test_external_view_embedder_);
}

// |PlatformView|
std::shared_ptr<ExternalViewEmbedder>
ShellTestPlatformViewVulkan::CreateExternalViewEmbedder() {
  return shell_test_external_view_embedder_;
}

// |PlatformView|
PointerDataDispatcherMaker ShellTestPlatformViewVulkan::GetDispatcherMaker() {
  return [](DefaultPointerDataDispatcher::Delegate& delegate) {
    return std::make_unique<SmoothPointerDataDispatcher>(delegate);
  };
}

// TODO(gw280): This code was forked from vulkan_window.cc specifically for
// shell_test.
//              We need to merge this functionality back into //vulkan.
//              https://github.com/flutter/flutter/issues/51132
ShellTestPlatformViewVulkan::OffScreenSurface::OffScreenSurface(
    fml::RefPtr<vulkan::VulkanProcTable> vk,
    std::shared_ptr<ShellTestExternalViewEmbedder>
        shell_test_external_view_embedder)
    : vk_(std::move(vk)),
      shell_test_external_view_embedder_(
          std::move(shell_test_external_view_embedder)) {
  if (!vk_ || !vk_->HasAcquiredMandatoryProcAddresses()) {
    FML_DLOG(ERROR) << "Proc table has not acquired mandatory proc addresses.";
    return;
  }

  // Create the application instance.
  std::vector<std::string> extensions = {
      VK_KHR_EXTERNAL_SEMAPHORE_CAPABILITIES_EXTENSION_NAME,
  };

  application_ = std::make_unique<vulkan::VulkanApplication>(
      *vk_, "FlutterTest", std::move(extensions), VK_MAKE_VERSION(1, 0, 0),
      VK_MAKE_VERSION(1, 1, 0), true);

  if (!application_->IsValid() || !vk_->AreInstanceProcsSetup()) {
    // Make certain the application instance was created and it set up the
    // instance proc table entries.
    FML_DLOG(ERROR) << "Instance proc addresses have not been set up.";
    return;
  }

  // Create the device.

  logical_device_ = application_->AcquireFirstCompatibleLogicalDevice();

  if (logical_device_ == nullptr || !logical_device_->IsValid() ||
      !vk_->AreDeviceProcsSetup()) {
    // Make certain the device was created and it set up the device proc table
    // entries.
    FML_DLOG(ERROR) << "Device proc addresses have not been set up.";
    return;
  }

  memory_allocator_ = FlutterSkiaVulkanMemoryAllocator::Make(
      application_->GetAPIVersion(), application_->GetInstance(),
      logical_device_->GetPhysicalDeviceHandle(), logical_device_->GetHandle(),
      vk_, true);

  // Create the Skia GrContext.
  if (!CreateSkiaGrContext()) {
    FML_DLOG(ERROR) << "Could not create Skia context.";
    return;
  }

  valid_ = true;
}

bool ShellTestPlatformViewVulkan::OffScreenSurface::CreateSkiaGrContext() {
  skgpu::VulkanBackendContext backend_context;
  skgpu::VulkanExtensions no_extensions;
  // For now, Skia crashes if fDeviceFeatures is set but fVkExtensions is not.
  backend_context.fVkExtensions = &no_extensions;
  VkPhysicalDeviceFeatures features;
  // It may be tempting to put features into backend_context here
  // and pass just backend_context into the below function, however the pointers
  // for features are const, so we won't be able to update them.

  if (!this->CreateSkiaBackendContext(&backend_context, &features)) {
    FML_DLOG(ERROR) << "Could not create Skia backend context.";
    return false;
  }

  const auto options =
      MakeDefaultContextOptions(ContextType::kRender, GrBackendApi::kVulkan);

  sk_sp<GrDirectContext> context =
      GrDirectContexts::MakeVulkan(backend_context, options);

  if (context == nullptr) {
    FML_DLOG(ERROR) << "Failed to create GrDirectContext";
    return false;
  }

  context->setResourceCacheLimit(vulkan::kGrCacheMaxByteSize);

  context_ = context;

  return true;
}

bool ShellTestPlatformViewVulkan::OffScreenSurface::CreateSkiaBackendContext(
    skgpu::VulkanBackendContext* context,
    VkPhysicalDeviceFeatures* features) {
  FML_CHECK(context);
  FML_CHECK(features);
  auto getProc = CreateSkiaGetProc(vk_);

  if (getProc == nullptr) {
    FML_DLOG(ERROR) << "GetProcAddress is null";
    return false;
  }

  if (!logical_device_->GetPhysicalDeviceFeatures(features)) {
    FML_DLOG(ERROR) << "Failed to get Physical Device features";
    return false;
  }

  context->fInstance = application_->GetInstance();
  context->fPhysicalDevice = logical_device_->GetPhysicalDeviceHandle();
  context->fDevice = logical_device_->GetHandle();
  context->fQueue = logical_device_->GetQueueHandle();
  context->fGraphicsQueueIndex = logical_device_->GetGraphicsQueueIndex();
  context->fMaxAPIVersion = application_->GetAPIVersion();
  context->fDeviceFeatures = features;
  context->fGetProc = std::move(getProc);
  context->fMemoryAllocator = memory_allocator_;

  return true;
}

ShellTestPlatformViewVulkan::OffScreenSurface::~OffScreenSurface() {}

bool ShellTestPlatformViewVulkan::OffScreenSurface::IsValid() {
  return valid_;
}

std::unique_ptr<SurfaceFrame>
ShellTestPlatformViewVulkan::OffScreenSurface::AcquireFrame(
    const SkISize& size) {
  auto image_info = SkImageInfo::Make(size, SkColorType::kRGBA_8888_SkColorType,
                                      SkAlphaType::kOpaque_SkAlphaType);
  auto surface = SkSurfaces::RenderTarget(context_.get(), skgpu::Budgeted::kNo,
                                          image_info, 0, nullptr);

  SurfaceFrame::EncodeCallback encode_callback = [](const SurfaceFrame&,
                                                    DlCanvas* canvas) -> bool {
    canvas->Flush();
    return true;
  };
  SurfaceFrame::SubmitCallback submit_callback =
      [](const SurfaceFrame&) -> bool { return true; };

  SurfaceFrame::FramebufferInfo framebuffer_info;
  framebuffer_info.supports_readback = true;

  return std::make_unique<SurfaceFrame>(std::move(surface), framebuffer_info,
                                        std::move(encode_callback),
                                        std::move(submit_callback),
                                        /*frame_size=*/SkISize::Make(800, 600));
}

GrDirectContext* ShellTestPlatformViewVulkan::OffScreenSurface::GetContext() {
  return context_.get();
}

SkMatrix ShellTestPlatformViewVulkan::OffScreenSurface::GetRootTransformation()
    const {
  SkMatrix matrix;
  matrix.reset();
  return matrix;
}

}  // namespace flutter::testing
