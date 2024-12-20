// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// FLUTTER_NOLINT: https://github.com/flutter/flutter/issues/68331

#include "vulkan_window.h"

#include <memory>
#include <string>
#include <utility>

#include "flutter/flutter_vma/flutter_skia_vma.h"
#include "flutter/vulkan/vulkan_skia_proc_table.h"
#include "vulkan_application.h"
#include "vulkan_device.h"
#include "vulkan_native_surface.h"
#include "vulkan_surface.h"
#include "vulkan_swapchain.h"

#include "third_party/skia/include/core/SkSurface.h"
#include "third_party/skia/include/gpu/ganesh/GrDirectContext.h"
#include "third_party/skia/include/gpu/ganesh/vk/GrVkDirectContext.h"
#include "third_party/skia/include/gpu/vk/VulkanExtensions.h"

namespace vulkan {

VulkanWindow::VulkanWindow(fml::RefPtr<VulkanProcTable> proc_table,
                           std::unique_ptr<VulkanNativeSurface> native_surface)
    : VulkanWindow(/*context/*/ nullptr,
                   std::move(proc_table),
                   std::move(native_surface)) {}

VulkanWindow::VulkanWindow(const sk_sp<GrDirectContext>& context,
                           fml::RefPtr<VulkanProcTable> proc_table,
                           std::unique_ptr<VulkanNativeSurface> native_surface)
    : valid_(false), vk_(std::move(proc_table)), skia_gr_context_(context) {
  if (!vk_ || !vk_->HasAcquiredMandatoryProcAddresses()) {
    FML_DLOG(INFO) << "Proc table has not acquired mandatory proc addresses.";
    return;
  }

  if (native_surface && !native_surface->IsValid()) {
    FML_DLOG(INFO) << "Native surface is invalid.";
    return;
  }

  // Create the application instance.

  std::vector<std::string> extensions = {
      VK_KHR_SURFACE_EXTENSION_NAME,      // parent extension
      native_surface->GetExtensionName()  // child extension
  };

  application_ = std::make_unique<VulkanApplication>(*vk_, "Flutter",
                                                     std::move(extensions));

  if (!application_->IsValid() || !vk_->AreInstanceProcsSetup()) {
    // Make certain the application instance was created and it set up the
    // instance proc table entries.
    FML_DLOG(INFO) << "Instance proc addresses have not been set up.";
    return;
  }

  // Create the device.

  logical_device_ = application_->AcquireFirstCompatibleLogicalDevice();

  if (logical_device_ == nullptr || !logical_device_->IsValid() ||
      !vk_->AreDeviceProcsSetup()) {
    // Make certain the device was created and it set up the device proc table
    // entries.
    FML_DLOG(INFO) << "Device proc addresses have not been set up.";
    return;
  }

  if (!native_surface) {
    return;
  }

  // Create the logical surface from the native platform surface.
  surface_ = std::make_unique<VulkanSurface>(*vk_, *application_,
                                             std::move(native_surface));

  if (!surface_->IsValid()) {
    FML_DLOG(INFO) << "Vulkan surface is invalid.";
    return;
  }

  // Needs to happen before GrDirectContext is created.
  memory_allocator_ = flutter::FlutterSkiaVulkanMemoryAllocator::Make(
      application_->GetAPIVersion(), application_->GetInstance(),
      logical_device_->GetPhysicalDeviceHandle(), logical_device_->GetHandle(),
      vk_, true);

  // Create the Skia GrDirectContext.

  if (!skia_gr_context_ && !CreateSkiaGrContext()) {
    FML_DLOG(INFO) << "Could not create Skia context.";
    return;
  }

  // Create the swapchain.

  if (!RecreateSwapchain()) {
    FML_DLOG(INFO) << "Could not set up the swapchain initially.";
    return;
  }

  valid_ = true;
}

VulkanWindow::~VulkanWindow() = default;

bool VulkanWindow::IsValid() const {
  return valid_;
}

GrDirectContext* VulkanWindow::GetSkiaGrContext() {
  return skia_gr_context_.get();
}

bool VulkanWindow::CreateSkiaGrContext() {
#ifdef SK_VULKAN
  skgpu::VulkanBackendContext backend_context;
  VkPhysicalDeviceFeatures features;
  skgpu::VulkanExtensions extensions;

  if (!this->CreateSkiaBackendContext(&backend_context, &features,
                                      &extensions)) {
    return false;
  }

  GrContextOptions options;
  options.fReduceOpsTaskSplitting = GrContextOptions::Enable::kNo;
  sk_sp<GrDirectContext> context =
      GrDirectContexts::MakeVulkan(backend_context, options);

  if (context == nullptr) {
    return false;
  }

  context->setResourceCacheLimit(kGrCacheMaxByteSize);

  skia_gr_context_ = context;

  return true;
#else
  return false;
#endif  // SK_VULKAN
}

bool VulkanWindow::CreateSkiaBackendContext(
    skgpu::VulkanBackendContext* context,
    VkPhysicalDeviceFeatures* features,
    skgpu::VulkanExtensions* extensions) {
#ifdef SK_VULKAN
  FML_CHECK(context);
  FML_CHECK(features);
  FML_CHECK(extensions);
  auto getProc = CreateSkiaGetProc(vk_);

  if (getProc == nullptr) {
    return false;
  }

  if (!logical_device_->GetPhysicalDeviceFeatures(features)) {
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

  constexpr uint32_t instance_extension_count = 2;
  const char* instance_extensions[instance_extension_count] = {
      // https://registry.khronos.org/vulkan/specs/1.3-extensions/man/html/VK_KHR_surface.html
      VK_KHR_SURFACE_EXTENSION_NAME,
      surface_->GetNativeSurface().GetExtensionName(),
  };
  constexpr uint32_t device_extension_count = 1;
  const char* device_extensions[device_extension_count] = {
      // https://registry.khronos.org/vulkan/specs/1.3-extensions/man/html/VK_KHR_swapchain.html
      VK_KHR_SWAPCHAIN_EXTENSION_NAME,
  };
  extensions->init(context->fGetProc, context->fInstance,
                   context->fPhysicalDevice, instance_extension_count,
                   instance_extensions, device_extension_count,
                   device_extensions);
  context->fVkExtensions = extensions;
  return true;
#else
  return false;
#endif
}

sk_sp<SkSurface> VulkanWindow::AcquireSurface() {
  if (!IsValid()) {
    FML_DLOG(INFO) << "Surface is invalid.";
    return nullptr;
  }

  auto surface_size = surface_->GetSize();

  // This check is theoretically unnecessary as the swapchain should report that
  // the surface is out-of-date and perform swapchain recreation at the new
  // configuration. However, on Android, the swapchain never reports that it is
  // of date. Hence this extra check. Platforms that don't have this issue, or,
  // cant report this information (which is optional anyway), report a zero
  // size.
  if (surface_size != SkISize::Make(0, 0) &&
      surface_size != swapchain_->GetSize()) {
    FML_DLOG(INFO) << "Swapchain and surface sizes are out of sync. Recreating "
                      "swapchain.";
    if (!RecreateSwapchain()) {
      FML_DLOG(INFO) << "Could not recreate swapchain.";
      valid_ = false;
      return nullptr;
    }
  }

  while (true) {
    sk_sp<SkSurface> surface;
    auto acquire_result = VulkanSwapchain::AcquireStatus::ErrorSurfaceLost;

    std::tie(acquire_result, surface) = swapchain_->AcquireSurface();

    if (acquire_result == VulkanSwapchain::AcquireStatus::Success) {
      // Successfully acquired a surface from the swapchain. Nothing more to do.
      return surface;
    }

    if (acquire_result == VulkanSwapchain::AcquireStatus::ErrorSurfaceLost) {
      // Surface is lost. This is an unrecoverable error.
      FML_DLOG(INFO) << "Swapchain reported surface was lost.";
      return nullptr;
    }

    if (acquire_result ==
        VulkanSwapchain::AcquireStatus::ErrorSurfaceOutOfDate) {
      // Surface out of date. Recreate the swapchain at the new configuration.
      if (RecreateSwapchain()) {
        // Swapchain was recreated, try surface acquisition again.
        continue;
      } else {
        // Could not recreate the swapchain at the new configuration.
        FML_DLOG(INFO) << "Swapchain reported surface was out of date but "
                          "could not recreate the swapchain at the new "
                          "configuration.";
        valid_ = false;
        return nullptr;
      }
    }

    break;
  }

  FML_DCHECK(false) << "Unhandled VulkanSwapchain::AcquireResult";
  return nullptr;
}

bool VulkanWindow::SwapBuffers() {
  if (!IsValid()) {
    FML_DLOG(INFO) << "Window was invalid.";
    return false;
  }

  return swapchain_->Submit();
}

bool VulkanWindow::RecreateSwapchain() {
  // This way, we always lose our reference to the old swapchain. Even if we
  // cannot create a new one to replace it.
  auto old_swapchain = std::move(swapchain_);

  if (!vk_->IsValid()) {
    return false;
  }

  if (logical_device_ == nullptr || !logical_device_->IsValid()) {
    return false;
  }

  if (surface_ == nullptr || !surface_->IsValid()) {
    return false;
  }

  if (skia_gr_context_ == nullptr) {
    return false;
  }

  auto swapchain = std::make_unique<VulkanSwapchain>(
      *vk_, *logical_device_, *surface_, skia_gr_context_.get(),
      std::move(old_swapchain), logical_device_->GetGraphicsQueueIndex());

  if (!swapchain->IsValid()) {
    return false;
  }

  swapchain_ = std::move(swapchain);
  return true;
}

}  // namespace vulkan
