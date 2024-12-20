// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "vulkan_surface_producer.h"

#include <lib/async/cpp/task.h>
#include <lib/async/default.h>

#include <memory>
#include <string>
#include <vector>

#include "flutter/fml/trace_event.h"
#include "flutter/vulkan/vulkan_skia_proc_table.h"
#include "flutter_vma/flutter_skia_vma.h"

#include "third_party/skia/include/core/SkSurface.h"
#include "third_party/skia/include/gpu/ganesh/GrBackendSemaphore.h"
#include "third_party/skia/include/gpu/ganesh/GrBackendSurface.h"
#include "third_party/skia/include/gpu/ganesh/SkSurfaceGanesh.h"
#include "third_party/skia/include/gpu/ganesh/vk/GrVkBackendSurface.h"
#include "third_party/skia/include/gpu/ganesh/vk/GrVkDirectContext.h"
#include "third_party/skia/include/gpu/ganesh/vk/GrVkTypes.h"
#include "third_party/skia/include/gpu/vk/VulkanBackendContext.h"
#include "third_party/skia/include/gpu/vk/VulkanExtensions.h"

namespace flutter_runner {

namespace {

// Tuning advice:
// If you see the following 3 things happening simultaneously in a trace:
//   * Over budget ("flutter", "GPURasterizer::Draw") durations
//   * Many ("skia", "GrGpu::createTexture") events within the
//     "GPURasterizer::Draw"s
//   * The Skia GPU resource cache is full, as indicated by the
//     "SkiaCacheBytes" field in the ("flutter", "SurfacePool") trace counter
//     (compare it to the bytes value here)
// then you should consider increasing the size of the GPU resource cache.
constexpr size_t kGrCacheMaxByteSize = 1024 * 600 * 12 * 4;

}  // namespace

VulkanSurfaceProducer::VulkanSurfaceProducer() {
  valid_ = Initialize();

  if (!valid_) {
    FML_LOG(FATAL) << "VulkanSurfaceProducer: Initialization failed";
  }
}

VulkanSurfaceProducer::~VulkanSurfaceProducer() {
  // Make sure queue is idle before we start destroying surfaces
  if (valid_) {
    VkResult wait_result = VK_CALL_LOG_ERROR(
        vk_->QueueWaitIdle(logical_device_->GetQueueHandle()));
    FML_DCHECK(wait_result == VK_SUCCESS);
  }
};

bool VulkanSurfaceProducer::Initialize() {
  vk_ = fml::MakeRefCounted<vulkan::VulkanProcTable>();

  std::vector<std::string> extensions = {
      VK_KHR_EXTERNAL_SEMAPHORE_CAPABILITIES_EXTENSION_NAME,
  };

  // On Fuchsia, the validation layers need to be packaged as part of the
  // flutter_runner in order to work. As a result, we can use the presence
  // of the layers to mean that we want the layers enabled.
  application_ = std::make_unique<vulkan::VulkanApplication>(
      *vk_, "FlutterRunner", std::move(extensions), VK_MAKE_VERSION(1, 0, 0),
      VK_MAKE_VERSION(1, 1, 0), true /* enable_validation_layers */);

  if (!application_->IsValid() || !vk_->AreInstanceProcsSetup()) {
    // Make certain the application instance was created and it set up the
    // instance proc table entries.
    FML_LOG(ERROR) << "VulkanSurfaceProducer: Instance proc addresses have not "
                      "been set up.";
    return false;
  }

  // Create the device.

  logical_device_ = application_->AcquireFirstCompatibleLogicalDevice();

  if (logical_device_ == nullptr || !logical_device_->IsValid() ||
      !vk_->AreDeviceProcsSetup()) {
    // Make certain the device was created and it set up the device proc table
    // entries.
    FML_LOG(ERROR)
        << "VulkanSurfaceProducer: Device proc addresses have not been set up.";
    return false;
  }

  if (!vk_->HasAcquiredMandatoryProcAddresses()) {
    FML_LOG(ERROR)
        << "VulkanSurfaceProducer: Failed to acquire mandatory proc addresses.";
    return false;
  }

  if (!vk_->IsValid()) {
    FML_LOG(ERROR) << "VulkanSurfaceProducer: VulkanProcTable invalid";
    return false;
  }

  auto getProc = CreateSkiaGetProc(vk_);

  if (getProc == nullptr) {
    FML_LOG(ERROR) << "VulkanSurfaceProducer: Failed to create skia getProc.";
    return false;
  }

  VkPhysicalDeviceFeatures features;
  if (!logical_device_->GetPhysicalDeviceFeatures(&features)) {
    FML_LOG(ERROR)
        << "VulkanSurfaceProducer: Failed to get physical device features.";

    return false;
  }

  memory_allocator_ = flutter::FlutterSkiaVulkanMemoryAllocator::Make(
      application_->GetAPIVersion(), application_->GetInstance(),
      logical_device_->GetPhysicalDeviceHandle(), logical_device_->GetHandle(),
      vk_, true);

  skgpu::VulkanBackendContext backend_context;
  backend_context.fInstance = application_->GetInstance();
  backend_context.fPhysicalDevice = logical_device_->GetPhysicalDeviceHandle();
  backend_context.fDevice = logical_device_->GetHandle();
  backend_context.fQueue = logical_device_->GetQueueHandle();
  backend_context.fGraphicsQueueIndex =
      logical_device_->GetGraphicsQueueIndex();
  backend_context.fMaxAPIVersion = application_->GetAPIVersion();
  backend_context.fDeviceFeatures = &features;
  backend_context.fGetProc = std::move(getProc);
  backend_context.fMemoryAllocator = memory_allocator_;

  // The memory_requirements_2 extension is required on Fuchsia as the AMD
  // memory allocator used by Skia benefit from it.
  const char* device_extensions[] = {
      VK_KHR_GET_MEMORY_REQUIREMENTS_2_EXTENSION_NAME,
  };
  const int device_extensions_count =
      sizeof(device_extensions) / sizeof(device_extensions[0]);
  skgpu::VulkanExtensions vk_extensions;
  vk_extensions.init(backend_context.fGetProc, backend_context.fInstance,
                     backend_context.fPhysicalDevice, 0, nullptr,
                     device_extensions_count, device_extensions);
  backend_context.fVkExtensions = &vk_extensions;
  GrContextOptions options;
  options.fReduceOpsTaskSplitting = GrContextOptions::Enable::kNo;

  context_ = GrDirectContexts::MakeVulkan(backend_context, options);

  if (context_ == nullptr) {
    FML_LOG(ERROR)
        << "VulkanSurfaceProducer: Failed to create GrDirectContext.";
    return false;
  }

  // Use local limits specified in this file above instead of flutter defaults.
  context_->setResourceCacheLimit(kGrCacheMaxByteSize);

  surface_pool_ = std::make_unique<VulkanSurfacePool>(*this, context_);

  return true;
}

void VulkanSurfaceProducer::SubmitSurfaces(
    std::vector<std::unique_ptr<SurfaceProducerSurface>> surfaces) {
  TRACE_EVENT0("flutter", "VulkanSurfaceProducer::SubmitSurfaces");

  // Do a single flush for all canvases derived from the context.
  {
    TRACE_EVENT0("flutter", "GrDirectContext::flushAndSignalSemaphores");
    context_->flushAndSubmit();
  }

  if (!TransitionSurfacesToExternal(surfaces))
    FML_LOG(ERROR) << "TransitionSurfacesToExternal failed";

  // Submit surface
  for (auto& surface : surfaces) {
    SubmitSurface(std::move(surface));
  }

  // Buffer management.
  surface_pool_->AgeAndCollectOldBuffers();

  // If no further surface production has taken place for 10 frames (TODO:
  // Don't hardcode refresh rate here), then shrink our surface pool to fit.
  constexpr auto kShouldShrinkThreshold = zx::msec(10 * 16.67);
  async::PostDelayedTask(
      async_get_default_dispatcher(),
      [self = weak_factory_.GetWeakPtr(), kShouldShrinkThreshold] {
        if (!self) {
          return;
        }
        auto time_since_last_produce =
            async::Now(async_get_default_dispatcher()) -
            self->last_produce_time_;
        if (time_since_last_produce >= kShouldShrinkThreshold) {
          self->surface_pool_->ShrinkToFit();
        }
      },
      kShouldShrinkThreshold);
}

bool VulkanSurfaceProducer::TransitionSurfacesToExternal(
    const std::vector<std::unique_ptr<SurfaceProducerSurface>>& surfaces) {
  for (auto& surface : surfaces) {
    auto vk_surface = static_cast<VulkanSurface*>(surface.get());
    if (!vk_surface) {
      continue;
    }

    vulkan::VulkanCommandBuffer* command_buffer =
        vk_surface->GetCommandBuffer(logical_device_->GetCommandPool());
    if (!command_buffer->Begin())
      return false;

    GrBackendRenderTarget backendRT = SkSurfaces::GetBackendRenderTarget(
        vk_surface->GetSkiaSurface().get(),
        SkSurfaces::BackendHandleAccess::kFlushRead);
    if (!backendRT.isValid()) {
      return false;
    }
    GrVkImageInfo imageInfo;
    if (!GrBackendRenderTargets::GetVkImageInfo(backendRT, &imageInfo)) {
      return false;
    }

    VkImageMemoryBarrier image_barrier = {
        .sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER,
        .pNext = nullptr,
        .srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT,
        .dstAccessMask = 0,
        .oldLayout = imageInfo.fImageLayout,
    // Understand why this is causing issues on Intel. TODO(fxb/53449)
#if defined(__aarch64__)
        .newLayout = imageInfo.fImageLayout,
#else
        .newLayout = VK_IMAGE_LAYOUT_GENERAL,
#endif
        .srcQueueFamilyIndex = 0,
        .dstQueueFamilyIndex = VK_QUEUE_FAMILY_EXTERNAL_KHR,
        .image = vk_surface->GetVkImage(),
        .subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1}};

    if (!command_buffer->InsertPipelineBarrier(
            VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,
            VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT,
            0,           // dependencyFlags
            0, nullptr,  // memory barriers
            0, nullptr,  // buffer barriers
            1, &image_barrier))
      return false;

    GrBackendRenderTargets::SetVkImageLayout(&backendRT,
                                             image_barrier.newLayout);

    if (!command_buffer->End())
      return false;

    if (!logical_device_->QueueSubmit(
            {}, {}, {vk_surface->GetAcquireVkSemaphore()},
            {command_buffer->Handle()}, vk_surface->GetCommandBufferFence()))
      return false;
  }
  return true;
}

std::unique_ptr<SurfaceProducerSurface> VulkanSurfaceProducer::ProduceSurface(
    const SkISize& size) {
  FML_CHECK(valid_);
  last_produce_time_ = async::Now(async_get_default_dispatcher());
  return surface_pool_->AcquireSurface(size);
}

void VulkanSurfaceProducer::SubmitSurface(
    std::unique_ptr<SurfaceProducerSurface> surface) {
  FML_CHECK(valid_);
  surface_pool_->SubmitSurface(std::move(surface));
}

std::unique_ptr<SurfaceProducerSurface>
VulkanSurfaceProducer::ProduceOffscreenSurface(const SkISize& size) {
  return surface_pool_->CreateSurface(size);
}

}  // namespace flutter_runner
