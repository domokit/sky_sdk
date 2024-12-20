// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/embedder/tests/embedder_test_context_vulkan.h"

#include <memory>
#include <utility>

#include "flutter/fml/logging.h"
#include "flutter/shell/platform/embedder/tests/embedder_test_compositor_vulkan.h"
#include "flutter/testing/test_vulkan_context.h"
#include "flutter/testing/test_vulkan_surface.h"
#include "flutter/vulkan/procs/vulkan_proc_table.h"
#include "flutter/vulkan/vulkan_device.h"
#include "third_party/skia/include/core/SkSurface.h"

namespace flutter::testing {

EmbedderTestContextVulkan::EmbedderTestContextVulkan(std::string assets_path)
    : EmbedderTestContext(std::move(assets_path)), surface_() {
  vulkan_context_ = fml::MakeRefCounted<TestVulkanContext>();
  renderer_config_.type = FlutterRendererType::kVulkan;
  renderer_config_.vulkan = {
      .struct_size = sizeof(FlutterVulkanRendererConfig),
      .version = vulkan_context_->application_->GetAPIVersion(),
      .instance = vulkan_context_->application_->GetInstance(),
      .physical_device = vulkan_context_->device_->GetPhysicalDeviceHandle(),
      .device = vulkan_context_->device_->GetHandle(),
      .queue_family_index = vulkan_context_->device_->GetGraphicsQueueIndex(),
      .queue = vulkan_context_->device_->GetQueueHandle(),
      .get_instance_proc_address_callback =
          EmbedderTestContextVulkan::InstanceProcAddr,
      .get_next_image_callback =
          [](void* context,
             const FlutterFrameInfo* frame_info) -> FlutterVulkanImage {
        VkImage image =
            reinterpret_cast<EmbedderTestContextVulkan*>(context)->GetNextImage(
                {static_cast<int>(frame_info->size.width),
                 static_cast<int>(frame_info->size.height)});
        return {
            .struct_size = sizeof(FlutterVulkanImage),
            .image = reinterpret_cast<uint64_t>(image),
            .format = VK_FORMAT_R8G8B8A8_UNORM,
        };
      },
      .present_image_callback = [](void* context,
                                   const FlutterVulkanImage* image) -> bool {
        return reinterpret_cast<EmbedderTestContextVulkan*>(context)
            ->PresentImage(reinterpret_cast<VkImage>(image->image));
      },
  };
}

EmbedderTestContextVulkan::~EmbedderTestContextVulkan() {}

EmbedderTestContextType EmbedderTestContextVulkan::GetContextType() const {
  return EmbedderTestContextType::kVulkanContext;
}

void EmbedderTestContextVulkan::SetVulkanInstanceProcAddressCallback(
    FlutterVulkanInstanceProcAddressCallback callback) {
  renderer_config_.vulkan.get_instance_proc_address_callback = callback;
}

size_t EmbedderTestContextVulkan::GetSurfacePresentCount() const {
  return present_count_;
}

VkImage EmbedderTestContextVulkan::GetNextImage(const SkISize& size) {
  return surface_->GetImage();
}

bool EmbedderTestContextVulkan::PresentImage(VkImage image) {
  FireRootSurfacePresentCallbackIfPresent(
      [&]() { return surface_->GetSurfaceSnapshot(); });
  present_count_++;
  return true;
}

void* EmbedderTestContextVulkan::InstanceProcAddr(
    void* user_data,
    FlutterVulkanInstanceHandle instance,
    const char* name) {
  auto proc_addr = reinterpret_cast<EmbedderTestContextVulkan*>(user_data)
                       ->vulkan_context_->vk_->GetInstanceProcAddr(
                           reinterpret_cast<VkInstance>(instance), name);
  return reinterpret_cast<void*>(proc_addr);
}

void EmbedderTestContextVulkan::SetSurface(SkISize surface_size) {
  FML_CHECK(surface_size_.isEmpty());
  surface_size_ = surface_size;
  surface_ = TestVulkanSurface::Create(*vulkan_context_, surface_size_);
}

void EmbedderTestContextVulkan::SetupCompositor() {
  FML_CHECK(!compositor_) << "Already set up a compositor in this context.";
  FML_CHECK(surface_)
      << "Set up the Vulkan surface before setting up a compositor.";
  compositor_ = std::make_unique<EmbedderTestCompositorVulkan>(
      surface_size_, vulkan_context_->GetGrDirectContext());
}

}  // namespace flutter::testing
