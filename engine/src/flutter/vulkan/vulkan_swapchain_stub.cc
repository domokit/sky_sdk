// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "vulkan_swapchain.h"

namespace vulkan {

VulkanSwapchain::VulkanSwapchain(const VulkanProcTable& p_vk,
                                 const VulkanDevice& device,
                                 const VulkanSurface& surface,
                                 GrDirectContext* skia_context,
                                 std::unique_ptr<VulkanSwapchain> old_swapchain,
                                 uint32_t queue_family_index) {}

VulkanSwapchain::~VulkanSwapchain() = default;

bool VulkanSwapchain::IsValid() const {
  return false;
}

VulkanSwapchain::AcquireResult VulkanSwapchain::AcquireSurface() {
  return {AcquireStatus::ErrorSurfaceLost, nullptr};
}

bool VulkanSwapchain::Submit() {
  return false;
}

SkISize VulkanSwapchain::GetSize() const {
  return SkISize::Make(0, 0);
}

}  // namespace vulkan
