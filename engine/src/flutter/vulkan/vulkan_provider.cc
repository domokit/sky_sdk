// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "vulkan_provider.h"

namespace vulkan {

vulkan::VulkanHandle<VkFence> VulkanProvider::CreateFence() {
  const VkFenceCreateInfo create_info = {
      .sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO,
      .pNext = nullptr,
      .flags = 0,
  };
  VkFence fence;
  if (VK_CALL_LOG_ERROR(vk().CreateFence(vk_device(), &create_info, nullptr,
                                         &fence)) != VK_SUCCESS)
    return vulkan::VulkanHandle<VkFence>();

  return {fence, [this](VkFence fence) {
            vk().DestroyFence(vk_device(), fence, nullptr);
          }};
}

}  // namespace vulkan
