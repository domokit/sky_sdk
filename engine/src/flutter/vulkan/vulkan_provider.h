// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_VULKAN_VULKAN_PROVIDER_H_
#define FLUTTER_VULKAN_VULKAN_PROVIDER_H_

#include "flutter/vulkan/procs/vulkan_handle.h"
#include "flutter/vulkan/procs/vulkan_proc_table.h"

namespace vulkan {

class VulkanProvider {
 public:
  virtual const vulkan::VulkanProcTable& vk() = 0;
  virtual const vulkan::VulkanHandle<VkDevice>& vk_device() = 0;

  vulkan::VulkanHandle<VkFence> CreateFence() {
    const VkFenceCreateInfo create_info = {
        .sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO,
        .pNext = nullptr,
        .flags = 0,
    };
    VkFence fence;
    if (VK_CALL_LOG_ERROR(vk().CreateFence(vk_device(), &create_info, nullptr,
                                           &fence)) != VK_SUCCESS)
      return vulkan::VulkanHandle<VkFence>();

    return VulkanHandle<VkFence>{fence, [this](VkFence fence) {
                                   vk().DestroyFence(vk_device(), fence,
                                                     nullptr);
                                 }};
  }
};

}  // namespace vulkan

#endif  // FLUTTER_VULKAN_VULKAN_PROVIDER_H_
