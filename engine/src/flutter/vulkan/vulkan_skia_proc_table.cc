// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/vulkan/vulkan_skia_proc_table.h"

namespace vulkan {

skgpu::VulkanGetProc CreateSkiaGetProc(
    const fml::RefPtr<vulkan::VulkanProcTable>& vk) {
  if (!vk || !vk->IsValid()) {
    return nullptr;
  }

  return [vk](const char* proc_name, VkInstance instance, VkDevice device) {
    if (device != VK_NULL_HANDLE) {
      auto result =
          vk->AcquireProc(proc_name, VulkanHandle<VkDevice>{device, nullptr});
      if (result != nullptr) {
        return result;
      }
    }

    return vk->AcquireProc(proc_name,
                           VulkanHandle<VkInstance>{instance, nullptr});
  };
}

}  // namespace vulkan
