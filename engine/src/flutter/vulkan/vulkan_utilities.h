// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_VULKAN_VULKAN_UTILITIES_H_
#define FLUTTER_VULKAN_VULKAN_UTILITIES_H_

#include <string>
#include <vector>

#include "flutter/fml/macros.h"
#include "flutter/vulkan/procs/vulkan_handle.h"
#include "flutter/vulkan/procs/vulkan_proc_table.h"

namespace vulkan {

bool ValidationLayerInfoMessagesEnabled();
bool ValidationErrorsFatal();

std::vector<std::string> InstanceLayersToEnable(const VulkanProcTable& vk,
                                                bool enable_validation_layers);

std::vector<std::string> DeviceLayersToEnable(
    const VulkanProcTable& vk,
    const VulkanHandle<VkPhysicalDevice>& physical_device,
    bool enable_validation_layers);

}  // namespace vulkan

#endif  // FLUTTER_VULKAN_VULKAN_UTILITIES_H_
