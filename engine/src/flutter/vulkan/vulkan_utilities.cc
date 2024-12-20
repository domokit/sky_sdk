// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "vulkan_utilities.h"

#include <algorithm>
#include <unordered_set>

#include "flutter/fml/build_config.h"

namespace vulkan {

// Whether to show Vulkan validation layer info messages in addition
// to the error messages.
bool ValidationLayerInfoMessagesEnabled() {
  return false;
}

bool ValidationErrorsFatal() {
#if OS_FUCHSIA
  return false;
#endif
  return true;
}

static std::vector<std::string> InstanceOrDeviceLayersToEnable(
    const VulkanProcTable& vk,
    VkPhysicalDevice physical_device,
    bool enable_validation_layers) {
  if (!enable_validation_layers) {
    return {};
  }

  // NOTE: The loader is sensitive to the ordering here. Please do not rearrange
  // this list.
#if OS_FUCHSIA
  // The other layers in the Fuchsia SDK seem to have a bug right now causing
  // crashes, so it is only recommended that we use VK_LAYER_KHRONOS_validation
  // until we have a confirmation that they are fixed.
  const std::vector<std::string> candidates = {"VK_LAYER_KHRONOS_validation"};
#else
  const std::vector<std::string> candidates = {
      "VK_LAYER_GOOGLE_threading",      "VK_LAYER_LUNARG_parameter_validation",
      "VK_LAYER_LUNARG_object_tracker", "VK_LAYER_LUNARG_core_validation",
      "VK_LAYER_LUNARG_device_limits",  "VK_LAYER_LUNARG_image",
      "VK_LAYER_LUNARG_swapchain",      "VK_LAYER_GOOGLE_unique_objects"};
#endif

  uint32_t count = 0;

  if (physical_device == VK_NULL_HANDLE) {
    if (VK_CALL_LOG_ERROR(vk.EnumerateInstanceLayerProperties(
            &count, nullptr)) != VK_SUCCESS) {
      return {};
    }
  } else {
    if (VK_CALL_LOG_ERROR(vk.EnumerateDeviceLayerProperties(
            physical_device, &count, nullptr)) != VK_SUCCESS) {
      return {};
    }
  }

  std::vector<VkLayerProperties> properties;
  properties.resize(count);

  if (physical_device == VK_NULL_HANDLE) {
    if (VK_CALL_LOG_ERROR(vk.EnumerateInstanceLayerProperties(
            &count, properties.data())) != VK_SUCCESS) {
      return {};
    }
  } else {
    if (VK_CALL_LOG_ERROR(vk.EnumerateDeviceLayerProperties(
            physical_device, &count, properties.data())) != VK_SUCCESS) {
      return {};
    }
  }

  std::unordered_set<std::string> available_extensions;

  for (size_t i = 0; i < count; i++) {
    available_extensions.emplace(properties[i].layerName);
  }

  std::vector<std::string> available_candidates;

  for (const auto& candidate : candidates) {
    auto found = available_extensions.find(candidate);
    if (found != available_extensions.end()) {
      available_candidates.emplace_back(candidate);
    }
  }

  return available_candidates;
}

std::vector<std::string> InstanceLayersToEnable(const VulkanProcTable& vk,
                                                bool enable_validation_layers) {
  return InstanceOrDeviceLayersToEnable(vk, VK_NULL_HANDLE,
                                        enable_validation_layers);
}

std::vector<std::string> DeviceLayersToEnable(
    const VulkanProcTable& vk,
    const VulkanHandle<VkPhysicalDevice>& physical_device,
    bool enable_validation_layers) {
  if (!physical_device) {
    return {};
  }

  return InstanceOrDeviceLayersToEnable(vk, physical_device,
                                        enable_validation_layers);
}

}  // namespace vulkan
