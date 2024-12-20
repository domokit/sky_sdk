// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "vulkan_debug_report.h"

#include <algorithm>
#include <iomanip>
#include <vector>

#include "vulkan_utilities.h"

namespace vulkan {

[[maybe_unused]]
static const VkDebugReportFlagsEXT kVulkanErrorFlags =
    VK_DEBUG_REPORT_WARNING_BIT_EXT |
    VK_DEBUG_REPORT_PERFORMANCE_WARNING_BIT_EXT | VK_DEBUG_REPORT_ERROR_BIT_EXT;

[[maybe_unused]]
static const VkDebugReportFlagsEXT kVulkanInfoFlags =
    VK_DEBUG_REPORT_INFORMATION_BIT_EXT | VK_DEBUG_REPORT_DEBUG_BIT_EXT;

std::string VulkanDebugReport::DebugExtensionName() {
  return VK_EXT_DEBUG_REPORT_EXTENSION_NAME;
}

static const char* VkDebugReportFlagsEXTToString(VkDebugReportFlagsEXT flags) {
  if (flags & VK_DEBUG_REPORT_INFORMATION_BIT_EXT) {
    return "Information";
  } else if (flags & VK_DEBUG_REPORT_WARNING_BIT_EXT) {
    return "Warning";
  } else if (flags & VK_DEBUG_REPORT_PERFORMANCE_WARNING_BIT_EXT) {
    return "Performance Warning";
  } else if (flags & VK_DEBUG_REPORT_ERROR_BIT_EXT) {
    return "Error";
  } else if (flags & VK_DEBUG_REPORT_DEBUG_BIT_EXT) {
    return "Debug";
  }
  return "UNKNOWN";
}

static const char* VkDebugReportObjectTypeEXTToString(
    VkDebugReportObjectTypeEXT type) {
  switch (type) {
    case VK_DEBUG_REPORT_OBJECT_TYPE_UNKNOWN_EXT:
      return "Unknown";
    case VK_DEBUG_REPORT_OBJECT_TYPE_INSTANCE_EXT:
      return "Instance";
    case VK_DEBUG_REPORT_OBJECT_TYPE_PHYSICAL_DEVICE_EXT:
      return "Physical Device";
    case VK_DEBUG_REPORT_OBJECT_TYPE_DEVICE_EXT:
      return "Device";
    case VK_DEBUG_REPORT_OBJECT_TYPE_QUEUE_EXT:
      return "Queue";
    case VK_DEBUG_REPORT_OBJECT_TYPE_SEMAPHORE_EXT:
      return "Semaphore";
    case VK_DEBUG_REPORT_OBJECT_TYPE_COMMAND_BUFFER_EXT:
      return "Command Buffer";
    case VK_DEBUG_REPORT_OBJECT_TYPE_FENCE_EXT:
      return "Fence";
    case VK_DEBUG_REPORT_OBJECT_TYPE_DEVICE_MEMORY_EXT:
      return "Device Memory";
    case VK_DEBUG_REPORT_OBJECT_TYPE_BUFFER_EXT:
      return "Buffer";
    case VK_DEBUG_REPORT_OBJECT_TYPE_IMAGE_EXT:
      return "Image";
    case VK_DEBUG_REPORT_OBJECT_TYPE_EVENT_EXT:
      return "Event";
    case VK_DEBUG_REPORT_OBJECT_TYPE_QUERY_POOL_EXT:
      return "Query Pool";
    case VK_DEBUG_REPORT_OBJECT_TYPE_BUFFER_VIEW_EXT:
      return "Buffer View";
    case VK_DEBUG_REPORT_OBJECT_TYPE_IMAGE_VIEW_EXT:
      return "Image_view";
    case VK_DEBUG_REPORT_OBJECT_TYPE_SHADER_MODULE_EXT:
      return "Shader Module";
    case VK_DEBUG_REPORT_OBJECT_TYPE_PIPELINE_CACHE_EXT:
      return "Pipeline Cache";
    case VK_DEBUG_REPORT_OBJECT_TYPE_PIPELINE_LAYOUT_EXT:
      return "Pipeline Layout";
    case VK_DEBUG_REPORT_OBJECT_TYPE_RENDER_PASS_EXT:
      return "Render Pass";
    case VK_DEBUG_REPORT_OBJECT_TYPE_PIPELINE_EXT:
      return "Pipeline";
    case VK_DEBUG_REPORT_OBJECT_TYPE_DESCRIPTOR_SET_LAYOUT_EXT:
      return "Descriptor Set Layout";
    case VK_DEBUG_REPORT_OBJECT_TYPE_SAMPLER_EXT:
      return "Sampler";
    case VK_DEBUG_REPORT_OBJECT_TYPE_DESCRIPTOR_POOL_EXT:
      return "Descriptor Pool";
    case VK_DEBUG_REPORT_OBJECT_TYPE_DESCRIPTOR_SET_EXT:
      return "Descriptor Set";
    case VK_DEBUG_REPORT_OBJECT_TYPE_FRAMEBUFFER_EXT:
      return "Framebuffer";
    case VK_DEBUG_REPORT_OBJECT_TYPE_COMMAND_POOL_EXT:
      return "Command Pool";
    case VK_DEBUG_REPORT_OBJECT_TYPE_SURFACE_KHR_EXT:
      return "Surface";
    case VK_DEBUG_REPORT_OBJECT_TYPE_SWAPCHAIN_KHR_EXT:
      return "Swapchain";
    case VK_DEBUG_REPORT_OBJECT_TYPE_DEBUG_REPORT_EXT:
      return "Debug";
    default:
      break;
  }

  return "Unknown";
}

static VKAPI_ATTR VkBool32
#ifdef WIN32
    __stdcall
#endif
    OnVulkanDebugReportCallback(VkDebugReportFlagsEXT flags,
                                VkDebugReportObjectTypeEXT object_type,
                                uint64_t object,
                                size_t location,
                                int32_t message_code,
                                const char* layer_prefix,
                                const char* message,
                                void* user_data) {
  std::vector<std::pair<std::string, std::string>> items;

  items.emplace_back("Severity", VkDebugReportFlagsEXTToString(flags));

  items.emplace_back("Object Type",
                     VkDebugReportObjectTypeEXTToString(object_type));

  items.emplace_back("Object Handle", std::to_string(object));

  if (location != 0) {
    items.emplace_back("Location", std::to_string(location));
  }

  if (message_code != 0) {
    items.emplace_back("Message Code", std::to_string(message_code));
  }

  if (layer_prefix != nullptr) {
    items.emplace_back("Layer", layer_prefix);
  }

  if (message != nullptr) {
    items.emplace_back("Message", message);
  }

  size_t padding = 0;

  for (const auto& item : items) {
    padding = std::max(padding, item.first.size());
  }

  padding += 1;

  std::stringstream stream;

  stream << std::endl;

  stream << "--- Vulkan Debug Report  ----------------------------------------";

  stream << std::endl;

  for (const auto& item : items) {
    stream << "| " << std::setw(static_cast<int>(padding)) << item.first
           << std::setw(0) << ": " << item.second << std::endl;
  }

  stream << "-----------------------------------------------------------------";

  if (flags & kVulkanErrorFlags) {
    if (ValidationErrorsFatal()) {
      FML_DCHECK(false) << stream.str();
    } else {
      FML_LOG(ERROR) << stream.str();
    }
  } else {
    FML_LOG(INFO) << stream.str();
  }

  // Returning false tells the layer not to stop when the event occurs, so
  // they see the same behavior with and without validation layers enabled.
  return VK_FALSE;
}

VulkanDebugReport::VulkanDebugReport(
    const VulkanProcTable& p_vk,
    const VulkanHandle<VkInstance>& application)
    : vk_(p_vk), application_(application), valid_(false) {
  if (!vk_.CreateDebugReportCallbackEXT || !vk_.DestroyDebugReportCallbackEXT) {
    return;
  }

  if (!application_) {
    return;
  }

  VkDebugReportFlagsEXT flags = kVulkanErrorFlags;
  if (ValidationLayerInfoMessagesEnabled()) {
    flags |= kVulkanInfoFlags;
  }
  const VkDebugReportCallbackCreateInfoEXT create_info = {
      .sType = VK_STRUCTURE_TYPE_DEBUG_REPORT_CREATE_INFO_EXT,
      .pNext = nullptr,
      .flags = flags,
      .pfnCallback = &vulkan::OnVulkanDebugReportCallback,
      .pUserData = nullptr,
  };

  VkDebugReportCallbackEXT handle = VK_NULL_HANDLE;
  if (VK_CALL_LOG_ERROR(vk_.CreateDebugReportCallbackEXT(
          application_, &create_info, nullptr, &handle)) != VK_SUCCESS) {
    return;
  }

  handle_ = VulkanHandle<VkDebugReportCallbackEXT>{
      handle, [this](VkDebugReportCallbackEXT handle) {
        vk_.DestroyDebugReportCallbackEXT(application_, handle, nullptr);
      }};

  valid_ = true;
}

VulkanDebugReport::~VulkanDebugReport() = default;

bool VulkanDebugReport::IsValid() const {
  return valid_;
}

}  // namespace vulkan
