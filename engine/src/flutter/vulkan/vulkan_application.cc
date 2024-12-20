// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "vulkan_application.h"

#include <utility>
#include <vector>

#include "flutter/vulkan/procs/vulkan_proc_table.h"
#include "vulkan_device.h"
#include "vulkan_utilities.h"

namespace vulkan {

// static
VKAPI_ATTR VkBool32 VulkanApplication::DebugReportCallback(
    VkDebugReportFlagsEXT flags,
    VkDebugReportObjectTypeEXT objectType,
    uint64_t object,
    size_t location,
    int32_t messageCode,
    const char* pLayerPrefix,
    const char* pMessage,
    void* pUserData) {
  auto application = static_cast<VulkanApplication*>(pUserData);
  if (application->initialization_logs_enabled_) {
    application->initialization_logs_ += pMessage;
    application->initialization_logs_ += "\n";
  }

  return VK_FALSE;
}

VulkanApplication::VulkanApplication(
    VulkanProcTable& p_vk,  // NOLINT
    const std::string& application_name,
    std::vector<std::string> enabled_extensions,
    uint32_t application_version,
    uint32_t api_version,
    bool enable_validation_layers)
    : valid_(false),
      enable_validation_layers_(enable_validation_layers),
      api_version_(api_version),
      vk_(p_vk) {
  // Check if we want to enable debugging.
  std::vector<VkExtensionProperties> supported_extensions =
      GetSupportedInstanceExtensions(vk_);
  bool enable_instance_debugging =
      enable_validation_layers_ &&
      ExtensionSupported(supported_extensions,
                         VulkanDebugReport::DebugExtensionName());

  // Configure extensions.

  if (enable_instance_debugging) {
    enabled_extensions.emplace_back(VulkanDebugReport::DebugExtensionName());
  }
#if OS_FUCHSIA
  if (ExtensionSupported(supported_extensions,
                         VK_KHR_EXTERNAL_MEMORY_CAPABILITIES_EXTENSION_NAME)) {
    // VK_KHR_get_physical_device_properties2 is a dependency of the memory
    // capabilities extension, so the validation layers require that it be
    // enabled.
    enabled_extensions.emplace_back(
        VK_KHR_GET_PHYSICAL_DEVICE_PROPERTIES_2_EXTENSION_NAME);
    enabled_extensions.emplace_back(
        VK_KHR_EXTERNAL_MEMORY_CAPABILITIES_EXTENSION_NAME);
  }
#endif

  std::vector<const char*> extensions;

  for (size_t i = 0; i < enabled_extensions.size(); i++) {
    extensions.push_back(enabled_extensions[i].c_str());
  }

  // Configure layers.

  const std::vector<std::string> enabled_layers =
      InstanceLayersToEnable(vk_, enable_validation_layers_);

  std::vector<const char*> layers;

  for (size_t i = 0; i < enabled_layers.size(); i++) {
    layers.push_back(enabled_layers[i].c_str());
  }

  // Configure init structs.

  const VkApplicationInfo info = {
      .sType = VK_STRUCTURE_TYPE_APPLICATION_INFO,
      .pNext = nullptr,
      .pApplicationName = application_name.c_str(),
      .applicationVersion = application_version,
      .pEngineName = "FlutterEngine",
      .engineVersion = VK_MAKE_VERSION(1, 0, 0),
      .apiVersion = api_version_,
  };

  const VkDebugReportCallbackCreateInfoEXT debug_report_info = {
      .sType = VK_STRUCTURE_TYPE_DEBUG_REPORT_CALLBACK_CREATE_INFO_EXT,
      .pNext = nullptr,
      .flags = VK_DEBUG_REPORT_INFORMATION_BIT_EXT |
               VK_DEBUG_REPORT_WARNING_BIT_EXT | VK_DEBUG_REPORT_ERROR_BIT_EXT |
               VK_DEBUG_REPORT_DEBUG_BIT_EXT,
      .pfnCallback = &DebugReportCallback,
      .pUserData = this};

  const VkInstanceCreateInfo create_info = {
      .sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO,
      .pNext = ExtensionSupported(supported_extensions,
                                  VK_EXT_DEBUG_REPORT_EXTENSION_NAME)
                   ? &debug_report_info
                   : nullptr,
      .flags = 0,
      .pApplicationInfo = &info,
      .enabledLayerCount = static_cast<uint32_t>(layers.size()),
      .ppEnabledLayerNames = layers.data(),
      .enabledExtensionCount = static_cast<uint32_t>(extensions.size()),
      .ppEnabledExtensionNames = extensions.data(),
  };

  // Perform initialization.

  VkInstance instance = VK_NULL_HANDLE;

  if (VK_CALL_LOG_ERROR(vk_.CreateInstance(&create_info, nullptr, &instance)) !=
      VK_SUCCESS) {
    FML_LOG(ERROR) << "Creating application instance failed with error:\n"
                   << initialization_logs_;
    return;
  }

  // The debug report callback will also be used in vkDestroyInstance, but we
  // don't need its data there.
  initialization_logs_enabled_ = false;
  initialization_logs_.clear();

  // Now that we have an instance, set up instance proc table entries.
  if (!vk_.SetupInstanceProcAddresses(VulkanHandle<VkInstance>(instance))) {
    FML_DLOG(INFO) << "Could not set up instance proc addresses.";
    return;
  }

  instance_ = VulkanHandle<VkInstance>{instance, [this](VkInstance i) {
                                         FML_DLOG(INFO)
                                             << "Destroying Vulkan instance";
                                         vk_.DestroyInstance(i, nullptr);
                                       }};

  if (enable_instance_debugging) {
    auto debug_report = std::make_unique<VulkanDebugReport>(vk_, instance_);
    if (!debug_report->IsValid()) {
      FML_DLOG(INFO) << "Vulkan debugging was enabled but could not be set up "
                        "for this instance.";
    } else {
      debug_report_ = std::move(debug_report);
      FML_DLOG(INFO) << "Debug reporting is enabled.";
    }
  }

  valid_ = true;
}

VulkanApplication::~VulkanApplication() = default;

bool VulkanApplication::IsValid() const {
  return valid_;
}

uint32_t VulkanApplication::GetAPIVersion() const {
  return api_version_;
}

const VulkanHandle<VkInstance>& VulkanApplication::GetInstance() const {
  return instance_;
}

void VulkanApplication::ReleaseInstanceOwnership() {
  instance_.ReleaseOwnership();
}

std::vector<VkPhysicalDevice> VulkanApplication::GetPhysicalDevices() const {
  if (!IsValid()) {
    return {};
  }

  uint32_t device_count = 0;
  if (VK_CALL_LOG_ERROR(vk_.EnumeratePhysicalDevices(instance_, &device_count,
                                                     nullptr)) != VK_SUCCESS) {
    FML_DLOG(INFO) << "Could not enumerate physical device.";
    return {};
  }

  if (device_count == 0) {
    // No available devices.
    FML_DLOG(INFO) << "No physical devices found.";
    return {};
  }

  std::vector<VkPhysicalDevice> physical_devices;

  physical_devices.resize(device_count);

  if (VK_CALL_LOG_ERROR(vk_.EnumeratePhysicalDevices(
          instance_, &device_count, physical_devices.data())) != VK_SUCCESS) {
    FML_DLOG(INFO) << "Could not enumerate physical device.";
    return {};
  }

  return physical_devices;
}

std::unique_ptr<VulkanDevice>
VulkanApplication::AcquireFirstCompatibleLogicalDevice() const {
  for (auto device_handle : GetPhysicalDevices()) {
    auto logical_device = std::make_unique<VulkanDevice>(
        vk_, VulkanHandle<VkPhysicalDevice>(device_handle),
        enable_validation_layers_);
    if (logical_device->IsValid()) {
      return logical_device;
    }
  }
  FML_DLOG(INFO) << "Could not acquire compatible logical device.";
  return nullptr;
}

std::vector<VkExtensionProperties>
VulkanApplication::GetSupportedInstanceExtensions(
    const VulkanProcTable& vk) const {
  if (!vk.EnumerateInstanceExtensionProperties) {
    return std::vector<VkExtensionProperties>();
  }

  uint32_t count = 0;
  if (VK_CALL_LOG_ERROR(vk.EnumerateInstanceExtensionProperties(
          nullptr, &count, nullptr)) != VK_SUCCESS) {
    return std::vector<VkExtensionProperties>();
  }

  if (count == 0) {
    return std::vector<VkExtensionProperties>();
  }

  std::vector<VkExtensionProperties> properties;
  properties.resize(count);
  if (VK_CALL_LOG_ERROR(vk.EnumerateInstanceExtensionProperties(
          nullptr, &count, properties.data())) != VK_SUCCESS) {
    return std::vector<VkExtensionProperties>();
  }

  return properties;
}

bool VulkanApplication::ExtensionSupported(
    const std::vector<VkExtensionProperties>& supported_instance_extensions,
    const std::string& extension_name) {
  uint32_t count = supported_instance_extensions.size();
  for (size_t i = 0; i < count; i++) {
    if (strncmp(supported_instance_extensions[i].extensionName,
                extension_name.c_str(), extension_name.size()) == 0) {
      return true;
    }
  }

  return false;
}

}  // namespace vulkan
