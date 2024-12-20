// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/vulkan/procs/vulkan_proc_table.h"

#include <utility>

#include "flutter/fml/logging.h"

#define ACQUIRE_PROC(name, context)                 \
  if (!(name = AcquireProc("vk" #name, context))) { \
    return false;                                   \
  }

#define ACQUIRE_PROC_EITHER(name, name2, context)     \
  if (!(name = AcquireProc("vk" #name, context)) &&   \
      !(name2 = AcquireProc("vk" #name2, context))) { \
    return false;                                     \
  }

namespace vulkan {

VulkanProcTable::VulkanProcTable() : VulkanProcTable("libvulkan.so"){};

VulkanProcTable::VulkanProcTable(const char* so_path)
    : handle_(nullptr), acquired_mandatory_proc_addresses_(false) {
  acquired_mandatory_proc_addresses_ = OpenLibraryHandle(so_path) &&
                                       SetupGetInstanceProcAddress() &&
                                       SetupLoaderProcAddresses();
}

VulkanProcTable::VulkanProcTable(
    PFN_vkGetInstanceProcAddr get_instance_proc_addr)
    : handle_(nullptr), acquired_mandatory_proc_addresses_(false) {
  GetInstanceProcAddr = get_instance_proc_addr;
  acquired_mandatory_proc_addresses_ = SetupLoaderProcAddresses();
}

VulkanProcTable::~VulkanProcTable() {
  CloseLibraryHandle();
}

bool VulkanProcTable::HasAcquiredMandatoryProcAddresses() const {
  return acquired_mandatory_proc_addresses_;
}

bool VulkanProcTable::IsValid() const {
  return instance_ && device_;
}

bool VulkanProcTable::AreInstanceProcsSetup() const {
  return instance_;
}

bool VulkanProcTable::AreDeviceProcsSetup() const {
  return device_;
}

bool VulkanProcTable::SetupGetInstanceProcAddress() {
  if (!handle_) {
    return true;
  }

  GetInstanceProcAddr = NativeGetInstanceProcAddr();
  if (!GetInstanceProcAddr) {
    FML_DLOG(WARNING) << "Could not acquire vkGetInstanceProcAddr.";
    return false;
  }

  return true;
}

PFN_vkGetInstanceProcAddr VulkanProcTable::NativeGetInstanceProcAddr() const {
  if (GetInstanceProcAddr) {
    return GetInstanceProcAddr;
  }

  auto instance_proc =
      const_cast<uint8_t*>(handle_->ResolveSymbol("vkGetInstanceProcAddr"));
  return reinterpret_cast<PFN_vkGetInstanceProcAddr>(instance_proc);
}

bool VulkanProcTable::SetupLoaderProcAddresses() {
  VulkanHandle<VkInstance> null_instance(VK_NULL_HANDLE, nullptr);

  ACQUIRE_PROC(CreateInstance, null_instance);
  ACQUIRE_PROC(EnumerateInstanceExtensionProperties, null_instance);
  ACQUIRE_PROC(EnumerateInstanceLayerProperties, null_instance);

  return true;
}

bool VulkanProcTable::SetupInstanceProcAddresses(
    const VulkanHandle<VkInstance>& handle) {
  ACQUIRE_PROC(CreateDevice, handle);
  ACQUIRE_PROC(DestroyDevice, handle);
  ACQUIRE_PROC(DestroyInstance, handle);
  ACQUIRE_PROC(EnumerateDeviceLayerProperties, handle);
  ACQUIRE_PROC(EnumeratePhysicalDevices, handle);
  ACQUIRE_PROC(GetDeviceProcAddr, handle);
  ACQUIRE_PROC(GetPhysicalDeviceFeatures, handle);
  ACQUIRE_PROC(GetPhysicalDeviceQueueFamilyProperties, handle);
  ACQUIRE_PROC(GetPhysicalDeviceProperties, handle);
  ACQUIRE_PROC(GetPhysicalDeviceMemoryProperties, handle);
  ACQUIRE_PROC_EITHER(GetPhysicalDeviceMemoryProperties2,
                      GetPhysicalDeviceMemoryProperties2KHR, handle);

#if FML_OS_ANDROID
  ACQUIRE_PROC(GetPhysicalDeviceSurfaceCapabilitiesKHR, handle);
  ACQUIRE_PROC(GetPhysicalDeviceSurfaceFormatsKHR, handle);
  ACQUIRE_PROC(GetPhysicalDeviceSurfacePresentModesKHR, handle);
  ACQUIRE_PROC(GetPhysicalDeviceSurfaceSupportKHR, handle);
  ACQUIRE_PROC(DestroySurfaceKHR, handle);
  ACQUIRE_PROC(CreateAndroidSurfaceKHR, handle);
#endif  // FML_OS_ANDROID

  // The debug report functions are optional. We don't want proc acquisition to
  // fail here because the optional methods were not present (since ACQUIRE_PROC
  // returns false on failure). Wrap the optional proc acquisitions in an
  // anonymous lambda and invoke it. We don't really care about the result since
  // users of Debug reporting functions check for their presence explicitly.
  [this, &handle]() -> bool {
    ACQUIRE_PROC(CreateDebugReportCallbackEXT, handle);
    ACQUIRE_PROC(DestroyDebugReportCallbackEXT, handle);
    return true;
  }();

  instance_ = VulkanHandle<VkInstance>{handle, nullptr};
  return true;
}

bool VulkanProcTable::SetupDeviceProcAddresses(
    const VulkanHandle<VkDevice>& handle) {
  ACQUIRE_PROC(AllocateCommandBuffers, handle);
  ACQUIRE_PROC(AllocateMemory, handle);
  ACQUIRE_PROC(BeginCommandBuffer, handle);
  ACQUIRE_PROC(BindImageMemory, handle);
  ACQUIRE_PROC(CmdPipelineBarrier, handle);
  ACQUIRE_PROC(CreateCommandPool, handle);
  ACQUIRE_PROC(CreateFence, handle);
  ACQUIRE_PROC(CreateImage, handle);
  ACQUIRE_PROC(CreateSemaphore, handle);
  ACQUIRE_PROC(DestroyCommandPool, handle);
  ACQUIRE_PROC(DestroyFence, handle);
  ACQUIRE_PROC(DestroyImage, handle);
  ACQUIRE_PROC(DestroySemaphore, handle);
  ACQUIRE_PROC(DeviceWaitIdle, handle);
  ACQUIRE_PROC(EndCommandBuffer, handle);
  ACQUIRE_PROC(FreeCommandBuffers, handle);
  ACQUIRE_PROC(FreeMemory, handle);
  ACQUIRE_PROC(GetDeviceQueue, handle);
  ACQUIRE_PROC(GetImageMemoryRequirements, handle);
  ACQUIRE_PROC(QueueSubmit, handle);
  ACQUIRE_PROC(QueueWaitIdle, handle);
  ACQUIRE_PROC(ResetCommandBuffer, handle);
  ACQUIRE_PROC(ResetFences, handle);
  ACQUIRE_PROC(WaitForFences, handle);
  ACQUIRE_PROC(MapMemory, handle);
  ACQUIRE_PROC(UnmapMemory, handle);
  ACQUIRE_PROC(FlushMappedMemoryRanges, handle);
  ACQUIRE_PROC(InvalidateMappedMemoryRanges, handle);
  ACQUIRE_PROC(BindBufferMemory, handle);
  ACQUIRE_PROC(GetBufferMemoryRequirements, handle);
  ACQUIRE_PROC(CreateBuffer, handle);
  ACQUIRE_PROC(DestroyBuffer, handle);
  ACQUIRE_PROC(CmdCopyBuffer, handle);

  ACQUIRE_PROC_EITHER(GetBufferMemoryRequirements2,
                      GetBufferMemoryRequirements2KHR, handle);
  ACQUIRE_PROC_EITHER(GetImageMemoryRequirements2,
                      GetImageMemoryRequirements2KHR, handle);
  ACQUIRE_PROC_EITHER(BindBufferMemory2, BindBufferMemory2KHR, handle);
  ACQUIRE_PROC_EITHER(BindImageMemory2, BindImageMemory2KHR, handle);

#ifndef TEST_VULKAN_PROCS
#if FML_OS_ANDROID
  ACQUIRE_PROC(AcquireNextImageKHR, handle);
  ACQUIRE_PROC(CreateSwapchainKHR, handle);
  ACQUIRE_PROC(DestroySwapchainKHR, handle);
  ACQUIRE_PROC(GetSwapchainImagesKHR, handle);
  ACQUIRE_PROC(QueuePresentKHR, handle);
#endif  // FML_OS_ANDROID
#if OS_FUCHSIA
  ACQUIRE_PROC(ImportSemaphoreZirconHandleFUCHSIA, handle);
  ACQUIRE_PROC(GetSemaphoreZirconHandleFUCHSIA, handle);
  ACQUIRE_PROC(GetMemoryZirconHandleFUCHSIA, handle);
  ACQUIRE_PROC(CreateBufferCollectionFUCHSIA, handle);
  ACQUIRE_PROC(DestroyBufferCollectionFUCHSIA, handle);
  ACQUIRE_PROC(SetBufferCollectionImageConstraintsFUCHSIA, handle);
  ACQUIRE_PROC(GetBufferCollectionPropertiesFUCHSIA, handle);
#endif  // OS_FUCHSIA
#endif  // TEST_VULKAN_PROCS
  device_ = VulkanHandle<VkDevice>{handle, nullptr};
  return true;
}

bool VulkanProcTable::OpenLibraryHandle(const char* path) {
  handle_ = fml::NativeLibrary::Create(path);
  if (!handle_) {
    FML_DLOG(ERROR) << "Could not open Vulkan library handle: " << path;
    return false;
  }
  return true;
}

bool VulkanProcTable::CloseLibraryHandle() {
  handle_ = nullptr;
  return true;
}

PFN_vkVoidFunction VulkanProcTable::AcquireProc(
    const char* proc_name,
    const VulkanHandle<VkInstance>& instance) const {
  if (proc_name == nullptr || !GetInstanceProcAddr) {
    return nullptr;
  }

  // A VK_NULL_HANDLE as the instance is an acceptable parameter.
  return reinterpret_cast<PFN_vkVoidFunction>(
      GetInstanceProcAddr(instance, proc_name));
}

PFN_vkVoidFunction VulkanProcTable::AcquireProc(
    const char* proc_name,
    const VulkanHandle<VkDevice>& device) const {
  if (proc_name == nullptr || !device || !GetDeviceProcAddr) {
    return nullptr;
  }

  return GetDeviceProcAddr(device, proc_name);
}

}  // namespace vulkan
