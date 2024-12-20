// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FLUTTER_VMA_FLUTTER_SKIA_VMA_H_
#define FLUTTER_FLUTTER_VMA_FLUTTER_SKIA_VMA_H_

#include "flutter/flutter_vma/flutter_vma.h"

#include "flutter/fml/memory/ref_ptr.h"
#include "flutter/vulkan/procs/vulkan_proc_table.h"
#include "third_party/skia/include/gpu/vk/VulkanMemoryAllocator.h"
#include "third_party/skia/include/gpu/vk/VulkanTypes.h"

namespace flutter {

class FlutterSkiaVulkanMemoryAllocator : public skgpu::VulkanMemoryAllocator {
 public:
  static sk_sp<VulkanMemoryAllocator> Make(
      uint32_t vulkan_api_version,
      VkInstance instance,
      VkPhysicalDevice physicalDevice,
      VkDevice device,
      const fml::RefPtr<vulkan::VulkanProcTable>& vk,
      bool mustUseCoherentHostVisibleMemory);

  ~FlutterSkiaVulkanMemoryAllocator() override;

  VkResult allocateImageMemory(VkImage image,
                               uint32_t allocationPropertyFlags,
                               skgpu::VulkanBackendMemory*) override;

  VkResult allocateBufferMemory(VkBuffer buffer,
                                BufferUsage usage,
                                uint32_t allocationPropertyFlags,
                                skgpu::VulkanBackendMemory*) override;

  void freeMemory(const skgpu::VulkanBackendMemory&) override;

  void getAllocInfo(const skgpu::VulkanBackendMemory&,
                    skgpu::VulkanAlloc*) const override;

  VkResult mapMemory(const skgpu::VulkanBackendMemory&, void** data) override;
  void unmapMemory(const skgpu::VulkanBackendMemory&) override;

  VkResult flushMemory(const skgpu::VulkanBackendMemory&,
                       VkDeviceSize offset,
                       VkDeviceSize size) override;
  VkResult invalidateMemory(const skgpu::VulkanBackendMemory&,
                            VkDeviceSize offset,
                            VkDeviceSize size) override;

  std::pair<uint64_t, uint64_t> totalAllocatedAndUsedMemory() const override;

 private:
  FlutterSkiaVulkanMemoryAllocator(
      fml::RefPtr<vulkan::VulkanProcTable> vk_proc_table,
      VmaAllocator allocator,
      bool mustUseCoherentHostVisibleMemory);

  fml::RefPtr<vulkan::VulkanProcTable> vk_proc_table_;
  VmaAllocator allocator_;

  // For host visible allocations do we require they are coherent or not. All
  // devices are required to support a host visible and coherent memory type.
  // This is used to work around bugs for devices that don't handle non coherent
  // memory correctly.
  bool must_use_coherent_host_visible_memory_;
};

}  // namespace flutter

#endif  // FLUTTER_FLUTTER_VMA_FLUTTER_SKIA_VMA_H_
