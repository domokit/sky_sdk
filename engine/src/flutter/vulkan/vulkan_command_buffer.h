// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_VULKAN_VULKAN_COMMAND_BUFFER_H_
#define FLUTTER_VULKAN_VULKAN_COMMAND_BUFFER_H_

#include "flutter/fml/macros.h"
#include "flutter/vulkan/procs/vulkan_handle.h"

namespace vulkan {

class VulkanProcTable;

class VulkanCommandBuffer {
 public:
  VulkanCommandBuffer(const VulkanProcTable& vk,
                      const VulkanHandle<VkDevice>& device,
                      const VulkanHandle<VkCommandPool>& pool);

  ~VulkanCommandBuffer();

  bool IsValid() const;

  VkCommandBuffer Handle() const;

  [[nodiscard]] bool Begin() const;

  [[nodiscard]] bool End() const;

  [[nodiscard]] bool InsertPipelineBarrier(
      VkPipelineStageFlagBits src_stage_flags,
      VkPipelineStageFlagBits dest_stage_flags,
      uint32_t /* mask of VkDependencyFlagBits */ dependency_flags,
      uint32_t memory_barrier_count,
      const VkMemoryBarrier* memory_barriers,
      uint32_t buffer_memory_barrier_count,
      const VkBufferMemoryBarrier* buffer_memory_barriers,
      uint32_t image_memory_barrier_count,
      const VkImageMemoryBarrier* image_memory_barriers) const;

 private:
  const VulkanProcTable& vk_;
  const VulkanHandle<VkDevice>& device_;
  const VulkanHandle<VkCommandPool>& pool_;
  VulkanHandle<VkCommandBuffer> handle_;
  bool valid_;

  FML_DISALLOW_COPY_AND_ASSIGN(VulkanCommandBuffer);
};

}  // namespace vulkan

#endif  // FLUTTER_VULKAN_VULKAN_COMMAND_BUFFER_H_
