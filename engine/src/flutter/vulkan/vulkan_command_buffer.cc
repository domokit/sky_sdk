// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "vulkan_command_buffer.h"

#include "flutter/vulkan/procs/vulkan_proc_table.h"

namespace vulkan {

VulkanCommandBuffer::VulkanCommandBuffer(
    const VulkanProcTable& p_vk,
    const VulkanHandle<VkDevice>& device,
    const VulkanHandle<VkCommandPool>& pool)
    : vk_(p_vk), device_(device), pool_(pool), valid_(false) {
  const VkCommandBufferAllocateInfo allocate_info = {
      .sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO,
      .pNext = nullptr,
      .commandPool = pool_,
      .level = VK_COMMAND_BUFFER_LEVEL_PRIMARY,
      .commandBufferCount = 1,
  };

  VkCommandBuffer buffer = VK_NULL_HANDLE;

  if (VK_CALL_LOG_ERROR(vk_.AllocateCommandBuffers(device_, &allocate_info,
                                                   &buffer)) != VK_SUCCESS) {
    FML_DLOG(INFO) << "Could not allocate command buffers.";
    return;
  }

  auto buffer_collect = [this](VkCommandBuffer buffer) {
    vk_.FreeCommandBuffers(device_, pool_, 1, &buffer);
  };

  handle_ = VulkanHandle<VkCommandBuffer>{buffer, buffer_collect};

  valid_ = true;
}

VulkanCommandBuffer::~VulkanCommandBuffer() = default;

bool VulkanCommandBuffer::IsValid() const {
  return valid_;
}

VkCommandBuffer VulkanCommandBuffer::Handle() const {
  return handle_;
}

bool VulkanCommandBuffer::Begin() const {
  const VkCommandBufferBeginInfo info{
      .sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO,
      .pNext = nullptr,
      .flags = 0,
      .pInheritanceInfo = nullptr,
  };

  return VK_CALL_LOG_ERROR(vk_.BeginCommandBuffer(handle_, &info)) ==
         VK_SUCCESS;
}

bool VulkanCommandBuffer::End() const {
  return VK_CALL_LOG_ERROR(vk_.EndCommandBuffer(handle_)) == VK_SUCCESS;
}

bool VulkanCommandBuffer::InsertPipelineBarrier(
    VkPipelineStageFlagBits src_stage_flags,
    VkPipelineStageFlagBits dest_stage_flags,
    uint32_t /* mask of VkDependencyFlagBits */ dependency_flags,
    uint32_t memory_barrier_count,
    const VkMemoryBarrier* memory_barriers,
    uint32_t buffer_memory_barrier_count,
    const VkBufferMemoryBarrier* buffer_memory_barriers,
    uint32_t image_memory_barrier_count,
    const VkImageMemoryBarrier* image_memory_barriers) const {
  vk_.CmdPipelineBarrier(handle_, src_stage_flags, dest_stage_flags,
                         dependency_flags, memory_barrier_count,
                         memory_barriers, buffer_memory_barrier_count,
                         buffer_memory_barriers, image_memory_barrier_count,
                         image_memory_barriers);
  return true;
}

}  // namespace vulkan
