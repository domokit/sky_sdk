// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "vulkan_backbuffer.h"

#include <limits>

#include "flutter/vulkan/procs/vulkan_proc_table.h"
#include "third_party/skia/include/gpu/ganesh/vk/GrVkTypes.h"
#include "vulkan/vulkan.h"

namespace vulkan {

VulkanBackbuffer::VulkanBackbuffer(const VulkanProcTable& p_vk,
                                   const VulkanHandle<VkDevice>& device,
                                   const VulkanHandle<VkCommandPool>& pool)
    : vk_(p_vk),
      device_(device),
      usage_command_buffer_(p_vk, device, pool),
      render_command_buffer_(p_vk, device, pool),
      valid_(false) {
  if (!usage_command_buffer_.IsValid() || !render_command_buffer_.IsValid()) {
    FML_DLOG(INFO) << "Command buffers were not valid.";
    return;
  }

  if (!CreateSemaphores()) {
    FML_DLOG(INFO) << "Could not create semaphores.";
    return;
  }

  if (!CreateFences()) {
    FML_DLOG(INFO) << "Could not create fences.";
    return;
  }

  valid_ = true;
}

VulkanBackbuffer::~VulkanBackbuffer() {
  [[maybe_unused]] auto result = WaitFences();
}

bool VulkanBackbuffer::IsValid() const {
  return valid_;
}

bool VulkanBackbuffer::CreateSemaphores() {
  const VkSemaphoreCreateInfo create_info = {
      .sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO,
      .pNext = nullptr,
      .flags = 0,
  };

  auto semaphore_collect = [this](VkSemaphore semaphore) {
    vk_.DestroySemaphore(device_, semaphore, nullptr);
  };

  for (size_t i = 0; i < semaphores_.size(); i++) {
    VkSemaphore semaphore = VK_NULL_HANDLE;

    if (VK_CALL_LOG_ERROR(vk_.CreateSemaphore(device_, &create_info, nullptr,
                                              &semaphore)) != VK_SUCCESS) {
      return false;
    }

    semaphores_[i] = VulkanHandle<VkSemaphore>{semaphore, semaphore_collect};
  }

  return true;
}

bool VulkanBackbuffer::CreateFences() {
  const VkFenceCreateInfo create_info = {
      .sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO,
      .pNext = nullptr,
      .flags = VK_FENCE_CREATE_SIGNALED_BIT,
  };

  auto fence_collect = [this](VkFence fence) {
    vk_.DestroyFence(device_, fence, nullptr);
  };

  for (size_t i = 0; i < use_fences_.size(); i++) {
    VkFence fence = VK_NULL_HANDLE;

    if (VK_CALL_LOG_ERROR(vk_.CreateFence(device_, &create_info, nullptr,
                                          &fence)) != VK_SUCCESS) {
      return false;
    }

    use_fences_[i] = VulkanHandle<VkFence>{fence, fence_collect};
  }

  return true;
}

bool VulkanBackbuffer::WaitFences() {
  VkFence fences[std::tuple_size_v<decltype(use_fences_)>];

  for (size_t i = 0; i < use_fences_.size(); i++) {
    fences[i] = use_fences_[i];
  }

  return VK_CALL_LOG_ERROR(vk_.WaitForFences(
             device_, static_cast<uint32_t>(use_fences_.size()), fences, true,
             std::numeric_limits<uint64_t>::max())) == VK_SUCCESS;
}

bool VulkanBackbuffer::ResetFences() {
  VkFence fences[std::tuple_size_v<decltype(use_fences_)>];

  for (size_t i = 0; i < use_fences_.size(); i++) {
    fences[i] = use_fences_[i];
  }

  return VK_CALL_LOG_ERROR(vk_.ResetFences(
             device_, static_cast<uint32_t>(use_fences_.size()), fences)) ==
         VK_SUCCESS;
}

const VulkanHandle<VkFence>& VulkanBackbuffer::GetUsageFence() const {
  return use_fences_[0];
}

const VulkanHandle<VkFence>& VulkanBackbuffer::GetRenderFence() const {
  return use_fences_[1];
}

const VulkanHandle<VkSemaphore>& VulkanBackbuffer::GetUsageSemaphore() const {
  return semaphores_[0];
}

const VulkanHandle<VkSemaphore>& VulkanBackbuffer::GetRenderSemaphore() const {
  return semaphores_[1];
}

VulkanCommandBuffer& VulkanBackbuffer::GetUsageCommandBuffer() {
  return usage_command_buffer_;
}

VulkanCommandBuffer& VulkanBackbuffer::GetRenderCommandBuffer() {
  return render_command_buffer_;
}

}  // namespace vulkan
