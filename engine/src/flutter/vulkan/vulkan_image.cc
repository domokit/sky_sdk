// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "vulkan_image.h"

#include "flutter/vulkan/procs/vulkan_proc_table.h"
#include "vulkan_command_buffer.h"

namespace vulkan {

VulkanImage::VulkanImage(VulkanHandle<VkImage> image)
    : handle_(std::move(image)),
      layout_(VK_IMAGE_LAYOUT_UNDEFINED),
      access_flags_(0),
      valid_(false) {
  if (!handle_) {
    return;
  }

  valid_ = true;
}

VulkanImage::~VulkanImage() = default;

bool VulkanImage::IsValid() const {
  return valid_;
}

bool VulkanImage::InsertImageMemoryBarrier(
    const VulkanCommandBuffer& command_buffer,
    VkPipelineStageFlagBits src_pipline_bits,
    VkPipelineStageFlagBits dest_pipline_bits,
    VkAccessFlags dest_access_flags,
    VkImageLayout dest_layout) {
  const VkImageMemoryBarrier image_memory_barrier = {
      .sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER,
      .pNext = nullptr,
      .srcAccessMask = access_flags_,
      .dstAccessMask = dest_access_flags,
      .oldLayout = layout_,
      .newLayout = dest_layout,
      .srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
      .dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
      .image = handle_,
      .subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1},
  };

  bool success = command_buffer.InsertPipelineBarrier(
      src_pipline_bits,      // src_stage_flags
      dest_pipline_bits,     // dest_stage_flags
      0,                     // dependency_flags
      0,                     // memory_barrier_count
      nullptr,               // memory_barriers
      0,                     // buffer_memory_barrier_count
      nullptr,               // buffer_memory_barriers
      1,                     // image_memory_barrier_count
      &image_memory_barrier  // image_memory_barriers
  );

  if (success) {
    access_flags_ = dest_access_flags;
    layout_ = dest_layout;
  }

  return success;
}

}  // namespace vulkan
