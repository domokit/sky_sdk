// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_VULKAN_VULKAN_BACKBUFFER_H_
#define FLUTTER_VULKAN_VULKAN_BACKBUFFER_H_

#include <array>

#include "flutter/fml/macros.h"
#include "flutter/vulkan/procs/vulkan_handle.h"
#include "third_party/skia/include/core/SkSize.h"
#include "third_party/skia/include/core/SkSurface.h"
#include "vulkan_command_buffer.h"

namespace vulkan {

class VulkanBackbuffer {
 public:
  VulkanBackbuffer(const VulkanProcTable& vk,
                   const VulkanHandle<VkDevice>& device,
                   const VulkanHandle<VkCommandPool>& pool);

  ~VulkanBackbuffer();

  bool IsValid() const;

  [[nodiscard]] bool WaitFences();

  [[nodiscard]] bool ResetFences();

  const VulkanHandle<VkFence>& GetUsageFence() const;

  const VulkanHandle<VkFence>& GetRenderFence() const;

  const VulkanHandle<VkSemaphore>& GetUsageSemaphore() const;

  const VulkanHandle<VkSemaphore>& GetRenderSemaphore() const;

  VulkanCommandBuffer& GetUsageCommandBuffer();

  VulkanCommandBuffer& GetRenderCommandBuffer();

 private:
  const VulkanProcTable& vk_;
  const VulkanHandle<VkDevice>& device_;
  std::array<VulkanHandle<VkSemaphore>, 2> semaphores_;
  std::array<VulkanHandle<VkFence>, 2> use_fences_;
  VulkanCommandBuffer usage_command_buffer_;
  VulkanCommandBuffer render_command_buffer_;
  bool valid_;

  bool CreateSemaphores();

  bool CreateFences();

  FML_DISALLOW_COPY_AND_ASSIGN(VulkanBackbuffer);
};

}  // namespace vulkan

#endif  // FLUTTER_VULKAN_VULKAN_BACKBUFFER_H_
