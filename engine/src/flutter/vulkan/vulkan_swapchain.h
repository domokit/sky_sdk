// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_VULKAN_VULKAN_SWAPCHAIN_H_
#define FLUTTER_VULKAN_VULKAN_SWAPCHAIN_H_

#include <memory>
#include <utility>
#include <vector>

#include "flutter/fml/macros.h"
#include "flutter/vulkan/procs/vulkan_handle.h"
#include "third_party/skia/include/core/SkSize.h"
#include "third_party/skia/include/core/SkSurface.h"

namespace vulkan {

class VulkanProcTable;
class VulkanDevice;
class VulkanSurface;
class VulkanBackbuffer;
class VulkanImage;

class VulkanSwapchain {
 public:
  VulkanSwapchain(const VulkanProcTable& vk,
                  const VulkanDevice& device,
                  const VulkanSurface& surface,
                  GrDirectContext* skia_context,
                  std::unique_ptr<VulkanSwapchain> old_swapchain,
                  uint32_t queue_family_index);

  ~VulkanSwapchain();

  bool IsValid() const;

  enum class AcquireStatus {
    /// A valid SkSurface was acquired successfully from the swapchain.
    Success,
    /// The underlying surface of the swapchain was permanently lost. This is an
    /// unrecoverable error. The entire surface must be recreated along with the
    /// swapchain.
    ErrorSurfaceLost,
    /// The swapchain surface is out-of-date with the underlying surface. The
    /// swapchain must be recreated.
    ErrorSurfaceOutOfDate,
  };
  using AcquireResult = std::pair<AcquireStatus, sk_sp<SkSurface>>;

  /// Acquire an SkSurface from the swapchain for the caller to render into for
  /// later submission via |Submit|. There must not be consecutive calls to
  /// |AcquireFrame| without and interleaving |Submit|.
  AcquireResult AcquireSurface();

  /// Submit a previously acquired. There must not be consecutive calls to
  /// |Submit| without and interleaving |AcquireFrame|.
  [[nodiscard]] bool Submit();

  SkISize GetSize() const;

#if FML_OS_ANDROID
 private:
  const VulkanProcTable& vk;
  const VulkanDevice& device_;
  VkSurfaceCapabilitiesKHR capabilities_;
  VkSurfaceFormatKHR surface_format_;
  VulkanHandle<VkSwapchainKHR> swapchain_;
  std::vector<std::unique_ptr<VulkanBackbuffer>> backbuffers_;
  std::vector<std::unique_ptr<VulkanImage>> images_;
  std::vector<sk_sp<SkSurface>> surfaces_;
  VkPipelineStageFlagBits current_pipeline_stage_;
  size_t current_backbuffer_index_;
  size_t current_image_index_;
  bool valid_;

  std::vector<VkImage> GetImages() const;

  bool CreateSwapchainImages(GrDirectContext* skia_context,
                             SkColorType color_type,
                             const sk_sp<SkColorSpace>& color_space,
                             VkImageUsageFlags usage_flags);

  sk_sp<SkSurface> CreateSkiaSurface(GrDirectContext* skia_context,
                                     VkImage image,
                                     VkImageUsageFlags usage_flags,
                                     const SkISize& size,
                                     SkColorType color_type,
                                     sk_sp<SkColorSpace> color_space) const;

  VulkanBackbuffer* GetNextBackbuffer();
#endif  // FML_OS_ANDROID

  FML_DISALLOW_COPY_AND_ASSIGN(VulkanSwapchain);
};

}  // namespace vulkan

#endif  // FLUTTER_VULKAN_VULKAN_SWAPCHAIN_H_
