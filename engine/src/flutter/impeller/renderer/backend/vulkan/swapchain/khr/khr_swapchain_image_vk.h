// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_SWAPCHAIN_KHR_KHR_SWAPCHAIN_IMAGE_VK_H_
#define FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_SWAPCHAIN_KHR_KHR_SWAPCHAIN_IMAGE_VK_H_

#include "impeller/geometry/size.h"
#include "impeller/renderer/backend/vulkan/formats_vk.h"
#include "impeller/renderer/backend/vulkan/texture_source_vk.h"
#include "impeller/renderer/backend/vulkan/vk.h"

namespace impeller {

class KHRSwapchainImageVK final : public TextureSourceVK {
 public:
  KHRSwapchainImageVK(TextureDescriptor desc,
                      const vk::Device& device,
                      vk::Image image);

  // |TextureSourceVK|
  ~KHRSwapchainImageVK() override;

  bool IsValid() const;

  // |TextureSourceVK|
  vk::Image GetImage() const override;

  // |TextureSourceVK|
  vk::ImageView GetImageView() const override;

  // |TextureSourceVK|
  vk::ImageView GetRenderTargetView() const override;

  // |TextureSourceVK|
  bool IsSwapchainImage() const override;

 private:
  vk::Image image_ = VK_NULL_HANDLE;
  vk::UniqueImageView image_view_ = {};
  bool is_valid_ = false;

  KHRSwapchainImageVK(const KHRSwapchainImageVK&) = delete;

  KHRSwapchainImageVK& operator=(const KHRSwapchainImageVK&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_SWAPCHAIN_KHR_KHR_SWAPCHAIN_IMAGE_VK_H_
