// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/backend/vulkan/swapchain/khr/khr_swapchain_image_vk.h"

namespace impeller {

KHRSwapchainImageVK::KHRSwapchainImageVK(TextureDescriptor desc,
                                         const vk::Device& device,
                                         vk::Image image)
    : TextureSourceVK(desc), image_(image) {
  vk::ImageViewCreateInfo view_info;
  view_info.image = image_;
  view_info.viewType = vk::ImageViewType::e2D;
  view_info.format = ToVKImageFormat(desc.format);
  view_info.subresourceRange.aspectMask = vk::ImageAspectFlagBits::eColor;
  view_info.subresourceRange.baseMipLevel = 0u;
  view_info.subresourceRange.baseArrayLayer = 0u;
  view_info.subresourceRange.levelCount = desc.mip_count;
  view_info.subresourceRange.layerCount = ToArrayLayerCount(desc.type);

  auto [view_result, view] = device.createImageViewUnique(view_info);
  if (view_result != vk::Result::eSuccess) {
    return;
  }

  image_view_ = std::move(view);
  is_valid_ = true;
}

KHRSwapchainImageVK::~KHRSwapchainImageVK() = default;

bool KHRSwapchainImageVK::IsValid() const {
  return is_valid_;
}

// |TextureSourceVK|
vk::Image KHRSwapchainImageVK::GetImage() const {
  return image_;
}

// |TextureSourceVK|
vk::ImageView KHRSwapchainImageVK::GetImageView() const {
  return image_view_.get();
}

// |TextureSourceVK|
vk::ImageView KHRSwapchainImageVK::GetRenderTargetView() const {
  return image_view_.get();
}

// |TextureSourceVK|
bool KHRSwapchainImageVK::IsSwapchainImage() const {
  return true;
}

}  // namespace impeller
