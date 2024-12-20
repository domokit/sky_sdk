// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_ANDROID_AHB_TEXTURE_SOURCE_VK_H_
#define FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_ANDROID_AHB_TEXTURE_SOURCE_VK_H_

#include "impeller/renderer/backend/vulkan/texture_source_vk.h"
#include "impeller/renderer/backend/vulkan/vk.h"
#include "impeller/renderer/backend/vulkan/yuv_conversion_vk.h"
#include "impeller/toolkit/android/hardware_buffer.h"

#include <android/hardware_buffer.h>
#include <android/hardware_buffer_jni.h>

namespace impeller {

class ContextVK;

//------------------------------------------------------------------------------
/// @brief      A texture source that wraps an instance of AHardwareBuffer.
///
///             The formats and conversions supported by Android Hardware
///             Buffers are a superset of those supported by Impeller (and
///             Vulkan for that matter). Impeller and Vulkan descriptors
///             obtained from the these texture sources are advisory and it
///             usually isn't possible to create copies of images and image
///             views held in these texture sources using the inferred
///             descriptors. The objects are meant to be used directly (either
///             as render targets or sources for sampling), not copied.
///
class AHBTextureSourceVK final : public TextureSourceVK {
 public:
  AHBTextureSourceVK(const std::shared_ptr<Context>& context,
                     struct AHardwareBuffer* hardware_buffer,
                     const AHardwareBuffer_Desc& hardware_buffer_desc);

  AHBTextureSourceVK(const std::shared_ptr<Context>& context,
                     std::unique_ptr<android::HardwareBuffer> backing_store,
                     bool is_swapchain_image);

  // |TextureSourceVK|
  ~AHBTextureSourceVK() override;

  // |TextureSourceVK|
  vk::Image GetImage() const override;

  // |TextureSourceVK|
  vk::ImageView GetImageView() const override;

  // |TextureSourceVK|
  vk::ImageView GetRenderTargetView() const override;

  bool IsValid() const;

  // |TextureSourceVK|
  bool IsSwapchainImage() const override;

  // |TextureSourceVK|
  std::shared_ptr<YUVConversionVK> GetYUVConversion() const override;

  const android::HardwareBuffer* GetBackingStore() const;

 private:
  std::unique_ptr<android::HardwareBuffer> backing_store_;
  vk::UniqueDeviceMemory device_memory_ = {};
  vk::UniqueImage image_ = {};
  vk::UniqueImageView image_view_ = {};
  std::shared_ptr<YUVConversionVK> yuv_conversion_ = {};
  bool needs_yuv_conversion_ = false;
  bool is_swapchain_image_ = false;
  bool is_valid_ = false;

  AHBTextureSourceVK(const AHBTextureSourceVK&) = delete;

  AHBTextureSourceVK& operator=(const AHBTextureSourceVK&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_ANDROID_AHB_TEXTURE_SOURCE_VK_H_
