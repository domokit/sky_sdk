// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_TEXTURE_SOURCE_VK_H_
#define FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_TEXTURE_SOURCE_VK_H_

#include "flutter/fml/status.h"
#include "impeller/base/thread.h"
#include "impeller/core/texture_descriptor.h"
#include "impeller/renderer/backend/vulkan/barrier_vk.h"
#include "impeller/renderer/backend/vulkan/formats_vk.h"
#include "impeller/renderer/backend/vulkan/shared_object_vk.h"
#include "impeller/renderer/backend/vulkan/vk.h"
#include "impeller/renderer/backend/vulkan/yuv_conversion_vk.h"
#include "vulkan/vulkan_handles.hpp"

namespace impeller {

//------------------------------------------------------------------------------
/// @brief      Abstract base class that represents a vkImage and an
///             vkImageView.
///
///             This is intended to be used with an impeller::TextureVK. Example
///             implementations represent swapchain images, uploaded textures,
///             Android Hardware Buffer backend textures, etc...
///
class TextureSourceVK {
 public:
  virtual ~TextureSourceVK();

  //----------------------------------------------------------------------------
  /// @brief      Gets the texture descriptor for this image source.
  ///
  /// @warning    Texture descriptors from texture sources whose capabilities
  ///             are a superset of those that can be expressed with Vulkan
  ///             (like Android Hardware Buffer) are inferred. Stuff like size,
  ///             mip-counts, types is reliable. So use these descriptors as
  ///             advisory. Creating copies of texture sources from these
  ///             descriptors is usually not possible and  depends on the
  ///             allocator used.
  ///
  /// @return     The texture descriptor.
  ///
  const TextureDescriptor& GetTextureDescriptor() const;

  //----------------------------------------------------------------------------
  /// @brief      Get the image handle for this texture source.
  ///
  /// @return     The image.
  ///
  virtual vk::Image GetImage() const = 0;

  //----------------------------------------------------------------------------
  /// @brief      Retrieve the image view used for sampling/blitting/compute
  ///             with this texture source.
  ///
  /// @return     The image view.
  ///
  virtual vk::ImageView GetImageView() const = 0;

  //----------------------------------------------------------------------------
  /// @brief      Retrieve the image view used for render target attachments
  ///             with this texture source.
  ///
  ///             ImageViews used as render target attachments cannot have any
  ///             mip levels. In cases where we want to generate mipmaps with
  ///             the result of this texture, we need to create multiple image
  ///             views.
  ///
  /// @return     The render target view.
  ///
  virtual vk::ImageView GetRenderTargetView() const = 0;

  //----------------------------------------------------------------------------
  /// @brief      Encodes the layout transition `barrier` to
  ///             `barrier.cmd_buffer` for the image.
  ///
  ///             The transition is from the layout stored via
  ///             `SetLayoutWithoutEncoding` to `barrier.new_layout`.
  ///
  /// @param[in]  barrier  The barrier.
  ///
  /// @return     If the layout transition was successfully made.
  ///
  fml::Status SetLayout(const BarrierVK& barrier) const;

  //----------------------------------------------------------------------------
  /// @brief      Store the layout of the image.
  ///
  ///             This just is bookkeeping on the CPU, to actually set the
  ///             layout use `SetLayout`.
  ///
  /// @param[in]  layout  The new layout.
  ///
  /// @return     The old layout.
  ///
  vk::ImageLayout SetLayoutWithoutEncoding(vk::ImageLayout layout) const;

  //----------------------------------------------------------------------------
  /// @brief      Get the last layout assigned to the TextureSourceVK.
  ///
  ///             This value is synchronized with the GPU via SetLayout so it
  ///             may not reflect the actual layout.
  ///
  /// @return     The last known layout of the texture source.
  ///
  vk::ImageLayout GetLayout() const;

  //----------------------------------------------------------------------------
  /// @brief      When sampling from textures whose formats are not known to
  ///             Vulkan, a custom conversion is necessary to setup custom
  ///             samplers. This accessor provides this conversion if one is
  ///             present. Most texture source have none.
  ///
  /// @return     The sampler conversion.
  ///
  virtual std::shared_ptr<YUVConversionVK> GetYUVConversion() const;

  //----------------------------------------------------------------------------
  /// @brief      Determines if swapchain image. That is, an image used as the
  ///             root render target.
  ///
  /// @return     Whether or not this is a swapchain image.
  ///
  virtual bool IsSwapchainImage() const = 0;

  // These methods should only be used by render_pass_vk.h

  /// Store the last framebuffer object used with this texture.
  ///
  /// This field is only set if this texture is used as the resolve texture
  /// of a render pass. By construction, this framebuffer should be compatible
  /// with any future render passes.
  void SetCachedFramebuffer(const SharedHandleVK<vk::Framebuffer>& framebuffer);

  /// Store the last render pass object used with this texture.
  ///
  /// This field is only set if this texture is used as the resolve texture
  /// of a render pass. By construction, this framebuffer should be compatible
  /// with any future render passes.
  void SetCachedRenderPass(const SharedHandleVK<vk::RenderPass>& render_pass);

  /// Retrieve the last framebuffer object used with this texture.
  ///
  /// May be nullptr if no previous framebuffer existed.
  SharedHandleVK<vk::Framebuffer> GetCachedFramebuffer() const;

  /// Retrieve the last render pass object used with this texture.
  ///
  /// May be nullptr if no previous render pass existed.
  SharedHandleVK<vk::RenderPass> GetCachedRenderPass() const;

 protected:
  const TextureDescriptor desc_;

  explicit TextureSourceVK(TextureDescriptor desc);

 private:
  SharedHandleVK<vk::Framebuffer> framebuffer_;
  SharedHandleVK<vk::RenderPass> render_pass_;
  mutable RWMutex layout_mutex_;
  mutable vk::ImageLayout layout_ IPLR_GUARDED_BY(layout_mutex_) =
      vk::ImageLayout::eUndefined;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_TEXTURE_SOURCE_VK_H_
