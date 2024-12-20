// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/backend/vulkan/texture_source_vk.h"

namespace impeller {

TextureSourceVK::TextureSourceVK(TextureDescriptor desc) : desc_(desc) {}

TextureSourceVK::~TextureSourceVK() = default;

const TextureDescriptor& TextureSourceVK::GetTextureDescriptor() const {
  return desc_;
}

std::shared_ptr<YUVConversionVK> TextureSourceVK::GetYUVConversion() const {
  return nullptr;
}

vk::ImageLayout TextureSourceVK::GetLayout() const {
  ReaderLock lock(layout_mutex_);
  return layout_;
}

vk::ImageLayout TextureSourceVK::SetLayoutWithoutEncoding(
    vk::ImageLayout layout) const {
  WriterLock lock(layout_mutex_);
  const auto old_layout = layout_;
  layout_ = layout;
  return old_layout;
}

fml::Status TextureSourceVK::SetLayout(const BarrierVK& barrier) const {
  const auto old_layout = SetLayoutWithoutEncoding(barrier.new_layout);
  if (barrier.new_layout == old_layout) {
    return {};
  }

  vk::ImageMemoryBarrier image_barrier;
  image_barrier.srcAccessMask = barrier.src_access;
  image_barrier.dstAccessMask = barrier.dst_access;
  image_barrier.oldLayout = old_layout;
  image_barrier.newLayout = barrier.new_layout;
  image_barrier.image = GetImage();
  image_barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
  image_barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
  image_barrier.subresourceRange.aspectMask = ToImageAspectFlags(desc_.format);
  image_barrier.subresourceRange.baseMipLevel = 0u;
  image_barrier.subresourceRange.levelCount = desc_.mip_count;
  image_barrier.subresourceRange.baseArrayLayer = 0u;
  image_barrier.subresourceRange.layerCount = ToArrayLayerCount(desc_.type);

  barrier.cmd_buffer.pipelineBarrier(barrier.src_stage,  // src stage
                                     barrier.dst_stage,  // dst stage
                                     {},                 // dependency flags
                                     nullptr,            // memory barriers
                                     nullptr,            // buffer barriers
                                     image_barrier       // image barriers
  );

  return {};
}

void TextureSourceVK::SetCachedFramebuffer(
    const SharedHandleVK<vk::Framebuffer>& framebuffer) {
  framebuffer_ = framebuffer;
}

void TextureSourceVK::SetCachedRenderPass(
    const SharedHandleVK<vk::RenderPass>& render_pass) {
  render_pass_ = render_pass;
}

SharedHandleVK<vk::Framebuffer> TextureSourceVK::GetCachedFramebuffer() const {
  return framebuffer_;
}

SharedHandleVK<vk::RenderPass> TextureSourceVK::GetCachedRenderPass() const {
  return render_pass_;
}

}  // namespace impeller
