// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/backend/vulkan/swapchain/swapchain_transients_vk.h"

#include "flutter/fml/trace_event.h"

namespace impeller {

SwapchainTransientsVK::SwapchainTransientsVK(std::weak_ptr<Context> context,
                                             TextureDescriptor desc,
                                             bool enable_msaa)
    : context_(std::move(context)), desc_(desc), enable_msaa_(enable_msaa) {}

SwapchainTransientsVK::~SwapchainTransientsVK() = default;

const std::shared_ptr<Texture>& SwapchainTransientsVK::GetMSAATexture() {
  if (cached_msaa_texture_) {
    return cached_msaa_texture_;
  }
  cached_msaa_texture_ = CreateMSAATexture();
  return cached_msaa_texture_;
}

const std::shared_ptr<Texture>&
SwapchainTransientsVK::GetDepthStencilTexture() {
  if (cached_depth_stencil_) {
    return cached_depth_stencil_;
  }
  cached_depth_stencil_ = CreateDepthStencilTexture();
  return cached_depth_stencil_;
}

std::shared_ptr<Texture> SwapchainTransientsVK::CreateMSAATexture() const {
  TRACE_EVENT0("impeller", __FUNCTION__);
  if (!enable_msaa_) {
    return nullptr;
  }
  TextureDescriptor msaa_desc;
  msaa_desc.storage_mode = StorageMode::kDeviceTransient;
  msaa_desc.type = TextureType::kTexture2DMultisample;
  msaa_desc.sample_count = SampleCount::kCount4;
  msaa_desc.format = desc_.format;
  msaa_desc.size = desc_.size;
  msaa_desc.usage = TextureUsage::kRenderTarget;

  auto context = context_.lock();
  if (!context) {
    return nullptr;
  }
  auto texture = context->GetResourceAllocator()->CreateTexture(msaa_desc);
  if (!texture) {
    return nullptr;
  }
  texture->SetLabel("SwapchainMSAA");
  return texture;
}

std::shared_ptr<Texture> SwapchainTransientsVK::CreateDepthStencilTexture()
    const {
  TRACE_EVENT0("impeller", __FUNCTION__);
  auto context = context_.lock();
  if (!context) {
    return nullptr;
  }
  TextureDescriptor depth_stencil_desc;
  depth_stencil_desc.storage_mode = StorageMode::kDeviceTransient;
  if (enable_msaa_) {
    depth_stencil_desc.type = TextureType::kTexture2DMultisample;
    depth_stencil_desc.sample_count = SampleCount::kCount4;
  } else {
    depth_stencil_desc.type = TextureType::kTexture2D;
    depth_stencil_desc.sample_count = SampleCount::kCount1;
  }
  depth_stencil_desc.format =
      context->GetCapabilities()->GetDefaultDepthStencilFormat();
  depth_stencil_desc.size = desc_.size;
  depth_stencil_desc.usage = TextureUsage::kRenderTarget;

  auto texture =
      context->GetResourceAllocator()->CreateTexture(depth_stencil_desc);
  if (!texture) {
    return nullptr;
  }
  texture->SetLabel("SwapchainDepthStencil");
  return texture;
}

bool SwapchainTransientsVK::IsMSAAEnabled() const {
  return enable_msaa_;
}

const std::weak_ptr<Context>& SwapchainTransientsVK::GetContext() const {
  return context_;
}

}  // namespace impeller
