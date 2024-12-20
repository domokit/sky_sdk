// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/backend/vulkan/swapchain/surface_vk.h"

#include "impeller/core/formats.h"
#include "impeller/renderer/backend/vulkan/texture_vk.h"
#include "impeller/renderer/surface.h"

namespace impeller {

std::unique_ptr<SurfaceVK> SurfaceVK::WrapSwapchainImage(
    const std::shared_ptr<SwapchainTransientsVK>& transients,
    const std::shared_ptr<TextureSourceVK>& swapchain_image,
    SwapCallback swap_callback) {
  if (!transients || !swapchain_image || !swap_callback) {
    return nullptr;
  }

  auto context = transients->GetContext().lock();
  if (!context) {
    return nullptr;
  }

  const auto enable_msaa = transients->IsMSAAEnabled();

  const auto swapchain_tex_desc = swapchain_image->GetTextureDescriptor();

  TextureDescriptor resolve_tex_desc;
  resolve_tex_desc.type = TextureType::kTexture2D;
  resolve_tex_desc.format = swapchain_tex_desc.format;
  resolve_tex_desc.size = swapchain_tex_desc.size;
  resolve_tex_desc.usage = TextureUsage::kRenderTarget;
  resolve_tex_desc.sample_count = SampleCount::kCount1;
  resolve_tex_desc.storage_mode = StorageMode::kDevicePrivate;

  std::shared_ptr<Texture> resolve_tex =
      std::make_shared<TextureVK>(context,         //
                                  swapchain_image  //
      );

  if (!resolve_tex) {
    return nullptr;
  }
  resolve_tex->SetLabel("ImpellerOnscreenResolve");

  ColorAttachment color0;
  color0.clear_color = Color::DarkSlateGray();
  color0.load_action = LoadAction::kClear;
  if (enable_msaa) {
    color0.texture = transients->GetMSAATexture();
    color0.store_action = StoreAction::kMultisampleResolve;
    color0.resolve_texture = resolve_tex;
  } else {
    color0.texture = resolve_tex;
    color0.store_action = StoreAction::kStore;
  }

  RenderTarget render_target_desc;
  render_target_desc.SetColorAttachment(color0, 0u);
  render_target_desc.SetupDepthStencilAttachments(
      /*context=*/*context,                            //
      /*allocator=*/*context->GetResourceAllocator(),  //
      /*size=*/swapchain_tex_desc.size,                //
      /*msaa=*/enable_msaa,                            //
      /*label=*/"Onscreen",                            //
      /*stencil_attachment_config=*/
      RenderTarget::kDefaultStencilAttachmentConfig,                  //
      /*depth_stencil_texture=*/transients->GetDepthStencilTexture()  //
  );

  // The constructor is private. So make_unique may not be used.
  return std::unique_ptr<SurfaceVK>(
      new SurfaceVK(render_target_desc, std::move(swap_callback)));
}

SurfaceVK::SurfaceVK(const RenderTarget& target, SwapCallback swap_callback)
    : Surface(target), swap_callback_(std::move(swap_callback)) {}

SurfaceVK::~SurfaceVK() = default;

bool SurfaceVK::Present() const {
  return swap_callback_ ? swap_callback_() : false;
}

}  // namespace impeller
