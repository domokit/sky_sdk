// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/testing/testing.h"  // IWYU pragma: keep
#include "gtest/gtest.h"
#include "impeller/renderer/backend/vulkan/swapchain/khr/khr_swapchain_vk.h"
#include "impeller/renderer/backend/vulkan/test/mock_vulkan.h"
#include "impeller/renderer/backend/vulkan/texture_vk.h"
#include "impeller/renderer/render_pass.h"
#include "vulkan/vulkan_enums.hpp"
#include "vulkan/vulkan_handles.hpp"

namespace impeller {
namespace testing {

vk::UniqueSurfaceKHR CreateSurface(const ContextVK& context) {
#if FML_OS_DARWIN
  impeller::vk::MetalSurfaceCreateInfoEXT createInfo = {};
  auto [result, surface] =
      context.GetInstance().createMetalSurfaceEXTUnique(createInfo);
  FML_DCHECK(result == vk::Result::eSuccess);
  return std::move(surface);
#else
  return {};
#endif  // FML_OS_DARWIN
}

TEST(SwapchainTest, CanCreateSwapchain) {
  auto const context = MockVulkanContextBuilder().Build();

  auto surface = CreateSurface(*context);
  auto swapchain =
      KHRSwapchainVK::Create(context, std::move(surface), ISize{1, 1});

  EXPECT_TRUE(swapchain->IsValid());
}

TEST(SwapchainTest, RecreateSwapchainWhenSizeChanges) {
  auto const context = MockVulkanContextBuilder().Build();

  auto surface = CreateSurface(*context);
  SetSwapchainImageSize(ISize{1, 1});
  auto swapchain =
      KHRSwapchainVK::Create(context, std::move(surface), ISize{1, 1},
                             /*enable_msaa=*/false);
  auto image = swapchain->AcquireNextDrawable();
  auto expected_size = ISize{1, 1};
  EXPECT_EQ(image->GetSize(), expected_size);

  SetSwapchainImageSize(ISize{100, 100});
  swapchain->UpdateSurfaceSize(ISize{100, 100});

  auto image_b = swapchain->AcquireNextDrawable();
  expected_size = ISize{100, 100};
  EXPECT_EQ(image_b->GetSize(), expected_size);
}

TEST(SwapchainTest, CachesRenderPassOnSwapchainImage) {
  auto const context = MockVulkanContextBuilder().Build();

  auto surface = CreateSurface(*context);
  auto swapchain =
      KHRSwapchainVK::Create(context, std::move(surface), ISize{1, 1});

  EXPECT_TRUE(swapchain->IsValid());

  // The mock swapchain will always create 3 images, verify each one starts
  // out with the same MSAA and depth+stencil texture, and no cached
  // framebuffer.
  std::vector<std::shared_ptr<Texture>> msaa_textures;
  std::vector<std::shared_ptr<Texture>> depth_stencil_textures;
  for (auto i = 0u; i < 3u; i++) {
    auto drawable = swapchain->AcquireNextDrawable();
    RenderTarget render_target = drawable->GetRenderTarget();

    auto texture = render_target.GetRenderTargetTexture();
    auto& texture_vk = TextureVK::Cast(*texture);
    EXPECT_EQ(texture_vk.GetCachedFramebuffer(), nullptr);
    EXPECT_EQ(texture_vk.GetCachedRenderPass(), nullptr);

    auto command_buffer = context->CreateCommandBuffer();
    auto render_pass = command_buffer->CreateRenderPass(render_target);
    render_pass->EncodeCommands();

    auto& depth = render_target.GetDepthAttachment();
    depth_stencil_textures.push_back(depth.has_value() ? depth->texture
                                                       : nullptr);
    msaa_textures.push_back(render_target.GetColorAttachment(0).texture);
  }

  for (auto i = 1; i < 3; i++) {
    EXPECT_EQ(msaa_textures[i - 1], msaa_textures[i]);
    EXPECT_EQ(depth_stencil_textures[i - 1], depth_stencil_textures[i]);
  }

  // After each images has been acquired once and the render pass presented,
  // each should have a cached framebuffer and render pass.

  std::vector<SharedHandleVK<vk::Framebuffer>> framebuffers;
  std::vector<SharedHandleVK<vk::RenderPass>> render_passes;
  for (auto i = 0u; i < 3u; i++) {
    auto drawable = swapchain->AcquireNextDrawable();
    RenderTarget render_target = drawable->GetRenderTarget();

    auto texture = render_target.GetRenderTargetTexture();
    auto& texture_vk = TextureVK::Cast(*texture);

    EXPECT_NE(texture_vk.GetCachedFramebuffer(), nullptr);
    EXPECT_NE(texture_vk.GetCachedRenderPass(), nullptr);
    framebuffers.push_back(texture_vk.GetCachedFramebuffer());
    render_passes.push_back(texture_vk.GetCachedRenderPass());
  }

  // Iterate through once more to verify render passes and framebuffers are
  // unchanged.
  for (auto i = 0u; i < 3u; i++) {
    auto drawable = swapchain->AcquireNextDrawable();
    RenderTarget render_target = drawable->GetRenderTarget();

    auto texture = render_target.GetRenderTargetTexture();
    auto& texture_vk = TextureVK::Cast(*texture);

    EXPECT_EQ(texture_vk.GetCachedFramebuffer(), framebuffers[i]);
    EXPECT_EQ(texture_vk.GetCachedRenderPass(), render_passes[i]);
  }
}

}  // namespace testing
}  // namespace impeller
