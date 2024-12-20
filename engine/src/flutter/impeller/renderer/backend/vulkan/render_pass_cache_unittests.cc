// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/testing/testing.h"
#include "gtest/gtest.h"
#include "impeller/playground/playground_test.h"
#include "impeller/renderer/backend/vulkan/texture_vk.h"
#include "impeller/renderer/render_target.h"

namespace impeller {
namespace testing {

using RendererTest = PlaygroundTest;

TEST_P(RendererTest, CachesRenderPassAndFramebuffer) {
  if (GetBackend() != PlaygroundBackend::kVulkan) {
    GTEST_SKIP() << "Test only applies to Vulkan";
  }

  auto allocator = std::make_shared<RenderTargetAllocator>(
      GetContext()->GetResourceAllocator());

  auto render_target =
      allocator->CreateOffscreenMSAA(*GetContext(), {100, 100}, 1);
  auto resolve_texture = render_target.GetColorAttachment(0).resolve_texture;
  auto& texture_vk = TextureVK::Cast(*resolve_texture);

  EXPECT_EQ(texture_vk.GetCachedFramebuffer(), nullptr);
  EXPECT_EQ(texture_vk.GetCachedRenderPass(), nullptr);

  auto buffer = GetContext()->CreateCommandBuffer();
  auto render_pass = buffer->CreateRenderPass(render_target);

  EXPECT_NE(texture_vk.GetCachedFramebuffer(), nullptr);
  EXPECT_NE(texture_vk.GetCachedRenderPass(), nullptr);

  render_pass->EncodeCommands();
  GetContext()->GetCommandQueue()->Submit({buffer});

  // Can be reused without error.
  auto buffer_2 = GetContext()->CreateCommandBuffer();
  auto render_pass_2 = buffer_2->CreateRenderPass(render_target);

  EXPECT_TRUE(render_pass_2->EncodeCommands());
  EXPECT_TRUE(GetContext()->GetCommandQueue()->Submit({buffer_2}).ok());
}

}  // namespace testing
}  // namespace impeller
