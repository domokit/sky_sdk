// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <memory>

#include "flutter/testing/testing.h"
#include "gtest/gtest.h"
#include "impeller/core/formats.h"
#include "impeller/entity/entity_pass_target.h"
#include "impeller/entity/entity_playground.h"

namespace impeller {
namespace testing {

using EntityPassTargetTest = EntityPlayground;
INSTANTIATE_PLAYGROUND_SUITE(EntityPassTargetTest);

TEST_P(EntityPassTargetTest, SwapWithMSAATexture) {
  if (GetContentContext()
          ->GetDeviceCapabilities()
          .SupportsImplicitResolvingMSAA()) {
    GTEST_SKIP() << "Implicit MSAA is used on this device.";
  }
  auto content_context = GetContentContext();
  auto buffer = content_context->GetContext()->CreateCommandBuffer();
  auto render_target =
      GetContentContext()->GetRenderTargetCache()->CreateOffscreenMSAA(
          *content_context->GetContext(), {100, 100},
          /*mip_count=*/1);

  auto entity_pass_target = EntityPassTarget(render_target, false, false);

  auto color0 = entity_pass_target.GetRenderTarget().GetColorAttachment(0);
  auto msaa_tex = color0.texture;
  auto resolve_tex = color0.resolve_texture;

  FML_DCHECK(content_context);
  entity_pass_target.Flip(*content_context);

  color0 = entity_pass_target.GetRenderTarget().GetColorAttachment(0);

  ASSERT_EQ(msaa_tex, color0.texture);
  ASSERT_NE(resolve_tex, color0.resolve_texture);
}

TEST_P(EntityPassTargetTest, SwapWithMSAAImplicitResolve) {
  auto content_context = GetContentContext();
  auto buffer = content_context->GetContext()->CreateCommandBuffer();
  auto context = content_context->GetContext();
  auto& allocator = *context->GetResourceAllocator();

  // Emulate implicit MSAA resolve by making color resolve and msaa texture the
  // same.
  RenderTarget render_target;
  {
    PixelFormat pixel_format =
        context->GetCapabilities()->GetDefaultColorFormat();

    // Create MSAA color texture.

    TextureDescriptor color0_tex_desc;
    color0_tex_desc.storage_mode = StorageMode::kDevicePrivate;
    color0_tex_desc.type = TextureType::kTexture2DMultisample;
    color0_tex_desc.sample_count = SampleCount::kCount4;
    color0_tex_desc.format = pixel_format;
    color0_tex_desc.size = ISize{100, 100};
    color0_tex_desc.usage = TextureUsage::kRenderTarget;

    auto color0_msaa_tex = allocator.CreateTexture(color0_tex_desc);

    // Color attachment.

    ColorAttachment color0;
    color0.load_action = LoadAction::kDontCare;
    color0.store_action = StoreAction::kStoreAndMultisampleResolve;
    color0.texture = color0_msaa_tex;
    color0.resolve_texture = color0_msaa_tex;

    render_target.SetColorAttachment(color0, 0u);
    render_target.SetStencilAttachment(std::nullopt);
  }

  auto entity_pass_target = EntityPassTarget(render_target, false, true);

  auto color0 = entity_pass_target.GetRenderTarget().GetColorAttachment(0);
  auto msaa_tex = color0.texture;
  auto resolve_tex = color0.resolve_texture;

  ASSERT_EQ(msaa_tex, resolve_tex);

  FML_DCHECK(content_context);
  entity_pass_target.Flip(*content_context);

  color0 = entity_pass_target.GetRenderTarget().GetColorAttachment(0);

  ASSERT_NE(msaa_tex, color0.texture);
  ASSERT_NE(resolve_tex, color0.resolve_texture);
  ASSERT_EQ(color0.texture, color0.resolve_texture);
}

}  // namespace testing
}  // namespace impeller
