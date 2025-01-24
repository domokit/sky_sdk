// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/testing/testing.h"
#include "impeller/core/formats.h"
#include "impeller/core/texture_descriptor.h"
#include "impeller/playground/playground_test.h"
#include "impeller/renderer/backend/metal/allocator_mtl.h"
#include "impeller/renderer/backend/metal/context_mtl.h"
#include "impeller/renderer/backend/metal/formats_mtl.h"
#include "impeller/renderer/backend/metal/swapchain_transients_mtl.h"
#include "impeller/renderer/backend/metal/texture_mtl.h"
#include "impeller/renderer/capabilities.h"

#include <memory>
#include <thread>

#include "gtest/gtest.h"

namespace impeller {
namespace testing {

using SwapchainTransientsMTLTest = PlaygroundTest;
INSTANTIATE_METAL_PLAYGROUND_SUITE(SwapchainTransientsMTLTest);

TEST_P(SwapchainTransientsMTLTest, CanAllocateSwapchainTextures) {
  const auto& transients = std::make_shared<SwapchainTransientsMTL>(
      GetContext()->GetResourceAllocator());

  transients->SetSizeAndFormat({1, 1}, PixelFormat::kB8G8R8A8UNormInt);

  auto resolve = transients->GetResolveTexture();
  EXPECT_NE(resolve, nullptr);
  EXPECT_NE(transients->GetMSAATexture(), nullptr);
  EXPECT_NE(transients->GetDepthStencilTexture(), nullptr);

  // Texture properties are correct for resolve.
  EXPECT_EQ(resolve->GetTextureDescriptor().size, ISize(1, 1));
  EXPECT_EQ(resolve->GetTextureDescriptor().format,
            PixelFormat::kB8G8R8A8UNormInt);
  EXPECT_EQ(resolve->GetTextureDescriptor().sample_count, SampleCount::kCount1);
  EXPECT_EQ(resolve->GetTextureDescriptor().storage_mode,
            StorageMode::kDevicePrivate);

  // Texture properties are correct for MSAA.
  auto msaa = transients->GetMSAATexture();
  EXPECT_EQ(msaa->GetTextureDescriptor().size, ISize(1, 1));
  EXPECT_EQ(msaa->GetTextureDescriptor().format,
            PixelFormat::kB8G8R8A8UNormInt);
  EXPECT_EQ(msaa->GetTextureDescriptor().sample_count, SampleCount::kCount4);
  EXPECT_EQ(msaa->GetTextureDescriptor().storage_mode,
            StorageMode::kDeviceTransient);

  // Texture properties are correct for Depth+Stencil.
  auto depth_stencil = transients->GetDepthStencilTexture();
  EXPECT_EQ(depth_stencil->GetTextureDescriptor().size, ISize(1, 1));
  EXPECT_EQ(depth_stencil->GetTextureDescriptor().format,
            PixelFormat::kD32FloatS8UInt);
  EXPECT_EQ(depth_stencil->GetTextureDescriptor().sample_count,
            SampleCount::kCount4);
  EXPECT_EQ(depth_stencil->GetTextureDescriptor().storage_mode,
            StorageMode::kDeviceTransient);

  // Textures are cached.
  EXPECT_EQ(transients->GetResolveTexture(), resolve);

  // Texture cache is invalidated when size changes.
  transients->SetSizeAndFormat({2, 2}, PixelFormat::kB8G8R8A8UNormInt);
  EXPECT_NE(resolve, transients->GetResolveTexture());
  resolve = transients->GetResolveTexture();

  // Texture cache is invalidated when pixel format changes.
  transients->SetSizeAndFormat({2, 2}, PixelFormat::kB10G10R10A10XR);
  EXPECT_NE(resolve, transients->GetResolveTexture());
}

}  // namespace testing
}  // namespace impeller
