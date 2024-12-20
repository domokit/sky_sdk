// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/impeller/playground/playground_test.h"
#include "flutter/impeller/renderer/backend/gles/context_gles.h"
#include "flutter/impeller/renderer/backend/gles/texture_gles.h"
#include "flutter/testing/testing.h"
#include "gtest/gtest.h"
#include "impeller/core/formats.h"
#include "impeller/core/texture_descriptor.h"
#include "impeller/renderer/backend/gles/handle_gles.h"
#include "impeller/renderer/backend/gles/proc_table_gles.h"

namespace impeller::testing {

using TextureGLESTest = PlaygroundTest;
INSTANTIATE_OPENGLES_PLAYGROUND_SUITE(TextureGLESTest);

TEST_P(TextureGLESTest, CanSetSyncFence) {
  ContextGLES& context_gles = ContextGLES::Cast(*GetContext());
  if (!context_gles.GetReactor()
           ->GetProcTable()
           .GetDescription()
           ->GetGlVersion()
           .IsAtLeast(Version{3, 0, 0})) {
    GTEST_SKIP() << "GL Version too low to test sync fence.";
  }

  TextureDescriptor desc;
  desc.storage_mode = StorageMode::kDevicePrivate;
  desc.size = {100, 100};
  desc.format = PixelFormat::kR8G8B8A8UNormInt;

  auto texture = GetContext()->GetResourceAllocator()->CreateTexture(desc);
  ASSERT_TRUE(!!texture);

  EXPECT_TRUE(GetContext()->AddTrackingFence(texture));
  EXPECT_TRUE(context_gles.GetReactor()->React());

  std::optional<HandleGLES> sync_fence =
      TextureGLES::Cast(*texture).GetSyncFence();
  ASSERT_TRUE(sync_fence.has_value());
  if (!sync_fence.has_value()) {
    return;
  }
  EXPECT_EQ(sync_fence.value().GetType(), HandleType::kFence);

  std::optional<GLsync> sync =
      context_gles.GetReactor()->GetGLFence(sync_fence.value());
  ASSERT_TRUE(sync.has_value());
  if (!sync.has_value()) {
    return;
  }

  // Now queue up operation that binds texture to verify that sync fence is
  // waited and removed.

  EXPECT_TRUE(
      context_gles.GetReactor()->AddOperation([&](const ReactorGLES& reactor) {
        return TextureGLES::Cast(*texture).Bind();
      }));

  sync_fence = TextureGLES::Cast(*texture).GetSyncFence();
  ASSERT_FALSE(sync_fence.has_value());
}

}  // namespace impeller::testing
