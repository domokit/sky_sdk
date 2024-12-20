// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/impeller/playground/playground_test.h"
#include "flutter/impeller/renderer/backend/gles/context_gles.h"
#include "flutter/impeller/renderer/backend/gles/surface_gles.h"
#include "flutter/impeller/renderer/backend/gles/texture_gles.h"
#include "flutter/testing/testing.h"

namespace impeller::testing {

using SurfaceGLESTest = PlaygroundTest;
INSTANTIATE_OPENGLES_PLAYGROUND_SUITE(SurfaceGLESTest);

TEST_P(SurfaceGLESTest, CanWrapNonZeroFBO) {
  const GLuint fbo = 1988;
  auto surface =
      SurfaceGLES::WrapFBO(GetContext(), []() { return true; }, fbo,
                           PixelFormat::kR8G8B8A8UNormInt, {100, 100});
  ASSERT_TRUE(!!surface);
  ASSERT_TRUE(surface->IsValid());
  ASSERT_TRUE(surface->GetRenderTarget().HasColorAttachment(0));
  const auto& texture = TextureGLES::Cast(
      *(surface->GetRenderTarget().GetColorAttachment(0).texture));
  auto wrapped = texture.GetFBO();
  ASSERT_TRUE(wrapped.has_value());
  // NOLINTNEXTLINE(bugprone-unchecked-optional-access)
  ASSERT_EQ(wrapped.value(), fbo);
}

}  // namespace impeller::testing
