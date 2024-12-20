// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/testing/testing.h"  // IWYU pragma: keep
#include "gtest/gtest.h"
#include "impeller/renderer/backend/gles/proc_table_gles.h"
#include "impeller/renderer/backend/gles/test/mock_gles.h"

namespace impeller {
namespace testing {

TEST(CapabilitiesGLES, CanInitializeWithDefaults) {
  auto mock_gles = MockGLES::Init();

  auto capabilities = mock_gles->GetProcTable().GetCapabilities();

  EXPECT_FALSE(capabilities->SupportsOffscreenMSAA());
  EXPECT_FALSE(capabilities->SupportsSSBO());
  EXPECT_TRUE(capabilities->SupportsTextureToTextureBlits());
  EXPECT_FALSE(capabilities->SupportsFramebufferFetch());
  EXPECT_FALSE(capabilities->SupportsCompute());
  EXPECT_FALSE(capabilities->SupportsComputeSubgroups());
  EXPECT_FALSE(capabilities->SupportsReadFromResolve());
  EXPECT_FALSE(capabilities->SupportsDecalSamplerAddressMode());
  EXPECT_FALSE(capabilities->SupportsDeviceTransientTextures());

  EXPECT_EQ(capabilities->GetDefaultColorFormat(),
            PixelFormat::kR8G8B8A8UNormInt);
  EXPECT_EQ(capabilities->GetDefaultStencilFormat(), PixelFormat::kS8UInt);
  EXPECT_EQ(capabilities->GetDefaultDepthStencilFormat(),
            PixelFormat::kD24UnormS8Uint);
}

TEST(CapabilitiesGLES, SupportsDecalSamplerAddressMode) {
  auto const extensions = std::vector<const char*>{
      "GL_KHR_debug",                 //
      "GL_EXT_texture_border_clamp",  //
  };
  auto mock_gles = MockGLES::Init(extensions);
  auto capabilities = mock_gles->GetProcTable().GetCapabilities();
  EXPECT_TRUE(capabilities->SupportsDecalSamplerAddressMode());
}

TEST(CapabilitiesGLES, SupportsDecalSamplerAddressModeNotOES) {
  auto const extensions = std::vector<const char*>{
      "GL_KHR_debug",                 //
      "GL_OES_texture_border_clamp",  //
  };
  auto mock_gles = MockGLES::Init(extensions);
  auto capabilities = mock_gles->GetProcTable().GetCapabilities();
  EXPECT_FALSE(capabilities->SupportsDecalSamplerAddressMode());
}

TEST(CapabilitiesGLES, SupportsFramebufferFetch) {
  auto const extensions = std::vector<const char*>{
      "GL_KHR_debug",                     //
      "GL_EXT_shader_framebuffer_fetch",  //
  };
  auto mock_gles = MockGLES::Init(extensions);
  auto capabilities = mock_gles->GetProcTable().GetCapabilities();
  EXPECT_TRUE(capabilities->SupportsFramebufferFetch());
}

TEST(CapabilitiesGLES, SupportsMSAA) {
  auto const extensions = std::vector<const char*>{
      "GL_EXT_multisampled_render_to_texture",
  };
  auto mock_gles = MockGLES::Init(extensions);
  auto capabilities = mock_gles->GetProcTable().GetCapabilities();
  EXPECT_TRUE(capabilities->SupportsImplicitResolvingMSAA());
  EXPECT_FALSE(capabilities->SupportsOffscreenMSAA());
}

}  // namespace testing
}  // namespace impeller
