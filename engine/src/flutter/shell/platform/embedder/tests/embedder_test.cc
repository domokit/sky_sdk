// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/embedder/tests/embedder_test.h"

#include <exception>
#include <utility>

#include "flutter/shell/platform/embedder/tests/embedder_test_context_software.h"

namespace flutter::testing {

EmbedderTest::EmbedderTest() = default;

std::string EmbedderTest::GetFixturesDirectory() const {
  return GetFixturesPath();
}

EmbedderTestContext& EmbedderTest::GetSoftwareContext() {
  if (!software_context_) {
    software_context_ =
        std::make_unique<EmbedderTestContextSoftware>(GetFixturesDirectory());
  }
  return *software_context_.get();
}

#ifndef SHELL_ENABLE_GL
// Fallback implementation.
// See: flutter/shell/platform/embedder/tests/embedder_test_gl.cc.
EmbedderTestContext& EmbedderTest::GetGLContext() {
  FML_LOG(FATAL) << "OpenGL is not supported in this build";
  std::terminate();
}
#endif

#ifndef SHELL_ENABLE_METAL
// Fallback implementation.
// See: flutter/shell/platform/embedder/tests/embedder_test_metal.mm.
EmbedderTestContext& EmbedderTest::GetMetalContext() {
  FML_LOG(FATAL) << "Metal is not supported in this build";
  std::terminate();
}
#endif

#ifndef SHELL_ENABLE_VULKAN
// Fallback implementation.
// See: flutter/shell/platform/embedder/tests/embedder_test_vulkan.cc.
EmbedderTestContext& EmbedderTest::GetVulkanContext() {
  FML_LOG(FATAL) << "Vulkan is not supported in this build";
  std::terminate();
}
#endif

EmbedderTestContext& EmbedderTestMultiBackend::GetEmbedderContext(
    EmbedderTestContextType type) {
  switch (type) {
    case EmbedderTestContextType::kOpenGLContext:
      return GetGLContext();
    case EmbedderTestContextType::kMetalContext:
      return GetMetalContext();
    case EmbedderTestContextType::kSoftwareContext:
      return GetSoftwareContext();
    case EmbedderTestContextType::kVulkanContext:
      return GetVulkanContext();
  }
}

}  // namespace flutter::testing
