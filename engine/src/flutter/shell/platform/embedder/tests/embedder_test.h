// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_EMBEDDER_TESTS_EMBEDDER_TEST_H_
#define FLUTTER_SHELL_PLATFORM_EMBEDDER_TESTS_EMBEDDER_TEST_H_

#include <map>
#include <memory>

#include "flutter/fml/macros.h"
#include "flutter/shell/platform/embedder/tests/embedder_test_context.h"
#include "flutter/testing/testing.h"
#include "flutter/testing/thread_test.h"
#include "gtest/gtest.h"

namespace flutter::testing {

class EmbedderTestContextGL;
class EmbedderTestContextMetal;
class EmbedderTestContextSoftware;
class EmbedderTestContextVulkan;

class EmbedderTest : public ThreadTest {
 public:
  EmbedderTest();

  std::string GetFixturesDirectory() const;

  template <typename T>
  T& GetEmbedderContext() {
    static_assert(false, "Unsupported test context type");
  }

  template <>
  EmbedderTestContextGL& GetEmbedderContext<EmbedderTestContextGL>() {
    return reinterpret_cast<EmbedderTestContextGL&>(GetGLContext());
  }

  template <>
  EmbedderTestContextMetal& GetEmbedderContext<EmbedderTestContextMetal>() {
    return reinterpret_cast<EmbedderTestContextMetal&>(GetMetalContext());
  }

  template <>
  EmbedderTestContextSoftware&
  GetEmbedderContext<EmbedderTestContextSoftware>() {
    return reinterpret_cast<EmbedderTestContextSoftware&>(GetSoftwareContext());
  }

  template <>
  EmbedderTestContextVulkan& GetEmbedderContext<EmbedderTestContextVulkan>() {
    return reinterpret_cast<EmbedderTestContextVulkan&>(GetVulkanContext());
  }

 protected:
  // We return the base class here and reinterpret_cast in the template
  // specializations because we're using forward declarations rather than
  // including the headers directly, and thus the relationship between the base
  // class and subclasses is unknown to the compiler here. We avoid including
  // the headers directly because the Metal headers include Objective-C types,
  // and thus cannot be included in pure C++ translation units.
  EmbedderTestContext& GetGLContext();
  EmbedderTestContext& GetMetalContext();
  EmbedderTestContext& GetSoftwareContext();
  EmbedderTestContext& GetVulkanContext();

  std::unique_ptr<EmbedderTestContext> gl_context_;
  std::unique_ptr<EmbedderTestContext> metal_context_;
  std::unique_ptr<EmbedderTestContext> software_context_;
  std::unique_ptr<EmbedderTestContext> vulkan_context_;

  FML_DISALLOW_COPY_AND_ASSIGN(EmbedderTest);
};

class EmbedderTestMultiBackend
    : public EmbedderTest,
      public ::testing::WithParamInterface<EmbedderTestContextType> {
 public:
  EmbedderTestContext& GetEmbedderContext(EmbedderTestContextType type);
};

}  // namespace flutter::testing

#endif  // FLUTTER_SHELL_PLATFORM_EMBEDDER_TESTS_EMBEDDER_TEST_H_
