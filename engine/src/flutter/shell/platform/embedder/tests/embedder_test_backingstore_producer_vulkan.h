// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_EMBEDDER_TESTS_EMBEDDER_TEST_BACKINGSTORE_PRODUCER_VULKAN_H_
#define FLUTTER_SHELL_PLATFORM_EMBEDDER_TESTS_EMBEDDER_TEST_BACKINGSTORE_PRODUCER_VULKAN_H_

#include "flutter/shell/platform/embedder/tests/embedder_test_backingstore_producer.h"

#include "flutter/testing/test_vulkan_context.h"

namespace flutter::testing {

class EmbedderTestBackingStoreProducerVulkan
    : public EmbedderTestBackingStoreProducer {
 public:
  EmbedderTestBackingStoreProducerVulkan(sk_sp<GrDirectContext> context,
                                         RenderTargetType type);

  virtual ~EmbedderTestBackingStoreProducerVulkan();

  bool Create(const FlutterBackingStoreConfig* config,
              FlutterBackingStore* backing_store_out) override;

  sk_sp<SkSurface> GetSurface(
      const FlutterBackingStore* backing_store) const override;

  sk_sp<SkImage> MakeImageSnapshot(
      const FlutterBackingStore* backing_store) const override;

 private:
  fml::RefPtr<TestVulkanContext> test_vulkan_context_;

  FML_DISALLOW_COPY_AND_ASSIGN(EmbedderTestBackingStoreProducerVulkan);
};

}  // namespace flutter::testing

#endif  // FLUTTER_SHELL_PLATFORM_EMBEDDER_TESTS_EMBEDDER_TEST_BACKINGSTORE_PRODUCER_VULKAN_H_
