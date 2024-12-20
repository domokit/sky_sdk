// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_EMBEDDER_TESTS_EMBEDDER_TEST_BACKINGSTORE_PRODUCER_H_
#define FLUTTER_SHELL_PLATFORM_EMBEDDER_TESTS_EMBEDDER_TEST_BACKINGSTORE_PRODUCER_H_

#include <memory>

#include "flutter/fml/macros.h"
#include "flutter/fml/memory/ref_ptr_internal.h"
#include "flutter/shell/platform/embedder/embedder.h"
#include "third_party/skia/include/core/SkSurface.h"
#include "third_party/skia/include/gpu/ganesh/GrDirectContext.h"

namespace flutter::testing {

class EmbedderTestBackingStoreProducer {
 public:
  enum class RenderTargetType {
    kSoftwareBuffer,
    kSoftwareBuffer2,
    kOpenGLFramebuffer,
    kOpenGLTexture,
    kOpenGLSurface,
    kMetalTexture,
    kVulkanImage,
  };

  EmbedderTestBackingStoreProducer(sk_sp<GrDirectContext> context,
                                   RenderTargetType type);
  virtual ~EmbedderTestBackingStoreProducer();

  virtual bool Create(const FlutterBackingStoreConfig* config,
                      FlutterBackingStore* backing_store_out) = 0;

  virtual sk_sp<SkSurface> GetSurface(
      const FlutterBackingStore* backing_store) const = 0;

  virtual sk_sp<SkImage> MakeImageSnapshot(
      const FlutterBackingStore* backing_store) const = 0;

 protected:
  sk_sp<GrDirectContext> context_;
  RenderTargetType type_;

  FML_DISALLOW_COPY_AND_ASSIGN(EmbedderTestBackingStoreProducer);
};

}  // namespace flutter::testing

#endif  // FLUTTER_SHELL_PLATFORM_EMBEDDER_TESTS_EMBEDDER_TEST_BACKINGSTORE_PRODUCER_H_
