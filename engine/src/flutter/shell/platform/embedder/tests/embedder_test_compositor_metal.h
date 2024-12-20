// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_EMBEDDER_TESTS_EMBEDDER_TEST_COMPOSITOR_METAL_H_
#define FLUTTER_SHELL_PLATFORM_EMBEDDER_TESTS_EMBEDDER_TEST_COMPOSITOR_METAL_H_

#include "flutter/fml/macros.h"
#include "flutter/shell/platform/embedder/embedder.h"
#include "flutter/shell/platform/embedder/tests/embedder_test_compositor.h"

namespace flutter {
namespace testing {

class EmbedderTestCompositorMetal : public EmbedderTestCompositor {
 public:
  EmbedderTestCompositorMetal(SkISize surface_size,
                              sk_sp<GrDirectContext> context);

  ~EmbedderTestCompositorMetal() override;

  void SetRenderTargetType(
      EmbedderTestBackingStoreProducer::RenderTargetType type,
      FlutterSoftwarePixelFormat software_pixfmt) override;

 private:
  bool UpdateOffscrenComposition(const FlutterLayer** layers,
                                 size_t layers_count) override;

  FML_DISALLOW_COPY_AND_ASSIGN(EmbedderTestCompositorMetal);
};

}  // namespace testing
}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_EMBEDDER_TESTS_EMBEDDER_TEST_COMPOSITOR_METAL_H_
