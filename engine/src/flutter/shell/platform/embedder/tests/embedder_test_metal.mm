// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/embedder/tests/embedder_test.h"

#include "flutter/shell/platform/embedder/tests/embedder_test_context_metal.h"

namespace flutter::testing {

EmbedderTestContext& EmbedderTest::GetMetalContext() {
  if (!metal_context_) {
    metal_context_ = std::make_unique<EmbedderTestContextMetal>(GetFixturesDirectory());
  }
  return *metal_context_.get();
}

}  // namespace flutter::testing
