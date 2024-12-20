// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/embedder/tests/embedder_test_backingstore_producer.h"

// TODO(zanderso): https://github.com/flutter/flutter/issues/127701
// NOLINTBEGIN(bugprone-unchecked-optional-access)

namespace flutter::testing {

EmbedderTestBackingStoreProducer::EmbedderTestBackingStoreProducer(
    sk_sp<GrDirectContext> context,
    RenderTargetType type)
    : context_(std::move(context)), type_(type) {}

EmbedderTestBackingStoreProducer::~EmbedderTestBackingStoreProducer() = default;

}  // namespace flutter::testing

// NOLINTEND(bugprone-unchecked-optional-access)
