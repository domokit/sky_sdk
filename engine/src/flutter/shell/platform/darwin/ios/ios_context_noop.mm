// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/ios/ios_context_noop.h"
#include "flutter/shell/platform/darwin/ios/rendering_api_selection.h"
#include "ios_context.h"

FLUTTER_ASSERT_ARC

namespace flutter {

IOSContextNoop::IOSContextNoop() = default;

// |IOSContext|
IOSContextNoop::~IOSContextNoop() = default;

// |IOSContext|
sk_sp<GrDirectContext> IOSContextNoop::CreateResourceContext() {
  return nullptr;
}

// |IOSContext|
sk_sp<GrDirectContext> IOSContextNoop::GetMainContext() const {
  return nullptr;
}

// |IOSContext|
IOSRenderingBackend IOSContextNoop::GetBackend() const {
  return IOSRenderingBackend::kImpeller;
}

// |IOSContext|
std::unique_ptr<GLContextResult> IOSContextNoop::MakeCurrent() {
  // This only makes sense for context that need to be bound to a specific thread.
  return std::make_unique<GLContextDefaultResult>(false);
}

// |IOSContext|
std::unique_ptr<Texture> IOSContextNoop::CreateExternalTexture(int64_t texture_id,
                                                               NSObject<FlutterTexture>* texture) {
  // Don't use FML for logging as it will contain engine specific details. This is a user facing
  // message.
  NSLog(@"Flutter: Attempted to composite external texture sources using the noop backend. "
        @"This backend is only used on simulators. This feature is only available on actual "
        @"devices where Metal is used for rendering.");

  // Not supported in this backend.
  return nullptr;
}

}  // namespace flutter
