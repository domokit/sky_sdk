// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/android/context/android_context.h"

namespace flutter {

AndroidContext::AndroidContext(AndroidRenderingAPI rendering_api)
    : rendering_api_(rendering_api) {}

AndroidContext::~AndroidContext() {
  if (main_context_) {
    main_context_->releaseResourcesAndAbandonContext();
  }
  if (impeller_context_) {
    impeller_context_->Shutdown();
  }
};

AndroidRenderingAPI AndroidContext::RenderingApi() const {
  return rendering_api_;
}

bool AndroidContext::IsValid() const {
  return true;
}

void AndroidContext::SetMainSkiaContext(
    const sk_sp<GrDirectContext>& main_context) {
  main_context_ = main_context;
}

sk_sp<GrDirectContext> AndroidContext::GetMainSkiaContext() const {
  return main_context_;
}

std::shared_ptr<impeller::Context> AndroidContext::GetImpellerContext() const {
  return impeller_context_;
}

void AndroidContext::SetImpellerContext(
    const std::shared_ptr<impeller::Context>& context) {
  impeller_context_ = context;
}

}  // namespace flutter
