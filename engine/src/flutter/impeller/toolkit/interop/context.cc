// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/toolkit/interop/context.h"

#include "impeller/typographer/backends/skia/typographer_context_skia.h"

namespace impeller::interop {

Context::Context(std::shared_ptr<impeller::Context> context)
    : context_(std::move(context), TypographerContextSkia::Make()) {}

Context::~Context() = default;

bool Context::IsValid() const {
  return context_.IsValid();
}

std::shared_ptr<impeller::Context> Context::GetContext() const {
  return context_.GetContext();
}

AiksContext& Context::GetAiksContext() {
  return context_;
}

bool Context::IsBackend(impeller::Context::BackendType type) const {
  if (!IsValid()) {
    return false;
  }
  return GetContext()->GetBackendType() == type;
}

bool Context::IsGL() const {
  return IsBackend(impeller::Context::BackendType::kOpenGLES);
}

bool Context::IsMetal() const {
  return IsBackend(impeller::Context::BackendType::kMetal);
}

bool Context::IsVulkan() const {
  return IsBackend(impeller::Context::BackendType::kVulkan);
}

}  // namespace impeller::interop
