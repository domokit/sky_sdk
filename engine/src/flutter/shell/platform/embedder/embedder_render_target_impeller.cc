// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/embedder/embedder_render_target_impeller.h"

#include "flutter/fml/logging.h"
#include "flutter/impeller/renderer/render_target.h"

namespace flutter {

EmbedderRenderTargetImpeller::EmbedderRenderTargetImpeller(
    FlutterBackingStore backing_store,
    std::shared_ptr<impeller::AiksContext> aiks_context,
    std::unique_ptr<impeller::RenderTarget> impeller_target,
    fml::closure on_release,
    fml::closure framebuffer_destruction_callback)
    : EmbedderRenderTarget(backing_store, std::move(on_release)),
      aiks_context_(std::move(aiks_context)),
      impeller_target_(std::move(impeller_target)),
      framebuffer_destruction_callback_(
          std::move(framebuffer_destruction_callback)) {
  FML_DCHECK(aiks_context_);
  FML_DCHECK(impeller_target_);
}

EmbedderRenderTargetImpeller::~EmbedderRenderTargetImpeller() {
  if (framebuffer_destruction_callback_) {
    framebuffer_destruction_callback_();
  }
}

sk_sp<SkSurface> EmbedderRenderTargetImpeller::GetSkiaSurface() const {
  return nullptr;
}

impeller::RenderTarget* EmbedderRenderTargetImpeller::GetImpellerRenderTarget()
    const {
  return impeller_target_.get();
}

std::shared_ptr<impeller::AiksContext>
EmbedderRenderTargetImpeller::GetAiksContext() const {
  return aiks_context_;
}

SkISize EmbedderRenderTargetImpeller::GetRenderTargetSize() const {
  auto size = impeller_target_->GetRenderTargetSize();
  return SkISize::Make(size.width, size.height);
}

}  // namespace flutter
