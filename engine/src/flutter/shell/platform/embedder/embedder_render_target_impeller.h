// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_EMBEDDER_EMBEDDER_RENDER_TARGET_IMPELLER_H_
#define FLUTTER_SHELL_PLATFORM_EMBEDDER_EMBEDDER_RENDER_TARGET_IMPELLER_H_

#include "flutter/shell/platform/embedder/embedder_render_target.h"

namespace flutter {

class EmbedderRenderTargetImpeller final : public EmbedderRenderTarget {
 public:
  EmbedderRenderTargetImpeller(
      FlutterBackingStore backing_store,
      std::shared_ptr<impeller::AiksContext> aiks_context,
      std::unique_ptr<impeller::RenderTarget> impeller_target,
      fml::closure on_release,
      fml::closure framebuffer_destruction_callback);

  // |EmbedderRenderTarget|
  ~EmbedderRenderTargetImpeller() override;

  // |EmbedderRenderTarget|
  sk_sp<SkSurface> GetSkiaSurface() const override;

  // |EmbedderRenderTarget|
  impeller::RenderTarget* GetImpellerRenderTarget() const override;

  // |EmbedderRenderTarget|
  std::shared_ptr<impeller::AiksContext> GetAiksContext() const override;

  // |EmbedderRenderTarget|
  SkISize GetRenderTargetSize() const override;

 private:
  std::shared_ptr<impeller::AiksContext> aiks_context_;
  std::unique_ptr<impeller::RenderTarget> impeller_target_;
  fml::closure framebuffer_destruction_callback_;

  FML_DISALLOW_COPY_AND_ASSIGN(EmbedderRenderTargetImpeller);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_EMBEDDER_EMBEDDER_RENDER_TARGET_IMPELLER_H_
