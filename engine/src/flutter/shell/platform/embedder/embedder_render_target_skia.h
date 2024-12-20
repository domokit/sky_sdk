// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_EMBEDDER_EMBEDDER_RENDER_TARGET_SKIA_H_
#define FLUTTER_SHELL_PLATFORM_EMBEDDER_EMBEDDER_RENDER_TARGET_SKIA_H_

#include "flutter/shell/platform/embedder/embedder_render_target.h"

namespace flutter {

class EmbedderRenderTargetSkia final : public EmbedderRenderTarget {
 public:
  EmbedderRenderTargetSkia(FlutterBackingStore backing_store,
                           sk_sp<SkSurface> render_surface,
                           fml::closure on_release,
                           MakeOrClearCurrentCallback on_make_current,
                           MakeOrClearCurrentCallback on_clear_current);

  // |EmbedderRenderTarget|
  ~EmbedderRenderTargetSkia() override;

  // |EmbedderRenderTarget|
  sk_sp<SkSurface> GetSkiaSurface() const override;

  // |EmbedderRenderTarget|
  impeller::RenderTarget* GetImpellerRenderTarget() const override;

  // |EmbedderRenderTarget|
  std::shared_ptr<impeller::AiksContext> GetAiksContext() const override;

  // |EmbedderRenderTarget|
  SkISize GetRenderTargetSize() const override;

  // |EmbedderRenderTarget|
  SetCurrentResult MaybeMakeCurrent() const override;

  // |EmbedderRenderTarget|
  SetCurrentResult MaybeClearCurrent() const override;

 private:
  sk_sp<SkSurface> render_surface_;

  MakeOrClearCurrentCallback on_make_current_;
  MakeOrClearCurrentCallback on_clear_current_;

  FML_DISALLOW_COPY_AND_ASSIGN(EmbedderRenderTargetSkia);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_EMBEDDER_EMBEDDER_RENDER_TARGET_SKIA_H_
