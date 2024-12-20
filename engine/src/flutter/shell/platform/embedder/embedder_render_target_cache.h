// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_EMBEDDER_EMBEDDER_RENDER_TARGET_CACHE_H_
#define FLUTTER_SHELL_PLATFORM_EMBEDDER_EMBEDDER_RENDER_TARGET_CACHE_H_

#include <set>
#include <stack>
#include <tuple>
#include <unordered_map>

#include "flutter/fml/macros.h"
#include "flutter/shell/platform/embedder/embedder_external_view.h"

namespace flutter {

//------------------------------------------------------------------------------
/// @brief      A cache used to reference render targets that are owned by the
///             embedder but needed by th engine to render a frame.
///
///             A map of class is managed by EmbedderExternalViewEmbedder. Each
///             instance of this class manages the cached render targets for a
///             view.
///
class EmbedderRenderTargetCache {
 public:
  EmbedderRenderTargetCache();

  ~EmbedderRenderTargetCache();

  std::unique_ptr<EmbedderRenderTarget> GetRenderTarget(
      const EmbedderExternalView::RenderTargetDescriptor& descriptor);

  std::set<std::unique_ptr<EmbedderRenderTarget>>
  ClearAllRenderTargetsInCache();

  void CacheRenderTarget(std::unique_ptr<EmbedderRenderTarget> target);

  size_t GetCachedTargetsCount() const;

 private:
  using CachedRenderTargets = std::unordered_multimap<
      EmbedderExternalView::RenderTargetDescriptor,
      std::unique_ptr<EmbedderRenderTarget>,
      EmbedderExternalView::RenderTargetDescriptor::Hash,
      EmbedderExternalView::RenderTargetDescriptor::Equal>;

  CachedRenderTargets cached_render_targets_;

  FML_DISALLOW_COPY_AND_ASSIGN(EmbedderRenderTargetCache);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_EMBEDDER_EMBEDDER_RENDER_TARGET_CACHE_H_
