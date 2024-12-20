// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/flow/layers/cacheable_layer.h"

namespace flutter {

AutoCache::AutoCache(RasterCacheItem* raster_cache_item,
                     PrerollContext* context,
                     const DlMatrix& matrix)
    : raster_cache_item_(raster_cache_item),
      context_(context),
      matrix_(matrix) {
#if !SLIMPELLER
  if (IsCacheEnabled()) {
    raster_cache_item->PrerollSetup(context, matrix);
  }
#endif  //  !SLIMPELLER
}

bool AutoCache::IsCacheEnabled() {
#if SLIMPELLER
  return false;
#else   // SLIMPELLER
  return raster_cache_item_ && context_ && context_->raster_cache;
#endif  //  SLIMPELLER
}

AutoCache::~AutoCache() {
#if !SLIMPELLER
  if (IsCacheEnabled()) {
    raster_cache_item_->PrerollFinalize(context_, matrix_);
  }
#endif  //  !SLIMPELLER
}

CacheableContainerLayer::CacheableContainerLayer(int layer_cached_threshold,
                                                 bool can_cache_children) {
#if !SLIMPELLER
  layer_raster_cache_item_ = LayerRasterCacheItem::Make(
      this, layer_cached_threshold, can_cache_children);
#endif  //  !SLIMPELLER
}

}  // namespace flutter
