// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FLOW_LAYERS_CACHEABLE_LAYER_H_
#define FLUTTER_FLOW_LAYERS_CACHEABLE_LAYER_H_

#include <memory>

#include "flutter/flow/layers/container_layer.h"
#include "flutter/flow/layers/layer_raster_cache_item.h"
#include "flutter/flow/raster_cache_util.h"

namespace flutter {

class AutoCache {
 public:
  AutoCache(RasterCacheItem* raster_cache_item,
            PrerollContext* context,
            const DlMatrix& matrix);

  void ShouldNotBeCached() { raster_cache_item_ = nullptr; }

  ~AutoCache();

 private:
  inline bool IsCacheEnabled();
  RasterCacheItem* raster_cache_item_ = nullptr;
  [[maybe_unused]] PrerollContext* context_ = nullptr;
  const DlMatrix matrix_;
};

class CacheableContainerLayer : public ContainerLayer {
 public:
  explicit CacheableContainerLayer(
      int layer_cached_threshold =
          RasterCacheUtil::kMinimumRendersBeforeCachingFilterLayer,
      bool can_cache_children = false);

#if !SLIMPELLER
  const LayerRasterCacheItem* raster_cache_item() const {
    return layer_raster_cache_item_.get();
  }

 protected:
  std::unique_ptr<LayerRasterCacheItem> layer_raster_cache_item_;
#endif  //  !SLIMPELLER
};

}  // namespace flutter

#endif  // FLUTTER_FLOW_LAYERS_CACHEABLE_LAYER_H_
