// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FLOW_LAYERS_LAYER_RASTER_CACHE_ITEM_H_
#define FLUTTER_FLOW_LAYERS_LAYER_RASTER_CACHE_ITEM_H_

#if !SLIMPELLER

#include <memory>
#include <optional>

#include "flutter/flow/raster_cache_item.h"

namespace flutter {

class LayerRasterCacheItem : public RasterCacheItem {
 public:
  explicit LayerRasterCacheItem(Layer* layer,
                                int layer_cached_threshold = 1,
                                bool can_cache_children = false);

  /**
   * @brief Create a LayerRasterCacheItem, connect a layer and manage the
   * Layer's raster cache
   *
   * @param layer_cache_threshold  after how many frames to start trying to
   * cache the layer self
   * @param can_cache_children the layer can do a cache for his children
   */
  static std::unique_ptr<LayerRasterCacheItem>
  Make(Layer*, int layer_cache_threshold, bool can_cache_children = false);

  std::optional<RasterCacheKeyID> GetId() const override;

  void PrerollSetup(PrerollContext* context, const DlMatrix& matrix) override;

  void PrerollFinalize(PrerollContext* context,
                       const DlMatrix& matrix) override;

  bool Draw(const PaintContext& context, const DlPaint* paint) const override;

  bool Draw(const PaintContext& context,
            DlCanvas* canvas,
            const DlPaint* paint) const override;

  bool TryToPrepareRasterCache(const PaintContext& context,
                               bool parent_cached = false) const override;

  void MarkCacheChildren() { can_cache_children_ = true; }

  void MarkNotCacheChildren() { can_cache_children_ = false; }

  bool IsCacheChildren() const { return cache_state_ == CacheState::kChildren; }

 protected:
  const SkRect* GetPaintBoundsFromLayer() const;

  Layer* layer_;

  // The id for cache the layer's children.
  std::optional<RasterCacheKeyID> layer_children_id_;

  int layer_cached_threshold_ = 1;

  // if the layer's children can be directly cache, set the param is true;
  bool can_cache_children_ = false;

  mutable int num_cache_attempts_ = 1;
};

}  // namespace flutter

#endif  //  !SLIMPELLER

#endif  // FLUTTER_FLOW_LAYERS_LAYER_RASTER_CACHE_ITEM_H_
