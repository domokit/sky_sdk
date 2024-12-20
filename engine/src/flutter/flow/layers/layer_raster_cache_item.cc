// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#if !SLIMPELLER

#include "flutter/flow/layers/layer_raster_cache_item.h"
#include "flutter/flow/layers/container_layer.h"
#include "flutter/flow/raster_cache_item.h"
#include "flutter/flow/raster_cache_util.h"
#include "third_party/skia/include/core/SkColorSpace.h"

namespace flutter {

LayerRasterCacheItem::LayerRasterCacheItem(Layer* layer,
                                           int layer_cached_threshold,
                                           bool can_cache_children)
    : RasterCacheItem(
          RasterCacheKeyID(layer->unique_id(), RasterCacheKeyType::kLayer),
          // The layer raster_cache_item's cache state default value is none.
          CacheState::kNone),
      layer_(layer),
      layer_cached_threshold_(layer_cached_threshold),
      can_cache_children_(can_cache_children) {}

void LayerRasterCacheItem::PrerollSetup(PrerollContext* context,
                                        const DlMatrix& matrix) {
  cache_state_ = CacheState::kNone;
  if (context->raster_cache && context->raster_cached_entries) {
    context->raster_cached_entries->push_back(this);
    child_items_ = context->raster_cached_entries->size();
    set_matrix(matrix);
  }
}

std::unique_ptr<LayerRasterCacheItem> LayerRasterCacheItem::Make(
    Layer* layer,
    int layer_cache_threshold,
    bool can_cache_children) {
  return std::make_unique<LayerRasterCacheItem>(layer, layer_cache_threshold,
                                                can_cache_children);
}

void LayerRasterCacheItem::PrerollFinalize(PrerollContext* context,
                                           const DlMatrix& matrix) {
  if (!context->raster_cache || !context->raster_cached_entries) {
    return;
  }
  // We've marked the cache entry that we would like to cache so it stays
  // alive, but if the following conditions apply then we need to set our
  // state back to kDoNotCache so that we don't populate the entry later.
  if (context->has_platform_view || context->has_texture_layer ||
      context->state_stack.content_culled(layer_->paint_bounds())) {
    return;
  }
  child_items_ = context->raster_cached_entries->size() - child_items_;
  if (num_cache_attempts_ >= layer_cached_threshold_) {
    // the layer can be cached
    cache_state_ = CacheState::kCurrent;
    context->raster_cache->MarkSeen(key_id_, matrix_, true);
  } else {
    num_cache_attempts_++;
    // access current layer
    if (can_cache_children_) {
      if (!layer_children_id_.has_value()) {
        auto ids = RasterCacheKeyID::LayerChildrenIds(layer_);
        if (!ids.has_value()) {
          return;
        }
        layer_children_id_.emplace(std::move(ids.value()),
                                   RasterCacheKeyType::kLayerChildren);
      }
      cache_state_ = CacheState::kChildren;
      context->raster_cache->MarkSeen(layer_children_id_.value(), matrix_,
                                      true);
    }
  }
}

std::optional<RasterCacheKeyID> LayerRasterCacheItem::GetId() const {
  switch (cache_state_) {
    case kCurrent:
      return key_id_;
    case kChildren:
      return layer_children_id_;
    default:
      return {};
  }
}

const SkRect* LayerRasterCacheItem::GetPaintBoundsFromLayer() const {
  switch (cache_state_) {
    case CacheState::kCurrent:
      return &ToSkRect(layer_->paint_bounds());
    case CacheState::kChildren:
      FML_DCHECK(layer_->as_container_layer());
      return &ToSkRect(layer_->as_container_layer()->child_paint_bounds());
    default:
      FML_DCHECK(cache_state_ != CacheState::kNone);
      return nullptr;
  }
}

bool Rasterize(RasterCacheItem::CacheState cache_state,
               Layer* layer,
               const PaintContext& paint_context,
               DlCanvas* canvas) {
  FML_DCHECK(cache_state != RasterCacheItem::CacheState::kNone);
  LayerStateStack state_stack;
  state_stack.set_delegate(canvas);
  PaintContext context = {
      // clang-format off
      .state_stack                   = state_stack,
      .canvas                        = canvas,
      .gr_context                    = paint_context.gr_context,
      .dst_color_space               = paint_context.dst_color_space,
      .view_embedder                 = paint_context.view_embedder,
      .raster_time                   = paint_context.raster_time,
      .ui_time                       = paint_context.ui_time,
      .texture_registry              = paint_context.texture_registry,
      .raster_cache                  = paint_context.raster_cache,
      // clang-format on
  };

  switch (cache_state) {
    case RasterCacheItem::CacheState::kCurrent:
      FML_DCHECK(layer->needs_painting(context));
      layer->Paint(context);
      break;
    case RasterCacheItem::CacheState::kChildren:
      layer->PaintChildren(context);
      break;
    case RasterCacheItem::CacheState::kNone:
      FML_DCHECK(cache_state != RasterCacheItem::CacheState::kNone);
      return false;
  }
  return true;
}

static const auto* flow_type = "RasterCacheFlow::Layer";

bool LayerRasterCacheItem::TryToPrepareRasterCache(const PaintContext& context,
                                                   bool parent_cached) const {
  auto maybe_id = GetId();
  if (!maybe_id.has_value() || !context.raster_cache || parent_cached) {
    return false;
  }
  if (cache_state_ != kNone) {
    if (const SkRect* paint_bounds = GetPaintBoundsFromLayer()) {
      RasterCache::Context r_context = {
          // clang-format off
          .gr_context         = context.gr_context,
          .dst_color_space    = context.dst_color_space,
          .matrix             = matrix_,
          .logical_rect       = *paint_bounds,
          .flow_type          = flow_type,
          // clang-format on
      };
      auto id = maybe_id.value();
      return context.raster_cache->UpdateCacheEntry(
          id, r_context,
          [ctx = context, cache_state = cache_state_,
           layer = layer_](DlCanvas* canvas) {
            Rasterize(cache_state, layer, ctx, canvas);
          });
    }
  }
  return false;
}

bool LayerRasterCacheItem::Draw(const PaintContext& context,
                                const DlPaint* paint) const {
  return Draw(context, context.canvas, paint);
}

bool LayerRasterCacheItem::Draw(const PaintContext& context,
                                DlCanvas* canvas,
                                const DlPaint* paint) const {
  if (!context.raster_cache || !canvas) {
    return false;
  }
  switch (cache_state_) {
    case RasterCacheItem::kNone:
      return false;
    case RasterCacheItem::kCurrent: {
      return context.raster_cache->Draw(key_id_, *canvas, paint,
                                        context.rendering_above_platform_view);
    }
    case RasterCacheItem::kChildren: {
      if (!layer_children_id_.has_value()) {
        return false;
      }
      return context.raster_cache->Draw(layer_children_id_.value(), *canvas,
                                        paint,
                                        context.rendering_above_platform_view);
    }
  }
}

}  // namespace flutter

#endif  //  !SLIMPELLER
