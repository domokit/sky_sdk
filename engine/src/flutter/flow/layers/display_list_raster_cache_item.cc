// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#if !SLIMPELLER

#include "flutter/flow/layers/display_list_raster_cache_item.h"

#include <optional>
#include <utility>

#include "flutter/display_list/benchmarking/dl_complexity.h"
#include "flutter/display_list/display_list.h"
#include "flutter/flow/layers/layer.h"
#include "flutter/flow/raster_cache.h"
#include "flutter/flow/raster_cache_item.h"
#include "flutter/flow/raster_cache_key.h"
#include "flutter/flow/raster_cache_util.h"
#include "flutter/third_party/skia/include/core/SkColorSpace.h"
#include "flutter/third_party/skia/include/gpu/ganesh/GrDirectContext.h"

namespace flutter {

static bool IsDisplayListWorthRasterizing(
    const DisplayList* display_list,
    bool will_change,
    bool is_complex,
    DisplayListComplexityCalculator* complexity_calculator) {
  if (will_change) {
    // If the display list is going to change in the future, there is no point
    // in doing to extra work to rasterize.
    return false;
  }

  if (display_list == nullptr ||
      !RasterCacheUtil::CanRasterizeRect(display_list->bounds())) {
    // No point in deciding whether the display list is worth rasterizing if it
    // cannot be rasterized at all.
    return false;
  }

  if (is_complex) {
    // The caller seems to have extra information about the display list and
    // thinks the display list is always worth rasterizing.
    return true;
  }

  unsigned int complexity_score = complexity_calculator->Compute(display_list);
  return complexity_calculator->ShouldBeCached(complexity_score);
}

DisplayListRasterCacheItem::DisplayListRasterCacheItem(
    const sk_sp<DisplayList>& display_list,
    const SkPoint& offset,
    bool is_complex,
    bool will_change)
    : RasterCacheItem(RasterCacheKeyID(display_list->unique_id(),
                                       RasterCacheKeyType::kDisplayList),
                      CacheState::kCurrent),
      display_list_(display_list),
      offset_(offset),
      is_complex_(is_complex),
      will_change_(will_change) {}

std::unique_ptr<DisplayListRasterCacheItem> DisplayListRasterCacheItem::Make(
    const sk_sp<DisplayList>& display_list,
    const SkPoint& offset,
    bool is_complex,
    bool will_change) {
  return std::make_unique<DisplayListRasterCacheItem>(display_list, offset,
                                                      is_complex, will_change);
}

void DisplayListRasterCacheItem::PrerollSetup(PrerollContext* context,
                                              const DlMatrix& matrix) {
  cache_state_ = CacheState::kNone;
  DisplayListComplexityCalculator* complexity_calculator =
      context->gr_context ? DisplayListComplexityCalculator::GetForBackend(
                                context->gr_context->backend())
                          : DisplayListComplexityCalculator::GetForSoftware();

  if (!IsDisplayListWorthRasterizing(display_list(), will_change_, is_complex_,
                                     complexity_calculator)) {
    // We only deal with display lists that are worthy of rasterization.
    return;
  }

  transformation_matrix_ = ToSkMatrix(matrix);
  transformation_matrix_.preTranslate(offset_.x(), offset_.y());

  if (!transformation_matrix_.invert(nullptr)) {
    // The matrix was singular. No point in going further.
    return;
  }

  if (context->raster_cached_entries && context->raster_cache) {
    context->raster_cached_entries->push_back(this);
    cache_state_ = CacheState::kCurrent;
  }
  return;
}

void DisplayListRasterCacheItem::PrerollFinalize(PrerollContext* context,
                                                 const DlMatrix& matrix) {
  if (cache_state_ == CacheState::kNone || !context->raster_cache ||
      !context->raster_cached_entries) {
    return;
  }
  auto* raster_cache = context->raster_cache;
  DlRect bounds = display_list_->GetBounds().Shift(offset_.x(), offset_.y());
  bool visible = !context->state_stack.content_culled(bounds);
  RasterCache::CacheInfo cache_info =
      raster_cache->MarkSeen(key_id_, ToSkMatrix(matrix), visible);
  if (!visible ||
      cache_info.accesses_since_visible <= raster_cache->access_threshold()) {
    cache_state_ = kNone;
  } else {
    if (cache_info.has_image) {
      context->renderable_state_flags |=
          LayerStateStack::kCallerCanApplyOpacity;
    }
    cache_state_ = kCurrent;
  }
  return;
}

bool DisplayListRasterCacheItem::Draw(const PaintContext& context,
                                      const DlPaint* paint) const {
  return Draw(context, context.canvas, paint);
}

bool DisplayListRasterCacheItem::Draw(const PaintContext& context,
                                      DlCanvas* canvas,
                                      const DlPaint* paint) const {
  if (!context.raster_cache || !canvas) {
    return false;
  }
  if (cache_state_ == CacheState::kCurrent) {
    return context.raster_cache->Draw(key_id_, *canvas, paint,
                                      context.rendering_above_platform_view);
  }
  return false;
}

static const auto* flow_type = "RasterCacheFlow::DisplayList";

bool DisplayListRasterCacheItem::TryToPrepareRasterCache(
    const PaintContext& context,
    bool parent_cached) const {
  // If we don't have raster_cache we should not cache the current display_list.
  // If the current node's ancestor has been cached we also should not cache the
  // current node. In the current frame, the raster_cache will collect all
  // display_list or picture_list to calculate the memory they used, we
  // shouldn't cache the current node if the memory is more significant than the
  // limit.
  auto id = GetId();
  FML_DCHECK(id.has_value());
  if (cache_state_ == kNone || !context.raster_cache || parent_cached ||
      !context.raster_cache->GenerateNewCacheInThisFrame() || !id.has_value()) {
    return false;
  }
  SkRect bounds = display_list_->bounds().makeOffset(offset_.x(), offset_.y());
  RasterCache::Context r_context = {
      // clang-format off
      .gr_context         = context.gr_context,
      .dst_color_space    = context.dst_color_space,
      .matrix             = transformation_matrix_,
      .logical_rect       = bounds,
      .flow_type          = flow_type,
      // clang-format on
  };
  return context.raster_cache->UpdateCacheEntry(
      id.value(), r_context,
      [display_list = display_list_](DlCanvas* canvas) {
        canvas->DrawDisplayList(display_list);
      },
      display_list_->rtree());
}
}  // namespace flutter

#endif  //  !SLIMPELLER
