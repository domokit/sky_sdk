// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/flow/testing/mock_raster_cache.h"

#include "flutter/flow/layers/display_list_raster_cache_item.h"
#include "flutter/flow/layers/layer.h"
#include "flutter/flow/raster_cache.h"
#include "flutter/flow/raster_cache_item.h"
#include "include/core/SkMatrix.h"

namespace flutter {
namespace testing {

MockRasterCacheResult::MockRasterCacheResult(SkRect device_rect)
    : RasterCacheResult(nullptr, SkRect::MakeEmpty(), "RasterCacheFlow::test"),
      device_rect_(device_rect) {}

void MockRasterCache::AddMockLayer(int width, int height) {
  SkMatrix ctm = SkMatrix::I();
  DlPath path = DlPath::MakeRectLTRB(100, 100, 100 + width, 100 + height);
  int layer_cached_threshold = 1;
  MockCacheableLayer layer =
      MockCacheableLayer(path, DlPaint(), layer_cached_threshold);
  layer.Preroll(&preroll_context_);
  layer.raster_cache_item()->TryToPrepareRasterCache(paint_context_);
  RasterCache::Context r_context = {
      // clang-format off
      .gr_context         = preroll_context_.gr_context,
      .dst_color_space    = preroll_context_.dst_color_space,
      .matrix             = ctm,
      .logical_rect       = ToSkRect(layer.paint_bounds()),
      // clang-format on
  };
  UpdateCacheEntry(
      RasterCacheKeyID(layer.unique_id(), RasterCacheKeyType::kLayer),
      r_context, [&](DlCanvas* canvas) {
        SkRect cache_rect = RasterCacheUtil::GetDeviceBounds(
            r_context.logical_rect, r_context.matrix);
        return std::make_unique<MockRasterCacheResult>(cache_rect);
      });
}

void MockRasterCache::AddMockPicture(int width, int height) {
  FML_DCHECK(access_threshold() > 0);
  SkMatrix ctm = SkMatrix::I();
  DisplayListBuilder builder(SkRect::MakeLTRB(0, 0, 200 + width, 200 + height));
  SkPath path;
  path.addRect(100, 100, 100 + width, 100 + height);
  builder.DrawPath(path, DlPaint());
  sk_sp<DisplayList> display_list = builder.Build();

  FixedRefreshRateStopwatch raster_time;
  FixedRefreshRateStopwatch ui_time;
  LayerStateStack state_stack;
  PaintContextHolder holder =
      GetSamplePaintContextHolder(state_stack, this, &raster_time, &ui_time);
  holder.paint_context.dst_color_space = color_space_;

  DisplayListRasterCacheItem display_list_item(display_list, SkPoint(), true,
                                               false);
  for (size_t i = 0; i < access_threshold(); i++) {
    AutoCache(&display_list_item, &preroll_context_, ToDlMatrix(ctm));
  }
  RasterCache::Context r_context = {
      // clang-format off
      .gr_context         = preroll_context_.gr_context,
      .dst_color_space    = preroll_context_.dst_color_space,
      .matrix             = ctm,
      .logical_rect       = display_list->bounds(),
      // clang-format on
  };
  UpdateCacheEntry(RasterCacheKeyID(display_list->unique_id(),
                                    RasterCacheKeyType::kDisplayList),
                   r_context, [&](DlCanvas* canvas) {
                     SkRect cache_rect = RasterCacheUtil::GetDeviceBounds(
                         r_context.logical_rect, r_context.matrix);
                     return std::make_unique<MockRasterCacheResult>(cache_rect);
                   });
}

PrerollContextHolder GetSamplePrerollContextHolder(
    LayerStateStack& state_stack,
    RasterCache* raster_cache,
    FixedRefreshRateStopwatch* raster_time,
    FixedRefreshRateStopwatch* ui_time) {
  sk_sp<SkColorSpace> srgb = SkColorSpace::MakeSRGB();

  PrerollContextHolder holder = {
      {
          // clang-format off
          .raster_cache                  = raster_cache,
          .gr_context                    = nullptr,
          .view_embedder                 = nullptr,
          .state_stack                   = state_stack,
          .dst_color_space               = srgb,
          .surface_needs_readback        = false,
          .raster_time                   = *raster_time,
          .ui_time                       = *ui_time,
          .texture_registry              = nullptr,
          .has_platform_view             = false,
          .has_texture_layer             = false,
          .raster_cached_entries         = &raster_cache_items_,
          // clang-format on
      },
      srgb};

  return holder;
}

PaintContextHolder GetSamplePaintContextHolder(
    LayerStateStack& state_stack,
    RasterCache* raster_cache,
    FixedRefreshRateStopwatch* raster_time,
    FixedRefreshRateStopwatch* ui_time) {
  sk_sp<SkColorSpace> srgb = SkColorSpace::MakeSRGB();
  PaintContextHolder holder = {// clang-format off
    {
        .state_stack                   = state_stack,
        .canvas                        = nullptr,
        .gr_context                    = nullptr,
        .dst_color_space               = srgb,
        .view_embedder                 = nullptr,
        .raster_time                   = *raster_time,
        .ui_time                       = *ui_time,
        .texture_registry              = nullptr,
        .raster_cache                  = raster_cache,
    },
                               // clang-format on
                               srgb};

  return holder;
}

bool RasterCacheItemPrerollAndTryToRasterCache(
    DisplayListRasterCacheItem& display_list_item,
    PrerollContext& context,
    PaintContext& paint_context,
    const DlMatrix& matrix) {
  RasterCacheItemPreroll(display_list_item, context, matrix);
  context.raster_cache->EvictUnusedCacheEntries();
  return RasterCacheItemTryToRasterCache(display_list_item, paint_context);
}

void RasterCacheItemPreroll(DisplayListRasterCacheItem& display_list_item,
                            PrerollContext& context,
                            const DlMatrix& matrix) {
  display_list_item.PrerollSetup(&context, matrix);
  display_list_item.PrerollFinalize(&context, matrix);
}

bool RasterCacheItemTryToRasterCache(
    DisplayListRasterCacheItem& display_list_item,
    PaintContext& paint_context) {
  if (display_list_item.cache_state() ==
      RasterCacheItem::CacheState::kCurrent) {
    return display_list_item.TryToPrepareRasterCache(paint_context);
  }
  return false;
}

}  // namespace testing
}  // namespace flutter
