// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/display_list/benchmarking/dl_complexity.h"
#include "flutter/display_list/display_list.h"
#include "flutter/display_list/dl_builder.h"
#include "flutter/display_list/testing/dl_test_snippets.h"
#include "flutter/flow/layers/container_layer.h"
#include "flutter/flow/layers/display_list_layer.h"
#include "flutter/flow/layers/image_filter_layer.h"
#include "flutter/flow/layers/layer_tree.h"
#include "flutter/flow/layers/transform_layer.h"
#include "flutter/flow/raster_cache.h"
#include "flutter/flow/raster_cache_item.h"
#include "flutter/flow/testing/layer_test.h"
#include "flutter/flow/testing/mock_raster_cache.h"
#include "flutter/testing/assertions_skia.h"
#include "gtest/gtest.h"
#include "third_party/skia/include/core/SkMatrix.h"
#include "third_party/skia/include/core/SkPoint.h"

// TODO(zanderso): https://github.com/flutter/flutter/issues/127701
// NOLINTBEGIN(bugprone-unchecked-optional-access)

namespace flutter {
namespace testing {

TEST(RasterCache, SimpleInitialization) {
  flutter::RasterCache cache;
  ASSERT_TRUE(true);
}

TEST(RasterCache, MetricsOmitUnpopulatedEntries) {
  size_t threshold = 2;
  flutter::RasterCache cache(threshold);

  DlMatrix matrix;

  auto display_list = GetSampleDisplayList();

  DisplayListBuilder dummy_canvas(1000, 1000);
  DlPaint paint;

  LayerStateStack preroll_state_stack;
  preroll_state_stack.set_preroll_delegate(kGiantRect, matrix);
  LayerStateStack paint_state_stack;
  preroll_state_stack.set_delegate(&dummy_canvas);

  FixedRefreshRateStopwatch raster_time;
  FixedRefreshRateStopwatch ui_time;
  PrerollContextHolder preroll_context_holder = GetSamplePrerollContextHolder(
      preroll_state_stack, &cache, &raster_time, &ui_time);
  PaintContextHolder paint_context_holder = GetSamplePaintContextHolder(
      paint_state_stack, &cache, &raster_time, &ui_time);
  auto& preroll_context = preroll_context_holder.preroll_context;
  auto& paint_context = paint_context_holder.paint_context;

  cache.BeginFrame();
  DisplayListRasterCacheItem display_list_item(display_list, SkPoint(), true,
                                               false);

  // 1st access.
  ASSERT_FALSE(RasterCacheItemPrerollAndTryToRasterCache(
      display_list_item, preroll_context, paint_context, matrix));
  ASSERT_FALSE(display_list_item.Draw(paint_context, &dummy_canvas, &paint));

  cache.EndFrame();
  ASSERT_EQ(cache.picture_metrics().total_count(), 0u);
  ASSERT_EQ(cache.picture_metrics().total_bytes(), 0u);
  cache.BeginFrame();

  // 2nd access.
  ASSERT_FALSE(RasterCacheItemPrerollAndTryToRasterCache(
      display_list_item, preroll_context, paint_context, matrix));
  ASSERT_FALSE(display_list_item.Draw(paint_context, &dummy_canvas, &paint));

  cache.EndFrame();
  ASSERT_EQ(cache.picture_metrics().total_count(), 0u);
  ASSERT_EQ(cache.picture_metrics().total_bytes(), 0u);
  cache.BeginFrame();

  // Now Prepare should cache it.
  ASSERT_TRUE(RasterCacheItemPrerollAndTryToRasterCache(
      display_list_item, preroll_context, paint_context, matrix));
  ASSERT_TRUE(display_list_item.Draw(paint_context, &dummy_canvas, &paint));

  cache.EndFrame();
  ASSERT_EQ(cache.picture_metrics().total_count(), 1u);
  // 80w * 80h * 4bpp + image object overhead
  ASSERT_EQ(cache.picture_metrics().total_bytes(), 25624u);
}

TEST(RasterCache, ThresholdIsRespectedForDisplayList) {
  size_t threshold = 2;
  flutter::RasterCache cache(threshold);

  DlMatrix matrix;

  auto display_list = GetSampleDisplayList();

  DisplayListBuilder dummy_canvas(1000, 1000);
  DlPaint paint;

  LayerStateStack preroll_state_stack;
  preroll_state_stack.set_preroll_delegate(kGiantRect, matrix);
  LayerStateStack paint_state_stack;
  preroll_state_stack.set_delegate(&dummy_canvas);

  FixedRefreshRateStopwatch raster_time;
  FixedRefreshRateStopwatch ui_time;
  PrerollContextHolder preroll_context_holder = GetSamplePrerollContextHolder(
      preroll_state_stack, &cache, &raster_time, &ui_time);
  PaintContextHolder paint_context_holder = GetSamplePaintContextHolder(
      paint_state_stack, &cache, &raster_time, &ui_time);
  auto& preroll_context = preroll_context_holder.preroll_context;
  auto& paint_context = paint_context_holder.paint_context;

  cache.BeginFrame();

  DisplayListRasterCacheItem display_list_item(display_list, SkPoint(), true,
                                               false);

  // 1st access.
  ASSERT_FALSE(RasterCacheItemPrerollAndTryToRasterCache(
      display_list_item, preroll_context, paint_context, matrix));
  ASSERT_FALSE(display_list_item.Draw(paint_context, &dummy_canvas, &paint));

  cache.EndFrame();
  cache.BeginFrame();

  // 2nd access.
  ASSERT_FALSE(RasterCacheItemPrerollAndTryToRasterCache(
      display_list_item, preroll_context, paint_context, matrix));
  ASSERT_FALSE(display_list_item.Draw(paint_context, &dummy_canvas, &paint));

  cache.EndFrame();
  cache.BeginFrame();

  // Now Prepare should cache it.
  ASSERT_TRUE(RasterCacheItemPrerollAndTryToRasterCache(
      display_list_item, preroll_context, paint_context, matrix));
  ASSERT_TRUE(display_list_item.Draw(paint_context, &dummy_canvas, &paint));
}

TEST(RasterCache, AccessThresholdOfZeroDisablesCachingForDisplayList) {
  size_t threshold = 0;
  flutter::RasterCache cache(threshold);

  DlMatrix matrix;

  auto display_list = GetSampleDisplayList();

  DisplayListBuilder dummy_canvas(1000, 1000);
  DlPaint paint;

  LayerStateStack preroll_state_stack;
  preroll_state_stack.set_preroll_delegate(kGiantRect, matrix);
  LayerStateStack paint_state_stack;
  preroll_state_stack.set_delegate(&dummy_canvas);

  FixedRefreshRateStopwatch raster_time;
  FixedRefreshRateStopwatch ui_time;
  PrerollContextHolder preroll_context_holder = GetSamplePrerollContextHolder(
      preroll_state_stack, &cache, &raster_time, &ui_time);
  PaintContextHolder paint_context_holder = GetSamplePaintContextHolder(
      paint_state_stack, &cache, &raster_time, &ui_time);
  auto& preroll_context = preroll_context_holder.preroll_context;
  auto& paint_context = paint_context_holder.paint_context;

  cache.BeginFrame();

  DisplayListRasterCacheItem display_list_item(display_list, SkPoint(), true,
                                               false);
  ASSERT_FALSE(RasterCacheItemPrerollAndTryToRasterCache(
      display_list_item, preroll_context, paint_context, matrix));
  ASSERT_FALSE(display_list_item.Draw(paint_context, &dummy_canvas, &paint));
}

TEST(RasterCache, PictureCacheLimitPerFrameIsRespectedWhenZeroForDisplayList) {
  size_t picture_cache_limit_per_frame = 0;
  flutter::RasterCache cache(3, picture_cache_limit_per_frame);

  DlMatrix matrix;

  auto display_list = GetSampleDisplayList();

  DisplayListBuilder dummy_canvas(1000, 1000);
  DlPaint paint;

  LayerStateStack preroll_state_stack;
  preroll_state_stack.set_preroll_delegate(kGiantRect, matrix);
  LayerStateStack paint_state_stack;
  preroll_state_stack.set_delegate(&dummy_canvas);

  FixedRefreshRateStopwatch raster_time;
  FixedRefreshRateStopwatch ui_time;
  PrerollContextHolder preroll_context_holder = GetSamplePrerollContextHolder(
      preroll_state_stack, &cache, &raster_time, &ui_time);
  PaintContextHolder paint_context_holder = GetSamplePaintContextHolder(
      paint_state_stack, &cache, &raster_time, &ui_time);
  auto& preroll_context = preroll_context_holder.preroll_context;
  auto& paint_context = paint_context_holder.paint_context;

  cache.BeginFrame();

  DisplayListRasterCacheItem display_list_item(display_list, SkPoint(), true,
                                               false);
  // 1st access.
  ASSERT_FALSE(RasterCacheItemPrerollAndTryToRasterCache(
      display_list_item, preroll_context, paint_context, matrix));
  ASSERT_FALSE(display_list_item.Draw(paint_context, &dummy_canvas, &paint));
  // 2nd access.
  ASSERT_FALSE(RasterCacheItemPrerollAndTryToRasterCache(
      display_list_item, preroll_context, paint_context, matrix));
  ASSERT_FALSE(display_list_item.Draw(paint_context, &dummy_canvas, &paint));
  // the picture_cache_limit_per_frame = 0, so don't cache it
  ASSERT_FALSE(RasterCacheItemPrerollAndTryToRasterCache(
      display_list_item, preroll_context, paint_context, matrix));
  ASSERT_FALSE(display_list_item.Draw(paint_context, &dummy_canvas, &paint));
}

TEST(RasterCache, EvictUnusedCacheEntries) {
  size_t threshold = 1;
  flutter::RasterCache cache(threshold);

  DlMatrix matrix;

  auto display_list_1 = GetSampleDisplayList();
  auto display_list_2 = GetSampleDisplayList();

  DisplayListBuilder dummy_canvas(1000, 1000);
  DlPaint paint;

  LayerStateStack preroll_state_stack;
  preroll_state_stack.set_preroll_delegate(kGiantRect, matrix);
  LayerStateStack paint_state_stack;
  preroll_state_stack.set_delegate(&dummy_canvas);

  FixedRefreshRateStopwatch raster_time;
  FixedRefreshRateStopwatch ui_time;
  PrerollContextHolder preroll_context_holder = GetSamplePrerollContextHolder(
      preroll_state_stack, &cache, &raster_time, &ui_time);
  PaintContextHolder paint_context_holder = GetSamplePaintContextHolder(
      paint_state_stack, &cache, &raster_time, &ui_time);
  auto& preroll_context = preroll_context_holder.preroll_context;
  auto& paint_context = paint_context_holder.paint_context;

  DisplayListRasterCacheItem display_list_item_1(display_list_1, SkPoint(),
                                                 true, false);
  DisplayListRasterCacheItem display_list_item_2(display_list_2, SkPoint(),
                                                 true, false);

  cache.BeginFrame();
  RasterCacheItemPreroll(display_list_item_1, preroll_context, matrix);
  RasterCacheItemPreroll(display_list_item_2, preroll_context, matrix);
  cache.EvictUnusedCacheEntries();
  ASSERT_EQ(cache.EstimatePictureCacheByteSize(), 0u);
  ASSERT_FALSE(
      RasterCacheItemTryToRasterCache(display_list_item_1, paint_context));
  ASSERT_FALSE(
      RasterCacheItemTryToRasterCache(display_list_item_2, paint_context));
  ASSERT_EQ(cache.EstimatePictureCacheByteSize(), 0u);
  ASSERT_FALSE(display_list_item_1.Draw(paint_context, &dummy_canvas, &paint));
  ASSERT_FALSE(display_list_item_2.Draw(paint_context, &dummy_canvas, &paint));
  cache.EndFrame();

  ASSERT_EQ(cache.EstimatePictureCacheByteSize(), 0u);
  ASSERT_EQ(cache.picture_metrics().total_count(), 0u);
  ASSERT_EQ(cache.picture_metrics().total_bytes(), 0u);

  cache.BeginFrame();
  RasterCacheItemPreroll(display_list_item_1, preroll_context, matrix);
  RasterCacheItemPreroll(display_list_item_2, preroll_context, matrix);
  cache.EvictUnusedCacheEntries();
  ASSERT_EQ(cache.EstimatePictureCacheByteSize(), 0u);
  ASSERT_TRUE(
      RasterCacheItemTryToRasterCache(display_list_item_1, paint_context));
  ASSERT_TRUE(
      RasterCacheItemTryToRasterCache(display_list_item_2, paint_context));
  ASSERT_EQ(cache.EstimatePictureCacheByteSize(), 51248u);
  ASSERT_TRUE(display_list_item_1.Draw(paint_context, &dummy_canvas, &paint));
  ASSERT_TRUE(display_list_item_2.Draw(paint_context, &dummy_canvas, &paint));
  cache.EndFrame();

  ASSERT_EQ(cache.EstimatePictureCacheByteSize(), 51248u);
  ASSERT_EQ(cache.picture_metrics().total_count(), 2u);
  ASSERT_EQ(cache.picture_metrics().total_bytes(), 51248u);

  cache.BeginFrame();
  RasterCacheItemPreroll(display_list_item_1, preroll_context, matrix);
  cache.EvictUnusedCacheEntries();
  ASSERT_EQ(cache.EstimatePictureCacheByteSize(), 25624u);
  ASSERT_TRUE(
      RasterCacheItemTryToRasterCache(display_list_item_1, paint_context));
  ASSERT_EQ(cache.EstimatePictureCacheByteSize(), 25624u);
  ASSERT_TRUE(display_list_item_1.Draw(paint_context, &dummy_canvas, &paint));
  cache.EndFrame();

  ASSERT_EQ(cache.EstimatePictureCacheByteSize(), 25624u);
  ASSERT_EQ(cache.picture_metrics().total_count(), 1u);
  ASSERT_EQ(cache.picture_metrics().total_bytes(), 25624u);

  cache.BeginFrame();
  cache.EvictUnusedCacheEntries();
  ASSERT_EQ(cache.EstimatePictureCacheByteSize(), 0u);
  cache.EndFrame();

  ASSERT_EQ(cache.EstimatePictureCacheByteSize(), 0u);
  ASSERT_EQ(cache.picture_metrics().total_count(), 0u);
  ASSERT_EQ(cache.picture_metrics().total_bytes(), 0u);

  cache.BeginFrame();
  ASSERT_FALSE(
      cache.Draw(display_list_item_1.GetId().value(), dummy_canvas, &paint));
  ASSERT_FALSE(display_list_item_1.Draw(paint_context, &dummy_canvas, &paint));
  ASSERT_FALSE(
      cache.Draw(display_list_item_2.GetId().value(), dummy_canvas, &paint));
  ASSERT_FALSE(display_list_item_2.Draw(paint_context, &dummy_canvas, &paint));
  cache.EndFrame();
}

TEST(RasterCache, ComputeDeviceRectBasedOnFractionalTranslation) {
  SkRect logical_rect = SkRect::MakeLTRB(0, 0, 300.2, 300.3);
  SkMatrix ctm = SkMatrix::MakeAll(2.0, 0, 0, 0, 2.0, 0, 0, 0, 1);
  auto result = RasterCacheUtil::GetDeviceBounds(logical_rect, ctm);
  ASSERT_EQ(result, SkRect::MakeLTRB(0.0, 0.0, 600.4, 600.6));
}

// Construct a cache result whose device target rectangle rounds out to be one
// pixel wider than the cached image.  Verify that it can be drawn without
// triggering any assertions.
TEST(RasterCache, DeviceRectRoundOutForDisplayList) {
  size_t threshold = 1;
  flutter::RasterCache cache(threshold);

  SkRect logical_rect = SkRect::MakeLTRB(28, 0, 354.56731, 310.288);
  DisplayListBuilder builder(logical_rect);
  builder.DrawRect(logical_rect, DlPaint(DlColor::kRed()));
  sk_sp<DisplayList> display_list = builder.Build();

  // clang-format off
  DlMatrix ctm(
      1.3312,      0, 0, 0,
           0, 1.3312, 0, 0,
           0,      0, 1, 0,
         233,    206, 0, 1
  );
  // clang-format on
  DlPaint paint;

  DisplayListBuilder canvas(1000, 1000);
  canvas.SetTransform(ctm);

  LayerStateStack preroll_state_stack;
  preroll_state_stack.set_preroll_delegate(kGiantRect, ctm);
  LayerStateStack paint_state_stack;
  preroll_state_stack.set_delegate(&canvas);

  FixedRefreshRateStopwatch raster_time;
  FixedRefreshRateStopwatch ui_time;
  PrerollContextHolder preroll_context_holder = GetSamplePrerollContextHolder(
      preroll_state_stack, &cache, &raster_time, &ui_time);
  PaintContextHolder paint_context_holder = GetSamplePaintContextHolder(
      paint_state_stack, &cache, &raster_time, &ui_time);
  auto& preroll_context = preroll_context_holder.preroll_context;
  auto& paint_context = paint_context_holder.paint_context;

  cache.BeginFrame();
  DisplayListRasterCacheItem display_list_item(display_list, SkPoint(), true,
                                               false);

  ASSERT_FALSE(RasterCacheItemPrerollAndTryToRasterCache(
      display_list_item, preroll_context, paint_context, ctm));
  ASSERT_FALSE(display_list_item.Draw(paint_context, &canvas, &paint));

  cache.EndFrame();
  cache.BeginFrame();

  ASSERT_TRUE(RasterCacheItemPrerollAndTryToRasterCache(
      display_list_item, preroll_context, paint_context, ctm));
  ASSERT_TRUE(display_list_item.Draw(paint_context, &canvas, &paint));

  canvas.Translate(248, 0);
  ASSERT_TRUE(cache.Draw(display_list_item.GetId().value(), canvas, &paint));
  ASSERT_TRUE(display_list_item.Draw(paint_context, &canvas, &paint));
}

TEST(RasterCache, NestedOpCountMetricUsedForDisplayList) {
  size_t threshold = 1;
  flutter::RasterCache cache(threshold);

  DlMatrix matrix;

  auto display_list = GetSampleNestedDisplayList();
  ASSERT_EQ(display_list->op_count(), 1u);
  ASSERT_EQ(display_list->op_count(true), 36u);

  DisplayListBuilder dummy_canvas(1000, 1000);
  DlPaint paint;

  LayerStateStack preroll_state_stack;
  preroll_state_stack.set_preroll_delegate(kGiantRect, matrix);
  LayerStateStack paint_state_stack;
  preroll_state_stack.set_delegate(&dummy_canvas);

  FixedRefreshRateStopwatch raster_time;
  FixedRefreshRateStopwatch ui_time;
  PrerollContextHolder preroll_context_holder = GetSamplePrerollContextHolder(
      preroll_state_stack, &cache, &raster_time, &ui_time);
  PaintContextHolder paint_context_holder = GetSamplePaintContextHolder(
      paint_state_stack, &cache, &raster_time, &ui_time);
  auto& preroll_context = preroll_context_holder.preroll_context;
  auto& paint_context = paint_context_holder.paint_context;

  cache.BeginFrame();

  DisplayListRasterCacheItem display_list_item(display_list, SkPoint(), false,
                                               false);

  ASSERT_FALSE(RasterCacheItemPrerollAndTryToRasterCache(
      display_list_item, preroll_context, paint_context, matrix));
  ASSERT_FALSE(display_list_item.Draw(paint_context, &dummy_canvas, &paint));

  cache.EndFrame();
  cache.BeginFrame();

  ASSERT_TRUE(RasterCacheItemPrerollAndTryToRasterCache(
      display_list_item, preroll_context, paint_context, matrix));
  ASSERT_TRUE(display_list_item.Draw(paint_context, &dummy_canvas, &paint));
}

TEST(RasterCache, NaiveComplexityScoringDisplayList) {
  DisplayListComplexityCalculator* calculator =
      DisplayListNaiveComplexityCalculator::GetInstance();

  size_t threshold = 1;
  flutter::RasterCache cache(threshold);

  DlMatrix matrix;

  // Five raster ops will not be cached
  auto display_list = GetSampleDisplayList(5);
  unsigned int complexity_score = calculator->Compute(display_list.get());

  ASSERT_EQ(complexity_score, 5u);
  ASSERT_EQ(display_list->op_count(), 5u);
  ASSERT_FALSE(calculator->ShouldBeCached(complexity_score));

  DisplayListBuilder dummy_canvas(1000, 1000);
  DlPaint paint;

  LayerStateStack preroll_state_stack;
  preroll_state_stack.set_preroll_delegate(kGiantRect, matrix);
  LayerStateStack paint_state_stack;
  preroll_state_stack.set_delegate(&dummy_canvas);

  FixedRefreshRateStopwatch raster_time;
  FixedRefreshRateStopwatch ui_time;
  PrerollContextHolder preroll_context_holder = GetSamplePrerollContextHolder(
      preroll_state_stack, &cache, &raster_time, &ui_time);
  PaintContextHolder paint_context_holder = GetSamplePaintContextHolder(
      paint_state_stack, &cache, &raster_time, &ui_time);
  auto& preroll_context = preroll_context_holder.preroll_context;
  auto& paint_context = paint_context_holder.paint_context;

  cache.BeginFrame();

  DisplayListRasterCacheItem display_list_item(display_list, SkPoint(), false,
                                               false);

  ASSERT_FALSE(RasterCacheItemPrerollAndTryToRasterCache(
      display_list_item, preroll_context, paint_context, matrix));
  ASSERT_FALSE(display_list_item.Draw(paint_context, &dummy_canvas, &paint));

  cache.EndFrame();
  cache.BeginFrame();

  ASSERT_FALSE(RasterCacheItemPrerollAndTryToRasterCache(
      display_list_item, preroll_context, paint_context, matrix));
  ASSERT_FALSE(display_list_item.Draw(paint_context, &dummy_canvas, &paint));

  // Six raster ops should be cached
  display_list = GetSampleDisplayList(6);
  complexity_score = calculator->Compute(display_list.get());

  ASSERT_EQ(complexity_score, 6u);
  ASSERT_EQ(display_list->op_count(), 6u);
  ASSERT_TRUE(calculator->ShouldBeCached(complexity_score));

  DisplayListRasterCacheItem display_list_item_2 =
      DisplayListRasterCacheItem(display_list, SkPoint(), false, false);
  cache.BeginFrame();

  ASSERT_FALSE(RasterCacheItemPrerollAndTryToRasterCache(
      display_list_item_2, preroll_context, paint_context, matrix));
  ASSERT_FALSE(display_list_item_2.Draw(paint_context, &dummy_canvas, &paint));

  cache.EndFrame();
  cache.BeginFrame();

  ASSERT_TRUE(RasterCacheItemPrerollAndTryToRasterCache(
      display_list_item_2, preroll_context, paint_context, matrix));
  ASSERT_TRUE(display_list_item_2.Draw(paint_context, &dummy_canvas, &paint));
}

TEST(RasterCache, DisplayListWithSingularMatrixIsNotCached) {
  size_t threshold = 2;
  flutter::RasterCache cache(threshold);

  DlMatrix matrices[] = {
      DlMatrix::MakeScale({0.0f, 1.0f, 1.0f}),
      DlMatrix::MakeScale({1.0f, 0.0f, 1.0f}),
      DlMatrix::MakeSkew(1, 1),
  };
  int matrix_count = sizeof(matrices) / sizeof(matrices[0]);

  auto display_list = GetSampleDisplayList();

  DisplayListBuilder dummy_canvas(1000, 1000);
  DlPaint paint;

  LayerStateStack preroll_state_stack;
  preroll_state_stack.set_preroll_delegate(kGiantRect, DlMatrix());
  LayerStateStack paint_state_stack;
  preroll_state_stack.set_delegate(&dummy_canvas);

  FixedRefreshRateStopwatch raster_time;
  FixedRefreshRateStopwatch ui_time;
  PrerollContextHolder preroll_context_holder = GetSamplePrerollContextHolder(
      preroll_state_stack, &cache, &raster_time, &ui_time);
  PaintContextHolder paint_context_holder = GetSamplePaintContextHolder(
      paint_state_stack, &cache, &raster_time, &ui_time);
  auto& preroll_context = preroll_context_holder.preroll_context;
  auto& paint_context = paint_context_holder.paint_context;

  DisplayListRasterCacheItem display_list_item(display_list, SkPoint(), true,
                                               false);

  for (int i = 0; i < 10; i++) {
    cache.BeginFrame();

    for (int j = 0; j < matrix_count; j++) {
      display_list_item.set_matrix(matrices[j]);
      ASSERT_FALSE(RasterCacheItemPrerollAndTryToRasterCache(
          display_list_item, preroll_context, paint_context, matrices[j]));
    }

    for (int j = 0; j < matrix_count; j++) {
      dummy_canvas.SetTransform(matrices[j]);
      ASSERT_FALSE(
          display_list_item.Draw(paint_context, &dummy_canvas, &paint));
    }

    cache.EndFrame();
  }
}

TEST(RasterCache, PrepareLayerTransform) {
  DlRect child_bounds = DlRect::MakeLTRB(10, 10, 50, 50);
  DlPath child_path = DlPath::MakeOval(child_bounds);
  auto child_layer = MockLayer::Make(child_path);
  auto blur_filter = DlBlurImageFilter::Make(5, 5, DlTileMode::kClamp);
  auto blur_layer = std::make_shared<ImageFilterLayer>(blur_filter);
  DlMatrix matrix = DlMatrix::MakeScale({2.0f, 2.0f, 1.0f});
  auto transform_layer = std::make_shared<TransformLayer>(matrix);
  DlMatrix cache_matrix = DlMatrix::MakeTranslation({-20.0f, -20.0f}) * matrix;
  child_layer->set_expected_paint_matrix(cache_matrix);

  blur_layer->Add(child_layer);
  transform_layer->Add(blur_layer);

  size_t threshold = 2;
  MockRasterCache cache(threshold);
  DisplayListBuilder dummy_canvas(1000, 1000);

  LayerStateStack preroll_state_stack;
  preroll_state_stack.set_preroll_delegate(kGiantRect, matrix);
  LayerStateStack paint_state_stack;
  preroll_state_stack.set_delegate(&dummy_canvas);

  FixedRefreshRateStopwatch raster_time;
  FixedRefreshRateStopwatch ui_time;
  std::vector<RasterCacheItem*> cache_items;

  cache.BeginFrame();

  auto preroll_holder = GetSamplePrerollContextHolder(
      preroll_state_stack, &cache, &raster_time, &ui_time);
  preroll_holder.preroll_context.raster_cached_entries = &cache_items;
  transform_layer->Preroll(&preroll_holder.preroll_context);

  auto paint_holder = GetSamplePaintContextHolder(paint_state_stack, &cache,
                                                  &raster_time, &ui_time);

  cache.EvictUnusedCacheEntries();
  LayerTree::TryToRasterCache(
      *preroll_holder.preroll_context.raster_cached_entries,
      &paint_holder.paint_context);

  // Condition tested inside MockLayer::Paint against expected paint matrix.
}

TEST(RasterCache, RasterCacheKeyHashFunction) {
  RasterCacheKey::Map<int> map;
  auto hash_function = map.hash_function();
  SkMatrix matrix = SkMatrix::I();
  uint64_t id = 5;
  RasterCacheKey layer_key(id, RasterCacheKeyType::kLayer, matrix);
  RasterCacheKey display_list_key(id, RasterCacheKeyType::kDisplayList, matrix);
  RasterCacheKey layer_children_key(id, RasterCacheKeyType::kLayerChildren,
                                    matrix);

  auto layer_cache_key_id = RasterCacheKeyID(id, RasterCacheKeyType::kLayer);
  auto layer_hash_code = hash_function(layer_key);
  ASSERT_EQ(layer_hash_code, layer_cache_key_id.GetHash());

  auto display_list_cache_key_id =
      RasterCacheKeyID(id, RasterCacheKeyType::kDisplayList);
  auto display_list_hash_code = hash_function(display_list_key);
  ASSERT_EQ(display_list_hash_code, display_list_cache_key_id.GetHash());

  auto layer_children_cache_key_id =
      RasterCacheKeyID(id, RasterCacheKeyType::kLayerChildren);
  auto layer_children_hash_code = hash_function(layer_children_key);
  ASSERT_EQ(layer_children_hash_code, layer_children_cache_key_id.GetHash());
}

TEST(RasterCache, RasterCacheKeySameID) {
  RasterCacheKey::Map<int> map;
  SkMatrix matrix = SkMatrix::I();
  uint64_t id = 5;
  RasterCacheKey layer_key(id, RasterCacheKeyType::kLayer, matrix);
  RasterCacheKey display_list_key(id, RasterCacheKeyType::kDisplayList, matrix);
  RasterCacheKey layer_children_key(id, RasterCacheKeyType::kLayerChildren,
                                    matrix);
  map[layer_key] = 100;
  map[display_list_key] = 300;
  map[layer_children_key] = 400;

  ASSERT_EQ(map[layer_key], 100);
  ASSERT_EQ(map[display_list_key], 300);
  ASSERT_EQ(map[layer_children_key], 400);
}

TEST(RasterCache, RasterCacheKeySameType) {
  RasterCacheKey::Map<int> map;
  SkMatrix matrix = SkMatrix::I();

  RasterCacheKeyType type = RasterCacheKeyType::kLayer;
  RasterCacheKey layer_first_key(5, type, matrix);
  RasterCacheKey layer_second_key(10, type, matrix);
  RasterCacheKey layer_third_key(15, type, matrix);
  map[layer_first_key] = 50;
  map[layer_second_key] = 100;
  map[layer_third_key] = 150;
  ASSERT_EQ(map[layer_first_key], 50);
  ASSERT_EQ(map[layer_second_key], 100);
  ASSERT_EQ(map[layer_third_key], 150);

  type = RasterCacheKeyType::kDisplayList;
  RasterCacheKey picture_first_key(20, type, matrix);
  RasterCacheKey picture_second_key(25, type, matrix);
  RasterCacheKey picture_third_key(30, type, matrix);
  map[picture_first_key] = 200;
  map[picture_second_key] = 250;
  map[picture_third_key] = 300;
  ASSERT_EQ(map[picture_first_key], 200);
  ASSERT_EQ(map[picture_second_key], 250);
  ASSERT_EQ(map[picture_third_key], 300);

  type = RasterCacheKeyType::kDisplayList;
  RasterCacheKey display_list_first_key(35, type, matrix);
  RasterCacheKey display_list_second_key(40, type, matrix);
  RasterCacheKey display_list_third_key(45, type, matrix);
  map[display_list_first_key] = 350;
  map[display_list_second_key] = 400;
  map[display_list_third_key] = 450;
  ASSERT_EQ(map[display_list_first_key], 350);
  ASSERT_EQ(map[display_list_second_key], 400);
  ASSERT_EQ(map[display_list_third_key], 450);

  type = RasterCacheKeyType::kLayerChildren;
  RasterCacheKeyID foo = RasterCacheKeyID(10, RasterCacheKeyType::kLayer);
  RasterCacheKeyID bar = RasterCacheKeyID(20, RasterCacheKeyType::kLayer);
  RasterCacheKeyID baz = RasterCacheKeyID(30, RasterCacheKeyType::kLayer);
  RasterCacheKey layer_children_first_key(
      RasterCacheKeyID({foo, bar, baz}, type), matrix);
  RasterCacheKey layer_children_second_key(
      RasterCacheKeyID({foo, baz, bar}, type), matrix);
  RasterCacheKey layer_children_third_key(
      RasterCacheKeyID({baz, bar, foo}, type), matrix);
  map[layer_children_first_key] = 100;
  map[layer_children_second_key] = 200;
  map[layer_children_third_key] = 300;
  ASSERT_EQ(map[layer_children_first_key], 100);
  ASSERT_EQ(map[layer_children_second_key], 200);
  ASSERT_EQ(map[layer_children_third_key], 300);
}

TEST(RasterCache, RasterCacheKeyIDEqual) {
  RasterCacheKeyID first = RasterCacheKeyID(1, RasterCacheKeyType::kLayer);
  RasterCacheKeyID second = RasterCacheKeyID(2, RasterCacheKeyType::kLayer);
  RasterCacheKeyID third =
      RasterCacheKeyID(1, RasterCacheKeyType::kLayerChildren);

  ASSERT_NE(first, second);
  ASSERT_NE(first, third);
  ASSERT_NE(second, third);

  RasterCacheKeyID fourth =
      RasterCacheKeyID({first, second}, RasterCacheKeyType::kLayer);
  RasterCacheKeyID fifth =
      RasterCacheKeyID({first, second}, RasterCacheKeyType::kLayerChildren);
  RasterCacheKeyID sixth =
      RasterCacheKeyID({second, first}, RasterCacheKeyType::kLayerChildren);
  ASSERT_NE(fourth, fifth);
  ASSERT_NE(fifth, sixth);
}

TEST(RasterCache, RasterCacheKeyIDHashCode) {
  uint64_t foo = 1;
  uint64_t bar = 2;
  RasterCacheKeyID first = RasterCacheKeyID(foo, RasterCacheKeyType::kLayer);
  RasterCacheKeyID second = RasterCacheKeyID(bar, RasterCacheKeyType::kLayer);
  std::size_t first_hash = first.GetHash();
  std::size_t second_hash = second.GetHash();

  ASSERT_EQ(first_hash, fml::HashCombine(foo, RasterCacheKeyType::kLayer));
  ASSERT_EQ(second_hash, fml::HashCombine(bar, RasterCacheKeyType::kLayer));

  RasterCacheKeyID third =
      RasterCacheKeyID({first, second}, RasterCacheKeyType::kLayerChildren);
  RasterCacheKeyID fourth =
      RasterCacheKeyID({second, first}, RasterCacheKeyType::kLayerChildren);
  std::size_t third_hash = third.GetHash();
  std::size_t fourth_hash = fourth.GetHash();

  ASSERT_EQ(third_hash, fml::HashCombine(RasterCacheKeyID::kDefaultUniqueID,
                                         RasterCacheKeyType::kLayerChildren,
                                         first.GetHash(), second.GetHash()));
  ASSERT_EQ(fourth_hash, fml::HashCombine(RasterCacheKeyID::kDefaultUniqueID,
                                          RasterCacheKeyType::kLayerChildren,
                                          second.GetHash(), first.GetHash()));

  // Verify that the cached hash code is correct.
  ASSERT_EQ(first_hash, first.GetHash());
  ASSERT_EQ(second_hash, second.GetHash());
  ASSERT_EQ(third_hash, third.GetHash());
  ASSERT_EQ(fourth_hash, fourth.GetHash());
}

using RasterCacheTest = LayerTest;

TEST_F(RasterCacheTest, RasterCacheKeyIDLayerChildrenIds) {
  auto layer = std::make_shared<ContainerLayer>();

  const DlPath child_path = DlPath::MakeRect(DlRect::MakeWH(5.0f, 5.0f));
  auto mock_layer = std::make_shared<MockLayer>(child_path);
  layer->Add(mock_layer);

  auto display_list = GetSampleDisplayList();
  auto display_list_layer =
      std::make_shared<DisplayListLayer>(DlPoint(), display_list, false, false);
  layer->Add(display_list_layer);

  auto ids = RasterCacheKeyID::LayerChildrenIds(layer.get()).value();
  std::vector<RasterCacheKeyID> expected_ids;
  expected_ids.emplace_back(
      RasterCacheKeyID(mock_layer->unique_id(), RasterCacheKeyType::kLayer));
  expected_ids.emplace_back(RasterCacheKeyID(display_list->unique_id(),
                                             RasterCacheKeyType::kDisplayList));
  ASSERT_EQ(expected_ids[0], mock_layer->caching_key_id());
  ASSERT_EQ(expected_ids[1], display_list_layer->caching_key_id());
  ASSERT_EQ(ids, expected_ids);
}

TEST(RasterCacheUtilsTest, SkMatrixIntegralTransCTM) {
#define EXPECT_EQ_WITH_TRANSLATE(test, expected, expected_tx, expected_ty) \
  do {                                                                     \
    EXPECT_EQ(test[SkMatrix::kMScaleX], expected[SkMatrix::kMScaleX]);     \
    EXPECT_EQ(test[SkMatrix::kMSkewX], expected[SkMatrix::kMSkewX]);       \
    EXPECT_EQ(test[SkMatrix::kMScaleY], expected[SkMatrix::kMScaleY]);     \
    EXPECT_EQ(test[SkMatrix::kMSkewY], expected[SkMatrix::kMSkewY]);       \
    EXPECT_EQ(test[SkMatrix::kMSkewX], expected[SkMatrix::kMSkewX]);       \
    EXPECT_EQ(test[SkMatrix::kMPersp0], expected[SkMatrix::kMPersp0]);     \
    EXPECT_EQ(test[SkMatrix::kMPersp1], expected[SkMatrix::kMPersp1]);     \
    EXPECT_EQ(test[SkMatrix::kMPersp2], expected[SkMatrix::kMPersp2]);     \
    EXPECT_EQ(test[SkMatrix::kMTransX], expected_tx);                      \
    EXPECT_EQ(test[SkMatrix::kMTransY], expected_ty);                      \
  } while (0)

#define EXPECT_NON_INTEGER_TRANSLATION(matrix)                        \
  EXPECT_TRUE(SkScalarFraction(matrix[SkMatrix::kMTransX]) != 0.0f || \
              SkScalarFraction(matrix[SkMatrix::kMTransY]) != 0.0f)

  {
    // Identity
    SkMatrix matrix = SkMatrix::I();
    SkMatrix get = RasterCacheUtil::GetIntegralTransCTM(matrix);
    SkMatrix compute;
    EXPECT_FALSE(RasterCacheUtil::ComputeIntegralTransCTM(matrix, &compute));
    EXPECT_EQ(get, matrix);
  }
  {
    // Integer translate
    SkMatrix matrix = SkMatrix::Translate(10.0f, 12.0f);
    SkMatrix get = RasterCacheUtil::GetIntegralTransCTM(matrix);
    SkMatrix compute;
    EXPECT_FALSE(RasterCacheUtil::ComputeIntegralTransCTM(matrix, &compute));
    EXPECT_EQ(get, matrix);
  }
  {
    // Fractional x translate
    SkMatrix matrix = SkMatrix::Translate(10.2f, 12.0f);
    EXPECT_NON_INTEGER_TRANSLATION(matrix);
    SkMatrix get = RasterCacheUtil::GetIntegralTransCTM(matrix);
    SkMatrix compute;
    EXPECT_TRUE(RasterCacheUtil::ComputeIntegralTransCTM(matrix, &compute));
    EXPECT_EQ_WITH_TRANSLATE(get, matrix, 10.0f, 12.0f);
    EXPECT_EQ(get, compute);
  }
  {
    // Fractional y translate
    SkMatrix matrix = SkMatrix::Translate(10.0f, 12.3f);
    EXPECT_NON_INTEGER_TRANSLATION(matrix);
    SkMatrix get = RasterCacheUtil::GetIntegralTransCTM(matrix);
    SkMatrix compute;
    EXPECT_TRUE(RasterCacheUtil::ComputeIntegralTransCTM(matrix, &compute));
    EXPECT_EQ_WITH_TRANSLATE(get, matrix, 10.0f, 12.0f);
    EXPECT_EQ(get, compute);
  }
  {
    // Fractional x & y translate
    SkMatrix matrix = SkMatrix::Translate(10.7f, 12.3f);
    EXPECT_NON_INTEGER_TRANSLATION(matrix);
    SkMatrix get = RasterCacheUtil::GetIntegralTransCTM(matrix);
    SkMatrix compute;
    EXPECT_TRUE(RasterCacheUtil::ComputeIntegralTransCTM(matrix, &compute));
    EXPECT_EQ_WITH_TRANSLATE(get, matrix, 11.0f, 12.0f);
    EXPECT_EQ(get, compute);
  }
  {
    // Scale
    SkMatrix matrix = SkMatrix::Scale(2.0f, 3.0f);
    SkMatrix get = RasterCacheUtil::GetIntegralTransCTM(matrix);
    SkMatrix compute;
    EXPECT_FALSE(RasterCacheUtil::ComputeIntegralTransCTM(matrix, &compute));
    EXPECT_EQ(get, matrix);
  }
  {
    // Scale, Integer translate
    SkMatrix matrix = SkMatrix::Scale(2.0f, 3.0f);
    matrix.preTranslate(10.0f, 12.0f);
    SkMatrix get = RasterCacheUtil::GetIntegralTransCTM(matrix);
    SkMatrix compute;
    EXPECT_FALSE(RasterCacheUtil::ComputeIntegralTransCTM(matrix, &compute));
    EXPECT_EQ(get, matrix);
  }
  {
    // Scale, Fractional translate
    SkMatrix matrix = SkMatrix::Scale(2.0f, 3.0f);
    matrix.preTranslate(10.7f, 12.1f);
    EXPECT_NON_INTEGER_TRANSLATION(matrix);
    SkMatrix get = RasterCacheUtil::GetIntegralTransCTM(matrix);
    SkMatrix compute;
    EXPECT_TRUE(RasterCacheUtil::ComputeIntegralTransCTM(matrix, &compute));
    EXPECT_EQ_WITH_TRANSLATE(get, matrix, 21.0f, 36.0f);
    EXPECT_EQ(get, compute);
  }
  {
    // Skew
    SkMatrix matrix = SkMatrix::Skew(0.5f, 0.1f);
    SkMatrix get = RasterCacheUtil::GetIntegralTransCTM(matrix);
    SkMatrix compute;
    EXPECT_FALSE(RasterCacheUtil::ComputeIntegralTransCTM(matrix, &compute));
    EXPECT_EQ(get, matrix);
  }
  {
    // Skew, Fractional translate - should be NOP
    SkMatrix matrix = SkMatrix::Skew(0.5f, 0.1f);
    matrix.preTranslate(10.7f, 12.1f);
    EXPECT_NON_INTEGER_TRANSLATION(matrix);
    SkMatrix get = RasterCacheUtil::GetIntegralTransCTM(matrix);
    SkMatrix compute;
    EXPECT_FALSE(RasterCacheUtil::ComputeIntegralTransCTM(matrix, &compute));
    EXPECT_EQ(get, matrix);
  }
  {
    // Rotate
    SkMatrix matrix = SkMatrix::RotateDeg(45);
    SkMatrix get = RasterCacheUtil::GetIntegralTransCTM(matrix);
    SkMatrix compute;
    EXPECT_FALSE(RasterCacheUtil::ComputeIntegralTransCTM(matrix, &compute));
    EXPECT_EQ(get, matrix);
  }
  {
    // Rotate, Fractional Translate - should be NOP
    SkMatrix matrix = SkMatrix::RotateDeg(45);
    matrix.preTranslate(10.7f, 12.1f);
    EXPECT_NON_INTEGER_TRANSLATION(matrix);
    SkMatrix get = RasterCacheUtil::GetIntegralTransCTM(matrix);
    SkMatrix compute;
    EXPECT_FALSE(RasterCacheUtil::ComputeIntegralTransCTM(matrix, &compute));
    EXPECT_EQ(get, matrix);
  }
  {
    // Perspective x
    SkMatrix matrix = SkMatrix::I();
    matrix.setPerspX(0.1);
    SkMatrix get = RasterCacheUtil::GetIntegralTransCTM(matrix);
    SkMatrix compute;
    EXPECT_FALSE(RasterCacheUtil::ComputeIntegralTransCTM(matrix, &compute));
    EXPECT_EQ(get, matrix);
  }
  {
    // Perspective x, Fractional Translate - should be NOP
    SkMatrix matrix = SkMatrix::I();
    matrix.setPerspX(0.1);
    matrix.preTranslate(10.7f, 12.1f);
    EXPECT_NON_INTEGER_TRANSLATION(matrix);
    SkMatrix get = RasterCacheUtil::GetIntegralTransCTM(matrix);
    SkMatrix compute;
    EXPECT_FALSE(RasterCacheUtil::ComputeIntegralTransCTM(matrix, &compute));
    EXPECT_EQ(get, matrix);
  }
  {
    // Perspective y
    SkMatrix matrix = SkMatrix::I();
    matrix.setPerspY(0.1);
    SkMatrix get = RasterCacheUtil::GetIntegralTransCTM(matrix);
    SkMatrix compute;
    EXPECT_FALSE(RasterCacheUtil::ComputeIntegralTransCTM(matrix, &compute));
    EXPECT_EQ(get, matrix);
  }
  {
    // Perspective y, Fractional Translate - should be NOP
    SkMatrix matrix = SkMatrix::I();
    matrix.setPerspY(0.1);
    matrix.preTranslate(10.7f, 12.1f);
    EXPECT_NON_INTEGER_TRANSLATION(matrix);
    SkMatrix get = RasterCacheUtil::GetIntegralTransCTM(matrix);
    SkMatrix compute;
    EXPECT_FALSE(RasterCacheUtil::ComputeIntegralTransCTM(matrix, &compute));
    EXPECT_EQ(get, matrix);
  }
  {
    // Perspective weight
    // clang-format off
    SkMatrix matrix = SkMatrix::MakeAll(
        1.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.9f);
    // clang-format on
    SkMatrix get = RasterCacheUtil::GetIntegralTransCTM(matrix);
    SkMatrix compute;
    EXPECT_FALSE(RasterCacheUtil::ComputeIntegralTransCTM(matrix, &compute));
    EXPECT_EQ(get, matrix);
  }
  {
    // Perspective weight, Fractional Translate - should be NOP
    // clang-format off
    SkMatrix matrix = SkMatrix::MakeAll(
        1.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.9f);
    // clang-format on
    matrix.preTranslate(10.7f, 12.1f);
    EXPECT_NON_INTEGER_TRANSLATION(matrix);
    SkMatrix get = RasterCacheUtil::GetIntegralTransCTM(matrix);
    SkMatrix compute;
    EXPECT_FALSE(RasterCacheUtil::ComputeIntegralTransCTM(matrix, &compute));
    EXPECT_EQ(get, matrix);
  }
#undef EXPECT_NON_INTEGER_TRANSLATION
#undef EXPECT_EQ_WITH_TRANSLATE
}

TEST(RasterCacheUtilsTest, SkM44IntegralTransCTM) {
#define EXPECT_EQ_WITH_TRANSLATE(test, expected, tx, ty, label) \
  do {                                                          \
    EXPECT_EQ(test.rc(0, 0), expected.rc(0, 0)) << label;       \
    EXPECT_EQ(test.rc(0, 1), expected.rc(0, 1)) << label;       \
    EXPECT_EQ(test.rc(0, 2), expected.rc(0, 2)) << label;       \
    EXPECT_EQ(test.rc(0, 3), tx) << label;                      \
    EXPECT_EQ(test.rc(1, 0), expected.rc(1, 0)) << label;       \
    EXPECT_EQ(test.rc(1, 1), expected.rc(1, 1)) << label;       \
    EXPECT_EQ(test.rc(1, 2), expected.rc(1, 2)) << label;       \
    EXPECT_EQ(test.rc(1, 3), ty) << label;                      \
    EXPECT_EQ(test.rc(2, 0), expected.rc(2, 0)) << label;       \
    EXPECT_EQ(test.rc(2, 1), expected.rc(2, 1)) << label;       \
    EXPECT_EQ(test.rc(2, 2), expected.rc(2, 2)) << label;       \
    EXPECT_EQ(test.rc(2, 3), expected.rc(2, 3)) << label;       \
    EXPECT_EQ(test.rc(3, 0), expected.rc(3, 0)) << label;       \
    EXPECT_EQ(test.rc(3, 1), expected.rc(3, 1)) << label;       \
    EXPECT_EQ(test.rc(3, 2), expected.rc(3, 2)) << label;       \
    EXPECT_EQ(test.rc(3, 3), expected.rc(3, 3)) << label;       \
  } while (0)

#define EXPECT_NON_INTEGER_TRANSLATION(matrix)             \
  EXPECT_TRUE(SkScalarFraction(matrix.rc(0, 3)) != 0.0f || \
              SkScalarFraction(matrix.rc(1, 3)) != 0.0f)

  for (int r = 0; r < 4; r++) {
    for (int c = 0; c < 4; c++) {
      bool snaps;
      switch (r) {
        case 0:  // X equation
          if (c == 3) {
            continue;  // TranslateX, the value we are testing, skip
          }
          snaps = (c == 0);  // X Scale value yes, Skew by Y or Z no
          break;
        case 1:  // Y equation
          if (c == 3) {
            continue;  // TranslateY, the value we are testing, skip
          }
          snaps = (c == 1);  // Y Scale value yes, Skew by X or Z no
          break;
        case 2:  // Z equation, ignored, will snap
          snaps = true;
          break;
        case 3:  // W equation, modifications prevent snapping
          snaps = false;
          break;
        default:
          FML_UNREACHABLE();
      }
      auto label = std::to_string(r) + ", " + std::to_string(c);
      SkM44 matrix = SkM44::Translate(10.7f, 12.1f);
      EXPECT_NON_INTEGER_TRANSLATION(matrix) << label;
      matrix.setRC(r, c, 0.5f);
      if (snaps) {
        SkM44 compute;
        SkM44 get = RasterCacheUtil::GetIntegralTransCTM(matrix);
        EXPECT_TRUE(RasterCacheUtil::ComputeIntegralTransCTM(matrix, &compute))
            << label;
        EXPECT_EQ_WITH_TRANSLATE(get, matrix, 11.0f, 12.0f, label);
        EXPECT_EQ(get, compute) << label;
      } else {
        SkM44 compute;
        SkM44 get = RasterCacheUtil::GetIntegralTransCTM(matrix);
        EXPECT_FALSE(RasterCacheUtil::ComputeIntegralTransCTM(matrix, &compute))
            << label;
        EXPECT_EQ(get, matrix) << label;
      }
    }
  }
#undef EXPECT_NON_INTEGER_TRANSLATION
#undef EXPECT_EQ_WITH_TRANSLATE
}

}  // namespace testing
}  // namespace flutter

// NOLINTEND(bugprone-unchecked-optional-access)
