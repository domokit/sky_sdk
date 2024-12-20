// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FLOW_TESTING_MOCK_RASTER_CACHE_H_
#define FLUTTER_FLOW_TESTING_MOCK_RASTER_CACHE_H_

#include <vector>
#include "flutter/flow/layers/layer.h"
#include "flutter/flow/raster_cache.h"
#include "flutter/flow/raster_cache_item.h"
#include "flutter/flow/testing/mock_layer.h"
#include "third_party/skia/include/core/SkColorSpace.h"
#include "third_party/skia/include/core/SkSize.h"

namespace flutter {
namespace testing {

/**
 * @brief A RasterCacheResult implementation that represents a cached Layer or
 * DisplayList without the overhead of storage.
 *
 * This implementation is used by MockRasterCache only for testing proper usage
 * of the RasterCache in layer unit tests.
 */
class MockRasterCacheResult : public RasterCacheResult {
 public:
  explicit MockRasterCacheResult(SkRect device_rect);

  void draw(DlCanvas& canvas,
            const DlPaint* paint = nullptr,
            bool preserve_rtree = false) const override {};

  SkISize image_dimensions() const override {
    return SkSize::Make(device_rect_.width(), device_rect_.height()).toCeil();
  };

  int64_t image_bytes() const override {
    return image_dimensions().area() *
           SkColorTypeBytesPerPixel(kBGRA_8888_SkColorType);
  }

 private:
  SkRect device_rect_;
};

static std::vector<RasterCacheItem*> raster_cache_items_;

/**
 * @brief A RasterCache implementation that simulates the act of rendering a
 * Layer or DisplayList without the overhead of rasterization or pixel storage.
 * This implementation is used only for testing proper usage of the RasterCache
 * in layer unit tests.
 */
class MockRasterCache : public RasterCache {
 public:
  explicit MockRasterCache(
      size_t access_threshold = 3,
      size_t picture_and_display_list_cache_limit_per_frame =
          RasterCacheUtil::kDefaultPictureAndDisplayListCacheLimitPerFrame)
      : RasterCache(access_threshold,
                    picture_and_display_list_cache_limit_per_frame) {
    preroll_state_stack_.set_preroll_delegate(DlMatrix());
    paint_state_stack_.set_delegate(&builder_);
  }

  void AddMockLayer(int width, int height);
  void AddMockPicture(int width, int height);

 private:
  LayerStateStack preroll_state_stack_;
  LayerStateStack paint_state_stack_;
  DisplayListBuilder builder_;
  sk_sp<SkColorSpace> color_space_ = SkColorSpace::MakeSRGB();
  MutatorsStack mutators_stack_;
  FixedRefreshRateStopwatch raster_time_;
  FixedRefreshRateStopwatch ui_time_;
  std::shared_ptr<TextureRegistry> texture_registry_;
  PrerollContext preroll_context_ = {
      // clang-format off
      .raster_cache                  = this,
      .gr_context                    = nullptr,
      .view_embedder                 = nullptr,
      .state_stack                   = preroll_state_stack_,
      .dst_color_space               = color_space_,
      .surface_needs_readback        = false,
      .raster_time                   = raster_time_,
      .ui_time                       = ui_time_,
      .texture_registry              = texture_registry_,
      .has_platform_view             = false,
      .has_texture_layer             = false,
      .raster_cached_entries         = &raster_cache_items_
      // clang-format on
  };

  PaintContext paint_context_ = {
      // clang-format off
      .state_stack                   = paint_state_stack_,
      .canvas                        = nullptr,
      .gr_context                    = nullptr,
      .dst_color_space               = color_space_,
      .view_embedder                 = nullptr,
      .raster_time                   = raster_time_,
      .ui_time                       = ui_time_,
      .texture_registry              = texture_registry_,
      .raster_cache                  = nullptr,
      // clang-format on
  };
};

struct PrerollContextHolder {
  PrerollContext preroll_context;
  sk_sp<SkColorSpace> srgb;
};

struct PaintContextHolder {
  PaintContext paint_context;
  sk_sp<SkColorSpace> srgb;
};

PrerollContextHolder GetSamplePrerollContextHolder(
    LayerStateStack& state_stack,
    RasterCache* raster_cache,
    FixedRefreshRateStopwatch* raster_time,
    FixedRefreshRateStopwatch* ui_time);

PaintContextHolder GetSamplePaintContextHolder(
    LayerStateStack& state_stack,
    RasterCache* raster_cache,
    FixedRefreshRateStopwatch* raster_time,
    FixedRefreshRateStopwatch* ui_time);

bool RasterCacheItemPrerollAndTryToRasterCache(
    DisplayListRasterCacheItem& display_list_item,
    PrerollContext& context,
    PaintContext& paint_context,
    const DlMatrix& matrix);

void RasterCacheItemPreroll(DisplayListRasterCacheItem& display_list_item,
                            PrerollContext& context,
                            const DlMatrix& matrix);

bool RasterCacheItemTryToRasterCache(
    DisplayListRasterCacheItem& display_list_item,
    PaintContext& paint_context);

}  // namespace testing
}  // namespace flutter

#endif  // FLUTTER_FLOW_TESTING_MOCK_RASTER_CACHE_H_
