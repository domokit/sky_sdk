// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FLOW_TESTING_LAYER_TEST_H_
#define FLUTTER_FLOW_TESTING_LAYER_TEST_H_

#include "display_list/dl_color.h"
#include "flutter/flow/layers/layer.h"

#include <optional>
#include <utility>
#include <vector>

#include "flutter/flow/testing/mock_raster_cache.h"
#include "flutter/fml/macros.h"
#include "flutter/testing/assertions_skia.h"
#include "flutter/testing/canvas_test.h"
#include "flutter/testing/display_list_testing.h"
#include "third_party/skia/include/core/SkCanvas.h"
#include "third_party/skia/include/core/SkImageInfo.h"
#include "third_party/skia/include/utils/SkNWayCanvas.h"

namespace flutter {
namespace testing {

// This fixture allows generating tests which can |Paint()| and |Preroll()|
// |Layer|'s.
// |LayerTest| is a default implementation based on |::testing::Test|.
//
// By default the preroll and paint contexts will not use a raster cache.
// If a test needs to verify the proper operation of a layer in the presence
// of a raster cache then a number of options can be enabled by using the
// methods |LayerTestBase::use_null_raster_cache()|,
// |LayerTestBase::use_mock_raster_cache()| or
// |LayerTestBase::use_skia_raster_cache()|
//
// |BaseT| should be the base test type, such as |::testing::Test| below.
template <typename BaseT>
class LayerTestBase : public CanvasTestBase<BaseT> {
  using TestT = CanvasTestBase<BaseT>;

  const SkRect k_dl_bounds_ = SkRect::MakeWH(500, 500);

 public:
  LayerTestBase()
      : texture_registry_(std::make_shared<TextureRegistry>()),
        preroll_context_{
            // clang-format off
            .raster_cache                  = nullptr,
            .gr_context                    = nullptr,
            .view_embedder                 = nullptr,
            .state_stack                   = preroll_state_stack_,
            .dst_color_space               = TestT::mock_color_space(),
            .surface_needs_readback        = false,
            .raster_time                   = raster_time_,
            .ui_time                       = ui_time_,
            .texture_registry              = texture_registry_,
            .has_platform_view             = false,
            .raster_cached_entries         = &cacheable_items_,
            // clang-format on
        },
        display_list_builder_(k_dl_bounds_),
        display_list_paint_context_{
            // clang-format off
            .state_stack                   = display_list_state_stack_,
            .canvas                        = &display_list_builder_,
            .gr_context                    = nullptr,
            .view_embedder                 = nullptr,
            .raster_time                   = raster_time_,
            .ui_time                       = ui_time_,
            .texture_registry              = texture_registry_,
            .raster_cache                  = nullptr,
            // clang-format on
        } {
    use_null_raster_cache();
    preroll_state_stack_.set_preroll_delegate(kGiantRect, DlMatrix());
    display_list_state_stack_.set_delegate(&display_list_builder_);
  }

  /**
   * @brief Use no raster cache in the preroll_context() and
   * paint_context() structures.
   *
   * This method must be called before using the preroll_context() and
   * paint_context() structures in calls to the Layer::Preroll() and
   * Layer::Paint() methods. This is the default mode of operation.
   *
   * @see use_mock_raster_cache()
   * @see use_skia_raster_cache()
   */
  void use_null_raster_cache() { set_raster_cache_(nullptr); }

  /**
   * @brief Use a mock raster cache in the preroll_context() and
   * paint_context() structures.
   *
   * This method must be called before using the preroll_context() and
   * paint_context() structures in calls to the Layer::Preroll() and
   * Layer::Paint() methods. The mock raster cache behaves like a normal
   * raster cache with respect to decisions about when layers and pictures
   * should be cached, but it does not incur the overhead of rendering the
   * layers or caching the resulting pixels.
   *
   * @see use_null_raster_cache()
   * @see use_skia_raster_cache()
   */
  void use_mock_raster_cache() {
    set_raster_cache_(std::make_unique<MockRasterCache>());
  }

  /**
   * @brief Use a normal raster cache in the preroll_context() and
   * paint_context() structures.
   *
   * This method must be called before using the preroll_context() and
   * paint_context() structures in calls to the Layer::Preroll() and
   * Layer::Paint() methods. The Skia raster cache will behave identically
   * to the raster cache typically used when handling a frame on a device
   * including rendering the contents of pictures and layers to an
   * SkImage, but using a software rather than a hardware renderer.
   *
   * @see use_null_raster_cache()
   * @see use_mock_raster_cache()
   */
  void use_skia_raster_cache() {
    set_raster_cache_(std::make_unique<RasterCache>());
  }

  std::vector<RasterCacheItem*>& cacheable_items() { return cacheable_items_; }

  std::shared_ptr<TextureRegistry> texture_registry() {
    return texture_registry_;
  }
  RasterCache* raster_cache() { return raster_cache_.get(); }
  PrerollContext* preroll_context() { return &preroll_context_; }
  PaintContext& paint_context() { return display_list_paint_context_; }
  PaintContext& display_list_paint_context() {
    return display_list_paint_context_;
  }

  sk_sp<DisplayList> display_list() {
    if (display_list_ == nullptr) {
      display_list_ = display_list_builder_.Build();
    }
    return display_list_;
  }

  void reset_display_list() {
    display_list_ = nullptr;
    // Build() will leave the builder in a state to start recording a new DL
    display_list_builder_.Build();
    // Make sure we are starting from a fresh state stack
    FML_DCHECK(display_list_state_stack_.is_empty());
  }

 private:
  void set_raster_cache_(std::unique_ptr<RasterCache> raster_cache) {
    raster_cache_ = std::move(raster_cache);
    preroll_context_.raster_cache = raster_cache_.get();
    display_list_paint_context_.raster_cache = raster_cache_.get();
  }

  LayerStateStack preroll_state_stack_;
  FixedRefreshRateStopwatch raster_time_;
  FixedRefreshRateStopwatch ui_time_;
  std::shared_ptr<TextureRegistry> texture_registry_;

  std::unique_ptr<RasterCache> raster_cache_;
  PrerollContext preroll_context_;
  DisplayListBuilder display_list_builder_;
  LayerStateStack display_list_state_stack_;
  sk_sp<DisplayList> display_list_;
  PaintContext display_list_paint_context_;

  std::vector<RasterCacheItem*> cacheable_items_;

  FML_DISALLOW_COPY_AND_ASSIGN(LayerTestBase);
};
using LayerTest = LayerTestBase<::testing::Test>;

}  // namespace testing
}  // namespace flutter

#endif  // FLUTTER_FLOW_TESTING_LAYER_TEST_H_
