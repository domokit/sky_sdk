// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FLOW_TESTING_MOCK_LAYER_H_
#define FLUTTER_FLOW_TESTING_MOCK_LAYER_H_

#include <functional>
#include <memory>
#include <utility>
#include "flutter/flow/diff_context.h"
#include "flutter/flow/layers/cacheable_layer.h"
#include "flutter/flow/layers/container_layer.h"
#include "flutter/flow/layers/layer.h"
#include "flutter/flow/layers/layer_raster_cache_item.h"
#include "flutter/flow/raster_cache.h"
#include "flutter/flow/raster_cache_item.h"

namespace flutter {
namespace testing {

// Mock implementation of the |Layer| interface that does nothing but paint
// the specified |path| into the canvas.  It records the |PrerollContext| and
// |PaintContext| data passed in by its parent |Layer|, so the test can later
// verify the data against expected values.
class MockLayer : public Layer {
 public:
  explicit MockLayer(const DlPath& path, DlPaint paint = DlPaint());

  static std::shared_ptr<MockLayer> Make(const DlPath& path,
                                         DlPaint paint = DlPaint()) {
    return std::make_shared<MockLayer>(path, std::move(paint));
  }

  static std::shared_ptr<MockLayer> MakeOpacityCompatible(const DlPath& path) {
    auto mock_layer = std::make_shared<MockLayer>(path, DlPaint());
    mock_layer->set_fake_opacity_compatible(true);
    return mock_layer;
  }

  void Preroll(PrerollContext* context) override;
  void Paint(PaintContext& context) const override;

  const MutatorsStack& parent_mutators() { return parent_mutators_; }
  const DlMatrix& parent_matrix() { return parent_matrix_; }
  const DlRect& parent_cull_rect() { return parent_cull_rect_; }

  bool IsReplacing(DiffContext* context, const Layer* layer) const override;
  void Diff(DiffContext* context, const Layer* old_layer) override;
  const MockLayer* as_mock_layer() const override { return this; }

  bool parent_has_platform_view() {
    return mock_flags_ & kParentHasPlatformView;
  }

  bool parent_has_texture_layer() {
    return mock_flags_ & kParentHasTextureLayer;
  }

  bool fake_has_platform_view() { return mock_flags_ & kFakeHasPlatformView; }

  bool fake_reads_surface() { return mock_flags_ & kFakeReadsSurface; }

  bool fake_opacity_compatible() {
    return mock_flags_ & kFakeOpacityCompatible;
  }

  bool fake_has_texture_layer() { return mock_flags_ & kFakeHasTextureLayer; }

  MockLayer& set_parent_has_platform_view(bool flag) {
    flag ? (mock_flags_ |= kParentHasPlatformView)
         : (mock_flags_ &= ~(kParentHasPlatformView));
    return *this;
  }

  MockLayer& set_parent_has_texture_layer(bool flag) {
    flag ? (mock_flags_ |= kParentHasTextureLayer)
         : (mock_flags_ &= ~(kParentHasTextureLayer));
    return *this;
  }

  MockLayer& set_fake_has_platform_view(bool flag) {
    flag ? (mock_flags_ |= kFakeHasPlatformView)
         : (mock_flags_ &= ~(kFakeHasPlatformView));
    return *this;
  }

  MockLayer& set_fake_reads_surface(bool flag) {
    flag ? (mock_flags_ |= kFakeReadsSurface)
         : (mock_flags_ &= ~(kFakeReadsSurface));
    return *this;
  }

  MockLayer& set_fake_opacity_compatible(bool flag) {
    flag ? (mock_flags_ |= kFakeOpacityCompatible)
         : (mock_flags_ &= ~(kFakeOpacityCompatible));
    return *this;
  }

  MockLayer& set_fake_has_texture_layer(bool flag) {
    flag ? (mock_flags_ |= kFakeHasTextureLayer)
         : (mock_flags_ &= ~(kFakeHasTextureLayer));
    return *this;
  }

  void set_expected_paint_matrix(const DlMatrix& matrix) {
    expected_paint_matrix_ = matrix;
  }

 private:
  MutatorsStack parent_mutators_;
  DlMatrix parent_matrix_;
  DlRect parent_cull_rect_;
  DlPath fake_paint_path_;
  DlPaint fake_paint_;
  std::optional<DlMatrix> expected_paint_matrix_;

  static constexpr int kParentHasPlatformView = 1 << 0;
  static constexpr int kParentHasTextureLayer = 1 << 1;
  static constexpr int kFakeHasPlatformView = 1 << 2;
  static constexpr int kFakeReadsSurface = 1 << 3;
  static constexpr int kFakeOpacityCompatible = 1 << 4;
  static constexpr int kFakeHasTextureLayer = 1 << 5;

  int mock_flags_ = 0;

  FML_DISALLOW_COPY_AND_ASSIGN(MockLayer);
};

class MockCacheableContainerLayer : public CacheableContainerLayer {
 public:
  // if render more than 3 frames, try to cache itself.
  // if less 3 frames, cache his children
  static std::shared_ptr<MockCacheableContainerLayer> CacheLayerOrChildren() {
    return std::make_shared<MockCacheableContainerLayer>(true);
  }

  // if render more than 3 frames, try to cache itself.
  // if less 3 frames, cache nothing
  static std::shared_ptr<MockCacheableContainerLayer> CacheLayerOnly() {
    return std::make_shared<MockCacheableContainerLayer>();
  }

  void Preroll(PrerollContext* context) override;

  explicit MockCacheableContainerLayer(bool cache_children = false)
      : CacheableContainerLayer(3, cache_children) {}
};

class MockLayerCacheableItem : public LayerRasterCacheItem {
 public:
  using LayerRasterCacheItem::LayerRasterCacheItem;
};
class MockCacheableLayer : public MockLayer {
 public:
  explicit MockCacheableLayer(const DlPath& path,
                              DlPaint paint = DlPaint(),
                              int render_limit = 3)
      : MockLayer(path, std::move(paint)) {
    raster_cache_item_ =
        std::make_unique<MockLayerCacheableItem>(this, render_limit);
  }

  const LayerRasterCacheItem* raster_cache_item() const {
    return raster_cache_item_.get();
  }

  void Preroll(PrerollContext* context) override;

 private:
  std::unique_ptr<LayerRasterCacheItem> raster_cache_item_;
};

}  // namespace testing
}  // namespace flutter

#endif  // FLUTTER_FLOW_TESTING_MOCK_LAYER_H_
