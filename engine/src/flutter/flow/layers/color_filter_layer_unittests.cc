// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/display_list/display_list.h"
#include "flutter/display_list/dl_color.h"
#include "flutter/display_list/dl_paint.h"
#include "flutter/flow/compositor_context.h"
#include "flutter/flow/layers/color_filter_layer.h"

#include "flutter/display_list/effects/dl_color_filter.h"
#include "flutter/flow/layers/layer_tree.h"
#include "flutter/flow/layers/opacity_layer.h"
#include "flutter/flow/raster_cache.h"
#include "flutter/flow/raster_cache_key.h"
#include "flutter/flow/testing/layer_test.h"
#include "flutter/flow/testing/mock_layer.h"
#include "flutter/fml/macros.h"

// TODO(zanderso): https://github.com/flutter/flutter/issues/127701
// NOLINTBEGIN(bugprone-unchecked-optional-access)

namespace flutter {
namespace testing {

using ColorFilterLayerTest = LayerTest;

#ifndef NDEBUG
TEST_F(ColorFilterLayerTest, PaintingEmptyLayerDies) {
  auto layer = std::make_shared<ColorFilterLayer>(nullptr);

  layer->Preroll(preroll_context());
  EXPECT_EQ(layer->paint_bounds(), DlRect());
  EXPECT_EQ(layer->child_paint_bounds(), DlRect());
  EXPECT_FALSE(layer->needs_painting(paint_context()));

  EXPECT_DEATH_IF_SUPPORTED(layer->Paint(paint_context()),
                            "needs_painting\\(context\\)");
}

TEST_F(ColorFilterLayerTest, PaintBeforePrerollDies) {
  const DlRect child_bounds = DlRect::MakeLTRB(5.0f, 6.0f, 20.5f, 21.5f);
  const DlPath child_path = DlPath::MakeRect(child_bounds);
  auto mock_layer = std::make_shared<MockLayer>(child_path);

  auto layer = std::make_shared<ColorFilterLayer>(nullptr);
  layer->Add(mock_layer);

  EXPECT_EQ(layer->paint_bounds(), DlRect());
  EXPECT_EQ(layer->child_paint_bounds(), DlRect());
  EXPECT_DEATH_IF_SUPPORTED(layer->Paint(paint_context()),
                            "needs_painting\\(context\\)");
}
#endif

TEST_F(ColorFilterLayerTest, EmptyFilter) {
  const DlMatrix initial_transform = DlMatrix::MakeTranslation({0.5f, 1.0f});
  const DlRect child_bounds = DlRect::MakeLTRB(5.0f, 6.0f, 20.5f, 21.5f);
  const DlPath child_path = DlPath::MakeRect(child_bounds);
  const DlPaint child_paint = DlPaint(DlColor::kYellow());
  auto mock_layer = std::make_shared<MockLayer>(child_path, child_paint);
  auto layer = std::make_shared<ColorFilterLayer>(nullptr);
  layer->Add(mock_layer);

  preroll_context()->state_stack.set_preroll_delegate(initial_transform);
  layer->Preroll(preroll_context());
  EXPECT_EQ(layer->paint_bounds(), child_bounds);
  EXPECT_EQ(layer->child_paint_bounds(), child_bounds);
  EXPECT_TRUE(layer->needs_painting(paint_context()));
  EXPECT_EQ(mock_layer->parent_matrix(), initial_transform);

  layer->Paint(display_list_paint_context());
  DisplayListBuilder expected_builder;
  /* (ColorFilter)layer::Paint */ {
    expected_builder.Save();
    {
      /* mock_layer::Paint */ {
        expected_builder.DrawPath(child_path, child_paint);
      }
    }
    expected_builder.Restore();
  }
  EXPECT_TRUE(DisplayListsEQ_Verbose(display_list(), expected_builder.Build()));
}

TEST_F(ColorFilterLayerTest, SimpleFilter) {
  const DlMatrix initial_transform = DlMatrix::MakeTranslation({0.5f, 1.0f});
  const DlRect child_bounds = DlRect::MakeLTRB(5.0f, 6.0f, 20.5f, 21.5f);
  const DlPath child_path = DlPath::MakeRect(child_bounds);
  const DlPaint child_paint = DlPaint(DlColor::kYellow());

  auto dl_color_filter = DlColorFilter::MakeLinearToSrgbGamma();
  auto mock_layer = std::make_shared<MockLayer>(child_path, child_paint);
  auto layer = std::make_shared<ColorFilterLayer>(dl_color_filter);
  layer->Add(mock_layer);

  preroll_context()->state_stack.set_preroll_delegate(initial_transform);
  layer->Preroll(preroll_context());
  EXPECT_EQ(layer->paint_bounds(), child_bounds);
  EXPECT_EQ(layer->child_paint_bounds(), child_bounds);
  EXPECT_TRUE(layer->needs_painting(paint_context()));
  EXPECT_EQ(mock_layer->parent_matrix(), initial_transform);

  DisplayListBuilder expected_builder;
  /* ColorFilterLayer::Paint() */ {
    DlPaint dl_paint;
    dl_paint.setColorFilter(dl_color_filter.get());
    expected_builder.SaveLayer(child_bounds, &dl_paint);
    {
      /* MockLayer::Paint() */ {
        expected_builder.DrawPath(child_path, DlPaint(DlColor::kYellow()));
      }
    }
  }
  expected_builder.Restore();
  auto expected_display_list = expected_builder.Build();

  layer->Paint(display_list_paint_context());
  EXPECT_TRUE(DisplayListsEQ_Verbose(display_list(), expected_display_list));
}

TEST_F(ColorFilterLayerTest, MultipleChildren) {
  const DlMatrix initial_transform = DlMatrix::MakeTranslation({0.5f, 1.0f});
  const DlRect child_bounds = DlRect::MakeLTRB(5.0f, 6.0f, 2.5f, 3.5f);
  const DlPath child_path1 = DlPath::MakeRect(child_bounds);
  const DlPath child_path2 = DlPath::MakeRect(child_bounds.Shift(3.0f, 0.0f));
  const DlPaint child_paint1 = DlPaint(DlColor::kYellow());
  const DlPaint child_paint2 = DlPaint(DlColor::kCyan());
  auto mock_layer1 = std::make_shared<MockLayer>(child_path1, child_paint1);
  auto mock_layer2 = std::make_shared<MockLayer>(child_path2, child_paint2);
  auto dl_color_filter = DlColorFilter::MakeSrgbToLinearGamma();
  auto layer = std::make_shared<ColorFilterLayer>(dl_color_filter);
  layer->Add(mock_layer1);
  layer->Add(mock_layer2);

  DlRect children_bounds =
      child_path1.GetBounds().Union(child_path2.GetBounds());
  preroll_context()->state_stack.set_preroll_delegate(initial_transform);
  layer->Preroll(preroll_context());
  EXPECT_EQ(mock_layer1->paint_bounds(), child_path1.GetBounds());
  EXPECT_EQ(mock_layer2->paint_bounds(), child_path2.GetBounds());
  EXPECT_EQ(layer->paint_bounds(), children_bounds);
  EXPECT_EQ(layer->child_paint_bounds(), children_bounds);
  EXPECT_TRUE(mock_layer1->needs_painting(paint_context()));
  EXPECT_TRUE(mock_layer2->needs_painting(paint_context()));
  EXPECT_TRUE(layer->needs_painting(paint_context()));
  EXPECT_EQ(mock_layer1->parent_matrix(), initial_transform);
  EXPECT_EQ(mock_layer2->parent_matrix(), initial_transform);

  DisplayListBuilder expected_builder;
  /* ColorFilterLayer::Paint() */ {
    DlPaint dl_paint;
    dl_paint.setColorFilter(dl_color_filter.get());
    expected_builder.SaveLayer(children_bounds, &dl_paint);
    {
      /* MockLayer::Paint() */ {
        expected_builder.DrawPath(child_path1, DlPaint(DlColor::kYellow()));
      }
      /* MockLayer::Paint() */ {
        expected_builder.DrawPath(child_path2, DlPaint(DlColor::kCyan()));
      }
    }
  }
  expected_builder.Restore();
  auto expected_display_list = expected_builder.Build();

  layer->Paint(display_list_paint_context());
  EXPECT_TRUE(DisplayListsEQ_Verbose(display_list(), expected_display_list));
}

TEST_F(ColorFilterLayerTest, Nested) {
  const DlMatrix initial_transform = DlMatrix::MakeTranslation({0.5f, 1.0f});
  const DlRect child_bounds = DlRect::MakeLTRB(5.0f, 6.0f, 2.5f, 3.5f);
  const DlPath child_path1 = DlPath::MakeRect(child_bounds);
  const DlPath child_path2 = DlPath::MakeRect(child_bounds.Shift(3.0f, 0.0f));
  const DlPaint child_paint1 = DlPaint(DlColor::kYellow());
  const DlPaint child_paint2 = DlPaint(DlColor::kCyan());
  auto mock_layer1 = std::make_shared<MockLayer>(child_path1, child_paint1);
  auto mock_layer2 = std::make_shared<MockLayer>(child_path2, child_paint2);
  auto dl_color_filter = DlColorFilter::MakeSrgbToLinearGamma();
  auto layer1 = std::make_shared<ColorFilterLayer>(dl_color_filter);

  auto layer2 = std::make_shared<ColorFilterLayer>(dl_color_filter);
  layer2->Add(mock_layer2);
  layer1->Add(mock_layer1);
  layer1->Add(layer2);

  const DlRect children_bounds =
      child_path1.GetBounds().Union(child_path2.GetBounds());
  preroll_context()->state_stack.set_preroll_delegate(initial_transform);
  layer1->Preroll(preroll_context());
  EXPECT_EQ(mock_layer1->paint_bounds(), child_path1.GetBounds());
  EXPECT_EQ(mock_layer2->paint_bounds(), child_path2.GetBounds());
  EXPECT_EQ(layer1->paint_bounds(), children_bounds);
  EXPECT_EQ(layer1->child_paint_bounds(), children_bounds);
  EXPECT_EQ(layer2->paint_bounds(), mock_layer2->paint_bounds());
  EXPECT_EQ(layer2->child_paint_bounds(), mock_layer2->paint_bounds());
  EXPECT_TRUE(mock_layer1->needs_painting(paint_context()));
  EXPECT_TRUE(mock_layer2->needs_painting(paint_context()));
  EXPECT_TRUE(layer1->needs_painting(paint_context()));
  EXPECT_TRUE(layer2->needs_painting(paint_context()));
  EXPECT_EQ(mock_layer1->parent_matrix(), initial_transform);
  EXPECT_EQ(mock_layer2->parent_matrix(), initial_transform);

  DisplayListBuilder expected_builder;
  /* ColorFilterLayer::Paint() */ {
    DlPaint dl_paint;
    dl_paint.setColorFilter(dl_color_filter.get());
    expected_builder.SaveLayer(children_bounds, &dl_paint);
    {
      /* MockLayer::Paint() */ {
        expected_builder.DrawPath(child_path1, DlPaint(DlColor::kYellow()));
      }
      /* ColorFilter::Paint() */ {
        DlPaint child_dl_paint;
        child_dl_paint.setColor(DlColor::kBlack());
        child_dl_paint.setColorFilter(dl_color_filter.get());
        expected_builder.SaveLayer(child_path2.GetBounds(), &child_dl_paint);

        /* MockLayer::Paint() */ {
          expected_builder.DrawPath(child_path2, DlPaint(DlColor::kCyan()));
        }
        expected_builder.Restore();
      }
    }
  }
  expected_builder.Restore();

  auto expected_display_list = expected_builder.Build();

  layer1->Paint(display_list_paint_context());
  EXPECT_TRUE(DisplayListsEQ_Verbose(display_list(), expected_display_list));
}

TEST_F(ColorFilterLayerTest, Readback) {
  DlMatrix initial_transform;

  // ColorFilterLayer does not read from surface
  auto layer = std::make_shared<ColorFilterLayer>(
      DlColorFilter::MakeLinearToSrgbGamma());
  preroll_context()->surface_needs_readback = false;
  preroll_context()->state_stack.set_preroll_delegate(initial_transform);
  layer->Preroll(preroll_context());
  EXPECT_FALSE(preroll_context()->surface_needs_readback);

  // ColorFilterLayer blocks child with readback
  auto mock_layer = std::make_shared<MockLayer>(DlPath(), DlPaint());
  mock_layer->set_fake_reads_surface(true);
  layer->Add(mock_layer);
  preroll_context()->surface_needs_readback = false;
  layer->Preroll(preroll_context());
  EXPECT_FALSE(preroll_context()->surface_needs_readback);
}

TEST_F(ColorFilterLayerTest, CacheChild) {
  auto layer_filter = DlColorFilter::MakeSrgbToLinearGamma();
  auto initial_transform = DlMatrix::MakeTranslation({50.0f, 25.5f});
  auto other_transform = DlMatrix::MakeScale({1.0f, 2.0f, 1.0f});
  const DlPath child_path = DlPath::MakeRect(DlRect::MakeWH(5.0f, 5.0f));
  DlPaint paint = DlPaint();
  auto mock_layer = std::make_shared<MockLayer>(child_path);
  auto layer = std::make_shared<ColorFilterLayer>(layer_filter);
  layer->Add(mock_layer);

  DlMatrix cache_ctm = initial_transform;
  DisplayListBuilder cache_canvas;
  cache_canvas.Transform(cache_ctm);
  DisplayListBuilder other_canvas;
  other_canvas.Transform(other_transform);

  use_mock_raster_cache();
  const auto* cacheable_color_filter_item = layer->raster_cache_item();

  EXPECT_EQ(raster_cache()->GetLayerCachedEntriesCount(), (size_t)0);
  EXPECT_EQ(cacheable_color_filter_item->cache_state(),
            RasterCacheItem::CacheState::kNone);
  EXPECT_FALSE(cacheable_color_filter_item->GetId().has_value());

  preroll_context()->state_stack.set_preroll_delegate(initial_transform);
  layer->Preroll(preroll_context());
  LayerTree::TryToRasterCache(cacheable_items(), &paint_context());

  EXPECT_EQ(raster_cache()->GetLayerCachedEntriesCount(), (size_t)1);
  EXPECT_EQ(cacheable_color_filter_item->cache_state(),
            RasterCacheItem::CacheState::kChildren);
  EXPECT_EQ(
      cacheable_color_filter_item->GetId().value(),
      RasterCacheKeyID(RasterCacheKeyID::LayerChildrenIds(layer.get()).value(),
                       RasterCacheKeyType::kLayerChildren));
  EXPECT_FALSE(raster_cache()->Draw(
      cacheable_color_filter_item->GetId().value(), other_canvas, &paint));
  EXPECT_TRUE(raster_cache()->Draw(cacheable_color_filter_item->GetId().value(),
                                   cache_canvas, &paint));
}

TEST_F(ColorFilterLayerTest, CacheChildren) {
  auto layer_filter = DlColorFilter::MakeSrgbToLinearGamma();
  auto initial_transform = DlMatrix::MakeTranslation({50.0f, 25.5f});
  auto other_transform = DlMatrix::MakeScale({1.0f, 2.0f, 1.0f});
  const DlPath child_path1 = DlPath::MakeRect(DlRect::MakeWH(5.0f, 5.0f));
  const DlPath child_path2 = DlPath::MakeRect(DlRect::MakeWH(5.0f, 5.0f));
  auto mock_layer1 = std::make_shared<MockLayer>(child_path1);
  auto mock_layer2 = std::make_shared<MockLayer>(child_path2);
  auto layer = std::make_shared<ColorFilterLayer>(layer_filter);
  layer->Add(mock_layer1);
  layer->Add(mock_layer2);
  DlPaint paint = DlPaint();

  DlMatrix cache_ctm = initial_transform;
  DisplayListBuilder cache_canvas;
  cache_canvas.Transform(cache_ctm);
  DisplayListBuilder other_canvas;
  other_canvas.Transform(other_transform);

  use_mock_raster_cache();

  EXPECT_EQ(raster_cache()->GetLayerCachedEntriesCount(), (size_t)0);
  const auto* cacheable_color_filter_item = layer->raster_cache_item();
  EXPECT_EQ(raster_cache()->GetLayerCachedEntriesCount(), (size_t)0);

  EXPECT_EQ(cacheable_color_filter_item->cache_state(),
            RasterCacheItem::CacheState::kNone);
  EXPECT_FALSE(cacheable_color_filter_item->GetId().has_value());

  preroll_context()->state_stack.set_preroll_delegate(initial_transform);
  layer->Preroll(preroll_context());
  LayerTree::TryToRasterCache(cacheable_items(), &paint_context());

  EXPECT_EQ(raster_cache()->GetLayerCachedEntriesCount(), (size_t)1);
  EXPECT_EQ(cacheable_color_filter_item->cache_state(),
            RasterCacheItem::CacheState::kChildren);
  EXPECT_EQ(
      cacheable_color_filter_item->GetId().value(),
      RasterCacheKeyID(RasterCacheKeyID::LayerChildrenIds(layer.get()).value(),
                       RasterCacheKeyType::kLayerChildren));
  EXPECT_FALSE(raster_cache()->Draw(
      cacheable_color_filter_item->GetId().value(), other_canvas, &paint));
  EXPECT_TRUE(raster_cache()->Draw(cacheable_color_filter_item->GetId().value(),
                                   cache_canvas, &paint));
}

TEST_F(ColorFilterLayerTest, CacheColorFilterLayerSelf) {
  auto layer_filter = DlColorFilter::MakeSrgbToLinearGamma();
  auto initial_transform = DlMatrix::MakeTranslation({50.0, 25.5});
  auto other_transform = DlMatrix::MakeScale({1.0, 2.0, 1.0f});
  const DlPath child_path1 = DlPath::MakeRect(DlRect::MakeWH(5.0f, 5.0f));
  const DlPath child_path2 = DlPath::MakeRect(DlRect::MakeWH(5.0f, 5.0f));
  auto mock_layer1 = std::make_shared<MockLayer>(child_path1);
  auto mock_layer2 = std::make_shared<MockLayer>(child_path2);
  auto layer = std::make_shared<ColorFilterLayer>(layer_filter);
  layer->Add(mock_layer1);
  layer->Add(mock_layer2);
  DlPaint paint = DlPaint();

  DlMatrix cache_ctm = initial_transform;
  DisplayListBuilder cache_canvas;
  cache_canvas.Transform(cache_ctm);
  DisplayListBuilder other_canvas;
  other_canvas.Transform(other_transform);

  use_mock_raster_cache();
  preroll_context()->state_stack.set_preroll_delegate(initial_transform);
  const auto* cacheable_color_filter_item = layer->raster_cache_item();

  // frame 1.
  layer->Preroll(preroll_context());
  layer->Paint(paint_context());
  // frame 2.
  layer->Preroll(preroll_context());
  LayerTree::TryToRasterCache(cacheable_items(), &paint_context());
  // ColorFilterLayer default cache children.
  EXPECT_EQ(cacheable_color_filter_item->cache_state(),
            RasterCacheItem::CacheState::kChildren);
  EXPECT_TRUE(raster_cache()->Draw(cacheable_color_filter_item->GetId().value(),
                                   cache_canvas, &paint));
  EXPECT_FALSE(raster_cache()->Draw(
      cacheable_color_filter_item->GetId().value(), other_canvas, &paint));
  layer->Paint(paint_context());

  // frame 3.
  layer->Preroll(preroll_context());
  layer->Paint(paint_context());

  LayerTree::TryToRasterCache(cacheable_items(), &paint_context());
  // frame1,2 cache the ColorFilterLayer's children layer, frame3 cache the
  // ColorFilterLayer
  EXPECT_EQ(raster_cache()->GetLayerCachedEntriesCount(), (size_t)2);

  // ColorFilterLayer default cache itself.
  EXPECT_EQ(cacheable_color_filter_item->cache_state(),
            RasterCacheItem::CacheState::kCurrent);
  EXPECT_EQ(cacheable_color_filter_item->GetId(),
            RasterCacheKeyID(layer->unique_id(), RasterCacheKeyType::kLayer));
  EXPECT_TRUE(raster_cache()->Draw(cacheable_color_filter_item->GetId().value(),
                                   cache_canvas, &paint));
  EXPECT_FALSE(raster_cache()->Draw(
      cacheable_color_filter_item->GetId().value(), other_canvas, &paint));
}

TEST_F(ColorFilterLayerTest, OpacityInheritance) {
  // clang-format off
  float matrix[20] = {
    0, 1, 0, 0, 0,
    0, 0, 1, 0, 0,
    1, 0, 0, 0, 0,
    0, 0, 0, 1, 0,
  };
  // clang-format on
  auto layer_filter = DlColorFilter::MakeMatrix(matrix);
  auto initial_transform = DlMatrix::MakeTranslation({50.0, 25.5});
  const DlPath child_path = DlPath::MakeRect(DlRect::MakeWH(5.0f, 5.0f));
  auto mock_layer = std::make_shared<MockLayer>(child_path);
  auto color_filter_layer =
      std::make_shared<ColorFilterLayer>(DlColorFilter::MakeMatrix(matrix));
  color_filter_layer->Add(mock_layer);

  PrerollContext* context = preroll_context();
  context->state_stack.set_preroll_delegate(initial_transform);
  color_filter_layer->Preroll(preroll_context());
  // ColorFilterLayer can always inherit opacity whether or not their
  // children are compatible.
  EXPECT_EQ(context->renderable_state_flags,
            LayerStateStack::kCallerCanApplyOpacity);

  int opacity_alpha = 0x7F;
  DlPoint offset = DlPoint(10, 10);
  auto opacity_layer = std::make_shared<OpacityLayer>(opacity_alpha, offset);
  opacity_layer->Add(color_filter_layer);
  preroll_context()->state_stack.set_preroll_delegate(DlMatrix());
  opacity_layer->Preroll(context);
  EXPECT_TRUE(opacity_layer->children_can_accept_opacity());

  DisplayListBuilder expected_builder;
  /* OpacityLayer::Paint() */ {
    expected_builder.Save();
    {
      expected_builder.Translate(offset.x, offset.y);
      /* ColorFilterLayer::Paint() */ {
        DlPaint dl_paint;
        dl_paint.setColor(DlColor(opacity_alpha << 24));
        dl_paint.setColorFilter(layer_filter);
        expected_builder.SaveLayer(child_path.GetBounds(), &dl_paint);
        /* MockLayer::Paint() */ {
          expected_builder.DrawPath(child_path, DlPaint(DlColor(0xFF000000)));
        }
        expected_builder.Restore();
      }
    }
    expected_builder.Restore();
  }

  opacity_layer->Paint(display_list_paint_context());
  EXPECT_TRUE(DisplayListsEQ_Verbose(expected_builder.Build(), display_list()));
}

}  // namespace testing
}  // namespace flutter

// NOLINTEND(bugprone-unchecked-optional-access)
