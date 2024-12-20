// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/flow/layers/image_filter_layer.h"

#include "flutter/display_list/dl_tile_mode.h"
#include "flutter/display_list/effects/dl_image_filter.h"
#include "flutter/flow/layers/layer_tree.h"
#include "flutter/flow/layers/transform_layer.h"
#include "flutter/flow/testing/diff_context_test.h"
#include "flutter/flow/testing/layer_test.h"
#include "flutter/flow/testing/mock_layer.h"
#include "flutter/fml/macros.h"
#include "gtest/gtest.h"

// TODO(zanderso): https://github.com/flutter/flutter/issues/127701
// NOLINTBEGIN(bugprone-unchecked-optional-access)

namespace flutter {
namespace testing {

using ImageFilterLayerTest = LayerTest;

#ifndef NDEBUG
TEST_F(ImageFilterLayerTest, PaintingEmptyLayerDies) {
  auto layer = std::make_shared<ImageFilterLayer>(nullptr);

  layer->Preroll(preroll_context());
  EXPECT_EQ(layer->paint_bounds(), DlRect());
  EXPECT_FALSE(layer->needs_painting(paint_context()));

  EXPECT_DEATH_IF_SUPPORTED(layer->Paint(paint_context()),
                            "needs_painting\\(context\\)");
}

TEST_F(ImageFilterLayerTest, PaintBeforePrerollDies) {
  const DlRect child_bounds = DlRect::MakeLTRB(5.0f, 6.0f, 20.5f, 21.5f);
  const DlPath child_path = DlPath::MakeRect(child_bounds);
  auto mock_layer = std::make_shared<MockLayer>(child_path);
  auto layer = std::make_shared<ImageFilterLayer>(nullptr);
  layer->Add(mock_layer);

  EXPECT_EQ(layer->paint_bounds(), DlRect());
  EXPECT_EQ(layer->child_paint_bounds(), DlRect());
  EXPECT_DEATH_IF_SUPPORTED(layer->Paint(paint_context()),
                            "needs_painting\\(context\\)");
}
#endif

TEST_F(ImageFilterLayerTest, EmptyFilter) {
  const DlMatrix initial_transform = DlMatrix::MakeTranslation({0.5f, 1.0f});
  const DlRect child_bounds = DlRect::MakeLTRB(5.0f, 6.0f, 20.5f, 21.5f);
  const DlPath child_path = DlPath::MakeRect(child_bounds);
  const DlPaint child_paint = DlPaint(DlColor::kYellow());
  auto mock_layer = std::make_shared<MockLayer>(child_path, child_paint);
  auto layer = std::make_shared<ImageFilterLayer>(nullptr);
  layer->Add(mock_layer);

  preroll_context()->state_stack.set_preroll_delegate(initial_transform);
  layer->Preroll(preroll_context());
  EXPECT_EQ(layer->paint_bounds(), child_bounds);
  EXPECT_EQ(layer->child_paint_bounds(), child_bounds);
  EXPECT_TRUE(layer->needs_painting(paint_context()));
  EXPECT_EQ(mock_layer->parent_matrix(), initial_transform);

  layer->Paint(display_list_paint_context());
  DisplayListBuilder expected_builder;
  /* (ImageFilter)layer::Paint */ {
    expected_builder.Save();
    /* mock_layer1::Paint */ {
      expected_builder.DrawPath(child_path, child_paint);
    }
    expected_builder.Restore();
  }
  EXPECT_TRUE(DisplayListsEQ_Verbose(display_list(), expected_builder.Build()));
}

TEST_F(ImageFilterLayerTest, SimpleFilter) {
  const DlMatrix initial_transform = DlMatrix::MakeTranslation({0.5f, 1.0f});
  const DlRect child_bounds = DlRect::MakeLTRB(5.0f, 6.0f, 20.5f, 21.5f);
  const DlPath child_path = DlPath::MakeRect(child_bounds);
  const DlPaint child_paint = DlPaint(DlColor::kYellow());
  auto dl_image_filter = DlImageFilter::MakeMatrix(
      DlMatrix::MakeTranslation({1.0, 2.0}), DlImageSampling::kMipmapLinear);
  auto mock_layer = std::make_shared<MockLayer>(child_path, child_paint);
  auto layer = std::make_shared<ImageFilterLayer>(dl_image_filter);
  layer->Add(mock_layer);

  const DlRect child_rounded_bounds =
      DlRect::MakeLTRB(6.0f, 8.0f, 22.0f, 24.0f);

  preroll_context()->state_stack.set_preroll_delegate(initial_transform);
  layer->Preroll(preroll_context());
  EXPECT_EQ(layer->paint_bounds(), child_rounded_bounds);
  EXPECT_EQ(layer->child_paint_bounds(), child_bounds);
  EXPECT_TRUE(layer->needs_painting(paint_context()));
  EXPECT_EQ(mock_layer->parent_matrix(), initial_transform);

  DisplayListBuilder expected_builder;
  /* ImageFilterLayer::Paint() */ {
    DlPaint dl_paint;
    dl_paint.setImageFilter(dl_image_filter.get());
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

TEST_F(ImageFilterLayerTest, SimpleFilterWithOffset) {
  const DlMatrix initial_transform = DlMatrix::MakeTranslation({0.5f, 1.0f});
  const DlRect initial_cull_rect = DlRect::MakeLTRB(0, 0, 100, 100);
  const DlRect child_bounds = DlRect::MakeLTRB(5.0f, 6.0f, 20.5f, 21.5f);
  const DlPath child_path = DlPath::MakeRect(child_bounds);
  const DlPaint child_paint = DlPaint(DlColor::kYellow());
  const DlPoint layer_offset = DlPoint(5.5, 6.5);
  auto dl_image_filter = DlImageFilter::MakeMatrix(
      DlMatrix::MakeTranslation({1.0, 2.0}), DlImageSampling::kMipmapLinear);
  auto mock_layer = std::make_shared<MockLayer>(child_path, child_paint);
  auto layer =
      std::make_shared<ImageFilterLayer>(dl_image_filter, layer_offset);
  layer->Add(mock_layer);

  DlMatrix child_matrix =
      DlMatrix::MakeTranslation(layer_offset) * initial_transform;
  const DlRect child_rounded_bounds =
      DlRect::MakeLTRB(11.5f, 14.5f, 27.5f, 30.5f);

  preroll_context()->state_stack.set_preroll_delegate(initial_cull_rect,
                                                      initial_transform);
  layer->Preroll(preroll_context());
  EXPECT_EQ(layer->paint_bounds(), child_rounded_bounds);
  EXPECT_EQ(layer->child_paint_bounds(), child_bounds);
  EXPECT_TRUE(layer->needs_painting(paint_context()));
  EXPECT_EQ(mock_layer->parent_matrix(), child_matrix);
  EXPECT_EQ(preroll_context()->state_stack.device_cull_rect(),
            initial_cull_rect);

  DisplayListBuilder expected_builder;
  /* ImageFilterLayer::Paint() */ {
    expected_builder.Save();
    {
      expected_builder.Translate(layer_offset.x, layer_offset.y);
      DlPaint dl_paint;
      dl_paint.setImageFilter(dl_image_filter.get());
      expected_builder.SaveLayer(child_bounds, &dl_paint);
      {
        /* MockLayer::Paint() */ {
          expected_builder.DrawPath(child_path, DlPaint(DlColor::kYellow()));
        }
      }
      expected_builder.Restore();
    }
    expected_builder.Restore();
  }
  auto expected_display_list = expected_builder.Build();

  layer->Paint(display_list_paint_context());
  EXPECT_TRUE(DisplayListsEQ_Verbose(display_list(), expected_display_list));
}

TEST_F(ImageFilterLayerTest, SimpleFilterBounds) {
  const DlMatrix initial_transform = DlMatrix::MakeTranslation({0.5f, 1.0f});
  const DlRect child_bounds = DlRect::MakeLTRB(5.0f, 6.0f, 20.5f, 21.5f);
  const DlPath child_path = DlPath::MakeRect(child_bounds);
  const DlPaint child_paint = DlPaint(DlColor::kYellow());
  const DlMatrix filter_transform = DlMatrix::MakeScale({2.0, 2.0, 1});

  auto dl_image_filter = DlImageFilter::MakeMatrix(
      filter_transform, DlImageSampling::kMipmapLinear);
  auto mock_layer = std::make_shared<MockLayer>(child_path, child_paint);
  auto layer = std::make_shared<ImageFilterLayer>(dl_image_filter);
  layer->Add(mock_layer);

  const DlRect filter_bounds = DlRect::MakeLTRB(10.0f, 12.0f, 42.0f, 44.0f);

  preroll_context()->state_stack.set_preroll_delegate(initial_transform);
  layer->Preroll(preroll_context());
  EXPECT_EQ(layer->paint_bounds(), filter_bounds);
  EXPECT_EQ(layer->child_paint_bounds(), child_bounds);
  EXPECT_TRUE(layer->needs_painting(paint_context()));
  EXPECT_EQ(mock_layer->parent_matrix(), initial_transform);

  DisplayListBuilder expected_builder;
  /* ImageFilterLayer::Paint() */ {
    DlPaint dl_paint;
    dl_paint.setImageFilter(dl_image_filter.get());
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

TEST_F(ImageFilterLayerTest, MultipleChildren) {
  const DlMatrix initial_transform = DlMatrix::MakeTranslation({0.5f, 1.0f});
  const DlRect child_bounds = DlRect::MakeLTRB(5.0f, 6.0f, 2.5f, 3.5f);
  const DlPath child_path1 = DlPath::MakeRect(child_bounds);
  const DlPath child_path2 = DlPath::MakeRect(child_bounds.Shift(3.0f, 0.0f));
  const DlPaint child_paint1 = DlPaint(DlColor::kYellow());
  const DlPaint child_paint2 = DlPaint(DlColor::kCyan());
  auto dl_image_filter = DlImageFilter::MakeMatrix(
      DlMatrix::MakeTranslation({1.0, 2.0}), DlImageSampling::kMipmapLinear);
  auto mock_layer1 = std::make_shared<MockLayer>(child_path1, child_paint1);
  auto mock_layer2 = std::make_shared<MockLayer>(child_path2, child_paint2);
  auto layer = std::make_shared<ImageFilterLayer>(dl_image_filter);
  layer->Add(mock_layer1);
  layer->Add(mock_layer2);

  const DlRect children_bounds =
      child_path1.GetBounds().Union(child_path2.GetBounds());
  DlRect children_rounded_bounds =
      DlRect::RoundOut(children_bounds).Shift(1.0f, 2.0f);

  preroll_context()->state_stack.set_preroll_delegate(initial_transform);
  layer->Preroll(preroll_context());
  EXPECT_EQ(mock_layer1->paint_bounds(), child_path1.GetBounds());
  EXPECT_EQ(mock_layer2->paint_bounds(), child_path2.GetBounds());
  EXPECT_EQ(layer->paint_bounds(), children_rounded_bounds);
  EXPECT_EQ(layer->child_paint_bounds(), children_bounds);
  EXPECT_TRUE(mock_layer1->needs_painting(paint_context()));
  EXPECT_TRUE(mock_layer2->needs_painting(paint_context()));
  EXPECT_TRUE(layer->needs_painting(paint_context()));
  EXPECT_EQ(mock_layer1->parent_matrix(), initial_transform);
  EXPECT_EQ(mock_layer2->parent_matrix(), initial_transform);

  DisplayListBuilder expected_builder;
  /* ImageFilterLayer::Paint() */ {
    DlPaint dl_paint;
    dl_paint.setImageFilter(dl_image_filter.get());
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

TEST_F(ImageFilterLayerTest, Nested) {
  const DlMatrix initial_transform = DlMatrix::MakeTranslation({0.5f, 1.0f});
  const DlRect child_bounds = DlRect::MakeLTRB(5.0f, 6.0f, 2.5f, 3.5f);
  const DlPath child_path1 = DlPath::MakeRect(child_bounds);
  const DlPath child_path2 = DlPath::MakeRect(child_bounds.Shift(3.0f, 0.0f));
  const DlPaint child_paint1 = DlPaint(DlColor::kYellow());
  const DlPaint child_paint2 = DlPaint(DlColor::kCyan());
  auto dl_image_filter1 = DlImageFilter::MakeMatrix(
      DlMatrix::MakeTranslation({1.0, 2.0}), DlImageSampling::kMipmapLinear);
  auto dl_image_filter2 = DlImageFilter::MakeMatrix(
      DlMatrix::MakeTranslation({3.0, 4.0}), DlImageSampling::kMipmapLinear);
  auto mock_layer1 = std::make_shared<MockLayer>(child_path1, child_paint1);
  auto mock_layer2 = std::make_shared<MockLayer>(child_path2, child_paint2);
  auto layer1 = std::make_shared<ImageFilterLayer>(dl_image_filter1);
  auto layer2 = std::make_shared<ImageFilterLayer>(dl_image_filter2);
  layer2->Add(mock_layer2);
  layer1->Add(mock_layer1);
  layer1->Add(layer2);

  //             Filter(translate by 1, 2)
  //             /                     |
  //   Mock(child_path1)     Filter(translate by 3, 4)
  //                                   |
  //                      Mock(child_path2 (shifted (3, 0)))

  DlRect filter2_bounds = DlRect::RoundOut(  //
      child_path2
          .GetBounds()      // includes shift(3, 0) on child_path2
          .Shift(3.0, 4.0)  // filter2 translation
  );
  const DlRect filter1_child_bounds =
      child_path1.GetBounds().Union(filter2_bounds);
  DlRect filter1_bounds = DlRect::RoundOut(  //
      filter1_child_bounds                   // no shift on child_path1
          .Shift(1.0, 2.0)                   // filter1 translation
  );

  preroll_context()->state_stack.set_preroll_delegate(initial_transform);
  layer1->Preroll(preroll_context());
  EXPECT_EQ(mock_layer1->paint_bounds(), child_path1.GetBounds());
  EXPECT_EQ(mock_layer2->paint_bounds(), child_path2.GetBounds());
  EXPECT_EQ(layer1->paint_bounds(), filter1_bounds);
  EXPECT_EQ(layer1->child_paint_bounds(), filter1_child_bounds);
  EXPECT_EQ(layer2->paint_bounds(), filter2_bounds);
  EXPECT_EQ(layer2->child_paint_bounds(), child_path2.GetBounds());
  EXPECT_TRUE(mock_layer1->needs_painting(paint_context()));
  EXPECT_TRUE(mock_layer2->needs_painting(paint_context()));
  EXPECT_TRUE(layer1->needs_painting(paint_context()));
  EXPECT_TRUE(layer2->needs_painting(paint_context()));
  EXPECT_EQ(mock_layer1->parent_matrix(), initial_transform);
  EXPECT_EQ(mock_layer2->parent_matrix(), initial_transform);

  DisplayListBuilder expected_builder;
  /* ImageFilterLayer::Paint() */ {
    DlPaint dl_paint;
    dl_paint.setImageFilter(dl_image_filter1.get());
    expected_builder.SaveLayer(filter1_child_bounds, &dl_paint);
    {
      /* MockLayer::Paint() */ {
        expected_builder.DrawPath(child_path1, DlPaint(DlColor::kYellow()));
      }
      /* ImageFilterLayer::Paint() */ {
        DlPaint child_paint;
        child_paint.setImageFilter(dl_image_filter2.get());
        expected_builder.SaveLayer(child_path2.GetBounds(), &child_paint);
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

TEST_F(ImageFilterLayerTest, Readback) {
  auto dl_image_filter = DlImageFilter::MakeMatrix(
      DlMatrix::MakeTranslation({1.0, 2.0}), DlImageSampling::kLinear);

  // ImageFilterLayer does not read from surface
  auto layer = std::make_shared<ImageFilterLayer>(dl_image_filter);
  preroll_context()->surface_needs_readback = false;
  layer->Preroll(preroll_context());
  EXPECT_FALSE(preroll_context()->surface_needs_readback);

  // ImageFilterLayer blocks child with readback
  auto mock_layer = std::make_shared<MockLayer>(DlPath(), DlPaint());
  mock_layer->set_fake_reads_surface(true);
  layer->Add(mock_layer);
  preroll_context()->surface_needs_readback = false;
  layer->Preroll(preroll_context());
  EXPECT_FALSE(preroll_context()->surface_needs_readback);
}

TEST_F(ImageFilterLayerTest, CacheChild) {
  auto dl_image_filter = DlImageFilter::MakeMatrix(
      DlMatrix::MakeTranslation({1.0f, 2.0f}), DlImageSampling::kMipmapLinear);
  auto initial_transform = DlMatrix::MakeTranslation({50.0f, 25.5f});
  auto other_transform = DlMatrix::MakeScale({1.0f, 2.0f, 1.0f});
  const DlPath child_path = DlPath::MakeRect(DlRect::MakeWH(5.0f, 5.0f));
  auto mock_layer = std::make_shared<MockLayer>(child_path);
  auto layer = std::make_shared<ImageFilterLayer>(dl_image_filter);
  layer->Add(mock_layer);

  DlMatrix cache_ctm = initial_transform;
  DisplayListBuilder cache_canvas;
  cache_canvas.Transform(cache_ctm);
  DisplayListBuilder other_canvas;
  other_canvas.Transform(other_transform);
  DlPaint paint;

  use_mock_raster_cache();
  const auto* cacheable_image_filter_item = layer->raster_cache_item();

  EXPECT_EQ(raster_cache()->GetLayerCachedEntriesCount(), (size_t)0);
  // ImageFilterLayer default cache itself.
  EXPECT_EQ(cacheable_image_filter_item->cache_state(),
            RasterCacheItem::CacheState::kNone);
  EXPECT_FALSE(cacheable_image_filter_item->Draw(paint_context(), &paint));

  preroll_context()->state_stack.set_preroll_delegate(initial_transform);
  layer->Preroll(preroll_context());
  LayerTree::TryToRasterCache(cacheable_items(), &paint_context());

  EXPECT_EQ(raster_cache()->GetLayerCachedEntriesCount(), (size_t)1);
  // The layer_cache_item's strategy is Children, mean we will must cache
  // his children
  EXPECT_EQ(cacheable_image_filter_item->cache_state(),
            RasterCacheItem::CacheState::kChildren);
  // We assert here because the lines after it will crash if this is not true
  // (It will generally only fail if another EXPECT test above also fails)
  ASSERT_TRUE(cacheable_image_filter_item->GetId().has_value());
  EXPECT_TRUE(raster_cache()->Draw(cacheable_image_filter_item->GetId().value(),
                                   cache_canvas, &paint));
  EXPECT_FALSE(raster_cache()->Draw(
      cacheable_image_filter_item->GetId().value(), other_canvas, &paint));
}

TEST_F(ImageFilterLayerTest, CacheChildren) {
  auto dl_image_filter = DlImageFilter::MakeMatrix(
      DlMatrix::MakeTranslation({1.0f, 2.0f}), DlImageSampling::kMipmapLinear);
  auto initial_transform = DlMatrix::MakeTranslation({50.0f, 25.5f});
  auto other_transform = DlMatrix::MakeScale({1.0f, 2.0f, 1.0f});
  DlPaint paint;
  const DlPath child_path1 = DlPath::MakeRect(DlRect::MakeWH(5.0f, 5.0f));
  const DlPath child_path2 = DlPath::MakeRect(DlRect::MakeWH(5.0f, 5.0f));
  auto mock_layer1 = std::make_shared<MockLayer>(child_path1);
  auto mock_layer2 = std::make_shared<MockLayer>(child_path2);
  auto offset = DlPoint(54, 24);
  auto layer = std::make_shared<ImageFilterLayer>(dl_image_filter, offset);
  layer->Add(mock_layer1);
  layer->Add(mock_layer2);

  DlMatrix cache_ctm = initial_transform;
  DisplayListBuilder cache_canvas;
  cache_canvas.Transform(cache_ctm);
  DisplayListBuilder other_canvas;
  other_canvas.Transform(other_transform);

  use_mock_raster_cache();

  const auto* cacheable_image_filter_item = layer->raster_cache_item();
  EXPECT_EQ(raster_cache()->GetLayerCachedEntriesCount(), (size_t)0);

  // ImageFilterLayer default cache itself.
  EXPECT_EQ(cacheable_image_filter_item->cache_state(),
            RasterCacheItem::CacheState::kNone);
  EXPECT_FALSE(cacheable_image_filter_item->Draw(paint_context(), &paint));

  preroll_context()->state_stack.set_preroll_delegate(initial_transform);
  layer->Preroll(preroll_context());
  LayerTree::TryToRasterCache(cacheable_items(), &paint_context());

  EXPECT_EQ(raster_cache()->GetLayerCachedEntriesCount(), (size_t)1);

  // The layer_cache_item's strategy is Children, mean we will must cache his
  // children
  EXPECT_EQ(cacheable_image_filter_item->cache_state(),
            RasterCacheItem::CacheState::kChildren);
  // We assert here because the lines after it will crash if this is not true
  // (It will generally only fail if another EXPECT test above also fails)
  ASSERT_TRUE(cacheable_image_filter_item->GetId().has_value());
  EXPECT_TRUE(raster_cache()->Draw(cacheable_image_filter_item->GetId().value(),
                                   cache_canvas, &paint));
  EXPECT_FALSE(raster_cache()->Draw(
      cacheable_image_filter_item->GetId().value(), other_canvas, &paint));

  layer->Preroll(preroll_context());

  DlMatrix snapped_matrix = DlMatrix::MakeTranslation(offset.Round());
  DlMatrix cache_matrix = snapped_matrix * initial_transform;
  auto transformed_filter = dl_image_filter->makeWithLocalMatrix(cache_matrix);

  layer->Paint(display_list_paint_context());
  DisplayListBuilder expected_builder;
  /* (ImageFilter)layer::Paint() */ {
    expected_builder.Save();
    {
      expected_builder.Translate(offset.x, offset.y);
      // translation components already snapped to pixels, intent to
      // use raster cache won't change them
      DlPaint dl_paint;
      dl_paint.setImageFilter(transformed_filter.get());
      raster_cache()->Draw(cacheable_image_filter_item->GetId().value(),
                           expected_builder, &dl_paint);
    }
    expected_builder.Restore();
  }
  expected_builder.Restore();
  EXPECT_TRUE(DisplayListsEQ_Verbose(display_list(), expected_builder.Build()));
}

TEST_F(ImageFilterLayerTest, CacheImageFilterLayerSelf) {
  auto dl_image_filter = DlImageFilter::MakeMatrix(
      DlMatrix::MakeTranslation({1.0, 2.0}), DlImageSampling::kMipmapLinear);

  auto initial_transform = DlMatrix::MakeTranslation({50.0f, 25.5f});
  auto other_transform = DlMatrix::MakeScale({1.0f, 2.0f, 1.0f});
  auto child_rect = DlRect::MakeWH(5.0f, 5.0f);
  const DlPath child_path = DlPath::MakeRect(child_rect);
  auto mock_layer = std::make_shared<MockLayer>(child_path);
  auto offset = DlPoint(53.8, 24.4);
  auto layer = std::make_shared<ImageFilterLayer>(dl_image_filter, offset);
  layer->Add(mock_layer);

  DlMatrix cache_ctm = initial_transform;
  DisplayListBuilder cache_canvas;
  cache_canvas.Transform(cache_ctm);
  DisplayListBuilder other_canvas;
  other_canvas.Transform(other_transform);
  DlPaint paint;

  DlMatrix snapped_matrix = DlMatrix::MakeTranslation(offset.Round());

  use_mock_raster_cache();
  preroll_context()->state_stack.set_preroll_delegate(initial_transform);
  const auto* cacheable_image_filter_item = layer->raster_cache_item();
  // frame 1.
  layer->Preroll(preroll_context());

  layer->Paint(display_list_paint_context());
  {
    DisplayListBuilder expected_builder;
    /* (ImageFilter)layer::Paint */ {
      expected_builder.Save();
      {
        expected_builder.Translate(offset.x, offset.y);
        // Snap to pixel translation due to use of raster cache
        expected_builder.TransformReset();
        expected_builder.Transform(snapped_matrix);
        DlPaint save_paint = DlPaint().setImageFilter(dl_image_filter);
        expected_builder.SaveLayer(child_rect, &save_paint);
        {
          /* mock_layer::Paint */ {
            expected_builder.DrawPath(child_path, DlPaint());
          }
        }
        expected_builder.Restore();
      }
      expected_builder.Restore();
    }
    EXPECT_TRUE(
        DisplayListsEQ_Verbose(display_list(), expected_builder.Build()));
  }

  // frame 2.
  layer->Preroll(preroll_context());
  layer->Paint(display_list_paint_context());
  // frame 3.
  layer->Preroll(preroll_context());
  layer->Paint(display_list_paint_context());

  LayerTree::TryToRasterCache(cacheable_items(), &paint_context());
  // frame1,2 cache the ImageFilter's children layer, frame3 cache the
  // ImageFilterLayer
  EXPECT_EQ(raster_cache()->GetLayerCachedEntriesCount(), (size_t)2);

  // ImageFilterLayer default cache itself.
  EXPECT_EQ(cacheable_image_filter_item->cache_state(),
            RasterCacheItem::CacheState::kCurrent);
  EXPECT_EQ(cacheable_image_filter_item->GetId(),
            RasterCacheKeyID(layer->unique_id(), RasterCacheKeyType::kLayer));
  EXPECT_TRUE(raster_cache()->Draw(cacheable_image_filter_item->GetId().value(),
                                   cache_canvas, &paint));
  EXPECT_FALSE(raster_cache()->Draw(
      cacheable_image_filter_item->GetId().value(), other_canvas, &paint));

  layer->Preroll(preroll_context());

  reset_display_list();
  layer->Paint(display_list_paint_context());
  {
    DisplayListBuilder expected_builder;
    /* (ImageFilter)layer::Paint */ {
      expected_builder.Save();
      {
        EXPECT_TRUE(
            raster_cache()->Draw(cacheable_image_filter_item->GetId().value(),
                                 expected_builder, nullptr));
      }
      expected_builder.Restore();
    }
    EXPECT_TRUE(
        DisplayListsEQ_Verbose(display_list(), expected_builder.Build()));
  }
}

TEST_F(ImageFilterLayerTest, OpacityInheritance) {
  const DlMatrix initial_transform = DlMatrix::MakeTranslation({0.5f, 1.0f});
  const DlRect child_bounds = DlRect::MakeLTRB(5.0f, 6.0f, 20.5f, 21.5f);
  const DlPath child_path = DlPath::MakeRect(child_bounds);
  const DlPaint child_paint = DlPaint(DlColor::kYellow());
  auto dl_image_filter = DlImageFilter::MakeMatrix(
      DlMatrix::MakeTranslation({1.0, 2.0}), DlImageSampling::kMipmapLinear);

  // The mock_layer child will not be compatible with opacity
  auto mock_layer = MockLayer::Make(child_path, child_paint);
  auto image_filter_layer = std::make_shared<ImageFilterLayer>(dl_image_filter);
  image_filter_layer->Add(mock_layer);

  PrerollContext* context = preroll_context();
  context->state_stack.set_preroll_delegate(initial_transform);
  image_filter_layer->Preroll(preroll_context());
  // ImageFilterLayers can always inherit opacity whether or not their
  // children are compatible.
  EXPECT_EQ(context->renderable_state_flags,
            LayerStateStack::kCallerCanApplyOpacity |
                LayerStateStack::kCallerCanApplyColorFilter);

  int opacity_alpha = 0x7F;
  DlPoint offset = DlPoint(10, 10);
  auto opacity_layer = std::make_shared<OpacityLayer>(opacity_alpha, offset);
  opacity_layer->Add(image_filter_layer);
  context->state_stack.set_preroll_delegate(DlMatrix());
  opacity_layer->Preroll(context);
  EXPECT_TRUE(opacity_layer->children_can_accept_opacity());

  DisplayListBuilder expected_builder;
  /* OpacityLayer::Paint() */ {
    expected_builder.Save();
    {
      expected_builder.Translate(offset.x, offset.y);
      /* ImageFilterLayer::Paint() */ {
        DlPaint image_filter_paint;
        image_filter_paint.setColor(DlColor(opacity_alpha << 24));
        image_filter_paint.setImageFilter(dl_image_filter.get());
        expected_builder.SaveLayer(child_path.GetBounds(), &image_filter_paint);
        /* MockLayer::Paint() */ {
          expected_builder.DrawPath(child_path,
                                    DlPaint(child_paint.getColor()));
        }
        expected_builder.Restore();
      }
    }
    expected_builder.Restore();
  }

  opacity_layer->Paint(display_list_paint_context());
  EXPECT_TRUE(DisplayListsEQ_Verbose(expected_builder.Build(), display_list()));
}

using ImageFilterLayerDiffTest = DiffContextTest;

TEST_F(ImageFilterLayerDiffTest, ImageFilterLayer) {
  auto dl_blur_filter = DlImageFilter::MakeBlur(10, 10, DlTileMode::kClamp);
  {
    // tests later assume 30px paint area, fail early if that's not the case
    DlIRect input_bounds;
    dl_blur_filter->get_input_device_bounds(DlIRect::MakeWH(10, 10), DlMatrix(),
                                            input_bounds);
    EXPECT_EQ(input_bounds, DlIRect::MakeLTRB(-30, -30, 40, 40));
  }

  MockLayerTree l1;
  auto filter_layer = std::make_shared<ImageFilterLayer>(dl_blur_filter);
  auto path = DlPath::MakeRectLTRB(100, 100, 110, 110);
  filter_layer->Add(std::make_shared<MockLayer>(path));
  l1.root()->Add(filter_layer);

  auto damage = DiffLayerTree(l1, MockLayerTree());
  EXPECT_EQ(damage.frame_damage, DlIRect::MakeLTRB(70, 70, 140, 140));

  MockLayerTree l2;
  auto scale =
      std::make_shared<TransformLayer>(DlMatrix::MakeScale({2.0f, 2.0f, 1.0f}));
  scale->Add(filter_layer);
  l2.root()->Add(scale);

  damage = DiffLayerTree(l2, MockLayerTree());
  EXPECT_EQ(damage.frame_damage, DlIRect::MakeLTRB(140, 140, 280, 280));

  MockLayerTree l3;
  l3.root()->Add(scale);

  // path outside of ImageFilterLayer
  auto path1 = DlPath::MakeRectLTRB(130, 130, 140, 140);
  l3.root()->Add(std::make_shared<MockLayer>(path1));
  damage = DiffLayerTree(l3, l2);
  EXPECT_EQ(damage.frame_damage, DlIRect::MakeLTRB(130, 130, 140, 140));

  // path intersecting ImageFilterLayer, shouldn't trigger entire
  // ImageFilterLayer repaint
  MockLayerTree l4;
  l4.root()->Add(scale);
  auto path2 = DlPath::MakeRectLTRB(130, 130, 141, 141);
  l4.root()->Add(std::make_shared<MockLayer>(path2));
  damage = DiffLayerTree(l4, l3);
  EXPECT_EQ(damage.frame_damage, DlIRect::MakeLTRB(130, 130, 141, 141));
}

TEST_F(ImageFilterLayerDiffTest, ImageFilterLayerInflatestChildSize) {
  auto dl_blur_filter = DlImageFilter::MakeBlur(10, 10, DlTileMode::kClamp);

  {
    // tests later assume 30px paint area, fail early if that's not the case
    DlIRect input_bounds;
    dl_blur_filter->get_input_device_bounds(DlIRect::MakeWH(10, 10), DlMatrix(),
                                            input_bounds);
    EXPECT_EQ(input_bounds, DlIRect::MakeLTRB(-30, -30, 40, 40));
  }

  MockLayerTree l1;

  // Use nested filter layers to check if both contribute to child bounds
  auto filter_layer_1_1 = std::make_shared<ImageFilterLayer>(dl_blur_filter);
  auto filter_layer_1_2 = std::make_shared<ImageFilterLayer>(dl_blur_filter);
  filter_layer_1_1->Add(filter_layer_1_2);
  auto path = DlPath::MakeRectLTRB(100, 100, 110, 110);
  filter_layer_1_2->Add(
      std::make_shared<MockLayer>(path, DlPaint(DlColor::kYellow())));
  l1.root()->Add(filter_layer_1_1);

  // second layer tree with identical filter layers but different child layer
  MockLayerTree l2;
  auto filter_layer2_1 = std::make_shared<ImageFilterLayer>(dl_blur_filter);
  filter_layer2_1->AssignOldLayer(filter_layer_1_1.get());
  auto filter_layer2_2 = std::make_shared<ImageFilterLayer>(dl_blur_filter);
  filter_layer2_2->AssignOldLayer(filter_layer_1_2.get());
  filter_layer2_1->Add(filter_layer2_2);
  filter_layer2_2->Add(
      std::make_shared<MockLayer>(path, DlPaint(DlColor::kRed())));
  l2.root()->Add(filter_layer2_1);

  DiffLayerTree(l1, MockLayerTree());
  auto damage = DiffLayerTree(l2, l1);

  // ensure that filter properly inflated child size
  EXPECT_EQ(damage.frame_damage, DlIRect::MakeLTRB(40, 40, 170, 170));
}

TEST_F(ImageFilterLayerTest, EmptyFilterWithOffset) {
  const DlRect child_bounds = DlRect::MakeLTRB(10.0f, 11.0f, 19.0f, 20.0f);
  const DlPath child_path = DlPath::MakeRect(child_bounds);
  const DlPaint child_paint = DlPaint(DlColor::kYellow());
  auto mock_layer = std::make_shared<MockLayer>(child_path, child_paint);
  const DlPoint offset = DlPoint(5.0f, 6.0f);
  auto layer = std::make_shared<ImageFilterLayer>(nullptr, offset);
  layer->Add(mock_layer);

  layer->Preroll(preroll_context());
  EXPECT_EQ(layer->paint_bounds(), child_bounds.Shift(offset));
}

}  // namespace testing
}  // namespace flutter

// NOLINTEND(bugprone-unchecked-optional-access)
