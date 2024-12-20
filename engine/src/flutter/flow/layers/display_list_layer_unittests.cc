// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#define FML_USED_ON_EMBEDDER

#include "flutter/flow/layers/display_list_layer.h"

#include "flutter/display_list/dl_builder.h"
#include "flutter/flow/layers/layer_tree.h"
#include "flutter/flow/testing/diff_context_test.h"
#include "flutter/fml/macros.h"

// TODO(zanderso): https://github.com/flutter/flutter/issues/127701
// NOLINTBEGIN(bugprone-unchecked-optional-access)

namespace flutter {
namespace testing {

using DisplayListLayerTest = LayerTest;

#ifndef NDEBUG
TEST_F(DisplayListLayerTest, PaintBeforePrerollInvalidDisplayListDies) {
  const DlPoint layer_offset = DlPoint(0.0f, 0.0f);
  auto layer = std::make_shared<DisplayListLayer>(
      layer_offset, sk_sp<DisplayList>(), false, false);

  EXPECT_DEATH_IF_SUPPORTED(layer->Paint(paint_context()), "display_list_");
}

TEST_F(DisplayListLayerTest, PaintBeforePrerollDies) {
  const DlPoint layer_offset = DlPoint(0.0f, 0.0f);
  const DlRect picture_bounds = DlRect::MakeLTRB(5.0f, 6.0f, 20.5f, 21.5f);
  DisplayListBuilder builder;
  builder.DrawRect(picture_bounds, DlPaint());
  auto display_list = builder.Build();
  auto layer = std::make_shared<DisplayListLayer>(layer_offset, display_list,
                                                  false, false);

  EXPECT_EQ(layer->paint_bounds(), DlRect());
  EXPECT_DEATH_IF_SUPPORTED(layer->Paint(paint_context()),
                            "needs_painting\\(context\\)");
}

TEST_F(DisplayListLayerTest, PaintingEmptyLayerDies) {
  const DlPoint layer_offset = DlPoint(0.0f, 0.0f);
  const DlRect picture_bounds;
  DisplayListBuilder builder;
  builder.DrawRect(picture_bounds, DlPaint());
  auto display_list = builder.Build();
  auto layer = std::make_shared<DisplayListLayer>(layer_offset, display_list,
                                                  false, false);

  layer->Preroll(preroll_context());
  EXPECT_EQ(layer->paint_bounds(), DlRect());
  EXPECT_FALSE(layer->needs_painting(paint_context()));

  EXPECT_DEATH_IF_SUPPORTED(layer->Paint(paint_context()),
                            "needs_painting\\(context\\)");
}

TEST_F(DisplayListLayerTest, InvalidDisplayListDies) {
  const DlPoint layer_offset = DlPoint(0.0f, 0.0f);
  auto layer = std::make_shared<DisplayListLayer>(
      layer_offset, sk_sp<DisplayList>(), false, false);

  // Crashes reading a nullptr.
  EXPECT_DEATH_IF_SUPPORTED(layer->Preroll(preroll_context()), "");
}
#endif

TEST_F(DisplayListLayerTest, SimpleDisplayList) {
  const DlPoint layer_offset = DlPoint(1.5f, -0.5f);
  const DlRect picture_bounds = DlRect::MakeLTRB(5.0f, 6.0f, 20.5f, 21.5f);
  DisplayListBuilder builder;
  builder.DrawRect(picture_bounds, DlPaint());
  auto display_list = builder.Build();
  auto layer = std::make_shared<DisplayListLayer>(layer_offset, display_list,
                                                  false, false);

  layer->Preroll(preroll_context());
  EXPECT_EQ(layer->paint_bounds(),
            picture_bounds.Shift(layer_offset.x, layer_offset.y));
  EXPECT_EQ(layer->display_list(), display_list.get());
  EXPECT_TRUE(layer->needs_painting(paint_context()));

  layer->Paint(display_list_paint_context());
  DisplayListBuilder expected_builder;
  /* (DisplayList)layer::Paint */ {
    expected_builder.Save();
    {
      expected_builder.Translate(layer_offset.x, layer_offset.y);
      expected_builder.DrawDisplayList(display_list);
    }
    expected_builder.Restore();
  }
  EXPECT_TRUE(
      DisplayListsEQ_Verbose(this->display_list(), expected_builder.Build()));
}

TEST_F(DisplayListLayerTest, CachingDoesNotChangeCullRect) {
  const DlPoint layer_offset = DlPoint(10, 10);
  DisplayListBuilder builder;
  builder.DrawRect(DlRect::MakeLTRB(10, 10, 20, 20), DlPaint());
  auto display_list = builder.Build();
  auto layer = std::make_shared<DisplayListLayer>(layer_offset, display_list,
                                                  true, false);

  DlRect original_cull_rect = preroll_context()->state_stack.device_cull_rect();
  use_mock_raster_cache();
  layer->Preroll(preroll_context());
  ASSERT_EQ(preroll_context()->state_stack.device_cull_rect(),
            original_cull_rect);
}

TEST_F(DisplayListLayerTest, SimpleDisplayListOpacityInheritance) {
  const DlPoint layer_offset = DlPoint(1.5f, -0.5f);
  const DlRect picture_bounds = DlRect::MakeLTRB(5.0f, 6.0f, 20.5f, 21.5f);
  DisplayListBuilder builder;
  builder.DrawRect(picture_bounds, DlPaint());
  auto display_list = builder.Build();
  auto display_list_layer = std::make_shared<DisplayListLayer>(
      layer_offset, display_list, false, false);
  EXPECT_TRUE(display_list->can_apply_group_opacity());

  auto context = preroll_context();
  display_list_layer->Preroll(preroll_context());
  EXPECT_EQ(context->renderable_state_flags,
            LayerStateStack::kCallerCanApplyOpacity);

  int opacity_alpha = 0x7F;
  DlScalar opacity = DlColor::toOpacity(opacity_alpha);
  DlPoint opacity_offset = DlPoint(10, 10);
  auto opacity_layer =
      std::make_shared<OpacityLayer>(opacity_alpha, opacity_offset);
  opacity_layer->Add(display_list_layer);
  opacity_layer->Preroll(context);
  EXPECT_TRUE(opacity_layer->children_can_accept_opacity());

  DisplayListBuilder child_builder;
  child_builder.DrawRect(picture_bounds, DlPaint());
  auto child_display_list = child_builder.Build();

  DisplayListBuilder expected_builder;
  /* opacity_layer::Paint() */ {
    expected_builder.Save();
    {
      expected_builder.Translate(opacity_offset.x, opacity_offset.y);
      /* display_list_layer::Paint() */ {
        expected_builder.Save();
        {
          expected_builder.Translate(layer_offset.x, layer_offset.y);
          expected_builder.DrawDisplayList(child_display_list, opacity);
        }
        expected_builder.Restore();
      }
    }
    expected_builder.Restore();
  }

  opacity_layer->Paint(display_list_paint_context());
  EXPECT_TRUE(
      DisplayListsEQ_Verbose(this->display_list(), expected_builder.Build()));
}

TEST_F(DisplayListLayerTest, IncompatibleDisplayListOpacityInheritance) {
  const DlPoint layer_offset = DlPoint(1.5f, -0.5f);
  const DlRect picture1_bounds = DlRect::MakeLTRB(5.0f, 6.0f, 20.5f, 21.5f);
  const DlRect picture2_bounds = DlRect::MakeLTRB(10.0f, 15.0f, 30.0f, 35.0f);
  DisplayListBuilder builder;
  builder.DrawRect(picture1_bounds, DlPaint());
  builder.DrawRect(picture2_bounds, DlPaint());
  auto display_list = builder.Build();
  auto display_list_layer = std::make_shared<DisplayListLayer>(
      layer_offset, display_list, false, false);
  EXPECT_FALSE(display_list->can_apply_group_opacity());

  auto context = preroll_context();
  display_list_layer->Preroll(preroll_context());
  EXPECT_EQ(context->renderable_state_flags, 0);

  int opacity_alpha = 0x7F;
  DlPoint opacity_offset = DlPoint(10, 10);
  auto opacity_layer =
      std::make_shared<OpacityLayer>(opacity_alpha, opacity_offset);
  opacity_layer->Add(display_list_layer);
  opacity_layer->Preroll(context);
  EXPECT_FALSE(opacity_layer->children_can_accept_opacity());

  DisplayListBuilder child_builder;
  child_builder.DrawRect(picture1_bounds, DlPaint());
  child_builder.DrawRect(picture2_bounds, DlPaint());
  auto child_display_list = child_builder.Build();

  auto display_list_bounds = picture1_bounds.Union(picture2_bounds);
  auto save_layer_bounds =
      display_list_bounds.Shift(layer_offset.x, layer_offset.y);
  DisplayListBuilder expected_builder;
  /* opacity_layer::Paint() */ {
    expected_builder.Save();
    {
      expected_builder.Translate(opacity_offset.x, opacity_offset.y);
      expected_builder.SaveLayer(save_layer_bounds,
                                 &DlPaint().setAlpha(opacity_alpha));
      {
        /* display_list_layer::Paint() */ {
          expected_builder.Save();
          {
            expected_builder.Translate(layer_offset.x, layer_offset.y);
            expected_builder.DrawDisplayList(child_display_list);
          }
          expected_builder.Restore();
        }
      }
      expected_builder.Restore();
    }
    expected_builder.Restore();
  }

  opacity_layer->Paint(display_list_paint_context());
  EXPECT_TRUE(
      DisplayListsEQ_Verbose(this->display_list(), expected_builder.Build()));
}

TEST_F(DisplayListLayerTest, CachedIncompatibleDisplayListOpacityInheritance) {
  const DlPoint layer_offset = DlPoint(1.5f, -0.5f);
  const DlRect picture1_bounds = DlRect::MakeLTRB(5.0f, 6.0f, 20.5f, 21.5f);
  const DlRect picture2_bounds = DlRect::MakeLTRB(10.0f, 15.0f, 30.0f, 35.0f);
  DisplayListBuilder builder;
  builder.DrawRect(picture1_bounds, DlPaint());
  builder.DrawRect(picture2_bounds, DlPaint());
  auto display_list = builder.Build();
  auto display_list_layer = std::make_shared<DisplayListLayer>(
      layer_offset, display_list, true, false);
  EXPECT_FALSE(display_list->can_apply_group_opacity());

  use_skia_raster_cache();

  auto context = preroll_context();
  display_list_layer->Preroll(preroll_context());
  EXPECT_EQ(context->renderable_state_flags, 0);

  // Pump the DisplayListLayer until it is ready to cache its DL
  display_list_layer->Preroll(preroll_context());
  display_list_layer->Preroll(preroll_context());
  display_list_layer->Preroll(preroll_context());
  LayerTree::TryToRasterCache(*preroll_context()->raster_cached_entries,
                              &paint_context(), false);

  int opacity_alpha = 0x7F;
  DlPoint opacity_offset = DlPoint(10.2, 10.2);
  auto opacity_layer =
      std::make_shared<OpacityLayer>(opacity_alpha, opacity_offset);
  opacity_layer->Add(display_list_layer);
  opacity_layer->Preroll(context);
  EXPECT_TRUE(opacity_layer->children_can_accept_opacity());

  auto opacity_integral_matrix = RasterCacheUtil::GetIntegralTransCTM(
      DlMatrix::MakeTranslation(opacity_offset));
  DlMatrix layer_offset_matrix =
      DlMatrix::MakeTranslation(layer_offset) * opacity_integral_matrix;
  auto layer_offset_integral_matrix =
      RasterCacheUtil::GetIntegralTransCTM(layer_offset_matrix);
  DisplayListBuilder expected(DlRect::MakeWH(1000, 1000));
  /* opacity_layer::Paint() */ {
    expected.Save();
    {
      expected.Translate(opacity_offset.x, opacity_offset.y);
      expected.TransformReset();
      expected.Transform(opacity_integral_matrix);
      /* display_list_layer::Paint() */ {
        expected.Save();
        {
          expected.Translate(layer_offset.x, layer_offset.y);
          expected.TransformReset();
          expected.Transform(layer_offset_integral_matrix);
          context->raster_cache->Draw(display_list_layer->caching_key_id(),
                                      expected,
                                      &DlPaint().setAlpha(opacity_alpha));
        }
        expected.Restore();
      }
    }
    expected.Restore();
  }

  opacity_layer->Paint(display_list_paint_context());
  EXPECT_TRUE(DisplayListsEQ_Verbose(expected.Build(), this->display_list()));
}

TEST_F(DisplayListLayerTest, RasterCachePreservesRTree) {
  const DlRect picture1_bounds = DlRect::MakeXYWH(10, 10, 10, 10);
  const DlRect picture2_bounds = DlRect::MakeXYWH(15, 15, 10, 10);
  DisplayListBuilder builder(true);
  builder.DrawRect(picture1_bounds, DlPaint());
  builder.DrawRect(picture2_bounds, DlPaint());
  auto display_list = builder.Build();
  auto display_list_layer = std::make_shared<DisplayListLayer>(
      DlPoint(3, 3), display_list, true, false);

  use_skia_raster_cache();

  auto context = preroll_context();
  {
    auto mutator = context->state_stack.save();
    mutator.transform(DlMatrix::MakeScale({2.0f, 2.0f, 1.0f}));
    display_list_layer->Preroll(preroll_context());
    EXPECT_EQ(context->renderable_state_flags, 0);

    // Pump the DisplayListLayer until it is ready to cache its DL
    display_list_layer->Preroll(preroll_context());
    display_list_layer->Preroll(preroll_context());
    display_list_layer->Preroll(preroll_context());
    LayerTree::TryToRasterCache(*preroll_context()->raster_cached_entries,
                                &paint_context(), false);
  }

  DisplayListBuilder expected_root_canvas(true);
  expected_root_canvas.Scale(2.0, 2.0);
  ASSERT_TRUE(context->raster_cache->Draw(display_list_layer->caching_key_id(),
                                          expected_root_canvas, nullptr,
                                          false));
  auto root_canvas_dl = expected_root_canvas.Build();
  const auto root_canvas_rects =
      root_canvas_dl->rtree()->searchAndConsolidateRects(kGiantRect, true);
  std::list<DlRect> root_canvas_rects_expected = {
      DlRect::MakeLTRB(26, 26, 56, 56),
  };
  EXPECT_EQ(root_canvas_rects_expected, root_canvas_rects);

  DisplayListBuilder expected_overlay_canvas(true);
  expected_overlay_canvas.Scale(2.0, 2.0);
  ASSERT_TRUE(context->raster_cache->Draw(display_list_layer->caching_key_id(),
                                          expected_overlay_canvas, nullptr,
                                          true));
  auto overlay_canvas_dl = expected_overlay_canvas.Build();
  const auto overlay_canvas_rects =
      overlay_canvas_dl->rtree()->searchAndConsolidateRects(kGiantRect, true);

  // Same bounds as root canvas, but preserves individual rects.
  std::list<DlRect> overlay_canvas_rects_expected = {
      DlRect::MakeLTRB(26, 26, 46, 36),
      DlRect::MakeLTRB(26, 36, 56, 46),
      DlRect::MakeLTRB(36, 46, 56, 56),
  };
  EXPECT_EQ(overlay_canvas_rects_expected, overlay_canvas_rects);
};

using DisplayListLayerDiffTest = DiffContextTest;

TEST_F(DisplayListLayerDiffTest, SimpleDisplayList) {
  auto display_list = CreateDisplayList(DlRect::MakeLTRB(10, 10, 60, 60));

  MockLayerTree tree1;
  tree1.root()->Add(CreateDisplayListLayer(display_list));

  auto damage = DiffLayerTree(tree1, MockLayerTree());
  EXPECT_EQ(damage.frame_damage, DlIRect::MakeLTRB(10, 10, 60, 60));

  MockLayerTree tree2;
  tree2.root()->Add(CreateDisplayListLayer(display_list));

  damage = DiffLayerTree(tree2, tree1);
  EXPECT_TRUE(damage.frame_damage.IsEmpty());

  MockLayerTree tree3;
  damage = DiffLayerTree(tree3, tree2);
  EXPECT_EQ(damage.frame_damage, DlIRect::MakeLTRB(10, 10, 60, 60));
}

TEST_F(DisplayListLayerDiffTest, FractionalTranslation) {
  auto display_list = CreateDisplayList(DlRect::MakeLTRB(10, 10, 60, 60));

  MockLayerTree tree1;
  tree1.root()->Add(CreateDisplayListLayer(display_list, DlPoint(0.5f, 0.5f)));

  auto damage = DiffLayerTree(tree1, MockLayerTree(), DlIRect(), 0, 0,
                              /*use_raster_cache=*/false);
  EXPECT_EQ(damage.frame_damage, DlIRect::MakeLTRB(10, 10, 61, 61));
}

TEST_F(DisplayListLayerDiffTest, FractionalTranslationWithRasterCache) {
  auto display_list = CreateDisplayList(DlRect::MakeLTRB(10, 10, 60, 60));

  MockLayerTree tree1;
  tree1.root()->Add(CreateDisplayListLayer(display_list, DlPoint(0.5f, 0.5f)));

  auto damage = DiffLayerTree(tree1, MockLayerTree(), DlIRect(), 0, 0,
                              /*use_raster_cache=*/true);
  EXPECT_EQ(damage.frame_damage, DlIRect::MakeLTRB(11, 11, 61, 61));
}

TEST_F(DisplayListLayerDiffTest, DisplayListCompare) {
  MockLayerTree tree1;
  auto display_list1 =
      CreateDisplayList(DlRect::MakeLTRB(10, 10, 60, 60), DlColor::kGreen());
  tree1.root()->Add(CreateDisplayListLayer(display_list1));

  auto damage = DiffLayerTree(tree1, MockLayerTree());
  EXPECT_EQ(damage.frame_damage, DlIRect::MakeLTRB(10, 10, 60, 60));

  MockLayerTree tree2;
  // same DL, same offset
  auto display_list2 =
      CreateDisplayList(DlRect::MakeLTRB(10, 10, 60, 60), DlColor::kGreen());
  tree2.root()->Add(CreateDisplayListLayer(display_list2));

  damage = DiffLayerTree(tree2, tree1);
  EXPECT_EQ(damage.frame_damage, DlIRect());

  MockLayerTree tree3;
  auto display_list3 =
      CreateDisplayList(DlRect::MakeLTRB(10, 10, 60, 60), DlColor::kGreen());
  // add offset
  tree3.root()->Add(CreateDisplayListLayer(display_list3, DlPoint(10, 10)));

  damage = DiffLayerTree(tree3, tree2);
  EXPECT_EQ(damage.frame_damage, DlIRect::MakeLTRB(10, 10, 70, 70));

  MockLayerTree tree4;
  // different color
  auto display_list4 =
      CreateDisplayList(DlRect::MakeLTRB(10, 10, 60, 60), DlColor::kRed());
  tree4.root()->Add(CreateDisplayListLayer(display_list4, DlPoint(10, 10)));

  damage = DiffLayerTree(tree4, tree3);
  EXPECT_EQ(damage.frame_damage, DlIRect::MakeLTRB(20, 20, 70, 70));
}

TEST_F(DisplayListLayerTest, DisplayListAccessCountDependsOnVisibility) {
  const DlPoint layer_offset = DlPoint(1.5f, -0.5f);
  const DlRect picture_bounds = DlRect::MakeLTRB(5.0f, 6.0f, 20.5f, 21.5f);
  const DlRect missed_cull_rect = DlRect::MakeLTRB(100, 100, 200, 200);
  const DlRect hit_cull_rect = DlRect::MakeLTRB(0, 0, 200, 200);
  DisplayListBuilder builder;
  builder.DrawRect(picture_bounds, DlPaint());
  auto display_list = builder.Build();
  auto layer = std::make_shared<DisplayListLayer>(layer_offset, display_list,
                                                  true, false);

  auto raster_cache_item = layer->raster_cache_item();
  use_mock_raster_cache();

  // First Preroll the DisplayListLayer a few times where it does not intersect
  // the cull rect. No caching progress should occur during this time, the
  // access_count should remain 0 because the DisplayList was never "visible".
  ASSERT_TRUE(preroll_context()->state_stack.is_empty());
  preroll_context()->state_stack.set_preroll_delegate(missed_cull_rect);
  for (int i = 0; i < 10; i++) {
    preroll_context()->raster_cached_entries->clear();
    layer->Preroll(preroll_context());
    ASSERT_EQ(raster_cache_item->cache_state(), RasterCacheItem::kNone);
    ASSERT_TRUE(raster_cache_item->GetId().has_value());
    ASSERT_EQ(preroll_context()->raster_cache->GetAccessCount(
                  raster_cache_item->GetId().value(), SkMatrix::I()),
              0);
    ASSERT_EQ(preroll_context()->raster_cached_entries->size(), size_t(1));
    ASSERT_EQ(preroll_context()->raster_cache->EstimatePictureCacheByteSize(),
              size_t(0));
    ASSERT_FALSE(raster_cache_item->TryToPrepareRasterCache(paint_context()));
    ASSERT_FALSE(raster_cache_item->Draw(paint_context(), nullptr));
  }

  // Next Preroll the DisplayListLayer once where it does intersect
  // the cull rect. No caching progress should occur during this time
  // since this is the first frame in which it was visible, but the
  // count should start incrementing.
  ASSERT_TRUE(preroll_context()->state_stack.is_empty());
  preroll_context()->state_stack.set_preroll_delegate(hit_cull_rect);
  preroll_context()->raster_cached_entries->clear();
  layer->Preroll(preroll_context());
  ASSERT_EQ(raster_cache_item->cache_state(), RasterCacheItem::kNone);
  ASSERT_TRUE(raster_cache_item->GetId().has_value());
  ASSERT_EQ(preroll_context()->raster_cache->GetAccessCount(
                raster_cache_item->GetId().value(), SkMatrix::I()),
            1);
  ASSERT_EQ(preroll_context()->raster_cached_entries->size(), size_t(1));
  ASSERT_EQ(preroll_context()->raster_cache->EstimatePictureCacheByteSize(),
            size_t(0));
  ASSERT_FALSE(raster_cache_item->TryToPrepareRasterCache(paint_context()));
  ASSERT_FALSE(raster_cache_item->Draw(paint_context(), nullptr));

  // Now we can Preroll the DisplayListLayer again with a cull rect that
  // it does not intersect and it should continue to count these operations
  // even though it is not visible. No actual caching should occur yet,
  // even though we will surpass its threshold.
  ASSERT_TRUE(preroll_context()->state_stack.is_empty());
  preroll_context()->state_stack.set_preroll_delegate(missed_cull_rect);
  for (int i = 0; i < 10; i++) {
    preroll_context()->raster_cached_entries->clear();
    layer->Preroll(preroll_context());
    ASSERT_EQ(raster_cache_item->cache_state(), RasterCacheItem::kNone);
    ASSERT_TRUE(raster_cache_item->GetId().has_value());
    ASSERT_EQ(preroll_context()->raster_cache->GetAccessCount(
                  raster_cache_item->GetId().value(), SkMatrix::I()),
              i + 2);
    ASSERT_EQ(preroll_context()->raster_cached_entries->size(), size_t(1));
    ASSERT_EQ(preroll_context()->raster_cache->EstimatePictureCacheByteSize(),
              size_t(0));
    ASSERT_FALSE(raster_cache_item->TryToPrepareRasterCache(paint_context()));
    ASSERT_FALSE(raster_cache_item->Draw(paint_context(), nullptr));
  }

  // Finally Preroll the DisplayListLayer again where it does intersect
  // the cull rect. Since we should have exhausted our access count
  // threshold in the loop above, these operations should result in the
  // DisplayList being cached.
  ASSERT_TRUE(preroll_context()->state_stack.is_empty());
  preroll_context()->state_stack.set_preroll_delegate(hit_cull_rect);
  preroll_context()->raster_cached_entries->clear();
  layer->Preroll(preroll_context());
  ASSERT_EQ(raster_cache_item->cache_state(), RasterCacheItem::kCurrent);
  ASSERT_TRUE(raster_cache_item->GetId().has_value());
  ASSERT_EQ(preroll_context()->raster_cache->GetAccessCount(
                raster_cache_item->GetId().value(), SkMatrix::I()),
            12);
  ASSERT_EQ(preroll_context()->raster_cached_entries->size(), size_t(1));
  ASSERT_EQ(preroll_context()->raster_cache->EstimatePictureCacheByteSize(),
            size_t(0));
  ASSERT_TRUE(raster_cache_item->TryToPrepareRasterCache(paint_context()));
  ASSERT_GT(preroll_context()->raster_cache->EstimatePictureCacheByteSize(),
            size_t(0));
  ASSERT_TRUE(raster_cache_item->Draw(paint_context(), nullptr));
}

TEST_F(DisplayListLayerTest, OverflowCachedDisplayListOpacityInheritance) {
  use_mock_raster_cache();
  PrerollContext* context = preroll_context();
  int per_frame =
      RasterCacheUtil::kDefaultPictureAndDisplayListCacheLimitPerFrame;
  int layer_count = per_frame + 1;
  DlPoint opacity_offset = DlPoint(10, 10);
  auto opacity_layer = std::make_shared<OpacityLayer>(0.5f, opacity_offset);
  std::vector<std::shared_ptr<DisplayListLayer>> layers;
  for (int i = 0; i < layer_count; i++) {
    DisplayListBuilder builder(false);
    builder.DrawRect(DlRect::MakeLTRB(0, 0, 100, 100), DlPaint());
    builder.DrawRect(DlRect::MakeLTRB(50, 50, 100, 100), DlPaint());
    auto display_list = builder.Build();
    ASSERT_FALSE(display_list->can_apply_group_opacity());
    DlPoint offset = DlPoint(i * 200.0f, 0);

    layers.push_back(
        std::make_shared<DisplayListLayer>(offset, display_list, true, false));
    opacity_layer->Add(layers.back());
  }
  for (size_t j = 0; j < context->raster_cache->access_threshold(); j++) {
    context->raster_cache->BeginFrame();
    for (int i = 0; i < layer_count; i++) {
      context->renderable_state_flags = 0;
      layers[i]->Preroll(context);
      ASSERT_EQ(context->renderable_state_flags, 0) << "pass " << (j + 1);
    }
  }
  opacity_layer->Preroll(context);
  ASSERT_FALSE(opacity_layer->children_can_accept_opacity());
  LayerTree::TryToRasterCache(*context->raster_cached_entries, &paint_context(),
                              false);
  context->raster_cached_entries->clear();
  context->raster_cache->BeginFrame();
  for (int i = 0; i < per_frame; i++) {
    context->renderable_state_flags = 0;
    layers[i]->Preroll(context);
    ASSERT_EQ(context->renderable_state_flags,
              LayerStateStack::kCallerCanApplyOpacity)
        << "layer " << (i + 1);
  }
  for (int i = per_frame; i < layer_count; i++) {
    context->renderable_state_flags = 0;
    layers[i]->Preroll(context);
    ASSERT_EQ(context->renderable_state_flags, 0) << "layer " << (i + 1);
  }
  opacity_layer->Preroll(context);
  ASSERT_FALSE(opacity_layer->children_can_accept_opacity());
  LayerTree::TryToRasterCache(*context->raster_cached_entries, &paint_context(),
                              false);
  context->raster_cached_entries->clear();
  context->raster_cache->BeginFrame();
  for (int i = 0; i < layer_count; i++) {
    context->renderable_state_flags = 0;
    layers[i]->Preroll(context);
    ASSERT_EQ(context->renderable_state_flags,
              LayerStateStack::kCallerCanApplyOpacity)
        << "layer " << (i + 1);
  }
  opacity_layer->Preroll(context);
  ASSERT_TRUE(opacity_layer->children_can_accept_opacity());
}

}  // namespace testing
}  // namespace flutter

// NOLINTEND(bugprone-unchecked-optional-access)
