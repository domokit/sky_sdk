// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/flow/layers/container_layer.h"

#include "flutter/flow/layers/layer.h"
#include "flutter/flow/layers/layer_tree.h"
#include "flutter/flow/testing/diff_context_test.h"
#include "flutter/flow/testing/layer_test.h"
#include "flutter/flow/testing/mock_layer.h"
#include "flutter/fml/macros.h"
#include "gtest/gtest.h"

// TODO(zanderso): https://github.com/flutter/flutter/issues/127701
// NOLINTBEGIN(bugprone-unchecked-optional-access)

namespace flutter {
namespace testing {

using ContainerLayerTest = LayerTest;

#ifndef NDEBUG
TEST_F(ContainerLayerTest, LayerWithParentHasPlatformView) {
  auto layer = std::make_shared<ContainerLayer>();

  preroll_context()->has_platform_view = true;
  EXPECT_DEATH_IF_SUPPORTED(layer->Preroll(preroll_context()),
                            "!context->has_platform_view");
}

TEST_F(ContainerLayerTest, LayerWithParentHasTextureLayer) {
  auto layer = std::make_shared<ContainerLayer>();

  preroll_context()->has_texture_layer = true;
  EXPECT_DEATH_IF_SUPPORTED(layer->Preroll(preroll_context()),
                            "!context->has_texture_layer");
}

TEST_F(ContainerLayerTest, PaintingEmptyLayerDies) {
  auto layer = std::make_shared<ContainerLayer>();

  layer->Preroll(preroll_context());
  EXPECT_EQ(layer->paint_bounds(), DlRect());
  EXPECT_EQ(layer->child_paint_bounds(), DlRect());
  EXPECT_FALSE(layer->needs_painting(paint_context()));

  EXPECT_DEATH_IF_SUPPORTED(layer->Paint(paint_context()),
                            "needs_painting\\(context\\)");
}

TEST_F(ContainerLayerTest, PaintBeforePrerollDies) {
  DlPath child_path = DlPath::MakeRectLTRB(5.0f, 6.0f, 20.5f, 21.5f);
  auto mock_layer = std::make_shared<MockLayer>(child_path);
  auto layer = std::make_shared<ContainerLayer>();
  layer->Add(mock_layer);

  EXPECT_EQ(layer->paint_bounds(), DlRect());
  EXPECT_EQ(layer->child_paint_bounds(), DlRect());
  EXPECT_DEATH_IF_SUPPORTED(layer->Paint(paint_context()),
                            "needs_painting\\(context\\)");
}
#endif

TEST_F(ContainerLayerTest, LayerWithParentHasTextureLayerNeedsResetFlag) {
  DlPath child_path1 = DlPath::MakeRectLTRB(5.0f, 6.0f, 20.5f, 21.5f);
  DlPath child_path2 = DlPath::MakeRectLTRB(8.0f, 2.0f, 16.5f, 14.5f);
  DlPaint child_paint1 = DlPaint(DlColor::kMidGrey());
  DlPaint child_paint2 = DlPaint(DlColor::kGreen());

  auto mock_layer1 = std::make_shared<MockLayer>(child_path1, child_paint1);
  mock_layer1->set_fake_has_texture_layer(true);
  auto mock_layer2 = std::make_shared<MockLayer>(child_path2, child_paint2);

  auto root = std::make_shared<ContainerLayer>();
  auto container_layer1 = std::make_shared<ContainerLayer>();
  auto container_layer2 = std::make_shared<ContainerLayer>();
  root->Add(container_layer1);
  root->Add(container_layer2);
  container_layer1->Add(mock_layer1);
  container_layer2->Add(mock_layer2);

  EXPECT_EQ(preroll_context()->has_texture_layer, false);
  root->Preroll(preroll_context());
  EXPECT_EQ(preroll_context()->has_texture_layer, true);
  // The flag for holding texture layer from parent needs to be clear
  EXPECT_EQ(mock_layer2->parent_has_texture_layer(), false);
}

TEST_F(ContainerLayerTest, Simple) {
  DlPath child_path = DlPath::MakeRectLTRB(5.0f, 6.0f, 20.5f, 21.5f);
  DlPaint child_paint = DlPaint(DlColor::kGreen());
  DlMatrix initial_transform = DlMatrix::MakeTranslation({-0.5f, -0.5f});

  auto mock_layer = std::make_shared<MockLayer>(child_path, child_paint);
  auto layer = std::make_shared<ContainerLayer>();
  layer->Add(mock_layer);

  preroll_context()->state_stack.set_preroll_delegate(initial_transform);
  layer->Preroll(preroll_context());
  EXPECT_FALSE(preroll_context()->has_platform_view);
  EXPECT_EQ(mock_layer->paint_bounds(), child_path.GetBounds());
  EXPECT_EQ(layer->paint_bounds(), child_path.GetBounds());
  EXPECT_EQ(layer->child_paint_bounds(), layer->paint_bounds());
  EXPECT_TRUE(mock_layer->needs_painting(paint_context()));
  EXPECT_TRUE(layer->needs_painting(paint_context()));
  EXPECT_EQ(mock_layer->parent_matrix(), initial_transform);
  EXPECT_EQ(mock_layer->parent_cull_rect(), kGiantRect);

  layer->Paint(display_list_paint_context());
  DisplayListBuilder expected_builder;
  /* (Container)layer::Paint */ {
    /* mock_layer::Paint */ {
      expected_builder.DrawPath(child_path, child_paint);
    }
  }
  EXPECT_TRUE(DisplayListsEQ_Verbose(display_list(), expected_builder.Build()));
}

TEST_F(ContainerLayerTest, Multiple) {
  DlPath child_path1 = DlPath::MakeRectLTRB(5.0f, 6.0f, 20.5f, 21.5f);
  DlPath child_path2 = DlPath::MakeRectLTRB(8.0f, 2.0f, 16.5f, 14.5f);
  DlPaint child_paint1 = DlPaint(DlColor::kMidGrey());
  DlPaint child_paint2 = DlPaint(DlColor::kGreen());
  DlMatrix initial_transform = DlMatrix::MakeTranslation({-0.5f, -0.5f});

  auto mock_layer1 = std::make_shared<MockLayer>(child_path1, child_paint1);
  mock_layer1->set_fake_has_platform_view(true);
  auto mock_layer2 = std::make_shared<MockLayer>(child_path2, child_paint2);
  auto layer = std::make_shared<ContainerLayer>();
  layer->Add(mock_layer1);
  layer->Add(mock_layer2);

  DlRect expected_total_bounds =
      child_path1.GetBounds().Union(child_path2.GetBounds());
  preroll_context()->state_stack.set_preroll_delegate(initial_transform);
  layer->Preroll(preroll_context());
  EXPECT_TRUE(preroll_context()->has_platform_view);
  EXPECT_EQ(mock_layer1->paint_bounds(), child_path1.GetBounds());
  EXPECT_EQ(mock_layer2->paint_bounds(), child_path2.GetBounds());
  EXPECT_EQ(layer->paint_bounds(), expected_total_bounds);
  EXPECT_EQ(layer->child_paint_bounds(), layer->paint_bounds());
  EXPECT_TRUE(mock_layer1->needs_painting(paint_context()));
  EXPECT_TRUE(mock_layer2->needs_painting(paint_context()));
  EXPECT_TRUE(layer->needs_painting(paint_context()));
  EXPECT_EQ(mock_layer1->parent_matrix(), initial_transform);
  EXPECT_EQ(mock_layer2->parent_matrix(), initial_transform);
  EXPECT_EQ(mock_layer1->parent_cull_rect(), kGiantRect);
  EXPECT_EQ(mock_layer2->parent_cull_rect(),
            kGiantRect);  // Siblings are independent

  layer->Paint(display_list_paint_context());
  DisplayListBuilder expected_builder;
  /* (Container)layer::Paint */ {
    /* mock_layer1::Paint */ {
      expected_builder.DrawPath(child_path1, child_paint1);
    }
    /* mock_layer2::Paint */ {
      expected_builder.DrawPath(child_path2, child_paint2);
    }
  }
  EXPECT_TRUE(DisplayListsEQ_Verbose(display_list(), expected_builder.Build()));
}

TEST_F(ContainerLayerTest, MultipleWithEmpty) {
  DlPath child_path1 = DlPath::MakeRectLTRB(5.0f, 6.0f, 20.5f, 21.5f);
  DlPaint child_paint1 = DlPaint(DlColor::kMidGrey());
  DlPaint child_paint2 = DlPaint(DlColor::kGreen());
  DlMatrix initial_transform = DlMatrix::MakeTranslation({-0.5f, -0.5f});

  auto mock_layer1 = std::make_shared<MockLayer>(child_path1, child_paint1);
  auto mock_layer2 = std::make_shared<MockLayer>(DlPath(), child_paint2);
  auto layer = std::make_shared<ContainerLayer>();
  layer->Add(mock_layer1);
  layer->Add(mock_layer2);

  preroll_context()->state_stack.set_preroll_delegate(initial_transform);
  layer->Preroll(preroll_context());
  EXPECT_FALSE(preroll_context()->has_platform_view);
  EXPECT_EQ(mock_layer1->paint_bounds(), child_path1.GetBounds());
  EXPECT_EQ(mock_layer2->paint_bounds(), DlPath().GetBounds());
  EXPECT_EQ(layer->paint_bounds(), child_path1.GetBounds());
  EXPECT_EQ(layer->child_paint_bounds(), layer->paint_bounds());
  EXPECT_TRUE(mock_layer1->needs_painting(paint_context()));
  EXPECT_FALSE(mock_layer2->needs_painting(paint_context()));
  EXPECT_TRUE(layer->needs_painting(paint_context()));
  EXPECT_EQ(mock_layer1->parent_matrix(), initial_transform);
  EXPECT_EQ(mock_layer2->parent_matrix(), initial_transform);
  EXPECT_EQ(mock_layer1->parent_cull_rect(), kGiantRect);
  EXPECT_EQ(mock_layer2->parent_cull_rect(), kGiantRect);

  layer->Paint(display_list_paint_context());
  DisplayListBuilder expected_builder;
  /* (Container)layer::Paint */ {
    /* mock_layer1::Paint */ {
      expected_builder.DrawPath(child_path1, child_paint1);
    }
    // mock_layer2 not drawn due to needs_painting() returning false
  }
  EXPECT_TRUE(DisplayListsEQ_Verbose(display_list(), expected_builder.Build()));
}

TEST_F(ContainerLayerTest, NeedsSystemComposite) {
  DlPath child_path1 = DlPath::MakeRectLTRB(5.0f, 6.0f, 20.5f, 21.5f);
  DlPath child_path2 = DlPath::MakeRectLTRB(8.0f, 2.0f, 16.5f, 14.5f);
  DlPaint child_paint1 = DlPaint(DlColor::kMidGrey());
  DlPaint child_paint2 = DlPaint(DlColor::kGreen());
  DlMatrix initial_transform = DlMatrix::MakeTranslation({-0.5f, -0.5f});

  auto mock_layer1 = std::make_shared<MockLayer>(child_path1, child_paint1);
  mock_layer1->set_fake_has_platform_view(false);
  auto mock_layer2 = std::make_shared<MockLayer>(child_path2, child_paint2);
  auto layer = std::make_shared<ContainerLayer>();
  layer->Add(mock_layer1);
  layer->Add(mock_layer2);

  DlRect expected_total_bounds =
      child_path1.GetBounds().Union(child_path2.GetBounds());
  preroll_context()->state_stack.set_preroll_delegate(initial_transform);
  layer->Preroll(preroll_context());
  EXPECT_FALSE(preroll_context()->has_platform_view);
  EXPECT_EQ(mock_layer1->paint_bounds(), child_path1.GetBounds());
  EXPECT_EQ(mock_layer2->paint_bounds(), child_path2.GetBounds());
  EXPECT_EQ(layer->paint_bounds(), expected_total_bounds);
  EXPECT_EQ(layer->child_paint_bounds(), layer->paint_bounds());
  EXPECT_TRUE(mock_layer1->needs_painting(paint_context()));
  EXPECT_TRUE(mock_layer2->needs_painting(paint_context()));
  EXPECT_TRUE(layer->needs_painting(paint_context()));
  EXPECT_EQ(mock_layer1->parent_matrix(), initial_transform);
  EXPECT_EQ(mock_layer2->parent_matrix(), initial_transform);
  EXPECT_EQ(mock_layer1->parent_cull_rect(), kGiantRect);
  EXPECT_EQ(mock_layer2->parent_cull_rect(), kGiantRect);

  layer->Paint(display_list_paint_context());
  DisplayListBuilder expected_builder;
  /* (Container)layer::Paint */ {
    /* mock_layer1::Paint */ {
      expected_builder.DrawPath(child_path1, child_paint1);
    }
    /* mock_layer2::Paint */ {
      expected_builder.DrawPath(child_path2, child_paint2);
    }
  }
  EXPECT_TRUE(DisplayListsEQ_Verbose(display_list(), expected_builder.Build()));
}

TEST_F(ContainerLayerTest, RasterCacheTest) {
  // LTRB
  const DlPath child_path1 = DlPath::MakeRectLTRB(5.0f, 6.0f, 20.5f, 21.5f);
  const DlPath child_path2 = DlPath::MakeRectLTRB(21.0f, 6.0f, 25.5f, 21.5f);
  const DlPath child_path3 = DlPath::MakeRectLTRB(26.0f, 6.0f, 30.5f, 21.5f);
  const DlPaint child_paint1 = DlPaint(DlColor::kMidGrey());
  const DlPaint child_paint2 = DlPaint(DlColor::kGreen());
  const DlPaint paint;
  auto cacheable_container_layer1 =
      MockCacheableContainerLayer::CacheLayerOrChildren();
  auto cacheable_container_layer2 =
      MockCacheableContainerLayer::CacheLayerOnly();
  auto cacheable_container_layer11 =
      MockCacheableContainerLayer::CacheLayerOrChildren();

  auto cacheable_layer111 =
      std::make_shared<MockCacheableLayer>(child_path3, paint);
  // if the frame had rendered 2 frames, we will cache the cacheable_layer21
  // layer
  auto cacheable_layer21 =
      std::make_shared<MockCacheableLayer>(child_path1, paint, 2);

  // clang-format off
//                                               layer
//                                                 |
//                 ________________________________ ________________________________
//                 |                               |                                |
//  cacheable_container_layer1                mock_layer2               cacheable_container_layer2
//                 |                                                                |
// cacheable_container_layer11                                             cacheable_layer21
//                 |
//        cacheable_layer111
  // clang-format on

  auto mock_layer1 = std::make_shared<MockLayer>(child_path1, child_paint1);
  auto mock_layer2 = std::make_shared<MockLayer>(DlPath(), child_paint2);
  auto mock_layer3 = std::make_shared<MockLayer>(child_path2, paint);

  cacheable_container_layer1->Add(mock_layer1);
  cacheable_container_layer1->Add(mock_layer3);

  cacheable_container_layer1->Add(cacheable_container_layer11);
  cacheable_container_layer11->Add(cacheable_layer111);

  cacheable_container_layer2->Add(cacheable_layer21);
  auto layer = std::make_shared<ContainerLayer>();
  layer->Add(cacheable_container_layer1);
  layer->Add(mock_layer2);
  layer->Add(cacheable_container_layer2);

  DisplayListBuilder cache_canvas;
  cache_canvas.TransformReset();

  // Initial Preroll for check the layer paint bounds
  layer->Preroll(preroll_context());

  EXPECT_EQ(mock_layer1->paint_bounds(),
            DlRect::MakeLTRB(5.f, 6.f, 20.5f, 21.5f));
  EXPECT_EQ(mock_layer3->paint_bounds(),
            DlRect::MakeLTRB(21.0f, 6.0f, 25.5f, 21.5f));
  EXPECT_EQ(cacheable_layer111->paint_bounds(),
            DlRect::MakeLTRB(26.0f, 6.0f, 30.5f, 21.5f));
  EXPECT_EQ(cacheable_container_layer1->paint_bounds(),
            DlRect::MakeLTRB(5.f, 6.f, 30.5f, 21.5f));

  // the preroll context's raster cache is nullptr
  EXPECT_EQ(preroll_context()->raster_cached_entries->size(),
            static_cast<unsigned long>(0));
  {
    // frame1
    use_mock_raster_cache();
    preroll_context()->raster_cache->BeginFrame();
    layer->Preroll(preroll_context());
    preroll_context()->raster_cache->EvictUnusedCacheEntries();
    // Cache the cacheable entries
    LayerTree::TryToRasterCache(*(preroll_context()->raster_cached_entries),
                                &paint_context());

    EXPECT_EQ(preroll_context()->raster_cached_entries->size(),
              static_cast<unsigned long>(5));

    // cacheable_container_layer1 will cache his children
    EXPECT_EQ(cacheable_container_layer1->raster_cache_item()->cache_state(),
              RasterCacheItem::CacheState::kChildren);
    EXPECT_TRUE(raster_cache()->HasEntry(
        cacheable_container_layer1->raster_cache_item()->GetId().value(),
        SkMatrix::I()));

    EXPECT_EQ(cacheable_container_layer11->raster_cache_item()->cache_state(),
              RasterCacheItem::CacheState::kChildren);
    EXPECT_TRUE(raster_cache()->HasEntry(
        cacheable_container_layer11->raster_cache_item()->GetId().value(),
        SkMatrix::I()));
    EXPECT_FALSE(raster_cache()->Draw(
        cacheable_container_layer11->raster_cache_item()->GetId().value(),
        cache_canvas, &paint));

    // The cacheable_layer111 should be cached when rended 3 frames
    EXPECT_EQ(cacheable_layer111->raster_cache_item()->cache_state(),
              RasterCacheItem::CacheState::kNone);
    // render count < 2 don't cache it
    EXPECT_EQ(cacheable_container_layer2->raster_cache_item()->cache_state(),
              RasterCacheItem::CacheState::kNone);
    preroll_context()->raster_cache->EndFrame();
  }

  {
    // frame2
    // new frame the layer tree will create new PrerollContext, so in here we
    // clear the cached_entries
    preroll_context()->raster_cached_entries->clear();
    preroll_context()->raster_cache->BeginFrame();
    layer->Preroll(preroll_context());
    preroll_context()->raster_cache->EvictUnusedCacheEntries();

    // Cache the cacheable entries
    LayerTree::TryToRasterCache(*(preroll_context()->raster_cached_entries),
                                &paint_context());
    EXPECT_EQ(preroll_context()->raster_cached_entries->size(),
              static_cast<unsigned long>(5));
    EXPECT_EQ(cacheable_container_layer1->raster_cache_item()->cache_state(),
              RasterCacheItem::CacheState::kChildren);
    EXPECT_TRUE(raster_cache()->HasEntry(
        cacheable_container_layer1->raster_cache_item()->GetId().value(),
        SkMatrix::I()));

    EXPECT_EQ(cacheable_container_layer11->raster_cache_item()->cache_state(),
              RasterCacheItem::CacheState::kChildren);
    EXPECT_TRUE(raster_cache()->HasEntry(
        cacheable_container_layer11->raster_cache_item()->GetId().value(),
        SkMatrix::I()));
    EXPECT_FALSE(raster_cache()->Draw(
        cacheable_container_layer11->raster_cache_item()->GetId().value(),
        cache_canvas, &paint));

    EXPECT_EQ(cacheable_container_layer2->raster_cache_item()->cache_state(),
              RasterCacheItem::CacheState::kNone);

    // render count == 2 cache it
    EXPECT_EQ(cacheable_layer21->raster_cache_item()->cache_state(),
              RasterCacheItem::CacheState::kCurrent);
    EXPECT_TRUE(raster_cache()->HasEntry(
        cacheable_layer21->raster_cache_item()->GetId().value(),
        SkMatrix::I()));
    EXPECT_TRUE(raster_cache()->Draw(
        cacheable_layer21->raster_cache_item()->GetId().value(), cache_canvas,
        &paint));
    preroll_context()->raster_cache->EndFrame();
  }

  {
    // frame3
    // new frame the layer tree will create new PrerollContext, so in here we
    // clear the cached_entries
    preroll_context()->raster_cache->BeginFrame();
    preroll_context()->raster_cached_entries->clear();
    layer->Preroll(preroll_context());
    preroll_context()->raster_cache->EvictUnusedCacheEntries();
    // Cache the cacheable entries
    LayerTree::TryToRasterCache(*(preroll_context()->raster_cached_entries),
                                &paint_context());
    EXPECT_EQ(preroll_context()->raster_cached_entries->size(),
              static_cast<unsigned long>(5));
    EXPECT_EQ(cacheable_container_layer1->raster_cache_item()->cache_state(),
              RasterCacheItem::CacheState::kCurrent);
    EXPECT_TRUE(raster_cache()->HasEntry(
        cacheable_container_layer1->raster_cache_item()->GetId().value(),
        SkMatrix::I()));
    EXPECT_TRUE(raster_cache()->HasEntry(
        cacheable_container_layer11->raster_cache_item()->GetId().value(),
        SkMatrix::I()));
    EXPECT_FALSE(raster_cache()->Draw(
        cacheable_container_layer11->raster_cache_item()->GetId().value(),
        cache_canvas, &paint));
    // The 3td frame, we will cache the cacheable_layer111, but his ancestor has
    // been cached, so cacheable_layer111 Draw is false
    EXPECT_TRUE(raster_cache()->HasEntry(
        cacheable_layer111->raster_cache_item()->GetId().value(),
        SkMatrix::I()));
    EXPECT_FALSE(raster_cache()->Draw(
        cacheable_layer111->raster_cache_item()->GetId().value(), cache_canvas,
        &paint));

    // The third frame, we will cache the cacheable_container_layer2
    EXPECT_EQ(cacheable_container_layer2->raster_cache_item()->cache_state(),
              RasterCacheItem::CacheState::kCurrent);

    EXPECT_TRUE(raster_cache()->HasEntry(
        cacheable_layer21->raster_cache_item()->GetId().value(),
        SkMatrix::I()));
    preroll_context()->raster_cache->EndFrame();
  }

  {
    preroll_context()->raster_cache->BeginFrame();
    // frame4
    preroll_context()->raster_cached_entries->clear();
    layer->Preroll(preroll_context());
    preroll_context()->raster_cache->EvictUnusedCacheEntries();
    LayerTree::TryToRasterCache(*(preroll_context()->raster_cached_entries),
                                &paint_context());
    preroll_context()->raster_cache->EndFrame();

    // frame5
    preroll_context()->raster_cache->BeginFrame();
    preroll_context()->raster_cached_entries->clear();
    layer->Preroll(preroll_context());
    LayerTree::TryToRasterCache(*(preroll_context()->raster_cached_entries),
                                &paint_context());
    preroll_context()->raster_cache->EndFrame();

    // frame6
    preroll_context()->raster_cache->BeginFrame();
    preroll_context()->raster_cached_entries->clear();
    layer->Preroll(preroll_context());
    LayerTree::TryToRasterCache(*(preroll_context()->raster_cached_entries),
                                &paint_context());
    preroll_context()->raster_cache->EndFrame();
  }
}

TEST_F(ContainerLayerTest, OpacityInheritance) {
  auto path1 = DlPath::MakeRectLTRB(10, 10, 30, 30);
  auto mock1 = MockLayer::MakeOpacityCompatible(path1);
  auto container1 = std::make_shared<ContainerLayer>();
  container1->Add(mock1);

  // ContainerLayer will pass through compatibility
  PrerollContext* context = preroll_context();
  container1->Preroll(context);
  EXPECT_EQ(context->renderable_state_flags,
            LayerStateStack::kCallerCanApplyOpacity);

  auto path2 = DlPath::MakeRectLTRB(40, 40, 50, 50);
  auto mock2 = MockLayer::MakeOpacityCompatible(path2);
  container1->Add(mock2);

  // ContainerLayer will pass through compatibility from multiple
  // non-overlapping compatible children
  container1->Preroll(context);
  EXPECT_EQ(context->renderable_state_flags,
            LayerStateStack::kCallerCanApplyOpacity);

  auto path3 = DlPath::MakeRectLTRB(20, 20, 40, 40);
  auto mock3 = MockLayer::MakeOpacityCompatible(path3);
  container1->Add(mock3);

  // ContainerLayer will not pass through compatibility from multiple
  // overlapping children even if they are individually compatible
  container1->Preroll(context);
  EXPECT_EQ(context->renderable_state_flags, 0);

  auto container2 = std::make_shared<ContainerLayer>();
  container2->Add(mock1);
  container2->Add(mock2);

  // Double check first two children are compatible and non-overlapping
  container2->Preroll(context);
  EXPECT_EQ(context->renderable_state_flags,
            LayerStateStack::kCallerCanApplyOpacity);

  auto path4 = DlPath::MakeRectLTRB(60, 60, 70, 70);
  auto mock4 = MockLayer::Make(path4);
  container2->Add(mock4);

  // The third child is non-overlapping, but not compatible so the
  // ContainerLayer should end up incompatible
  container2->Preroll(context);
  EXPECT_EQ(context->renderable_state_flags, 0);
}

TEST_F(ContainerLayerTest, CollectionCacheableLayer) {
  DlPath child_path = DlPath::MakeRectLTRB(5.0f, 6.0f, 20.5f, 21.5f);
  DlPaint child_paint = DlPaint(DlColor::kGreen());
  DlMatrix initial_transform = DlMatrix::MakeTranslation({-0.5f, -0.5f});

  auto mock_layer1 = std::make_shared<MockLayer>(DlPath(), child_paint);
  auto mock_cacheable_container_layer1 =
      std::make_shared<MockCacheableContainerLayer>();
  auto mock_container_layer = std::make_shared<ContainerLayer>();
  auto mock_cacheable_layer =
      std::make_shared<MockCacheableLayer>(child_path, child_paint);
  mock_cacheable_container_layer1->Add(mock_cacheable_layer);

  // ContainerLayer
  //   |- MockLayer
  //   |- MockCacheableContainerLayer
  //        |- MockCacheableLayer
  auto layer = std::make_shared<ContainerLayer>();
  layer->Add(mock_cacheable_container_layer1);
  layer->Add(mock_layer1);

  preroll_context()->state_stack.set_preroll_delegate(initial_transform);
  layer->Preroll(preroll_context());
  // raster cache is null, so no entry
  ASSERT_EQ(preroll_context()->raster_cached_entries->size(),
            static_cast<const unsigned long>(0));

  use_mock_raster_cache();
  // preroll_context()->raster_cache = raster_cache();
  layer->Preroll(preroll_context());
  ASSERT_EQ(preroll_context()->raster_cached_entries->size(),
            static_cast<const unsigned long>(2));
}

using ContainerLayerDiffTest = DiffContextTest;

// Insert PictureLayer amongst container layers
TEST_F(ContainerLayerDiffTest, PictureLayerInsertion) {
  auto pic1 = CreateDisplayList(DlRect::MakeLTRB(0, 0, 50, 50));
  auto pic2 = CreateDisplayList(DlRect::MakeLTRB(100, 0, 150, 50));
  auto pic3 = CreateDisplayList(DlRect::MakeLTRB(200, 0, 250, 50));

  MockLayerTree t1;

  auto t1_c1 = CreateContainerLayer(CreateDisplayListLayer(pic1));
  t1.root()->Add(t1_c1);

  auto t1_c2 = CreateContainerLayer(CreateDisplayListLayer(pic2));
  t1.root()->Add(t1_c2);

  auto damage = DiffLayerTree(t1, MockLayerTree());
  EXPECT_EQ(damage.frame_damage, DlIRect::MakeLTRB(0, 0, 150, 50));

  // Add in the middle

  MockLayerTree t2;
  auto t2_c1 = CreateContainerLayer(CreateDisplayListLayer(pic1));
  t2_c1->AssignOldLayer(t1_c1.get());
  t2.root()->Add(t2_c1);

  t2.root()->Add(CreateDisplayListLayer(pic3));

  auto t2_c2 = CreateContainerLayer(CreateDisplayListLayer(pic2));
  t2_c2->AssignOldLayer(t1_c2.get());
  t2.root()->Add(t2_c2);

  damage = DiffLayerTree(t2, t1);
  EXPECT_EQ(damage.frame_damage, DlIRect::MakeLTRB(200, 0, 250, 50));

  // Add in the beginning

  t2 = MockLayerTree();
  t2.root()->Add(CreateDisplayListLayer(pic3));
  t2.root()->Add(t2_c1);
  t2.root()->Add(t2_c2);
  damage = DiffLayerTree(t2, t1);
  EXPECT_EQ(damage.frame_damage, DlIRect::MakeLTRB(200, 0, 250, 50));

  // Add at the end

  t2 = MockLayerTree();
  t2.root()->Add(t2_c1);
  t2.root()->Add(t2_c2);
  t2.root()->Add(CreateDisplayListLayer(pic3));
  damage = DiffLayerTree(t2, t1);
  EXPECT_EQ(damage.frame_damage, DlIRect::MakeLTRB(200, 0, 250, 50));
}

// Insert picture layer amongst other picture layers
TEST_F(ContainerLayerDiffTest, PictureInsertion) {
  auto pic1 = CreateDisplayList(DlRect::MakeLTRB(0, 0, 50, 50));
  auto pic2 = CreateDisplayList(DlRect::MakeLTRB(100, 0, 150, 50));
  auto pic3 = CreateDisplayList(DlRect::MakeLTRB(200, 0, 250, 50));

  MockLayerTree t1;
  t1.root()->Add(CreateDisplayListLayer(pic1));
  t1.root()->Add(CreateDisplayListLayer(pic2));

  auto damage = DiffLayerTree(t1, MockLayerTree());
  EXPECT_EQ(damage.frame_damage, DlIRect::MakeLTRB(0, 0, 150, 50));

  MockLayerTree t2;
  t2.root()->Add(CreateDisplayListLayer(pic3));
  t2.root()->Add(CreateDisplayListLayer(pic1));
  t2.root()->Add(CreateDisplayListLayer(pic2));

  damage = DiffLayerTree(t2, t1);
  EXPECT_EQ(damage.frame_damage, DlIRect::MakeLTRB(200, 0, 250, 50));

  MockLayerTree t3;
  t3.root()->Add(CreateDisplayListLayer(pic1));
  t3.root()->Add(CreateDisplayListLayer(pic3));
  t3.root()->Add(CreateDisplayListLayer(pic2));

  damage = DiffLayerTree(t3, t1);
  EXPECT_EQ(damage.frame_damage, DlIRect::MakeLTRB(200, 0, 250, 50));

  MockLayerTree t4;
  t4.root()->Add(CreateDisplayListLayer(pic1));
  t4.root()->Add(CreateDisplayListLayer(pic2));
  t4.root()->Add(CreateDisplayListLayer(pic3));

  damage = DiffLayerTree(t4, t1);
  EXPECT_EQ(damage.frame_damage, DlIRect::MakeLTRB(200, 0, 250, 50));
}

TEST_F(ContainerLayerDiffTest, LayerDeletion) {
  auto path1 = DlPath::MakeRectLTRB(0, 0, 50, 50);
  auto path2 = DlPath::MakeRectLTRB(100, 0, 150, 50);
  auto path3 = DlPath::MakeRectLTRB(200, 0, 250, 50);

  auto c1 = CreateContainerLayer(std::make_shared<MockLayer>(path1));
  auto c2 = CreateContainerLayer(std::make_shared<MockLayer>(path2));
  auto c3 = CreateContainerLayer(std::make_shared<MockLayer>(path3));

  MockLayerTree t1;
  t1.root()->Add(c1);
  t1.root()->Add(c2);
  t1.root()->Add(c3);

  auto damage = DiffLayerTree(t1, MockLayerTree());
  EXPECT_EQ(damage.frame_damage, DlIRect::MakeLTRB(0, 0, 250, 50));

  MockLayerTree t2;
  t2.root()->Add(c2);
  t2.root()->Add(c3);

  damage = DiffLayerTree(t2, t1);
  EXPECT_EQ(damage.frame_damage, DlIRect::MakeLTRB(0, 0, 50, 50));

  MockLayerTree t3;
  t3.root()->Add(c1);
  t3.root()->Add(c3);

  damage = DiffLayerTree(t3, t1);
  EXPECT_EQ(damage.frame_damage, DlIRect::MakeLTRB(100, 0, 150, 50));

  MockLayerTree t4;
  t4.root()->Add(c1);
  t4.root()->Add(c2);

  damage = DiffLayerTree(t4, t1);
  EXPECT_EQ(damage.frame_damage, DlIRect::MakeLTRB(200, 0, 250, 50));

  MockLayerTree t5;
  t5.root()->Add(c1);

  damage = DiffLayerTree(t5, t1);
  EXPECT_EQ(damage.frame_damage, DlIRect::MakeLTRB(100, 0, 250, 50));

  MockLayerTree t6;
  t6.root()->Add(c2);

  damage = DiffLayerTree(t6, t1);
  EXPECT_EQ(damage.frame_damage, DlIRect::MakeLTRB(0, 0, 250, 50));

  MockLayerTree t7;
  t7.root()->Add(c3);

  damage = DiffLayerTree(t7, t1);
  EXPECT_EQ(damage.frame_damage, DlIRect::MakeLTRB(0, 0, 150, 50));
}

TEST_F(ContainerLayerDiffTest, ReplaceLayer) {
  auto path1 = DlPath::MakeRectLTRB(0, 0, 50, 50);
  auto path2 = DlPath::MakeRectLTRB(100, 0, 150, 50);
  auto path3 = DlPath::MakeRectLTRB(200, 0, 250, 50);

  auto path1a = DlPath::MakeRectLTRB(0, 100, 50, 150);
  auto path2a = DlPath::MakeRectLTRB(100, 100, 150, 150);
  auto path3a = DlPath::MakeRectLTRB(200, 100, 250, 150);

  auto c1 = CreateContainerLayer(std::make_shared<MockLayer>(path1));
  auto c2 = CreateContainerLayer(std::make_shared<MockLayer>(path2));
  auto c3 = CreateContainerLayer(std::make_shared<MockLayer>(path3));

  MockLayerTree t1;
  t1.root()->Add(c1);
  t1.root()->Add(c2);
  t1.root()->Add(c3);

  auto damage = DiffLayerTree(t1, MockLayerTree());
  EXPECT_EQ(damage.frame_damage, DlIRect::MakeLTRB(0, 0, 250, 50));

  MockLayerTree t2;
  t2.root()->Add(c1);
  t2.root()->Add(c2);
  t2.root()->Add(c3);

  damage = DiffLayerTree(t2, t1);
  EXPECT_TRUE(damage.frame_damage.IsEmpty());

  MockLayerTree t3;
  t3.root()->Add(CreateContainerLayer({std::make_shared<MockLayer>(path1a)}));
  t3.root()->Add(c2);
  t3.root()->Add(c3);

  damage = DiffLayerTree(t3, t1);
  EXPECT_EQ(damage.frame_damage, DlIRect::MakeLTRB(0, 0, 50, 150));

  MockLayerTree t4;
  t4.root()->Add(c1);
  t4.root()->Add(CreateContainerLayer(std::make_shared<MockLayer>(path2a)));
  t4.root()->Add(c3);

  damage = DiffLayerTree(t4, t1);
  EXPECT_EQ(damage.frame_damage, DlIRect::MakeLTRB(100, 0, 150, 150));

  MockLayerTree t5;
  t5.root()->Add(c1);
  t5.root()->Add(c2);
  t5.root()->Add(CreateContainerLayer(std::make_shared<MockLayer>(path3a)));

  damage = DiffLayerTree(t5, t1);
  EXPECT_EQ(damage.frame_damage, DlIRect::MakeLTRB(200, 0, 250, 150));
}

}  // namespace testing
}  // namespace flutter

// NOLINTEND(bugprone-unchecked-optional-access)
