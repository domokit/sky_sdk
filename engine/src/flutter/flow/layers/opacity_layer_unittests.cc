// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/flow/layers/opacity_layer.h"

#include "flutter/display_list/effects/dl_image_filter.h"
#include "flutter/flow/layers/clip_rect_layer.h"
#include "flutter/flow/layers/image_filter_layer.h"
#include "flutter/flow/layers/layer_tree.h"
#include "flutter/flow/layers/platform_view_layer.h"
#include "flutter/flow/layers/transform_layer.h"
#include "flutter/flow/raster_cache_util.h"
#include "flutter/flow/testing/diff_context_test.h"
#include "flutter/flow/testing/layer_test.h"
#include "flutter/flow/testing/mock_embedder.h"
#include "flutter/flow/testing/mock_layer.h"
#include "flutter/fml/macros.h"
#include "flutter/testing/display_list_testing.h"
#include "gtest/gtest.h"

// TODO(zanderso): https://github.com/flutter/flutter/issues/127701
// NOLINTBEGIN(bugprone-unchecked-optional-access)

namespace flutter {
namespace testing {

using OpacityLayerTest = LayerTest;

#ifndef NDEBUG
TEST_F(OpacityLayerTest, PaintingEmptyLayerDies) {
  auto mock_layer = std::make_shared<MockLayer>(DlPath());
  auto layer =
      std::make_shared<OpacityLayer>(DlColor::toAlpha(1.0f), DlPoint());
  layer->Add(mock_layer);

  layer->Preroll(preroll_context());
  EXPECT_EQ(mock_layer->paint_bounds(), DlPath().GetBounds());
  EXPECT_EQ(layer->paint_bounds(), mock_layer->paint_bounds());
  EXPECT_EQ(layer->child_paint_bounds(), mock_layer->paint_bounds());
  EXPECT_FALSE(mock_layer->needs_painting(paint_context()));
  EXPECT_FALSE(layer->needs_painting(paint_context()));

  EXPECT_DEATH_IF_SUPPORTED(layer->Paint(paint_context()),
                            "needs_painting\\(context\\)");
}

TEST_F(OpacityLayerTest, PaintBeforePrerollDies) {
  const DlPath child_path = DlPath::MakeRectLTRB(5.0f, 6.0f, 20.5f, 21.5f);
  auto mock_layer = std::make_shared<MockLayer>(child_path);
  auto layer =
      std::make_shared<OpacityLayer>(DlColor::toAlpha(1.0f), DlPoint());
  layer->Add(mock_layer);

  EXPECT_DEATH_IF_SUPPORTED(layer->Paint(paint_context()),
                            "needs_painting\\(context\\)");
}
#endif

TEST_F(OpacityLayerTest, TranslateChildren) {
  const DlPath child_path1 = DlPath::MakeRectLTRB(10.0f, 10.0f, 20.0f, 20.f);
  DlPaint child_paint1 = DlPaint(DlColor::kMidGrey());
  auto layer =
      std::make_shared<OpacityLayer>(DlColor::toAlpha(0.5f), DlPoint(10, 10));
  auto mock_layer1 = std::make_shared<MockLayer>(child_path1, child_paint1);
  layer->Add(mock_layer1);

  auto initial_transform = DlMatrix::MakeScale({2.0f, 2.0f, 1.0f});
  preroll_context()->state_stack.set_preroll_delegate(initial_transform);
  layer->Preroll(preroll_context());

  DlRect layer_bounds = mock_layer1->paint_bounds().TransformAndClipBounds(
      mock_layer1->parent_matrix());

  EXPECT_EQ(layer_bounds, DlRect::MakeXYWH(40, 40, 20, 20));
}

TEST_F(OpacityLayerTest, CacheChild) {
  const uint8_t alpha_half = DlColor::toAlpha(0.5f);
  auto initial_transform = DlMatrix::MakeTranslation({50.0f, 25.5f});
  auto other_transform = DlMatrix::MakeScale({1.0f, 2.0f, 1.0f});
  const DlPath child_path = DlPath::MakeRect(DlRect::MakeWH(5.0f, 5.0f));
  auto mock_layer = std::make_shared<MockLayer>(child_path);
  auto layer = std::make_shared<OpacityLayer>(alpha_half, DlPoint());
  layer->Add(mock_layer);
  DlPaint paint;

  DlMatrix cache_ctm = initial_transform;
  DisplayListBuilder cache_canvas;
  cache_canvas.Transform(cache_ctm);
  DisplayListBuilder other_canvas;
  other_canvas.Transform(other_transform);

  use_mock_raster_cache();

  EXPECT_EQ(raster_cache()->GetLayerCachedEntriesCount(), (size_t)0);

  const auto* cacheable_opacity_item = layer->raster_cache_item();

  EXPECT_EQ(raster_cache()->GetLayerCachedEntriesCount(), (size_t)0);
  EXPECT_EQ(cacheable_opacity_item->cache_state(),
            RasterCacheItem::CacheState::kNone);
  EXPECT_FALSE(cacheable_opacity_item->GetId().has_value());

  preroll_context()->state_stack.set_preroll_delegate(initial_transform);
  layer->Preroll(preroll_context());
  LayerTree::TryToRasterCache(cacheable_items(), &paint_context());

  EXPECT_EQ(raster_cache()->GetLayerCachedEntriesCount(), (size_t)1);

  EXPECT_EQ(cacheable_opacity_item->cache_state(),
            RasterCacheItem::CacheState::kChildren);
  EXPECT_EQ(
      cacheable_opacity_item->GetId().value(),
      RasterCacheKeyID(RasterCacheKeyID::LayerChildrenIds(layer.get()).value(),
                       RasterCacheKeyType::kLayerChildren));
  EXPECT_FALSE(raster_cache()->Draw(cacheable_opacity_item->GetId().value(),
                                    other_canvas, &paint));
  EXPECT_TRUE(raster_cache()->Draw(cacheable_opacity_item->GetId().value(),
                                   cache_canvas, &paint));
}

TEST_F(OpacityLayerTest, CacheChildren) {
  const uint8_t alpha_half = DlColor::toAlpha(0.5f);
  auto initial_transform = DlMatrix::MakeTranslation({50.0f, 25.5f});
  auto other_transform = DlMatrix::MakeScale({1.0f, 2.0f, 1.0f});
  const DlPath child_path1 = DlPath::MakeRect(DlRect::MakeWH(5.0f, 5.0f));
  const DlPath child_path2 = DlPath::MakeRect(DlRect::MakeWH(5.0f, 5.0f));
  DlPaint paint;
  auto mock_layer1 = std::make_shared<MockLayer>(child_path1);
  auto mock_layer2 = std::make_shared<MockLayer>(child_path2);
  auto layer = std::make_shared<OpacityLayer>(alpha_half, DlPoint());
  layer->Add(mock_layer1);
  layer->Add(mock_layer2);

  DlMatrix cache_ctm = initial_transform;
  DisplayListBuilder cache_canvas;
  cache_canvas.Transform(cache_ctm);
  DisplayListBuilder other_canvas;
  other_canvas.Transform(other_transform);

  use_mock_raster_cache();

  EXPECT_EQ(raster_cache()->GetLayerCachedEntriesCount(), (size_t)0);

  const auto* cacheable_opacity_item = layer->raster_cache_item();

  EXPECT_EQ(raster_cache()->GetLayerCachedEntriesCount(), (size_t)0);
  EXPECT_EQ(cacheable_opacity_item->cache_state(),
            RasterCacheItem::CacheState::kNone);
  EXPECT_FALSE(cacheable_opacity_item->GetId().has_value());

  preroll_context()->state_stack.set_preroll_delegate(initial_transform);
  layer->Preroll(preroll_context());
  LayerTree::TryToRasterCache(cacheable_items(), &paint_context());

  EXPECT_EQ(raster_cache()->GetLayerCachedEntriesCount(), (size_t)1);

  EXPECT_EQ(cacheable_opacity_item->cache_state(),
            RasterCacheItem::CacheState::kChildren);
  EXPECT_EQ(
      cacheable_opacity_item->GetId().value(),
      RasterCacheKeyID(RasterCacheKeyID::LayerChildrenIds(layer.get()).value(),
                       RasterCacheKeyType::kLayerChildren));
  EXPECT_FALSE(raster_cache()->Draw(cacheable_opacity_item->GetId().value(),
                                    other_canvas, &paint));
  EXPECT_TRUE(raster_cache()->Draw(cacheable_opacity_item->GetId().value(),
                                   cache_canvas, &paint));
}

TEST_F(OpacityLayerTest, ShouldNotCacheChildren) {
  DlPaint paint;
  auto opacity_layer = std::make_shared<OpacityLayer>(128, DlPoint(20, 20));
  auto mock_layer = MockLayer::MakeOpacityCompatible(DlPath());
  opacity_layer->Add(mock_layer);

  PrerollContext* context = preroll_context();

  use_mock_raster_cache();

  EXPECT_EQ(raster_cache()->GetLayerCachedEntriesCount(), (size_t)0);

  const auto* cacheable_opacity_item = opacity_layer->raster_cache_item();

  EXPECT_EQ(raster_cache()->GetLayerCachedEntriesCount(), (size_t)0);
  EXPECT_EQ(cacheable_opacity_item->cache_state(),
            RasterCacheItem::CacheState::kNone);
  EXPECT_FALSE(cacheable_opacity_item->GetId().has_value());

  opacity_layer->Preroll(preroll_context());

  EXPECT_EQ(context->renderable_state_flags,
            LayerStateStack::kCallerCanApplyOpacity);
  EXPECT_TRUE(opacity_layer->children_can_accept_opacity());
  LayerTree::TryToRasterCache(cacheable_items(), &paint_context());
  EXPECT_EQ(raster_cache()->GetLayerCachedEntriesCount(), (size_t)0);
  EXPECT_EQ(cacheable_opacity_item->cache_state(),
            RasterCacheItem::CacheState::kNone);
  EXPECT_FALSE(cacheable_opacity_item->Draw(paint_context(), &paint));
}

TEST_F(OpacityLayerTest, FullyOpaque) {
  const DlPath child_path = DlPath::MakeRect(DlRect::MakeWH(5.0f, 5.0f));
  const DlPoint layer_offset = DlPoint(0.5f, 1.5f);
  const DlMatrix initial_transform = DlMatrix::MakeTranslation({0.5f, 0.5f});
  const DlMatrix layer_transform = DlMatrix::MakeTranslation(layer_offset);
  const DlPaint child_paint = DlPaint(DlColor::kGreen());
  const DlRect expected_layer_bounds =
      child_path.GetBounds().TransformAndClipBounds(layer_transform);
  auto mock_layer = std::make_shared<MockLayer>(child_path, child_paint);
  auto layer =
      std::make_shared<OpacityLayer>(DlColor::toAlpha(1.0f), layer_offset);
  layer->Add(mock_layer);

  preroll_context()->state_stack.set_preroll_delegate(initial_transform);
  layer->Preroll(preroll_context());
  EXPECT_EQ(mock_layer->paint_bounds(), child_path.GetBounds());
  EXPECT_EQ(layer->paint_bounds(), expected_layer_bounds);
  EXPECT_EQ(layer->child_paint_bounds(), child_path.GetBounds());
  EXPECT_TRUE(mock_layer->needs_painting(paint_context()));
  EXPECT_TRUE(layer->needs_painting(paint_context()));
  EXPECT_EQ(mock_layer->parent_matrix(), initial_transform * layer_transform);
  EXPECT_EQ(mock_layer->parent_mutators(),
            std::vector({Mutator(layer_transform)}));

  DisplayListBuilder expected_builder;
  /* (Opacity)layer::Paint */ {
    expected_builder.Save();
    {
      expected_builder.Translate(layer_offset.x, layer_offset.y);
      // Opaque alpha needs no SaveLayer, just recurse into painting mock_layer
      /* mock_layer::Paint */ {
        expected_builder.DrawPath(child_path, child_paint);
      }
    }
    expected_builder.Restore();
  }
  layer->Paint(display_list_paint_context());
  EXPECT_TRUE(DisplayListsEQ_Verbose(display_list(), expected_builder.Build()));
}

TEST_F(OpacityLayerTest, FullyTransparent) {
  const DlRect child_bounds = DlRect::MakeWH(5.0f, 5.0f);
  const DlPath child_path = DlPath::MakeRect(child_bounds);
  const DlPoint layer_offset = DlPoint(0.5f, 1.5f);
  const DlMatrix initial_transform = DlMatrix::MakeTranslation({0.5f, 0.5f});
  const DlMatrix layer_transform = DlMatrix::MakeTranslation(layer_offset);
  const DlPaint child_paint = DlPaint(DlColor::kGreen());
  const DlRect expected_layer_bounds =
      child_path.GetBounds().TransformAndClipBounds(layer_transform);
  auto mock_layer = std::make_shared<MockLayer>(child_path, child_paint);
  auto layer = std::make_shared<OpacityLayer>(0u, layer_offset);
  layer->Add(mock_layer);

  preroll_context()->state_stack.set_preroll_delegate(initial_transform);
  layer->Preroll(preroll_context());
  EXPECT_EQ(mock_layer->paint_bounds(), child_path.GetBounds());
  EXPECT_EQ(layer->paint_bounds(), expected_layer_bounds);
  EXPECT_EQ(layer->child_paint_bounds(), child_path.GetBounds());
  EXPECT_TRUE(mock_layer->needs_painting(paint_context()));
  EXPECT_TRUE(layer->needs_painting(paint_context()));
  EXPECT_EQ(mock_layer->parent_matrix(), initial_transform * layer_transform);
  EXPECT_EQ(mock_layer->parent_mutators(),
            std::vector({Mutator(layer_transform), Mutator(0u)}));

  DisplayListBuilder expected_builder;
  /* (Opacity)layer::Paint */ {
    expected_builder.Save();
    {
      expected_builder.Translate(layer_offset.x, layer_offset.y);
      /* (Opacity)layer::PaintChildren */ {
        DlPaint save_paint(DlPaint().setOpacity(layer->opacity()));
        expected_builder.SaveLayer(child_bounds, &save_paint);
        /* mock_layer::Paint */ {
          expected_builder.DrawPath(child_path, child_paint);
        }
        expected_builder.Restore();
      }
    }
    expected_builder.Restore();
  }
  layer->Paint(display_list_paint_context());
  EXPECT_TRUE(DisplayListsEQ_Verbose(display_list(), expected_builder.Build()));
}

TEST_F(OpacityLayerTest, HalfTransparent) {
  const DlPath child_path = DlPath::MakeRect(DlRect::MakeWH(5.0f, 5.0f));
  const DlPoint layer_offset = DlPoint(0.5f, 1.5f);
  const DlMatrix initial_transform = DlMatrix::MakeTranslation({0.5f, 0.5f});
  const DlMatrix layer_transform = DlMatrix::MakeTranslation(layer_offset);
  const DlPaint child_paint = DlPaint(DlColor::kGreen());
  const DlRect expected_layer_bounds =
      child_path.GetBounds().TransformAndClipBounds(layer_transform);
  const uint8_t alpha_half = DlColor::toAlpha(0.5f);
  auto mock_layer = std::make_shared<MockLayer>(child_path, child_paint);
  auto layer = std::make_shared<OpacityLayer>(alpha_half, layer_offset);
  layer->Add(mock_layer);

  preroll_context()->state_stack.set_preroll_delegate(initial_transform);
  layer->Preroll(preroll_context());
  EXPECT_EQ(mock_layer->paint_bounds(), child_path.GetBounds());
  EXPECT_EQ(layer->paint_bounds(), expected_layer_bounds);
  EXPECT_EQ(layer->child_paint_bounds(), child_path.GetBounds());
  EXPECT_TRUE(mock_layer->needs_painting(paint_context()));
  EXPECT_TRUE(layer->needs_painting(paint_context()));
  EXPECT_EQ(mock_layer->parent_matrix(), initial_transform * layer_transform);
  EXPECT_EQ(mock_layer->parent_mutators(),
            std::vector({Mutator(layer_transform), Mutator(alpha_half)}));

  DlRect opacity_bounds =
      DlRect::RoundOut(expected_layer_bounds.Shift(-layer_offset));
  DlPaint save_paint = DlPaint().setAlpha(alpha_half);
  DlPaint child_dl_paint = DlPaint(DlColor::kGreen());

  auto expected_builder = DisplayListBuilder();
  /* (Opacity)layer::Paint */ {
    expected_builder.Save();
    expected_builder.Translate(layer_offset.x, layer_offset.y);
    /* (Opacity)layer::PaintChildren */ {
      expected_builder.SaveLayer(opacity_bounds, &save_paint);
      /* mock_layer::Paint */ {
        expected_builder.DrawPath(child_path, child_dl_paint);
      }
      expected_builder.Restore();
    }
    expected_builder.Restore();
  }
  sk_sp<DisplayList> expected_display_list = expected_builder.Build();

  layer->Paint(display_list_paint_context());
  EXPECT_TRUE(DisplayListsEQ_Verbose(display_list(), expected_display_list));
}

TEST_F(OpacityLayerTest, Nested) {
  const DlPath child1_path = DlPath::MakeRect(DlRect::MakeWH(5.0f, 6.0f));
  const DlPath child2_path = DlPath::MakeRect(DlRect::MakeWH(2.0f, 7.0f));
  const DlPath child3_path = DlPath::MakeRect(DlRect::MakeWH(6.0f, 6.0f));
  const DlPoint layer1_offset = DlPoint(0.5f, 1.5f);
  const DlPoint layer2_offset = DlPoint(2.5f, 0.5f);
  const DlMatrix initial_transform = DlMatrix::MakeTranslation({0.5f, 0.5f});
  const DlMatrix layer1_transform = DlMatrix::MakeTranslation(layer1_offset);
  const DlMatrix layer2_transform = DlMatrix::MakeTranslation(layer2_offset);
  const DlPaint child1_paint = DlPaint(DlColor::kRed());
  const DlPaint child2_paint = DlPaint(DlColor::kBlue());
  const DlPaint child3_paint = DlPaint(DlColor::kGreen());
  const uint8_t alpha1 = 155u;
  const uint8_t alpha2 = 224u;
  auto mock_layer1 = std::make_shared<MockLayer>(child1_path, child1_paint);
  auto mock_layer2 = std::make_shared<MockLayer>(child2_path, child2_paint);
  auto mock_layer3 = std::make_shared<MockLayer>(child3_path, child3_paint);
  auto layer1 = std::make_shared<OpacityLayer>(alpha1, layer1_offset);
  auto layer2 = std::make_shared<OpacityLayer>(alpha2, layer2_offset);
  layer2->Add(mock_layer2);
  layer1->Add(mock_layer1);
  layer1->Add(layer2);
  layer1->Add(mock_layer3);  // Ensure something is processed after recursion

  const DlRect expected_layer2_bounds =
      child2_path.GetBounds().TransformAndClipBounds(layer2_transform);
  const DlRect layer1_child_bounds =       //
      expected_layer2_bounds               //
          .Union(child1_path.GetBounds())  //
          .Union(child3_path.GetBounds());
  const DlRect expected_layer1_bounds =
      layer1_child_bounds.TransformAndClipBounds(layer1_transform);
  preroll_context()->state_stack.set_preroll_delegate(initial_transform);
  layer1->Preroll(preroll_context());
  EXPECT_EQ(mock_layer1->paint_bounds(), child1_path.GetBounds());
  EXPECT_EQ(mock_layer2->paint_bounds(), child2_path.GetBounds());
  EXPECT_EQ(mock_layer3->paint_bounds(), child3_path.GetBounds());
  EXPECT_EQ(layer1->paint_bounds(), expected_layer1_bounds);
  EXPECT_EQ(layer1->child_paint_bounds(), layer1_child_bounds);
  EXPECT_EQ(layer2->paint_bounds(), expected_layer2_bounds);
  EXPECT_EQ(layer2->child_paint_bounds(), child2_path.GetBounds());
  EXPECT_TRUE(mock_layer1->needs_painting(paint_context()));
  EXPECT_TRUE(mock_layer2->needs_painting(paint_context()));
  EXPECT_TRUE(mock_layer3->needs_painting(paint_context()));
  EXPECT_TRUE(layer1->needs_painting(paint_context()));
  EXPECT_TRUE(layer2->needs_painting(paint_context()));
  EXPECT_EQ(mock_layer1->parent_matrix(), initial_transform * layer1_transform);
  EXPECT_EQ(mock_layer1->parent_mutators(),
            std::vector({Mutator(layer1_transform), Mutator(alpha1)}));
  EXPECT_EQ(mock_layer2->parent_matrix(),
            (initial_transform * layer1_transform) * layer2_transform);
  EXPECT_EQ(mock_layer2->parent_mutators(),
            std::vector({Mutator(layer1_transform), Mutator(alpha1),
                         Mutator(layer2_transform), Mutator(alpha2)}));
  EXPECT_EQ(mock_layer3->parent_matrix(), initial_transform * layer1_transform);
  EXPECT_EQ(mock_layer3->parent_mutators(),
            std::vector({Mutator(layer1_transform), Mutator(alpha1)}));

  DlRect opacity1_bounds = expected_layer1_bounds.Shift(-layer1_offset);
  DlRect opacity2_bounds = expected_layer2_bounds.Shift(-layer2_offset);
  DlPaint opacity1_paint = DlPaint().setAlpha(alpha1);
  DlPaint opacity2_paint = DlPaint().setAlpha(alpha2);

  DisplayListBuilder expected_builder;
  /* (Opacity)layer1::Paint */ {
    expected_builder.Save();
    {
      expected_builder.Translate(layer1_offset.x, layer1_offset.y);
      /* (Opacity)layer1::PaintChildren */ {
        expected_builder.SaveLayer(opacity1_bounds, &opacity1_paint);
        /* mock_layer1::Paint */ {
          expected_builder.DrawPath(child1_path, child1_paint);
        }
        /* (Opacity)layer2::Paint */ {
          expected_builder.Save();
          {
            expected_builder.Translate(layer2_offset.x, layer2_offset.y);
            /* (Opacity)layer2::PaintChidren */ {
              expected_builder.SaveLayer(opacity2_bounds, &opacity2_paint);
              {
                /* mock_layer2::Paint */ {
                  expected_builder.DrawPath(child2_path, child2_paint);
                }
              }
              expected_builder.Restore();
            }
          }
          expected_builder.Restore();
        }
        /* mock_layer3::Paint */ {
          expected_builder.DrawPath(child3_path, child3_paint);
        }
        expected_builder.Restore();
      }
    }
    expected_builder.Restore();
  }
  layer1->Paint(display_list_paint_context());
  EXPECT_TRUE(DisplayListsEQ_Verbose(display_list(), expected_builder.Build()));
}

TEST_F(OpacityLayerTest, Readback) {
  auto layer = std::make_shared<OpacityLayer>(0xff, DlPoint());
  layer->Add(std::make_shared<MockLayer>(DlPath()));

  // OpacityLayer does not read from surface
  preroll_context()->surface_needs_readback = false;
  layer->Preroll(preroll_context());
  EXPECT_FALSE(preroll_context()->surface_needs_readback);

  // OpacityLayer blocks child with readback
  auto mock_layer = std::make_shared<MockLayer>(DlPath(), DlPaint());
  mock_layer->set_fake_reads_surface(true);
  layer->Add(mock_layer);
  preroll_context()->surface_needs_readback = false;
  layer->Preroll(preroll_context());
  EXPECT_FALSE(preroll_context()->surface_needs_readback);
}

TEST_F(OpacityLayerTest, CullRectIsTransformed) {
  auto clip_rect_layer = std::make_shared<ClipRectLayer>(
      DlRect::MakeLTRB(0, 0, 10, 10), Clip::kHardEdge);
  auto opacity_layer = std::make_shared<OpacityLayer>(128u, DlPoint(20, 20));
  auto mock_layer = std::make_shared<MockLayer>(DlPath());
  clip_rect_layer->Add(opacity_layer);
  opacity_layer->Add(mock_layer);
  clip_rect_layer->Preroll(preroll_context());
  EXPECT_EQ(mock_layer->parent_cull_rect().GetLeft(), -20);
  EXPECT_EQ(mock_layer->parent_cull_rect().GetTop(), -20);
}

TEST_F(OpacityLayerTest, OpacityInheritanceCompatibleChild) {
  auto opacity_layer = std::make_shared<OpacityLayer>(128u, DlPoint(20, 20));
  auto mock_layer = MockLayer::MakeOpacityCompatible(DlPath());
  opacity_layer->Add(mock_layer);

  PrerollContext* context = preroll_context();
  opacity_layer->Preroll(context);
  EXPECT_EQ(context->renderable_state_flags,
            LayerStateStack::kCallerCanApplyOpacity);
  EXPECT_TRUE(opacity_layer->children_can_accept_opacity());
}

TEST_F(OpacityLayerTest, OpacityInheritanceIncompatibleChild) {
  auto opacity_layer = std::make_shared<OpacityLayer>(128u, DlPoint(20, 20));
  auto mock_layer = MockLayer::Make(DlPath());
  opacity_layer->Add(mock_layer);

  PrerollContext* context = preroll_context();
  opacity_layer->Preroll(context);
  EXPECT_EQ(context->renderable_state_flags,
            LayerStateStack::kCallerCanApplyOpacity);
  EXPECT_FALSE(opacity_layer->children_can_accept_opacity());
}

TEST_F(OpacityLayerTest, OpacityInheritanceThroughContainer) {
  auto opacity_layer = std::make_shared<OpacityLayer>(128u, DlPoint(20, 20));
  auto container_layer = std::make_shared<ContainerLayer>();
  auto mock_layer = MockLayer::MakeOpacityCompatible(DlPath());
  container_layer->Add(mock_layer);
  opacity_layer->Add(container_layer);

  PrerollContext* context = preroll_context();
  opacity_layer->Preroll(context);
  EXPECT_EQ(context->renderable_state_flags,
            LayerStateStack::kCallerCanApplyOpacity);
  EXPECT_TRUE(opacity_layer->children_can_accept_opacity());
}

TEST_F(OpacityLayerTest, OpacityInheritanceThroughTransform) {
  auto opacity_layer = std::make_shared<OpacityLayer>(128u, DlPoint(20, 20));
  auto transformLayer =
      std::make_shared<TransformLayer>(DlMatrix::MakeScale({2.0f, 2.0f, 1.0f}));
  auto mock_layer = MockLayer::MakeOpacityCompatible(DlPath());
  transformLayer->Add(mock_layer);
  opacity_layer->Add(transformLayer);

  PrerollContext* context = preroll_context();
  opacity_layer->Preroll(context);
  EXPECT_EQ(context->renderable_state_flags,
            LayerStateStack::kCallerCanApplyOpacity);
  EXPECT_TRUE(opacity_layer->children_can_accept_opacity());
}

TEST_F(OpacityLayerTest, OpacityInheritanceThroughImageFilter) {
  auto opacity_layer = std::make_shared<OpacityLayer>(128u, DlPoint(20, 20));
  auto filter_layer = std::make_shared<ImageFilterLayer>(
      DlImageFilter::MakeBlur(5.0, 5.0, DlTileMode::kClamp));
  auto mock_layer = MockLayer::MakeOpacityCompatible(DlPath());
  filter_layer->Add(mock_layer);
  opacity_layer->Add(filter_layer);

  PrerollContext* context = preroll_context();
  opacity_layer->Preroll(context);
  EXPECT_EQ(context->renderable_state_flags,
            LayerStateStack::kCallerCanApplyOpacity);
  EXPECT_TRUE(opacity_layer->children_can_accept_opacity());
}

TEST_F(OpacityLayerTest, OpacityInheritanceNestedWithCompatibleChild) {
  DlPoint offset1 = DlPoint(10, 20);
  DlPoint offset2 = DlPoint(20, 10);
  DlPath mock_path = DlPath::MakeRectLTRB(10, 10, 20, 20);
  auto opacity_layer_1 = std::make_shared<OpacityLayer>(128u, offset1);
  auto opacity_layer_2 = std::make_shared<OpacityLayer>(64u, offset2);
  auto mock_layer = MockLayer::MakeOpacityCompatible(mock_path);
  opacity_layer_2->Add(mock_layer);
  opacity_layer_1->Add(opacity_layer_2);

  PrerollContext* context = preroll_context();
  opacity_layer_1->Preroll(context);
  EXPECT_EQ(context->renderable_state_flags,
            LayerStateStack::kCallerCanApplyOpacity);
  EXPECT_TRUE(opacity_layer_1->children_can_accept_opacity());
  EXPECT_TRUE(opacity_layer_2->children_can_accept_opacity());

  DlPaint savelayer_paint;
  DlScalar inherited_opacity = DlColor::toOpacity(128u);
  inherited_opacity *= DlColor::toOpacity(64u);
  savelayer_paint.setOpacity(inherited_opacity);

  DisplayListBuilder expected_builder;
  /* opacity_layer_1::Paint */ {
    expected_builder.Save();
    {
      expected_builder.Translate(offset1.x, offset1.y);
      /* opacity_layer_2::Paint */ {
        expected_builder.Save();
        {
          expected_builder.Translate(offset2.x, offset2.y);
          /* mock_layer::Paint */ {
            expected_builder.DrawPath(mock_path,
                                      DlPaint().setOpacity(inherited_opacity));
          }
        }
        expected_builder.Restore();
      }
    }
    expected_builder.Restore();
  }

  opacity_layer_1->Paint(display_list_paint_context());
  EXPECT_TRUE(DisplayListsEQ_Verbose(expected_builder.Build(), display_list()));
}

TEST_F(OpacityLayerTest, OpacityInheritanceNestedWithIncompatibleChild) {
  DlPoint offset1 = DlPoint(10, 20);
  DlPoint offset2 = DlPoint(20, 10);
  DlPath mock_path = DlPath::MakeRectLTRB(10, 10, 20, 20);
  auto opacity_layer_1 = std::make_shared<OpacityLayer>(128u, offset1);
  auto opacity_layer_2 = std::make_shared<OpacityLayer>(64u, offset2);
  auto mock_layer = MockLayer::Make(mock_path);
  opacity_layer_2->Add(mock_layer);
  opacity_layer_1->Add(opacity_layer_2);

  PrerollContext* context = preroll_context();
  opacity_layer_1->Preroll(context);
  EXPECT_EQ(context->renderable_state_flags,
            LayerStateStack::kCallerCanApplyOpacity);
  EXPECT_TRUE(opacity_layer_1->children_can_accept_opacity());
  EXPECT_FALSE(opacity_layer_2->children_can_accept_opacity());

  DlPaint savelayer_paint;
  DlScalar inherited_opacity = DlColor::toOpacity(128);
  inherited_opacity *= DlColor::toOpacity(64u);
  savelayer_paint.setOpacity(inherited_opacity);

  DisplayListBuilder expected_builder;
  /* opacity_layer_1::Paint */ {
    expected_builder.Save();
    {
      expected_builder.Translate(offset1.x, offset1.y);
      /* opacity_layer_2::Paint */ {
        expected_builder.Save();
        {
          expected_builder.Translate(offset2.x, offset2.y);
          expected_builder.SaveLayer(mock_layer->paint_bounds(),
                                     &savelayer_paint);
          /* mock_layer::Paint */ {
            expected_builder.DrawPath(mock_path, DlPaint());
          }
        }
        expected_builder.Restore();
      }
    }
    expected_builder.Restore();
  }

  opacity_layer_1->Paint(display_list_paint_context());
  EXPECT_TRUE(DisplayListsEQ_Verbose(expected_builder.Build(), display_list()));
}

using OpacityLayerDiffTest = DiffContextTest;

TEST_F(OpacityLayerDiffTest, FractionalTranslation) {
  auto picture = CreateDisplayListLayer(
      CreateDisplayList(DlRect::MakeLTRB(10, 10, 60, 60)));
  auto layer = CreateOpacityLater({picture}, 128u, DlPoint(0.5f, 0.5f));

  MockLayerTree tree1;
  tree1.root()->Add(layer);

  auto damage = DiffLayerTree(tree1, MockLayerTree(), DlIRect(), 0, 0,
                              /*use_raster_cache=*/false);
  EXPECT_EQ(damage.frame_damage, DlIRect::MakeLTRB(10, 10, 61, 61));
}

TEST_F(OpacityLayerDiffTest, FractionalTranslationWithRasterCache) {
  auto picture = CreateDisplayListLayer(
      CreateDisplayList(DlRect::MakeLTRB(10, 10, 60, 60)));
  auto layer = CreateOpacityLater({picture}, 128u, DlPoint(0.5f, 0.5f));

  MockLayerTree tree1;
  tree1.root()->Add(layer);

  auto damage = DiffLayerTree(tree1, MockLayerTree(), DlIRect(), 0, 0,
                              /*use_raster_cache=*/true);
  EXPECT_EQ(damage.frame_damage, DlIRect::MakeLTRB(11, 11, 61, 61));
}

TEST_F(OpacityLayerTest, FullyOpaqueWithFractionalValues) {
  use_mock_raster_cache();  // Ensure pixel-snapped alignment.

  const DlPath child_path = DlPath::MakeRect(DlRect::MakeWH(5.0f, 5.0f));
  const DlPoint layer_offset = DlPoint(0.5f, 1.5f);
  const DlMatrix initial_transform = DlMatrix::MakeTranslation({0.5f, 0.5f});
  const DlPaint child_paint = DlPaint(DlColor::kGreen());
  auto mock_layer = std::make_shared<MockLayer>(child_path, child_paint);
  auto layer = std::make_shared<OpacityLayer>(0xff, layer_offset);
  layer->Add(mock_layer);

  preroll_context()->state_stack.set_preroll_delegate(initial_transform);
  layer->Preroll(preroll_context());

  auto expected_builder = DisplayListBuilder();
  /* (Opacity)layer::Paint */ {
    expected_builder.Save();
    expected_builder.Translate(layer_offset.x, layer_offset.y);
    // Opaque alpha needs no SaveLayer, just recurse into painting mock_layer
    // but since we use the mock raster cache we pixel snap the transform
    expected_builder.TransformReset();
    expected_builder.Transform2DAffine(1, 0, std::round(layer_offset.x),  //
                                       0, 1, std::round(layer_offset.y));
    /* mock_layer::Paint */ {
      expected_builder.DrawPath(child_path, child_paint);
    }
    expected_builder.Restore();
  }
  sk_sp<DisplayList> expected_display_list = expected_builder.Build();

  layer->Paint(display_list_paint_context());
  EXPECT_TRUE(DisplayListsEQ_Verbose(display_list(), expected_display_list));
}

TEST_F(OpacityLayerTest, FullyTransparentDoesNotCullPlatformView) {
  const DlPoint opacity_offset = DlPoint(0.5f, 1.5f);
  const DlPoint view_offset = DlPoint(0.0f, 0.0f);
  const DlSize view_size = DlSize(8.0f, 8.0f);
  const int64_t view_id = 42;
  auto platform_view =
      std::make_shared<PlatformViewLayer>(view_offset, view_size, view_id);

  auto opacity = std::make_shared<OpacityLayer>(0u, opacity_offset);
  opacity->Add(platform_view);

  auto embedder = MockViewEmbedder();
  DisplayListBuilder fake_overlay_builder;
  embedder.AddCanvas(&fake_overlay_builder);
  preroll_context()->view_embedder = &embedder;
  paint_context().view_embedder = &embedder;

  opacity->Preroll(preroll_context());
  EXPECT_EQ(embedder.prerolled_views(), std::vector<int64_t>({view_id}));

  opacity->Paint(paint_context());
  EXPECT_EQ(embedder.painted_views(), std::vector<int64_t>({view_id}));
}

}  // namespace testing
}  // namespace flutter

// NOLINTEND(bugprone-unchecked-optional-access)
