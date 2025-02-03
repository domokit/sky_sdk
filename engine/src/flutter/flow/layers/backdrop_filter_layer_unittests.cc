// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/flow/layers/backdrop_filter_layer.h"

#include "flutter/display_list/effects/dl_image_filter.h"
#include "flutter/flow/layers/clip_rect_layer.h"
#include "flutter/flow/layers/transform_layer.h"
#include "flutter/flow/testing/diff_context_test.h"
#include "flutter/flow/testing/layer_test.h"
#include "flutter/flow/testing/mock_layer.h"
#include "flutter/fml/macros.h"

namespace flutter {
namespace testing {

using BackdropFilterLayerTest = LayerTest;

#ifndef NDEBUG
TEST_F(BackdropFilterLayerTest, PaintingEmptyLayerDies) {
  auto filter = DlImageFilter::MakeBlur(5, 5, DlTileMode::kClamp);
  auto layer =
      std::make_shared<BackdropFilterLayer>(filter, DlBlendMode::kSrcOver);
  auto parent = std::make_shared<ClipRectLayer>(DlRect(), Clip::kHardEdge);
  parent->Add(layer);

  parent->Preroll(preroll_context());
  EXPECT_EQ(layer->paint_bounds(), DlRect());
  EXPECT_EQ(layer->child_paint_bounds(), DlRect());
  EXPECT_FALSE(layer->needs_painting(paint_context()));

  EXPECT_DEATH_IF_SUPPORTED(layer->Paint(paint_context()),
                            "needs_painting\\(context\\)");
}

TEST_F(BackdropFilterLayerTest, PaintBeforePrerollDies) {
  const DlRect child_bounds = DlRect::MakeLTRB(5.0f, 6.0f, 20.5f, 21.5f);
  const DlPath child_path = DlPath::MakeRect(child_bounds);
  auto mock_layer = std::make_shared<MockLayer>(child_path);
  auto filter = DlImageFilter::MakeBlur(5, 5, DlTileMode::kClamp);
  auto layer =
      std::make_shared<BackdropFilterLayer>(filter, DlBlendMode::kSrcOver);
  layer->Add(mock_layer);

  EXPECT_EQ(layer->paint_bounds(), DlRect());
  EXPECT_EQ(layer->child_paint_bounds(), DlRect());
  EXPECT_DEATH_IF_SUPPORTED(layer->Paint(paint_context()),
                            "needs_painting\\(context\\)");
}
#endif

TEST_F(BackdropFilterLayerTest, EmptyFilter) {
  const DlMatrix initial_transform = DlMatrix::MakeTranslation({0.5f, 1.0f});
  const DlRect child_bounds = DlRect::MakeLTRB(5.0f, 6.0f, 20.5f, 21.5f);
  const DlPath child_path = DlPath::MakeRect(child_bounds);
  const DlPaint child_paint = DlPaint(DlColor::kYellow());
  auto mock_layer = std::make_shared<MockLayer>(child_path, child_paint);
  auto layer =
      std::make_shared<BackdropFilterLayer>(nullptr, DlBlendMode::kSrcOver);
  layer->Add(mock_layer);
  auto parent = std::make_shared<ClipRectLayer>(child_bounds, Clip::kHardEdge);
  parent->Add(layer);

  preroll_context()->state_stack.set_preroll_delegate(initial_transform);
  parent->Preroll(preroll_context());
  EXPECT_EQ(layer->paint_bounds(), child_bounds);
  EXPECT_EQ(layer->child_paint_bounds(), child_bounds);
  EXPECT_TRUE(layer->needs_painting(paint_context()));
  EXPECT_EQ(mock_layer->parent_matrix(), initial_transform);

  parent->Paint(display_list_paint_context());
  DisplayListBuilder expected_builder;
  /* (ClipRect)parent::Paint */ {
    expected_builder.Save();
    {
      expected_builder.ClipRect(child_bounds, DlClipOp::kIntersect, false);
      /* (BackdropFilter)layer::Paint */ {
        expected_builder.Save();
        {
          expected_builder.SaveLayer(child_bounds, nullptr, nullptr);
          {
            /* mock_layer::Paint */ {
              expected_builder.DrawPath(child_path, child_paint);
            }
          }
          expected_builder.Restore();
        }
        expected_builder.Restore();
      }
    }
    expected_builder.Restore();
  }
  EXPECT_TRUE(DisplayListsEQ_Verbose(display_list(), expected_builder.Build()));
}

TEST_F(BackdropFilterLayerTest, SimpleFilter) {
  const DlMatrix initial_transform = DlMatrix::MakeTranslation({0.5f, 1.0f});
  const DlRect child_bounds = DlRect::MakeLTRB(5.0f, 6.0f, 20.5f, 21.5f);
  const DlPath child_path = DlPath::MakeRect(child_bounds);
  const DlPaint child_paint = DlPaint(DlColor::kYellow());
  auto layer_filter = DlImageFilter::MakeBlur(2.5, 3.2, DlTileMode::kClamp);
  auto mock_layer = std::make_shared<MockLayer>(child_path, child_paint);
  auto layer = std::make_shared<BackdropFilterLayer>(layer_filter,
                                                     DlBlendMode::kSrcOver);
  layer->Add(mock_layer);
  auto parent = std::make_shared<ClipRectLayer>(child_bounds, Clip::kHardEdge);
  parent->Add(layer);

  preroll_context()->state_stack.set_preroll_delegate(initial_transform);
  parent->Preroll(preroll_context());
  EXPECT_EQ(layer->paint_bounds(), child_bounds);
  EXPECT_EQ(layer->child_paint_bounds(), child_bounds);
  EXPECT_TRUE(layer->needs_painting(paint_context()));
  EXPECT_EQ(mock_layer->parent_matrix(), initial_transform);

  parent->Paint(display_list_paint_context());
  DisplayListBuilder expected_builder;
  /* (ClipRect)parent::Paint */ {
    expected_builder.Save();
    {
      expected_builder.ClipRect(child_bounds, DlClipOp::kIntersect, false);
      /* (BackdropFilter)layer::Paint */ {
        expected_builder.Save();
        {
          expected_builder.SaveLayer(child_bounds, nullptr, layer_filter.get());
          {
            /* mock_layer::Paint */ {
              expected_builder.DrawPath(child_path, child_paint);
            }
          }
          expected_builder.Restore();
        }
        expected_builder.Restore();
      }
    }
    expected_builder.Restore();
  }
  EXPECT_TRUE(DisplayListsEQ_Verbose(display_list(), expected_builder.Build()));
}

TEST_F(BackdropFilterLayerTest, NonSrcOverBlend) {
  const DlMatrix initial_transform = DlMatrix::MakeTranslation({0.5f, 1.0f});
  const DlRect child_bounds = DlRect::MakeLTRB(5.0f, 6.0f, 20.5f, 21.5f);
  const DlPath child_path = DlPath::MakeRect(child_bounds);
  const DlPaint child_paint = DlPaint(DlColor::kYellow());
  auto layer_filter = DlImageFilter::MakeBlur(2.5, 3.2, DlTileMode::kClamp);
  auto mock_layer = std::make_shared<MockLayer>(child_path, child_paint);
  auto layer =
      std::make_shared<BackdropFilterLayer>(layer_filter, DlBlendMode::kSrc);
  layer->Add(mock_layer);
  auto parent = std::make_shared<ClipRectLayer>(child_bounds, Clip::kHardEdge);
  parent->Add(layer);

  preroll_context()->state_stack.set_preroll_delegate(initial_transform);
  parent->Preroll(preroll_context());
  EXPECT_EQ(layer->paint_bounds(), child_bounds);
  EXPECT_EQ(layer->child_paint_bounds(), child_bounds);
  EXPECT_TRUE(layer->needs_painting(paint_context()));
  EXPECT_EQ(mock_layer->parent_matrix(), initial_transform);

  DlPaint filter_paint = DlPaint();
  filter_paint.setBlendMode(DlBlendMode::kSrc);

  parent->Paint(display_list_paint_context());
  DisplayListBuilder expected_builder;
  /* (ClipRect)parent::Paint */ {
    expected_builder.Save();
    {
      expected_builder.ClipRect(child_bounds, DlClipOp::kIntersect, false);
      /* (BackdropFilter)layer::Paint */ {
        expected_builder.Save();
        {
          DlPaint save_paint = DlPaint().setBlendMode(DlBlendMode::kSrc);
          expected_builder.SaveLayer(child_bounds, &save_paint,
                                     layer_filter.get());
          {
            /* mock_layer::Paint */ {
              expected_builder.DrawPath(child_path, child_paint);
            }
          }
          expected_builder.Restore();
        }
        expected_builder.Restore();
      }
    }
    expected_builder.Restore();
  }
  EXPECT_TRUE(DisplayListsEQ_Verbose(display_list(), expected_builder.Build()));
}

TEST_F(BackdropFilterLayerTest, MultipleChildren) {
  const DlMatrix initial_transform = DlMatrix::MakeTranslation({0.5f, 1.0f});
  const DlRect child_bounds = DlRect::MakeLTRB(5.0f, 6.0f, 2.5f, 3.5f);
  const DlPath child_path1 = DlPath::MakeRect(child_bounds);
  const DlPath child_path2 = DlPath::MakeRect(child_bounds.Shift(3.0f, 0.0f));
  const DlPaint child_paint1 = DlPaint(DlColor::kYellow());
  const DlPaint child_paint2 = DlPaint(DlColor::kCyan());
  const DlRect children_bounds =
      child_path1.GetBounds().Union(child_path2.GetBounds());
  auto layer_filter = DlImageFilter::MakeBlur(2.5, 3.2, DlTileMode::kClamp);
  auto mock_layer1 = std::make_shared<MockLayer>(child_path1, child_paint1);
  auto mock_layer2 = std::make_shared<MockLayer>(child_path2, child_paint2);
  auto layer = std::make_shared<BackdropFilterLayer>(layer_filter,
                                                     DlBlendMode::kSrcOver);
  layer->Add(mock_layer1);
  layer->Add(mock_layer2);
  auto parent =
      std::make_shared<ClipRectLayer>(children_bounds, Clip::kHardEdge);
  parent->Add(layer);

  preroll_context()->state_stack.set_preroll_delegate(initial_transform);
  parent->Preroll(preroll_context());
  EXPECT_EQ(mock_layer1->paint_bounds(), child_path1.GetBounds());
  EXPECT_EQ(mock_layer2->paint_bounds(), child_path2.GetBounds());
  EXPECT_EQ(layer->paint_bounds(), children_bounds);
  EXPECT_EQ(layer->child_paint_bounds(), children_bounds);
  EXPECT_TRUE(mock_layer1->needs_painting(paint_context()));
  EXPECT_TRUE(mock_layer2->needs_painting(paint_context()));
  EXPECT_TRUE(layer->needs_painting(paint_context()));
  EXPECT_EQ(mock_layer1->parent_matrix(), initial_transform);
  EXPECT_EQ(mock_layer2->parent_matrix(), initial_transform);

  parent->Paint(display_list_paint_context());
  DisplayListBuilder expected_builder;
  /* (ClipRect)parent::Paint */ {
    expected_builder.Save();
    {
      expected_builder.ClipRect(children_bounds, DlClipOp::kIntersect, false);
      /* (BackdropFilter)layer::Paint */ {
        expected_builder.Save();
        {
          expected_builder.SaveLayer(children_bounds, nullptr,
                                     layer_filter.get());
          {
            /* mock_layer1::Paint */ {
              expected_builder.DrawPath(child_path1, child_paint1);
            }
            /* mock_layer2::Paint */ {
              expected_builder.DrawPath(child_path2, child_paint2);
            }
          }
          expected_builder.Restore();
        }
        expected_builder.Restore();
      }
    }
    expected_builder.Restore();
  }
  EXPECT_TRUE(DisplayListsEQ_Verbose(display_list(), expected_builder.Build()));
}

TEST_F(BackdropFilterLayerTest, Nested) {
  const DlMatrix initial_transform = DlMatrix::MakeTranslation({0.5f, 1.0f});
  const DlRect child_bounds = DlRect::MakeLTRB(5.0f, 6.0f, 2.5f, 3.5f);
  const DlPath child_path1 = DlPath::MakeRect(child_bounds);
  const DlPath child_path2 = DlPath::MakeRect(child_bounds.Shift(3.0f, 0.0f));
  const DlPaint child_paint1 = DlPaint(DlColor::kYellow());
  const DlPaint child_paint2 = DlPaint(DlColor::kCyan());
  const DlRect children_bounds =
      child_path1.GetBounds().Union(child_path2.GetBounds());
  auto layer_filter1 = DlImageFilter::MakeBlur(2.5, 3.2, DlTileMode::kClamp);
  auto layer_filter2 = DlImageFilter::MakeBlur(2.7, 3.1, DlTileMode::kDecal);
  auto mock_layer1 = std::make_shared<MockLayer>(child_path1, child_paint1);
  auto mock_layer2 = std::make_shared<MockLayer>(child_path2, child_paint2);
  auto layer1 = std::make_shared<BackdropFilterLayer>(layer_filter1,
                                                      DlBlendMode::kSrcOver);
  auto layer2 = std::make_shared<BackdropFilterLayer>(layer_filter2,
                                                      DlBlendMode::kSrcOver);
  layer2->Add(mock_layer2);
  layer1->Add(mock_layer1);
  layer1->Add(layer2);
  auto parent =
      std::make_shared<ClipRectLayer>(children_bounds, Clip::kHardEdge);
  parent->Add(layer1);

  preroll_context()->state_stack.set_preroll_delegate(initial_transform);
  parent->Preroll(preroll_context());

  EXPECT_EQ(mock_layer1->paint_bounds(), child_path1.GetBounds());
  EXPECT_EQ(mock_layer2->paint_bounds(), child_path2.GetBounds());
  EXPECT_EQ(layer1->paint_bounds(), children_bounds);
  EXPECT_EQ(layer2->paint_bounds(), children_bounds);
  EXPECT_TRUE(mock_layer1->needs_painting(paint_context()));
  EXPECT_TRUE(mock_layer2->needs_painting(paint_context()));
  EXPECT_TRUE(layer1->needs_painting(paint_context()));
  EXPECT_TRUE(layer2->needs_painting(paint_context()));
  EXPECT_EQ(mock_layer1->parent_matrix(), initial_transform);
  EXPECT_EQ(mock_layer2->parent_matrix(), initial_transform);

  parent->Paint(display_list_paint_context());
  DisplayListBuilder expected_builder;
  /* (ClipRect)parent::Paint */ {
    expected_builder.Save();
    {
      expected_builder.ClipRect(children_bounds, DlClipOp::kIntersect, false);
      /* (BackdropFilter)layer1::Paint */ {
        expected_builder.Save();
        {
          expected_builder.SaveLayer(children_bounds, nullptr,
                                     layer_filter1.get());
          {
            /* mock_layer1::Paint */ {
              expected_builder.DrawPath(child_path1, child_paint1);
            }
            /* (BackdropFilter)layer2::Paint */ {
              expected_builder.Save();
              {
                expected_builder.SaveLayer(children_bounds, nullptr,
                                           layer_filter2.get());
                {
                  /* mock_layer2::Paint */ {
                    expected_builder.DrawPath(child_path2, child_paint2);
                  }
                }
                expected_builder.Restore();
              }
              expected_builder.Restore();
            }
          }
          expected_builder.Restore();
        }
        expected_builder.Restore();
      }
    }
    expected_builder.Restore();
  }
  EXPECT_TRUE(DisplayListsEQ_Verbose(display_list(), expected_builder.Build()));
}

TEST_F(BackdropFilterLayerTest, Readback) {
  std::shared_ptr<DlImageFilter> no_filter;
  auto layer_filter = DlImageFilter::MakeBlur(5, 5, DlTileMode::kClamp);
  auto initial_transform = DlMatrix();

  // BDF with filter always reads from surface
  auto layer1 = std::make_shared<BackdropFilterLayer>(layer_filter,
                                                      DlBlendMode::kSrcOver);
  preroll_context()->surface_needs_readback = false;
  preroll_context()->state_stack.set_preroll_delegate(initial_transform);
  layer1->Preroll(preroll_context());
  EXPECT_TRUE(preroll_context()->surface_needs_readback);

  // BDF with no filter does not read from surface itself
  auto layer2 =
      std::make_shared<BackdropFilterLayer>(no_filter, DlBlendMode::kSrcOver);
  preroll_context()->surface_needs_readback = false;
  layer2->Preroll(preroll_context());
  EXPECT_FALSE(preroll_context()->surface_needs_readback);

  // BDF with no filter does not block prior readback value
  preroll_context()->surface_needs_readback = true;
  layer2->Preroll(preroll_context());
  EXPECT_TRUE(preroll_context()->surface_needs_readback);

  // BDF with no filter blocks child with readback
  auto mock_layer = std::make_shared<MockLayer>(DlPath(), DlPaint());
  mock_layer->set_fake_reads_surface(true);
  layer2->Add(mock_layer);
  preroll_context()->surface_needs_readback = false;
  layer2->Preroll(preroll_context());
  EXPECT_FALSE(preroll_context()->surface_needs_readback);
}

TEST_F(BackdropFilterLayerTest, OpacityInheritance) {
  auto backdrop_filter = DlImageFilter::MakeBlur(5, 5, DlTileMode::kClamp);
  const DlPath mock_path = DlPath::MakeRectLTRB(0, 0, 10, 10);
  const DlPaint mock_paint = DlPaint(DlColor::kRed());
  const DlRect clip_rect = DlRect::MakeLTRB(0, 0, 100, 100);

  auto clip = std::make_shared<ClipRectLayer>(clip_rect, Clip::kHardEdge);
  auto parent = std::make_shared<OpacityLayer>(128, DlPoint());
  auto layer = std::make_shared<BackdropFilterLayer>(backdrop_filter,
                                                     DlBlendMode::kSrcOver);
  auto child = std::make_shared<MockLayer>(mock_path, mock_paint);
  layer->Add(child);
  parent->Add(layer);
  clip->Add(parent);

  clip->Preroll(preroll_context());

  clip->Paint(display_list_paint_context());

  DisplayListBuilder expected_builder;
  /* ClipRectLayer::Paint */ {
    expected_builder.Save();
    {
      expected_builder.ClipRect(clip_rect, DlClipOp::kIntersect, false);
      /* OpacityLayer::Paint */ {
        // NOP - it hands opacity down to BackdropFilterLayer
        /* BackdropFilterLayer::Paint */ {
          DlPaint save_paint;
          save_paint.setAlpha(128);
          expected_builder.SaveLayer(clip_rect, &save_paint,
                                     backdrop_filter.get());
          {
            /* MockLayer::Paint */ {
              DlPaint child_paint;
              child_paint.setColor(DlColor::kRed());
              expected_builder.DrawPath(mock_path, child_paint);
            }
          }
          expected_builder.Restore();
        }
      }
    }
    expected_builder.Restore();
  }
  EXPECT_TRUE(DisplayListsEQ_Verbose(display_list(), expected_builder.Build()));
}

using BackdropLayerDiffTest = DiffContextTest;

TEST_F(BackdropLayerDiffTest, BackdropLayer) {
  auto filter = DlImageFilter::MakeBlur(10, 10, DlTileMode::kClamp);

  {
    // tests later assume 30px readback area, fail early if that's not the case
    DlIRect readback;
    EXPECT_EQ(filter->get_input_device_bounds(DlIRect::MakeWH(10, 10),
                                              DlMatrix(), readback),
              &readback);
    EXPECT_EQ(readback, DlIRect::MakeLTRB(-30, -30, 40, 40));
  }

  MockLayerTree l1(DlISize(100, 100));
  l1.root()->Add(
      std::make_shared<BackdropFilterLayer>(filter, DlBlendMode::kSrcOver));

  // no clip, effect over entire surface
  auto damage = DiffLayerTree(l1, MockLayerTree(DlISize(100, 100)));
  EXPECT_EQ(damage.frame_damage, DlIRect::MakeWH(100, 100));

  MockLayerTree l2(DlISize(100, 100));

  auto clip = std::make_shared<ClipRectLayer>(DlRect::MakeLTRB(20, 20, 60, 60),
                                              Clip::kHardEdge);
  clip->Add(
      std::make_shared<BackdropFilterLayer>(filter, DlBlendMode::kSrcOver));
  l2.root()->Add(clip);
  damage = DiffLayerTree(l2, MockLayerTree(DlISize(100, 100)));

  EXPECT_EQ(damage.frame_damage, DlIRect::MakeLTRB(0, 0, 90, 90));

  MockLayerTree l3;
  auto scale =
      std::make_shared<TransformLayer>(DlMatrix::MakeScale({2.0, 2.0, 1.0f}));
  scale->Add(clip);
  l3.root()->Add(scale);

  damage = DiffLayerTree(l3, MockLayerTree());
  EXPECT_EQ(damage.frame_damage, DlIRect::MakeLTRB(0, 0, 180, 180));

  MockLayerTree l4;
  l4.root()->Add(scale);

  // path just outside of readback region, doesn't affect blur
  auto path1 = DlPath::MakeRectLTRB(180, 180, 190, 190);
  l4.root()->Add(std::make_shared<MockLayer>(path1));
  damage = DiffLayerTree(l4, l3);
  EXPECT_EQ(damage.frame_damage, DlIRect::MakeLTRB(180, 180, 190, 190));

  MockLayerTree l5;
  l5.root()->Add(scale);

  // path just inside of readback region, must trigger backdrop repaint
  auto path2 = DlPath::MakeRectLTRB(179, 179, 189, 189);
  l5.root()->Add(std::make_shared<MockLayer>(path2));
  damage = DiffLayerTree(l5, l4);
  EXPECT_EQ(damage.frame_damage, DlIRect::MakeLTRB(0, 0, 190, 190));
}

TEST_F(BackdropLayerDiffTest, ReadbackOutsideOfPaintArea) {
  auto filter = DlImageFilter::MakeMatrix(DlMatrix::MakeTranslation({50, 50}),
                                          DlImageSampling::kLinear);

  MockLayerTree l1(DlISize(100, 100));

  auto clip = std::make_shared<ClipRectLayer>(DlRect::MakeLTRB(60, 60, 80, 80),
                                              Clip::kHardEdge);
  clip->Add(
      std::make_shared<BackdropFilterLayer>(filter, DlBlendMode::kSrcOver));
  l1.root()->Add(clip);
  auto damage = DiffLayerTree(l1, MockLayerTree(DlISize(100, 100)));

  EXPECT_EQ(damage.frame_damage, DlIRect::MakeLTRB(60 - 50, 60 - 50, 80, 80));

  MockLayerTree l2(DlISize(100, 100));
  // path inside readback area must trigger whole readback repaint + filter
  // repaint.
  auto path2 = DlPath::MakeRectXYWH(60 - 50, 60 - 50, 10, 10);
  l2.root()->Add(clip);
  l2.root()->Add(std::make_shared<MockLayer>(path2));
  damage = DiffLayerTree(l2, l1);
  EXPECT_EQ(damage.frame_damage, DlIRect::MakeLTRB(60 - 50, 60 - 50, 80, 80));
}

TEST_F(BackdropLayerDiffTest, BackdropLayerInvalidTransform) {
  auto filter = DlImageFilter::MakeBlur(10, 10, DlTileMode::kClamp);

  {
    // tests later assume 30px readback area, fail early if that's not the case
    DlIRect readback;
    EXPECT_EQ(filter->get_input_device_bounds(DlIRect::MakeWH(10, 10),
                                              DlMatrix(), readback),
              &readback);
    EXPECT_EQ(readback, DlIRect::MakeLTRB(-30, -30, 40, 40));
  }

  MockLayerTree l1(DlISize(100, 100));
  // clang-format off
  DlMatrix transform(
      1, 0, 0, 0.1f,
      0, 1, 0, 0.1f,
      0, 0, 1, 0,
      0, 0, 0, 1
  );
  // clang-format on

  auto transform_layer = std::make_shared<TransformLayer>(transform);
  l1.root()->Add(transform_layer);
  transform_layer->Add(
      std::make_shared<BackdropFilterLayer>(filter, DlBlendMode::kSrcOver));

  auto damage = DiffLayerTree(l1, MockLayerTree(DlISize(100, 100)));
  EXPECT_EQ(damage.frame_damage, DlIRect::MakeWH(100, 100));
}

}  // namespace testing
}  // namespace flutter
