// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/flow/testing/mock_layer.h"

#include "flutter/flow/testing/layer_test.h"
#include "flutter/fml/macros.h"

namespace flutter {
namespace testing {

using MockLayerTest = LayerTest;

#ifndef NDEBUG
TEST_F(MockLayerTest, PaintBeforePrerollDies) {
  DlPath path = DlPath::MakeRectLTRB(5.0f, 6.0f, 20.5f, 21.5f);
  auto layer = std::make_shared<MockLayer>(path, DlPaint());

  EXPECT_DEATH_IF_SUPPORTED(layer->Paint(paint_context()),
                            "needs_painting\\(context\\)");
}

TEST_F(MockLayerTest, PaintingEmptyLayerDies) {
  auto layer = std::make_shared<MockLayer>(DlPath(), DlPaint());

  layer->Preroll(preroll_context());
  EXPECT_EQ(layer->paint_bounds(), DlPath().GetBounds());

  EXPECT_DEATH_IF_SUPPORTED(layer->Paint(paint_context()),
                            "needs_painting\\(context\\)");
}
#endif

TEST_F(MockLayerTest, SimpleParams) {
  const DlPath path = DlPath::MakeRectLTRB(5.0f, 6.0f, 20.5f, 21.5f);
  const DlPaint paint = DlPaint(DlColor::kBlue());
  const DlMatrix start_matrix = DlMatrix::MakeTranslation({1.0f, 2.0f});
  const DlMatrix scale_matrix = DlMatrix::MakeScale({0.5f, 0.5f, 1.0f});
  const DlMatrix combined_matrix = start_matrix * scale_matrix;
  const DlRect local_cull_rect = DlRect::MakeWH(5.0f, 5.0f);
  const DlRect device_cull_rect =
      local_cull_rect.TransformBounds(combined_matrix);
  const bool parent_has_platform_view = true;
  auto layer = std::make_shared<MockLayer>(path, paint);

  preroll_context()->state_stack.set_preroll_delegate(device_cull_rect,
                                                      start_matrix);
  auto mutator = preroll_context()->state_stack.save();
  mutator.transform(scale_matrix);
  preroll_context()->has_platform_view = parent_has_platform_view;
  layer->Preroll(preroll_context());
  EXPECT_EQ(preroll_context()->has_platform_view, false);
  EXPECT_EQ(layer->paint_bounds(), path.GetBounds());
  EXPECT_TRUE(layer->needs_painting(paint_context()));
  EXPECT_EQ(layer->parent_mutators(),
            std::vector{Mutator(ToSkMatrix(scale_matrix))});
  EXPECT_EQ(layer->parent_matrix(), combined_matrix);
  EXPECT_EQ(layer->parent_cull_rect(), local_cull_rect);
  EXPECT_EQ(layer->parent_has_platform_view(), parent_has_platform_view);

  layer->Paint(display_list_paint_context());

  DisplayListBuilder expected_builder;
  expected_builder.DrawPath(DlPath(path), paint);
  auto expected_dl = expected_builder.Build();

  EXPECT_TRUE(DisplayListsEQ_Verbose(display_list(), expected_dl));
}

TEST_F(MockLayerTest, FakePlatformView) {
  auto layer = std::make_shared<MockLayer>(DlPath(), DlPaint());
  layer->set_fake_has_platform_view(true);
  EXPECT_EQ(preroll_context()->has_platform_view, false);

  layer->Preroll(preroll_context());
  EXPECT_EQ(preroll_context()->has_platform_view, true);
}

TEST_F(MockLayerTest, SaveLayerOnLeafNodesCanvas) {
  auto layer = std::make_shared<MockLayer>(DlPath(), DlPaint());
  layer->set_fake_has_platform_view(true);
  EXPECT_EQ(preroll_context()->has_platform_view, false);

  layer->Preroll(preroll_context());
  EXPECT_EQ(preroll_context()->has_platform_view, true);
}

TEST_F(MockLayerTest, OpacityInheritance) {
  auto path1 = DlPath::MakeRectLTRB(10, 10, 30, 30);
  PrerollContext* context = preroll_context();

  auto mock1 = std::make_shared<MockLayer>(path1);
  mock1->Preroll(context);
  EXPECT_EQ(context->renderable_state_flags, 0);

  auto mock2 = MockLayer::MakeOpacityCompatible(path1);
  mock2->Preroll(context);
  EXPECT_EQ(context->renderable_state_flags,
            LayerStateStack::kCallerCanApplyOpacity);
}

TEST_F(MockLayerTest, FlagGetSet) {
  auto mock_layer = std::make_shared<MockLayer>(DlPath());

  EXPECT_EQ(mock_layer->parent_has_platform_view(), false);
  mock_layer->set_parent_has_platform_view(true);
  EXPECT_EQ(mock_layer->parent_has_platform_view(), true);

  EXPECT_EQ(mock_layer->parent_has_texture_layer(), false);
  mock_layer->set_parent_has_texture_layer(true);
  EXPECT_EQ(mock_layer->parent_has_texture_layer(), true);

  EXPECT_EQ(mock_layer->fake_has_platform_view(), false);
  mock_layer->set_fake_has_platform_view(true);
  EXPECT_EQ(mock_layer->fake_has_platform_view(), true);

  EXPECT_EQ(mock_layer->fake_reads_surface(), false);
  mock_layer->set_fake_reads_surface(true);
  EXPECT_EQ(mock_layer->fake_reads_surface(), true);

  EXPECT_EQ(mock_layer->fake_opacity_compatible(), false);
  mock_layer->set_fake_opacity_compatible(true);
  EXPECT_EQ(mock_layer->fake_opacity_compatible(), true);

  EXPECT_EQ(mock_layer->fake_has_texture_layer(), false);
  mock_layer->set_fake_has_texture_layer(true);
  EXPECT_EQ(mock_layer->fake_has_texture_layer(), true);
}

}  // namespace testing
}  // namespace flutter
