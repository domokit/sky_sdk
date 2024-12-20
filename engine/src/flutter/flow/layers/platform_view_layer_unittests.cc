// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/flow/layers/clip_rect_layer.h"
#include "flutter/flow/layers/platform_view_layer.h"
#include "flutter/flow/layers/transform_layer.h"

#include "flutter/flow/testing/layer_test.h"
#include "flutter/flow/testing/mock_embedder.h"
#include "flutter/flow/testing/mock_layer.h"
#include "flutter/fml/macros.h"

namespace flutter {
namespace testing {

using PlatformViewLayerTest = LayerTest;

using ClipOp = DlCanvas::ClipOp;

TEST_F(PlatformViewLayerTest, NullViewEmbedderDoesntPrerollCompositeOrPaint) {
  const DlPoint layer_offset = DlPoint();
  const DlSize layer_size = DlSize(8.0f, 8.0f);
  const int64_t view_id = 0;
  auto layer =
      std::make_shared<PlatformViewLayer>(layer_offset, layer_size, view_id);

  layer->Preroll(preroll_context());
  EXPECT_FALSE(preroll_context()->has_platform_view);
  EXPECT_EQ(layer->paint_bounds(),
            DlRect::MakeOriginSize(layer_offset, layer_size));
  EXPECT_TRUE(layer->needs_painting(paint_context()));
  EXPECT_FALSE(layer->subtree_has_platform_view());

  layer->Paint(display_list_paint_context());

  DisplayListBuilder expected_builder;
  auto expected_dl = expected_builder.Build();

  EXPECT_TRUE(DisplayListsEQ_Verbose(display_list(), expected_dl));
}

TEST_F(PlatformViewLayerTest, ClippedPlatformViewPrerollsAndPaintsNothing) {
  const DlPoint layer_offset = DlPoint();
  const DlSize layer_size = DlSize(8.0f, 8.0f);
  const DlRect child_clip = DlRect::MakeLTRB(20.0f, 20.0f, 40.0f, 40.0f);
  const DlRect parent_clip = DlRect::MakeLTRB(50.0f, 50.0f, 80.0f, 80.0f);
  const int64_t view_id = 0;
  auto layer =
      std::make_shared<PlatformViewLayer>(layer_offset, layer_size, view_id);
  auto child_clip_layer =
      std::make_shared<ClipRectLayer>(child_clip, Clip::kHardEdge);
  auto parent_clip_layer =
      std::make_shared<ClipRectLayer>(parent_clip, Clip::kHardEdge);
  parent_clip_layer->Add(child_clip_layer);
  child_clip_layer->Add(layer);

  auto embedder = MockViewEmbedder();
  preroll_context()->view_embedder = &embedder;

  parent_clip_layer->Preroll(preroll_context());
  EXPECT_TRUE(preroll_context()->has_platform_view);
  EXPECT_EQ(layer->paint_bounds(),
            DlRect::MakeOriginSize(layer_offset, layer_size));
  EXPECT_TRUE(layer->needs_painting(paint_context()));
  EXPECT_TRUE(child_clip_layer->needs_painting(paint_context()));
  EXPECT_TRUE(parent_clip_layer->needs_painting(paint_context()));
  EXPECT_TRUE(layer->subtree_has_platform_view());
  EXPECT_TRUE(child_clip_layer->subtree_has_platform_view());
  EXPECT_TRUE(parent_clip_layer->subtree_has_platform_view());

  parent_clip_layer->Paint(display_list_paint_context());

  DisplayListBuilder expected_builder;
  expected_builder.Save();
  expected_builder.ClipRect(parent_clip, ClipOp::kIntersect, false);

  // In reality the following save/clip/restore are elided due to reaching
  // a nop state (and the save is then unnecessary), but this is the order
  // of operations that the layers will do...
  expected_builder.Save();
  expected_builder.ClipRect(child_clip, ClipOp::kIntersect, false);
  expected_builder.Restore();
  // End of section that gets ignored during recording

  expected_builder.Restore();
  auto expected_dl = expected_builder.Build();

  EXPECT_TRUE(DisplayListsEQ_Verbose(display_list(), expected_dl));
}

TEST_F(PlatformViewLayerTest, OpacityInheritance) {
  const DlPoint layer_offset = DlPoint();
  const DlSize layer_size = DlSize(8.0f, 8.0f);
  const int64_t view_id = 0;
  auto layer =
      std::make_shared<PlatformViewLayer>(layer_offset, layer_size, view_id);

  PrerollContext* context = preroll_context();
  layer->Preroll(preroll_context());
  EXPECT_EQ(context->renderable_state_flags, 0);
}

TEST_F(PlatformViewLayerTest, StateTransfer) {
  const DlMatrix transform1 = DlMatrix::MakeTranslation({5.0f, 5.0f});
  const DlMatrix transform2 = DlMatrix::MakeTranslation({15.0f, 15.0f});
  const DlMatrix combined_transform = DlMatrix::MakeTranslation({20.0f, 20.0f});
  const DlPoint layer_offset = DlPoint(0.0f, 0.0f);
  const DlSize layer_size = DlSize(8.0f, 8.0f);
  const int64_t view_id = 0;
  const DlPath path1 = DlPath::MakeOvalLTRB(10, 10, 20, 20);
  const DlPath path2 = DlPath::MakeOvalLTRB(15, 15, 30, 30);

  // transform_layer1
  //   |- child1
  //   |- platform_layer
  //   |- transform_layer2
  //        |- child2
  auto transform_layer1 = std::make_shared<TransformLayer>(transform1);
  auto transform_layer2 = std::make_shared<TransformLayer>(transform2);
  auto platform_layer =
      std::make_shared<PlatformViewLayer>(layer_offset, layer_size, view_id);
  auto child1 = std::make_shared<MockLayer>(path1);
  child1->set_expected_paint_matrix(transform1);
  auto child2 = std::make_shared<MockLayer>(path2);
  child2->set_expected_paint_matrix(combined_transform);
  transform_layer1->Add(child1);
  transform_layer1->Add(platform_layer);
  transform_layer1->Add(transform_layer2);
  transform_layer2->Add(child2);

  auto embedder = MockViewEmbedder();
  DisplayListBuilder builder(DlRect::MakeWH(500, 500));
  embedder.AddCanvas(&builder);

  PrerollContext* preroll_ctx = preroll_context();
  preroll_ctx->view_embedder = &embedder;
  transform_layer1->Preroll(preroll_ctx);

  PaintContext& paint_ctx = paint_context();
  paint_ctx.view_embedder = &embedder;
  transform_layer1->Paint(paint_ctx);
}

}  // namespace testing
}  // namespace flutter
