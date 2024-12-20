// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <memory>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "flutter/display_list/display_list.h"
#include "flutter/display_list/dl_blend_mode.h"
#include "flutter/display_list/dl_builder.h"
#include "flutter/display_list/dl_paint.h"
#include "flutter/display_list/effects/dl_image_filters.h"
#include "flutter/display_list/geometry/dl_rtree.h"
#include "flutter/display_list/skia/dl_sk_dispatcher.h"
#include "flutter/display_list/testing/dl_test_snippets.h"
#include "flutter/display_list/utils/dl_receiver_utils.h"
#include "flutter/fml/logging.h"
#include "flutter/fml/math.h"
#include "flutter/impeller/typographer/backends/skia/text_frame_skia.h"
#include "flutter/testing/assertions_skia.h"
#include "flutter/testing/display_list_testing.h"
#include "flutter/testing/testing.h"

#include "third_party/skia/include/core/SkBBHFactory.h"
#include "third_party/skia/include/core/SkColorFilter.h"
#include "third_party/skia/include/core/SkPictureRecorder.h"
#include "third_party/skia/include/core/SkRSXform.h"
#include "third_party/skia/include/core/SkSurface.h"

namespace flutter {

DlOpReceiver& DisplayListBuilderTestingAccessor(DisplayListBuilder& builder) {
  return builder.asReceiver();
}

DlPaint DisplayListBuilderTestingAttributes(DisplayListBuilder& builder) {
  return builder.CurrentAttributes();
}

int DisplayListBuilderTestingLastOpIndex(DisplayListBuilder& builder) {
  return builder.LastOpIndex();
}

namespace testing {

static std::vector<testing::DisplayListInvocationGroup> allGroups =
    CreateAllGroups();

using ClipOp = DlCanvas::ClipOp;
using PointMode = DlCanvas::PointMode;

template <typename BaseT>
class DisplayListTestBase : public BaseT {
 public:
  DisplayListTestBase() = default;

  static DlOpReceiver& ToReceiver(DisplayListBuilder& builder) {
    return DisplayListBuilderTestingAccessor(builder);
  }

  static sk_sp<DisplayList> Build(DisplayListInvocation& invocation) {
    DisplayListBuilder builder;
    invocation.Invoke(ToReceiver(builder));
    return builder.Build();
  }

  static sk_sp<DisplayList> Build(size_t g_index, size_t v_index) {
    DisplayListBuilder builder;
    DlOpReceiver& receiver =
        DisplayListTestBase<::testing::Test>::ToReceiver(builder);
    uint32_t op_count = 0u;
    size_t byte_count = 0u;
    uint32_t depth = 0u;
    uint32_t render_op_depth_cost = 1u;
    for (size_t i = 0; i < allGroups.size(); i++) {
      DisplayListInvocationGroup& group = allGroups[i];
      size_t j = (i == g_index ? v_index : 0);
      if (j >= group.variants.size()) {
        continue;
      }
      DisplayListInvocation& invocation = group.variants[j];
      op_count += invocation.op_count();
      byte_count += invocation.raw_byte_count();
      depth += invocation.depth_accumulated(render_op_depth_cost);
      invocation.Invoke(receiver);
      render_op_depth_cost =
          invocation.adjust_render_op_depth_cost(render_op_depth_cost);
    }
    sk_sp<DisplayList> dl = builder.Build();
    std::string name;
    if (g_index >= allGroups.size()) {
      name = "Default";
    } else {
      name = allGroups[g_index].op_name;
      if (v_index >= allGroups[g_index].variants.size()) {
        name += " skipped";
      } else {
        name += " variant " + std::to_string(v_index + 1);
      }
    }
    EXPECT_EQ(dl->op_count(false), op_count) << name;
    EXPECT_EQ(dl->bytes(false), byte_count + sizeof(DisplayList)) << name;
    EXPECT_EQ(dl->total_depth(), depth) << name;
    return dl;
  }

  static void check_defaults(
      DisplayListBuilder& builder,
      const DlRect& cull_rect = DisplayListBuilder::kMaxCullRect) {
    DlPaint builder_paint = DisplayListBuilderTestingAttributes(builder);
    DlPaint defaults;

    EXPECT_EQ(builder_paint.isAntiAlias(), defaults.isAntiAlias());
    EXPECT_EQ(builder_paint.isInvertColors(), defaults.isInvertColors());
    EXPECT_EQ(builder_paint.getColor(), defaults.getColor());
    EXPECT_EQ(builder_paint.getBlendMode(), defaults.getBlendMode());
    EXPECT_EQ(builder_paint.getDrawStyle(), defaults.getDrawStyle());
    EXPECT_EQ(builder_paint.getStrokeWidth(), defaults.getStrokeWidth());
    EXPECT_EQ(builder_paint.getStrokeMiter(), defaults.getStrokeMiter());
    EXPECT_EQ(builder_paint.getStrokeCap(), defaults.getStrokeCap());
    EXPECT_EQ(builder_paint.getStrokeJoin(), defaults.getStrokeJoin());
    EXPECT_EQ(builder_paint.getColorSource(), defaults.getColorSource());
    EXPECT_EQ(builder_paint.getColorFilter(), defaults.getColorFilter());
    EXPECT_EQ(builder_paint.getImageFilter(), defaults.getImageFilter());
    EXPECT_EQ(builder_paint.getMaskFilter(), defaults.getMaskFilter());
    EXPECT_EQ(builder_paint, defaults);
    EXPECT_TRUE(builder_paint.isDefault());

    EXPECT_EQ(builder.GetTransform(), SkMatrix());
    EXPECT_EQ(builder.GetTransformFullPerspective(), SkM44());

    EXPECT_EQ(builder.GetLocalClipCoverage(), cull_rect);
    EXPECT_EQ(builder.GetDestinationClipCoverage(), cull_rect);

    EXPECT_EQ(builder.GetSaveCount(), 1);
  }

  typedef const std::function<void(DlCanvas&)> DlSetup;
  typedef const std::function<void(DlCanvas&, DlPaint&, SkRect& rect)>
      DlRenderer;

  static void verify_inverted_bounds(DlSetup& setup,
                                     DlRenderer& renderer,
                                     DlPaint paint,
                                     SkRect render_rect,
                                     SkRect expected_bounds,
                                     const std::string& desc) {
    DisplayListBuilder builder;
    setup(builder);
    renderer(builder, paint, render_rect);
    auto dl = builder.Build();
    EXPECT_EQ(dl->op_count(), 1u) << desc;
    EXPECT_EQ(dl->bounds(), expected_bounds) << desc;
  }

  static void check_inverted_bounds(DlRenderer& renderer,
                                    const std::string& desc) {
    SkRect rect = SkRect::MakeLTRB(0.0f, 0.0f, 10.0f, 10.0f);
    SkRect invertedLR = SkRect::MakeLTRB(rect.fRight, rect.fTop,  //
                                         rect.fLeft, rect.fBottom);
    SkRect invertedTB = SkRect::MakeLTRB(rect.fLeft, rect.fBottom,  //
                                         rect.fRight, rect.fTop);
    SkRect invertedLTRB = SkRect::MakeLTRB(rect.fRight, rect.fBottom,  //
                                           rect.fLeft, rect.fTop);
    auto empty_setup = [](DlCanvas&) {};

    ASSERT_TRUE(rect.fLeft < rect.fRight);
    ASSERT_TRUE(rect.fTop < rect.fBottom);
    ASSERT_FALSE(rect.isEmpty());
    ASSERT_TRUE(invertedLR.fLeft > invertedLR.fRight);
    ASSERT_TRUE(invertedLR.isEmpty());
    ASSERT_TRUE(invertedTB.fTop > invertedTB.fBottom);
    ASSERT_TRUE(invertedTB.isEmpty());
    ASSERT_TRUE(invertedLTRB.fLeft > invertedLTRB.fRight);
    ASSERT_TRUE(invertedLTRB.fTop > invertedLTRB.fBottom);
    ASSERT_TRUE(invertedLTRB.isEmpty());

    DlPaint ref_paint = DlPaint();
    SkRect ref_bounds = rect;
    verify_inverted_bounds(empty_setup, renderer, ref_paint, invertedLR,
                           ref_bounds, desc + " LR swapped");
    verify_inverted_bounds(empty_setup, renderer, ref_paint, invertedTB,
                           ref_bounds, desc + " TB swapped");
    verify_inverted_bounds(empty_setup, renderer, ref_paint, invertedLTRB,
                           ref_bounds, desc + " LR&TB swapped");

    // Round joins are used because miter joins greatly pad the bounds,
    // but only on paths. So we use round joins for consistency there.
    // We aren't fully testing all stroke-related bounds computations here,
    // those are more fully tested in the render tests. We are simply
    // checking that they are applied to the ordered bounds.
    DlPaint stroke_paint = DlPaint()                                 //
                               .setDrawStyle(DlDrawStyle::kStroke)   //
                               .setStrokeJoin(DlStrokeJoin::kRound)  //
                               .setStrokeWidth(2.0f);
    SkRect stroke_bounds = rect.makeOutset(1.0f, 1.0f);
    verify_inverted_bounds(empty_setup, renderer, stroke_paint, invertedLR,
                           stroke_bounds, desc + " LR swapped, sw 2");
    verify_inverted_bounds(empty_setup, renderer, stroke_paint, invertedTB,
                           stroke_bounds, desc + " TB swapped, sw 2");
    verify_inverted_bounds(empty_setup, renderer, stroke_paint, invertedLTRB,
                           stroke_bounds, desc + " LR&TB swapped, sw 2");

    DlBlurMaskFilter mask_filter(DlBlurStyle::kNormal, 2.0f);
    DlPaint maskblur_paint = DlPaint()  //
                                 .setMaskFilter(&mask_filter);
    SkRect maskblur_bounds = rect.makeOutset(6.0f, 6.0f);
    verify_inverted_bounds(empty_setup, renderer, maskblur_paint, invertedLR,
                           maskblur_bounds, desc + " LR swapped, mask 2");
    verify_inverted_bounds(empty_setup, renderer, maskblur_paint, invertedTB,
                           maskblur_bounds, desc + " TB swapped, mask 2");
    verify_inverted_bounds(empty_setup, renderer, maskblur_paint, invertedLTRB,
                           maskblur_bounds, desc + " LR&TB swapped, mask 2");

    DlErodeImageFilter erode_filter(2.0f, 2.0f);
    DlPaint erode_paint = DlPaint()  //
                              .setImageFilter(&erode_filter);
    SkRect erode_bounds = rect.makeInset(2.0f, 2.0f);
    verify_inverted_bounds(empty_setup, renderer, erode_paint, invertedLR,
                           erode_bounds, desc + " LR swapped, erode 2");
    verify_inverted_bounds(empty_setup, renderer, erode_paint, invertedTB,
                           erode_bounds, desc + " TB swapped, erode 2");
    verify_inverted_bounds(empty_setup, renderer, erode_paint, invertedLTRB,
                           erode_bounds, desc + " LR&TB swapped, erode 2");
  }

 private:
  FML_DISALLOW_COPY_AND_ASSIGN(DisplayListTestBase);
};
using DisplayListTest = DisplayListTestBase<::testing::Test>;

TEST_F(DisplayListTest, Defaults) {
  DisplayListBuilder builder;
  check_defaults(builder);
}

TEST_F(DisplayListTest, EmptyBuild) {
  DisplayListBuilder builder;
  auto dl = builder.Build();
  EXPECT_EQ(dl->op_count(), 0u);
  EXPECT_EQ(dl->bytes(), sizeof(DisplayList));
  EXPECT_EQ(dl->total_depth(), 0u);
}

TEST_F(DisplayListTest, EmptyRebuild) {
  DisplayListBuilder builder;
  auto dl1 = builder.Build();
  auto dl2 = builder.Build();
  auto dl3 = builder.Build();
  ASSERT_TRUE(dl1->Equals(dl2));
  ASSERT_TRUE(dl2->Equals(dl3));
}

TEST_F(DisplayListTest, NopReusedBuildIsReallyEmpty) {
  DisplayListBuilder builder;
  builder.DrawRect(DlRect::MakeLTRB(0.0f, 0.0f, 10.0f, 10.0f), DlPaint());

  {
    auto dl1 = builder.Build();
    EXPECT_EQ(dl1->op_count(), 1u);
    EXPECT_GT(dl1->bytes(), sizeof(DisplayList));
    EXPECT_EQ(dl1->GetBounds(), DlRect::MakeLTRB(0.0f, 0.0f, 10.0f, 10.0f));
  }

  {
    auto dl2 = builder.Build();
    EXPECT_EQ(dl2->op_count(), 0u);
    EXPECT_EQ(dl2->bytes(), sizeof(DisplayList));
    EXPECT_EQ(dl2->GetBounds(), DlRect());
  }
}

TEST_F(DisplayListTest, GeneralReceiverInitialValues) {
  DisplayListGeneralReceiver receiver;

  EXPECT_EQ(receiver.GetOpsReceived(), 0u);

  auto max_type = static_cast<int>(DisplayListOpType::kMaxOp);
  for (int i = 0; i <= max_type; i++) {
    DisplayListOpType type = static_cast<DisplayListOpType>(i);
    EXPECT_EQ(receiver.GetOpsReceived(type), 0u) << type;
  }

  auto max_category = static_cast<int>(DisplayListOpCategory::kMaxCategory);
  for (int i = 0; i <= max_category; i++) {
    DisplayListOpCategory category = static_cast<DisplayListOpCategory>(i);
    EXPECT_EQ(receiver.GetOpsReceived(category), 0u) << category;
  }
}

TEST_F(DisplayListTest, Iteration) {
  DisplayListBuilder builder;
  builder.DrawRect(SkRect{10, 10, 20, 20}, DlPaint());
  auto dl = builder.Build();
  for (DlIndex i : *dl) {
    EXPECT_EQ(dl->GetOpType(i), DisplayListOpType::kDrawRect)  //
        << "at " << i;
    EXPECT_EQ(dl->GetOpCategory(i), DisplayListOpCategory::kRendering)
        << "at " << i;
  }
}

TEST_F(DisplayListTest, InvalidIndices) {
  DisplayListBuilder builder;
  builder.DrawRect(kTestSkBounds, DlPaint());
  auto dl = builder.Build();
  DisplayListGeneralReceiver receiver;

  EXPECT_FALSE(dl->Dispatch(receiver, -1));
  EXPECT_FALSE(dl->Dispatch(receiver, dl->GetRecordCount()));
  EXPECT_EQ(dl->GetOpType(-1), DisplayListOpType::kInvalidOp);
  EXPECT_EQ(dl->GetOpType(dl->GetRecordCount()), DisplayListOpType::kInvalidOp);
  EXPECT_EQ(dl->GetOpCategory(-1), DisplayListOpCategory::kInvalidCategory);
  EXPECT_EQ(dl->GetOpCategory(dl->GetRecordCount()),
            DisplayListOpCategory::kInvalidCategory);
  EXPECT_EQ(dl->GetOpCategory(-1), DisplayListOpCategory::kInvalidCategory);
  EXPECT_EQ(dl->GetOpCategory(DisplayListOpType::kInvalidOp),
            DisplayListOpCategory::kInvalidCategory);
  EXPECT_EQ(dl->GetOpCategory(DisplayListOpType::kMaxOp),
            DisplayListOpCategory::kInvalidCategory);
  EXPECT_EQ(receiver.GetOpsReceived(), 0u);
}

TEST_F(DisplayListTest, ValidIndices) {
  DisplayListBuilder builder;
  builder.DrawRect(kTestSkBounds, DlPaint());
  auto dl = builder.Build();
  DisplayListGeneralReceiver receiver;

  EXPECT_EQ(dl->GetRecordCount(), 1u);
  EXPECT_TRUE(dl->Dispatch(receiver, 0u));
  EXPECT_EQ(dl->GetOpType(0u), DisplayListOpType::kDrawRect);
  EXPECT_EQ(dl->GetOpCategory(0u), DisplayListOpCategory::kRendering);
  EXPECT_EQ(receiver.GetOpsReceived(), 1u);
}

TEST_F(DisplayListTest, BuilderCanBeReused) {
  DisplayListBuilder builder(kTestSkBounds);
  builder.DrawRect(kTestSkBounds, DlPaint());
  auto dl = builder.Build();
  builder.DrawRect(kTestSkBounds, DlPaint());
  auto dl2 = builder.Build();
  ASSERT_TRUE(dl->Equals(dl2));
}

TEST_F(DisplayListTest, SaveRestoreRestoresTransform) {
  DlRect cull_rect = DlRect::MakeLTRB(-10.0f, -10.0f, 500.0f, 500.0f);
  DisplayListBuilder builder(cull_rect);

  builder.Save();
  builder.Translate(10.0f, 10.0f);
  builder.Restore();
  check_defaults(builder, cull_rect);

  builder.Save();
  builder.Scale(10.0f, 10.0f);
  builder.Restore();
  check_defaults(builder, cull_rect);

  builder.Save();
  builder.Skew(0.1f, 0.1f);
  builder.Restore();
  check_defaults(builder, cull_rect);

  builder.Save();
  builder.Rotate(45.0f);
  builder.Restore();
  check_defaults(builder, cull_rect);

  builder.Save();
  builder.Transform(SkMatrix::Scale(10.0f, 10.0f));
  builder.Restore();
  check_defaults(builder, cull_rect);

  builder.Save();
  builder.Transform2DAffine(1.0f, 0.0f, 12.0f,  //
                            0.0f, 1.0f, 35.0f);
  builder.Restore();
  check_defaults(builder, cull_rect);

  builder.Save();
  builder.Transform(SkM44(SkMatrix::Scale(10.0f, 10.0f)));
  builder.Restore();
  check_defaults(builder, cull_rect);

  builder.Save();
  builder.TransformFullPerspective(1.0f, 0.0f, 0.0f, 12.0f,  //
                                   0.0f, 1.0f, 0.0f, 35.0f,  //
                                   0.0f, 0.0f, 1.0f, 5.0f,   //
                                   0.0f, 0.0f, 0.0f, 1.0f);
  builder.Restore();
  check_defaults(builder, cull_rect);
}

TEST_F(DisplayListTest, BuildRestoresTransform) {
  DlRect cull_rect = DlRect::MakeLTRB(-10.0f, -10.0f, 500.0f, 500.0f);
  DisplayListBuilder builder(cull_rect);

  builder.Translate(10.0f, 10.0f);
  builder.Build();
  check_defaults(builder, cull_rect);

  builder.Scale(10.0f, 10.0f);
  builder.Build();
  check_defaults(builder, cull_rect);

  builder.Skew(0.1f, 0.1f);
  builder.Build();
  check_defaults(builder, cull_rect);

  builder.Rotate(45.0f);
  builder.Build();
  check_defaults(builder, cull_rect);

  builder.Transform(SkMatrix::Scale(10.0f, 10.0f));
  builder.Build();
  check_defaults(builder, cull_rect);

  builder.Transform2DAffine(1.0f, 0.0f, 12.0f,  //
                            0.0f, 1.0f, 35.0f);
  builder.Build();
  check_defaults(builder, cull_rect);

  builder.Transform(SkM44(SkMatrix::Scale(10.0f, 10.0f)));
  builder.Build();
  check_defaults(builder, cull_rect);

  builder.TransformFullPerspective(1.0f, 0.0f, 0.0f, 12.0f,  //
                                   0.0f, 1.0f, 0.0f, 35.0f,  //
                                   0.0f, 0.0f, 1.0f, 5.0f,   //
                                   0.0f, 0.0f, 0.0f, 1.0f);
  builder.Build();
  check_defaults(builder, cull_rect);
}

TEST_F(DisplayListTest, SaveRestoreRestoresClip) {
  DlRect cull_rect = DlRect::MakeLTRB(-10.0f, -10.0f, 500.0f, 500.0f);
  DisplayListBuilder builder(cull_rect);

  builder.Save();
  builder.ClipRect(SkRect{0.0f, 0.0f, 10.0f, 10.0f});
  builder.Restore();
  check_defaults(builder, cull_rect);

  builder.Save();
  builder.ClipRRect(SkRRect::MakeRectXY({0.0f, 0.0f, 5.0f, 5.0f}, 2.0f, 2.0f));
  builder.Restore();
  check_defaults(builder, cull_rect);

  builder.Save();
  builder.ClipPath(SkPath().addOval({0.0f, 0.0f, 10.0f, 10.0f}));
  builder.Restore();
  check_defaults(builder, cull_rect);
}

TEST_F(DisplayListTest, BuildRestoresClip) {
  DlRect cull_rect = DlRect::MakeLTRB(-10.0f, -10.0f, 500.0f, 500.0f);
  DisplayListBuilder builder(cull_rect);

  builder.ClipRect(SkRect{0.0f, 0.0f, 10.0f, 10.0f});
  builder.Build();
  check_defaults(builder, cull_rect);

  builder.ClipRRect(SkRRect::MakeRectXY({0.0f, 0.0f, 5.0f, 5.0f}, 2.0f, 2.0f));
  builder.Build();
  check_defaults(builder, cull_rect);

  builder.ClipPath(SkPath().addOval({0.0f, 0.0f, 10.0f, 10.0f}));
  builder.Build();
  check_defaults(builder, cull_rect);
}

TEST_F(DisplayListTest, BuildRestoresAttributes) {
  DlRect cull_rect = DlRect::MakeLTRB(-10.0f, -10.0f, 500.0f, 500.0f);
  DisplayListBuilder builder(cull_rect);
  DlOpReceiver& receiver = ToReceiver(builder);

  receiver.setAntiAlias(true);
  builder.Build();
  check_defaults(builder, cull_rect);

  receiver.setInvertColors(true);
  builder.Build();
  check_defaults(builder, cull_rect);

  receiver.setColor(DlColor::kRed());
  builder.Build();
  check_defaults(builder, cull_rect);

  receiver.setBlendMode(DlBlendMode::kColorBurn);
  builder.Build();
  check_defaults(builder, cull_rect);

  receiver.setDrawStyle(DlDrawStyle::kStrokeAndFill);
  builder.Build();
  check_defaults(builder, cull_rect);

  receiver.setStrokeWidth(300.0f);
  builder.Build();
  check_defaults(builder, cull_rect);

  receiver.setStrokeMiter(300.0f);
  builder.Build();
  check_defaults(builder, cull_rect);

  receiver.setStrokeCap(DlStrokeCap::kRound);
  builder.Build();
  check_defaults(builder, cull_rect);

  receiver.setStrokeJoin(DlStrokeJoin::kRound);
  builder.Build();
  check_defaults(builder, cull_rect);

  receiver.setColorSource(kTestSource1.get());
  builder.Build();
  check_defaults(builder, cull_rect);

  receiver.setColorFilter(kTestMatrixColorFilter1.get());
  builder.Build();
  check_defaults(builder, cull_rect);

  receiver.setImageFilter(&kTestBlurImageFilter1);
  builder.Build();
  check_defaults(builder, cull_rect);

  receiver.setMaskFilter(&kTestMaskFilter1);
  builder.Build();
  check_defaults(builder, cull_rect);
}

TEST_F(DisplayListTest, BuilderBoundsTransformComparedToSkia) {
  const SkRect frame_rect = SkRect::MakeLTRB(10, 10, 100, 100);
  DisplayListBuilder builder(frame_rect);
  SkPictureRecorder recorder;
  SkCanvas* canvas = recorder.beginRecording(frame_rect);
  ASSERT_EQ(builder.GetDestinationClipBounds(),
            SkRect::Make(canvas->getDeviceClipBounds()));
  ASSERT_EQ(builder.GetLocalClipBounds().makeOutset(1, 1),
            canvas->getLocalClipBounds());
  ASSERT_EQ(builder.GetTransform(), canvas->getTotalMatrix());
}

TEST_F(DisplayListTest, BuilderInitialClipBounds) {
  SkRect cull_rect = SkRect::MakeWH(100, 100);
  SkRect clip_bounds = SkRect::MakeWH(100, 100);
  DisplayListBuilder builder(cull_rect);
  ASSERT_EQ(builder.GetDestinationClipBounds(), clip_bounds);
}

TEST_F(DisplayListTest, BuilderInitialClipBoundsNaN) {
  SkRect cull_rect = SkRect::MakeWH(SK_ScalarNaN, SK_ScalarNaN);
  SkRect clip_bounds = SkRect::MakeEmpty();
  DisplayListBuilder builder(cull_rect);
  ASSERT_EQ(builder.GetDestinationClipBounds(), clip_bounds);
}

TEST_F(DisplayListTest, BuilderClipBoundsAfterClipRect) {
  SkRect cull_rect = SkRect::MakeWH(100, 100);
  SkRect clip_rect = SkRect::MakeLTRB(10, 10, 20, 20);
  SkRect clip_bounds = SkRect::MakeLTRB(10, 10, 20, 20);
  DisplayListBuilder builder(cull_rect);
  builder.ClipRect(clip_rect, ClipOp::kIntersect, false);
  ASSERT_EQ(builder.GetDestinationClipBounds(), clip_bounds);
}

TEST_F(DisplayListTest, BuilderClipBoundsAfterClipRRect) {
  SkRect cull_rect = SkRect::MakeWH(100, 100);
  SkRect clip_rect = SkRect::MakeLTRB(10, 10, 20, 20);
  SkRRect clip_rrect = SkRRect::MakeRectXY(clip_rect, 2, 2);
  SkRect clip_bounds = SkRect::MakeLTRB(10, 10, 20, 20);
  DisplayListBuilder builder(cull_rect);
  builder.ClipRRect(clip_rrect, ClipOp::kIntersect, false);
  ASSERT_EQ(builder.GetDestinationClipBounds(), clip_bounds);
}

TEST_F(DisplayListTest, BuilderClipBoundsAfterClipPath) {
  SkRect cull_rect = SkRect::MakeWH(100, 100);
  SkPath clip_path = SkPath().addRect(10, 10, 15, 15).addRect(15, 15, 20, 20);
  SkRect clip_bounds = SkRect::MakeLTRB(10, 10, 20, 20);
  DisplayListBuilder builder(cull_rect);
  builder.ClipPath(clip_path, ClipOp::kIntersect, false);
  ASSERT_EQ(builder.GetDestinationClipBounds(), clip_bounds);
}

TEST_F(DisplayListTest, BuilderInitialClipBoundsNonZero) {
  SkRect cull_rect = SkRect::MakeLTRB(10, 10, 100, 100);
  SkRect clip_bounds = SkRect::MakeLTRB(10, 10, 100, 100);
  DisplayListBuilder builder(cull_rect);
  ASSERT_EQ(builder.GetDestinationClipBounds(), clip_bounds);
}

TEST_F(DisplayListTest, UnclippedSaveLayerContentAccountsForFilter) {
  SkRect cull_rect = SkRect::MakeLTRB(0.0f, 0.0f, 300.0f, 300.0f);
  SkRect clip_rect = SkRect::MakeLTRB(100.0f, 100.0f, 200.0f, 200.0f);
  SkRect draw_rect = SkRect::MakeLTRB(50.0f, 140.0f, 101.0f, 160.0f);
  auto filter = DlImageFilter::MakeBlur(10.0f, 10.0f, DlTileMode::kDecal);
  DlPaint layer_paint = DlPaint().setImageFilter(filter);

  ASSERT_TRUE(clip_rect.intersects(draw_rect));
  ASSERT_TRUE(cull_rect.contains(clip_rect));
  ASSERT_TRUE(cull_rect.contains(draw_rect));

  DisplayListBuilder builder;
  builder.Save();
  {
    builder.ClipRect(clip_rect, ClipOp::kIntersect, false);
    builder.SaveLayer(&cull_rect, &layer_paint);
    {  //
      builder.DrawRect(draw_rect, DlPaint());
    }
    builder.Restore();
  }
  builder.Restore();
  auto display_list = builder.Build();

  EXPECT_EQ(display_list->op_count(), 6u);
  EXPECT_EQ(display_list->total_depth(), 2u);

  SkRect result_rect = draw_rect.makeOutset(30.0f, 30.0f);
  ASSERT_TRUE(result_rect.intersect(clip_rect));
  ASSERT_EQ(result_rect, SkRect::MakeLTRB(100.0f, 110.0f, 131.0f, 190.0f));
  EXPECT_EQ(display_list->bounds(), result_rect);
}

TEST_F(DisplayListTest, ClippedSaveLayerContentAccountsForFilter) {
  SkRect cull_rect = SkRect::MakeLTRB(0.0f, 0.0f, 300.0f, 300.0f);
  SkRect clip_rect = SkRect::MakeLTRB(100.0f, 100.0f, 200.0f, 200.0f);
  SkRect draw_rect = SkRect::MakeLTRB(50.0f, 140.0f, 99.0f, 160.0f);
  auto filter = DlImageFilter::MakeBlur(10.0f, 10.0f, DlTileMode::kDecal);
  DlPaint layer_paint = DlPaint().setImageFilter(filter);

  ASSERT_FALSE(clip_rect.intersects(draw_rect));
  ASSERT_TRUE(cull_rect.contains(clip_rect));
  ASSERT_TRUE(cull_rect.contains(draw_rect));

  DisplayListBuilder builder;
  builder.Save();
  {
    builder.ClipRect(clip_rect, ClipOp::kIntersect, false);
    builder.SaveLayer(&cull_rect, &layer_paint);
    {  //
      builder.DrawRect(draw_rect, DlPaint());
    }
    builder.Restore();
  }
  builder.Restore();
  auto display_list = builder.Build();

  EXPECT_EQ(display_list->op_count(), 6u);
  EXPECT_EQ(display_list->total_depth(), 2u);

  SkRect result_rect = draw_rect.makeOutset(30.0f, 30.0f);
  ASSERT_TRUE(result_rect.intersect(clip_rect));
  ASSERT_EQ(result_rect, SkRect::MakeLTRB(100.0f, 110.0f, 129.0f, 190.0f));
  EXPECT_EQ(display_list->bounds(), result_rect);
}

TEST_F(DisplayListTest, OOBSaveLayerContentCulledWithBlurFilter) {
  DlRect cull_rect = DlRect::MakeLTRB(100.0f, 100.0f, 200.0f, 200.0f);
  DlRect draw_rect = DlRect::MakeLTRB(25.0f, 25.0f, 99.0f, 75.0f);
  auto filter = DlImageFilter::MakeBlur(10.0f, 10.0f, DlTileMode::kDecal);
  DlPaint layer_paint = DlPaint().setImageFilter(filter);

  // We want a draw rect that is outside the layer bounds even though its
  // filtered output might be inside. The drawn rect should be culled by
  // the expectations of the layer bounds even though it is close enough
  // to be visible due to filtering.
  ASSERT_FALSE(cull_rect.IntersectsWithRect(draw_rect));
  DlRect mapped_rect;
  ASSERT_TRUE(filter->map_local_bounds(draw_rect, mapped_rect));
  ASSERT_TRUE(mapped_rect.IntersectsWithRect(cull_rect));

  DisplayListBuilder builder;
  builder.SaveLayer(cull_rect, &layer_paint);
  {  //
    builder.DrawRect(draw_rect, DlPaint());
  }
  builder.Restore();
  auto display_list = builder.Build();

  EXPECT_EQ(display_list->op_count(), 2u);
  EXPECT_EQ(display_list->total_depth(), 1u);

  EXPECT_TRUE(display_list->bounds().isEmpty()) << display_list->bounds();
}

TEST_F(DisplayListTest, OOBSaveLayerContentCulledWithMatrixFilter) {
  DlRect cull_rect = DlRect::MakeLTRB(100.0f, 100.0f, 200.0f, 200.0f);
  DlRect draw_rect = DlRect::MakeLTRB(25.0f, 125.0f, 75.0f, 175.0f);
  auto filter = DlImageFilter::MakeMatrix(
      DlMatrix::MakeTranslation({100.0f, 0.0f}), DlImageSampling::kLinear);
  DlPaint layer_paint = DlPaint().setImageFilter(filter);

  // We want a draw rect that is outside the layer bounds even though its
  // filtered output might be inside. The drawn rect should be culled by
  // the expectations of the layer bounds even though it is close enough
  // to be visible due to filtering.
  ASSERT_FALSE(cull_rect.IntersectsWithRect(draw_rect));
  DlRect mapped_rect;
  ASSERT_TRUE(filter->map_local_bounds(draw_rect, mapped_rect));
  ASSERT_TRUE(mapped_rect.IntersectsWithRect(cull_rect));

  DisplayListBuilder builder;
  builder.SaveLayer(cull_rect, &layer_paint);
  {  //
    builder.DrawRect(draw_rect, DlPaint());
  }
  builder.Restore();
  auto display_list = builder.Build();

  EXPECT_EQ(display_list->op_count(), 2u);
  EXPECT_EQ(display_list->total_depth(), 1u);

  EXPECT_TRUE(display_list->bounds().isEmpty()) << display_list->bounds();
}

TEST_F(DisplayListTest, SingleOpSizes) {
  for (auto& group : allGroups) {
    for (size_t i = 0; i < group.variants.size(); i++) {
      auto& invocation = group.variants[i];
      sk_sp<DisplayList> dl = Build(invocation);
      auto desc = group.op_name + "(variant " + std::to_string(i + 1) + ")";
      EXPECT_EQ(dl->op_count(false), invocation.op_count()) << desc;
      EXPECT_EQ(dl->bytes(false), invocation.byte_count()) << desc;
      EXPECT_EQ(dl->total_depth(), invocation.depth_accumulated()) << desc;
    }
  }
}

TEST_F(DisplayListTest, SingleOpDisplayListsNotEqualEmpty) {
  sk_sp<DisplayList> empty = DisplayListBuilder().Build();
  for (auto& group : allGroups) {
    for (size_t i = 0; i < group.variants.size(); i++) {
      sk_sp<DisplayList> dl = Build(group.variants[i]);
      auto desc =
          group.op_name + "(variant " + std::to_string(i + 1) + " != empty)";
      if (group.variants[i].is_empty()) {
        ASSERT_TRUE(DisplayListsEQ_Verbose(dl, empty));
        ASSERT_TRUE(empty->Equals(*dl)) << desc;
      } else {
        ASSERT_TRUE(DisplayListsNE_Verbose(dl, empty));
        ASSERT_FALSE(empty->Equals(*dl)) << desc;
      }
    }
  }
}

TEST_F(DisplayListTest, SingleOpDisplayListsRecapturedAreEqual) {
  for (auto& group : allGroups) {
    for (size_t i = 0; i < group.variants.size(); i++) {
      sk_sp<DisplayList> dl = Build(group.variants[i]);
      // Verify recapturing the replay of the display list is Equals()
      // when dispatching directly from the DL to another builder
      DisplayListBuilder copy_builder;
      DlOpReceiver& r = ToReceiver(copy_builder);
      dl->Dispatch(r);
      sk_sp<DisplayList> copy = copy_builder.Build();
      auto desc =
          group.op_name + "(variant " + std::to_string(i + 1) + " == copy)";
      ASSERT_TRUE(DisplayListsEQ_Verbose(dl, copy));
      ASSERT_EQ(copy->op_count(false), dl->op_count(false)) << desc;
      ASSERT_EQ(copy->GetRecordCount(), dl->GetRecordCount());
      ASSERT_EQ(copy->bytes(false), dl->bytes(false)) << desc;
      ASSERT_EQ(copy->op_count(true), dl->op_count(true)) << desc;
      ASSERT_EQ(copy->bytes(true), dl->bytes(true)) << desc;
      ASSERT_EQ(copy->total_depth(), dl->total_depth()) << desc;
      ASSERT_EQ(copy->bounds(), dl->bounds()) << desc;
      ASSERT_TRUE(copy->Equals(*dl)) << desc;
      ASSERT_TRUE(dl->Equals(*copy)) << desc;
    }
  }
}

TEST_F(DisplayListTest, SingleOpDisplayListsRecapturedByIndexAreEqual) {
  for (auto& group : allGroups) {
    for (size_t i = 0; i < group.variants.size(); i++) {
      sk_sp<DisplayList> dl = Build(group.variants[i]);
      // Verify recapturing the replay of the display list is Equals()
      // when dispatching directly from the DL to another builder
      DisplayListBuilder copy_builder;
      DlOpReceiver& r = ToReceiver(copy_builder);
      for (DlIndex i = 0; i < dl->GetRecordCount(); i++) {
        EXPECT_NE(dl->GetOpType(i), DisplayListOpType::kInvalidOp);
        EXPECT_NE(dl->GetOpCategory(i),
                  DisplayListOpCategory::kInvalidCategory);
        EXPECT_TRUE(dl->Dispatch(r, i));
      }
      sk_sp<DisplayList> copy = copy_builder.Build();
      auto desc =
          group.op_name + "(variant " + std::to_string(i + 1) + " == copy)";
      ASSERT_TRUE(DisplayListsEQ_Verbose(dl, copy));
      ASSERT_EQ(copy->op_count(false), dl->op_count(false)) << desc;
      ASSERT_EQ(copy->GetRecordCount(), dl->GetRecordCount());
      ASSERT_EQ(copy->bytes(false), dl->bytes(false)) << desc;
      ASSERT_EQ(copy->op_count(true), dl->op_count(true)) << desc;
      ASSERT_EQ(copy->bytes(true), dl->bytes(true)) << desc;
      ASSERT_EQ(copy->total_depth(), dl->total_depth()) << desc;
      ASSERT_EQ(copy->bounds(), dl->bounds()) << desc;
      ASSERT_TRUE(copy->Equals(*dl)) << desc;
      ASSERT_TRUE(dl->Equals(*copy)) << desc;
    }
  }
}

TEST_F(DisplayListTest, SingleOpDisplayListsCompareToEachOther) {
  for (auto& group : allGroups) {
    std::vector<sk_sp<DisplayList>> lists_a;
    std::vector<sk_sp<DisplayList>> lists_b;
    for (size_t i = 0; i < group.variants.size(); i++) {
      lists_a.push_back(Build(group.variants[i]));
      lists_b.push_back(Build(group.variants[i]));
    }

    for (size_t i = 0; i < lists_a.size(); i++) {
      sk_sp<DisplayList> listA = lists_a[i];
      for (size_t j = 0; j < lists_b.size(); j++) {
        sk_sp<DisplayList> listB = lists_b[j];
        auto desc = group.op_name + "(variant " + std::to_string(i + 1) +
                    " ==? variant " + std::to_string(j + 1) + ")";
        if (i == j ||
            (group.variants[i].is_empty() && group.variants[j].is_empty())) {
          // They are the same variant, or both variants are NOPs
          ASSERT_EQ(listA->op_count(false), listB->op_count(false)) << desc;
          ASSERT_EQ(listA->bytes(false), listB->bytes(false)) << desc;
          ASSERT_EQ(listA->op_count(true), listB->op_count(true)) << desc;
          ASSERT_EQ(listA->bytes(true), listB->bytes(true)) << desc;
          EXPECT_EQ(listA->total_depth(), listB->total_depth()) << desc;
          ASSERT_EQ(listA->bounds(), listB->bounds()) << desc;
          ASSERT_TRUE(listA->Equals(*listB)) << desc;
          ASSERT_TRUE(listB->Equals(*listA)) << desc;
        } else {
          // No assertion on op/byte counts or bounds
          // they may or may not be equal between variants
          ASSERT_FALSE(listA->Equals(*listB)) << desc;
          ASSERT_FALSE(listB->Equals(*listA)) << desc;
        }
      }
    }
  }
}

TEST_F(DisplayListTest, SingleOpDisplayListsAreEqualWithOrWithoutRtree) {
  for (auto& group : allGroups) {
    for (size_t i = 0; i < group.variants.size(); i++) {
      DisplayListBuilder builder1(/*prepare_rtree=*/false);
      DisplayListBuilder builder2(/*prepare_rtree=*/true);
      group.variants[i].Invoke(ToReceiver(builder1));
      group.variants[i].Invoke(ToReceiver(builder2));
      sk_sp<DisplayList> dl1 = builder1.Build();
      sk_sp<DisplayList> dl2 = builder2.Build();

      auto desc = group.op_name + "(variant " + std::to_string(i + 1) + " )";
      ASSERT_EQ(dl1->op_count(false), dl2->op_count(false)) << desc;
      ASSERT_EQ(dl1->bytes(false), dl2->bytes(false)) << desc;
      ASSERT_EQ(dl1->op_count(true), dl2->op_count(true)) << desc;
      ASSERT_EQ(dl1->bytes(true), dl2->bytes(true)) << desc;
      EXPECT_EQ(dl1->total_depth(), dl2->total_depth()) << desc;
      ASSERT_EQ(dl1->bounds(), dl2->bounds()) << desc;
      ASSERT_EQ(dl1->total_depth(), dl2->total_depth()) << desc;
      ASSERT_TRUE(DisplayListsEQ_Verbose(dl1, dl2)) << desc;
      ASSERT_TRUE(DisplayListsEQ_Verbose(dl2, dl2)) << desc;
      ASSERT_EQ(dl1->rtree().get(), nullptr) << desc;
      ASSERT_NE(dl2->rtree().get(), nullptr) << desc;
    }
  }
}

TEST_F(DisplayListTest, FullRotationsAreNop) {
  DisplayListBuilder builder;
  builder.Rotate(0);
  builder.Rotate(360);
  builder.Rotate(720);
  builder.Rotate(1080);
  builder.Rotate(1440);
  sk_sp<DisplayList> dl = builder.Build();
  ASSERT_EQ(dl->bytes(false), sizeof(DisplayList));
  ASSERT_EQ(dl->bytes(true), sizeof(DisplayList));
  ASSERT_EQ(dl->op_count(false), 0u);
  ASSERT_EQ(dl->op_count(true), 0u);
  EXPECT_EQ(dl->total_depth(), 0u);
}

TEST_F(DisplayListTest, AllBlendModeNops) {
  DisplayListBuilder builder;
  DlOpReceiver& receiver = ToReceiver(builder);
  receiver.setBlendMode(DlBlendMode::kSrcOver);
  sk_sp<DisplayList> dl = builder.Build();
  ASSERT_EQ(dl->bytes(false), sizeof(DisplayList));
  ASSERT_EQ(dl->bytes(true), sizeof(DisplayList));
  ASSERT_EQ(dl->op_count(false), 0u);
  ASSERT_EQ(dl->op_count(true), 0u);
  EXPECT_EQ(dl->total_depth(), 0u);
}

TEST_F(DisplayListTest, DisplayListsWithVaryingOpComparisons) {
  sk_sp<DisplayList> default_dl = Build(allGroups.size(), 0);
  ASSERT_TRUE(default_dl->Equals(*default_dl)) << "Default == itself";
  for (size_t gi = 0; gi < allGroups.size(); gi++) {
    DisplayListInvocationGroup& group = allGroups[gi];
    sk_sp<DisplayList> missing_dl = Build(gi, group.variants.size());
    auto desc = "[Group " + group.op_name + " omitted]";
    ASSERT_TRUE(missing_dl->Equals(*missing_dl)) << desc << " == itself";
    ASSERT_FALSE(missing_dl->Equals(*default_dl)) << desc << " != Default";
    ASSERT_FALSE(default_dl->Equals(*missing_dl)) << "Default != " << desc;
    for (size_t vi = 0; vi < group.variants.size(); vi++) {
      auto desc = "[Group " + group.op_name + " variant " +
                  std::to_string(vi + 1) + "]";
      sk_sp<DisplayList> variant_dl = Build(gi, vi);
      ASSERT_TRUE(variant_dl->Equals(*variant_dl)) << desc << " == itself";
      if (vi == 0) {
        ASSERT_TRUE(variant_dl->Equals(*default_dl)) << desc << " == Default";
        ASSERT_TRUE(default_dl->Equals(*variant_dl)) << "Default == " << desc;
      } else {
        ASSERT_FALSE(variant_dl->Equals(*default_dl)) << desc << " != Default";
        ASSERT_FALSE(default_dl->Equals(*variant_dl)) << "Default != " << desc;
      }
      if (group.variants[vi].is_empty()) {
        ASSERT_TRUE(variant_dl->Equals(*missing_dl)) << desc << " != omitted";
        ASSERT_TRUE(missing_dl->Equals(*variant_dl)) << "omitted != " << desc;
      } else {
        ASSERT_FALSE(variant_dl->Equals(*missing_dl)) << desc << " != omitted";
        ASSERT_FALSE(missing_dl->Equals(*variant_dl)) << "omitted != " << desc;
      }
    }
  }
}

TEST_F(DisplayListTest, DisplayListSaveLayerBoundsWithAlphaFilter) {
  SkRect build_bounds = SkRect::MakeLTRB(-100, -100, 200, 200);
  SkRect save_bounds = SkRect::MakeWH(100, 100);
  SkRect rect = SkRect::MakeLTRB(30, 30, 70, 70);
  // clang-format off
  const float color_matrix[] = {
    0, 0, 0, 0, 0,
    0, 1, 0, 0, 0,
    0, 0, 1, 0, 0,
    0, 0, 0, 1, 0,
  };
  // clang-format on
  auto base_color_filter = DlColorFilter::MakeMatrix(color_matrix);
  // clang-format off
  const float alpha_matrix[] = {
    0, 0, 0, 0, 0,
    0, 1, 0, 0, 0,
    0, 0, 1, 0, 0,
    0, 0, 0, 0, 1,
  };
  // clang-format on
  auto alpha_color_filter = DlColorFilter::MakeMatrix(alpha_matrix);
  sk_sp<SkColorFilter> sk_alpha_color_filter =
      SkColorFilters::Matrix(alpha_matrix);

  {
    // No tricky stuff, just verifying drawing a rect produces rect bounds
    DisplayListBuilder builder(build_bounds);
    builder.SaveLayer(&save_bounds, nullptr);
    builder.DrawRect(rect, DlPaint());
    builder.Restore();
    sk_sp<DisplayList> display_list = builder.Build();
    ASSERT_EQ(display_list->bounds(), rect);
  }

  {
    // Now checking that a normal color filter still produces rect bounds
    DisplayListBuilder builder(build_bounds);
    DlPaint save_paint;
    save_paint.setColorFilter(base_color_filter);
    builder.SaveLayer(&save_bounds, &save_paint);
    builder.DrawRect(rect, DlPaint());
    builder.Restore();
    sk_sp<DisplayList> display_list = builder.Build();
    ASSERT_EQ(display_list->bounds(), rect);
  }

  {
    // Now checking how SkPictureRecorder deals with a color filter
    // that modifies alpha channels (save layer bounds are meaningless
    // under those circumstances)
    SkPictureRecorder recorder;
    SkRTreeFactory rtree_factory;
    SkCanvas* canvas = recorder.beginRecording(build_bounds, &rtree_factory);
    SkPaint p1;
    p1.setColorFilter(sk_alpha_color_filter);
    canvas->saveLayer(save_bounds, &p1);
    SkPaint p2;
    canvas->drawRect(rect, p2);
    canvas->restore();
    sk_sp<SkPicture> picture = recorder.finishRecordingAsPicture();
    ASSERT_EQ(picture->cullRect(), build_bounds);
  }

  {
    // Now checking that DisplayList has the same behavior that we
    // saw in the SkPictureRecorder example above - returning the
    // cull rect of the DisplayListBuilder when it encounters a
    // save layer that modifies an unbounded region
    DisplayListBuilder builder(build_bounds);
    DlPaint save_paint;
    save_paint.setColorFilter(alpha_color_filter);
    builder.SaveLayer(&save_bounds, &save_paint);
    builder.DrawRect(rect, DlPaint());
    builder.Restore();
    sk_sp<DisplayList> display_list = builder.Build();
    ASSERT_EQ(display_list->bounds(), build_bounds);
  }

  {
    // Verifying that the save layer bounds are not relevant
    // to the behavior in the previous example
    DisplayListBuilder builder(build_bounds);
    DlPaint save_paint;
    save_paint.setColorFilter(alpha_color_filter);
    builder.SaveLayer(nullptr, &save_paint);
    builder.DrawRect(rect, DlPaint());
    builder.Restore();
    sk_sp<DisplayList> display_list = builder.Build();
    ASSERT_EQ(display_list->bounds(), build_bounds);
  }

  {
    // Making sure hiding a ColorFilter as an ImageFilter will
    // generate the same behavior as setting it as a ColorFilter
    DisplayListBuilder builder(build_bounds);
    DlColorFilterImageFilter color_filter_image_filter(base_color_filter);
    DlPaint save_paint;
    save_paint.setImageFilter(&color_filter_image_filter);
    builder.SaveLayer(&save_bounds, &save_paint);
    builder.DrawRect(rect, DlPaint());
    builder.Restore();
    sk_sp<DisplayList> display_list = builder.Build();
    ASSERT_EQ(display_list->bounds(), rect);
  }

  {
    // Making sure hiding a problematic ColorFilter as an ImageFilter
    // will generate the same behavior as setting it as a ColorFilter
    DisplayListBuilder builder(build_bounds);
    DlColorFilterImageFilter color_filter_image_filter(alpha_color_filter);
    DlPaint save_paint;
    save_paint.setImageFilter(&color_filter_image_filter);
    builder.SaveLayer(&save_bounds, &save_paint);
    builder.DrawRect(rect, DlPaint());
    builder.Restore();
    sk_sp<DisplayList> display_list = builder.Build();
    ASSERT_EQ(display_list->bounds(), build_bounds);
  }

  {
    // Same as above (ImageFilter hiding ColorFilter) with no save bounds
    DisplayListBuilder builder(build_bounds);
    DlColorFilterImageFilter color_filter_image_filter(alpha_color_filter);
    DlPaint save_paint;
    save_paint.setImageFilter(&color_filter_image_filter);
    builder.SaveLayer(nullptr, &save_paint);
    builder.DrawRect(rect, DlPaint());
    builder.Restore();
    sk_sp<DisplayList> display_list = builder.Build();
    ASSERT_EQ(display_list->bounds(), build_bounds);
  }

  {
    // Testing behavior with an unboundable blend mode
    DisplayListBuilder builder(build_bounds);
    DlPaint save_paint;
    save_paint.setBlendMode(DlBlendMode::kClear);
    builder.SaveLayer(&save_bounds, &save_paint);
    builder.DrawRect(rect, DlPaint());
    builder.Restore();
    sk_sp<DisplayList> display_list = builder.Build();
    ASSERT_EQ(display_list->bounds(), build_bounds);
  }

  {
    // Same as previous with no save bounds
    DisplayListBuilder builder(build_bounds);
    DlPaint save_paint;
    save_paint.setBlendMode(DlBlendMode::kClear);
    builder.SaveLayer(nullptr, &save_paint);
    builder.DrawRect(rect, DlPaint());
    builder.Restore();
    sk_sp<DisplayList> display_list = builder.Build();
    ASSERT_EQ(display_list->bounds(), build_bounds);
  }
}

TEST_F(DisplayListTest, NestedOpCountMetricsSameAsSkPicture) {
  SkPictureRecorder recorder;
  recorder.beginRecording(SkRect::MakeWH(150, 100));
  SkCanvas* canvas = recorder.getRecordingCanvas();
  SkPaint sk_paint;
  for (int y = 10; y <= 60; y += 10) {
    for (int x = 10; x <= 60; x += 10) {
      sk_paint.setColor(((x + y) % 20) == 10 ? SK_ColorRED : SK_ColorBLUE);
      canvas->drawRect(SkRect::MakeXYWH(x, y, 80, 80), sk_paint);
    }
  }
  SkPictureRecorder outer_recorder;
  outer_recorder.beginRecording(SkRect::MakeWH(150, 100));
  canvas = outer_recorder.getRecordingCanvas();
  canvas->drawPicture(recorder.finishRecordingAsPicture());

  auto picture = outer_recorder.finishRecordingAsPicture();
  ASSERT_EQ(picture->approximateOpCount(), 1);
  ASSERT_EQ(picture->approximateOpCount(true), 36);

  DisplayListBuilder builder(SkRect::MakeWH(150, 100));
  DlPaint dl_paint;
  for (int y = 10; y <= 60; y += 10) {
    for (int x = 10; x <= 60; x += 10) {
      dl_paint.setColor(((x + y) % 20) == 10 ? DlColor(SK_ColorRED)
                                             : DlColor(SK_ColorBLUE));
      builder.DrawRect(SkRect::MakeXYWH(x, y, 80, 80), dl_paint);
    }
  }

  DisplayListBuilder outer_builder(SkRect::MakeWH(150, 100));
  outer_builder.DrawDisplayList(builder.Build());
  auto display_list = outer_builder.Build();

  ASSERT_EQ(display_list->op_count(), 1u);
  ASSERT_EQ(display_list->op_count(true), 36u);
  EXPECT_EQ(display_list->total_depth(), 37u);

  ASSERT_EQ(picture->approximateOpCount(),
            static_cast<int>(display_list->op_count()));
  ASSERT_EQ(picture->approximateOpCount(true),
            static_cast<int>(display_list->op_count(true)));
}

TEST_F(DisplayListTest, DisplayListFullPerspectiveTransformHandling) {
  // SkM44 constructor takes row-major order
  SkM44 sk_matrix = SkM44(
      // clang-format off
       1,  2,  3,  4,
       5,  6,  7,  8,
       9, 10, 11, 12,
      13, 14, 15, 16
      // clang-format on
  );

  {  // First test ==
    DisplayListBuilder builder;
    // builder.TransformFullPerspective takes row-major order
    builder.TransformFullPerspective(
        // clang-format off
         1,  2,  3,  4,
         5,  6,  7,  8,
         9, 10, 11, 12,
        13, 14, 15, 16
        // clang-format on
    );
    sk_sp<DisplayList> display_list = builder.Build();
    sk_sp<SkSurface> surface =
        SkSurfaces::Raster(SkImageInfo::MakeN32Premul(10, 10));
    SkCanvas* canvas = surface->getCanvas();
    // We can't use DlSkCanvas.DrawDisplayList as that method protects
    // the canvas against mutations from the display list being drawn.
    auto dispatcher = DlSkCanvasDispatcher(surface->getCanvas());
    display_list->Dispatch(dispatcher);
    SkM44 dl_matrix = canvas->getLocalToDevice();
    ASSERT_EQ(sk_matrix, dl_matrix);
  }
  {  // Next test !=
    DisplayListBuilder builder;
    // builder.TransformFullPerspective takes row-major order
    builder.TransformFullPerspective(
        // clang-format off
         1,  5,  9, 13,
         2,  6,  7, 11,
         3,  7, 11, 15,
         4,  8, 12, 16
        // clang-format on
    );
    sk_sp<DisplayList> display_list = builder.Build();
    sk_sp<SkSurface> surface =
        SkSurfaces::Raster(SkImageInfo::MakeN32Premul(10, 10));
    SkCanvas* canvas = surface->getCanvas();
    // We can't use DlSkCanvas.DrawDisplayList as that method protects
    // the canvas against mutations from the display list being drawn.
    auto dispatcher = DlSkCanvasDispatcher(surface->getCanvas());
    display_list->Dispatch(dispatcher);
    SkM44 dl_matrix = canvas->getLocalToDevice();
    ASSERT_NE(sk_matrix, dl_matrix);
  }
}

TEST_F(DisplayListTest, DisplayListTransformResetHandling) {
  DisplayListBuilder builder;
  builder.Scale(20.0, 20.0);
  builder.TransformReset();
  auto display_list = builder.Build();
  ASSERT_NE(display_list, nullptr);
  sk_sp<SkSurface> surface =
      SkSurfaces::Raster(SkImageInfo::MakeN32Premul(10, 10));
  SkCanvas* canvas = surface->getCanvas();
  // We can't use DlSkCanvas.DrawDisplayList as that method protects
  // the canvas against mutations from the display list being drawn.
  auto dispatcher = DlSkCanvasDispatcher(surface->getCanvas());
  display_list->Dispatch(dispatcher);
  ASSERT_TRUE(canvas->getTotalMatrix().isIdentity());
}

TEST_F(DisplayListTest, SingleOpsMightSupportGroupOpacityBlendMode) {
  auto run_tests = [](const std::string& name,
                      void build(DlCanvas & canvas, const DlPaint& paint),
                      bool expect_for_op, bool expect_with_kSrc) {
    {
      // First test is the draw op, by itself
      // (usually supports group opacity)
      DisplayListBuilder builder;
      DlPaint paint;
      build(builder, paint);
      auto display_list = builder.Build();
      EXPECT_EQ(display_list->can_apply_group_opacity(), expect_for_op)
          << "{" << std::endl
          << "  " << name << std::endl
          << "}";
    }
    {
      // Second test is the draw op with kSrc,
      // (usually fails group opacity)
      DisplayListBuilder builder;
      DlPaint paint;
      paint.setBlendMode(DlBlendMode::kSrc);
      build(builder, paint);
      auto display_list = builder.Build();
      EXPECT_EQ(display_list->can_apply_group_opacity(), expect_with_kSrc)
          << "{" << std::endl
          << "  receiver.setBlendMode(kSrc);" << std::endl
          << "  " << name << std::endl
          << "}";
    }
  };

#define RUN_TESTS(body) \
  run_tests(            \
      #body, [](DlCanvas& canvas, const DlPaint& paint) { body }, true, false)

#define RUN_TESTS2(body, expect)                                          \
  run_tests(                                                              \
      #body, [](DlCanvas& canvas, const DlPaint& paint) { body }, expect, \
      expect)

  RUN_TESTS(canvas.DrawPaint(paint););
  RUN_TESTS2(canvas.DrawColor(DlColor(SK_ColorRED), DlBlendMode::kSrcOver);
             , true);
  RUN_TESTS2(canvas.DrawColor(DlColor(SK_ColorRED), DlBlendMode::kSrc);, false);
  RUN_TESTS(canvas.DrawLine(SkPoint{0, 0}, SkPoint{10, 10}, paint););
  RUN_TESTS(canvas.DrawRect(SkRect{0, 0, 10, 10}, paint););
  RUN_TESTS(canvas.DrawOval(SkRect{0, 0, 10, 10}, paint););
  RUN_TESTS(canvas.DrawCircle(SkPoint{10, 10}, 5, paint););
  RUN_TESTS(
      canvas.DrawRRect(SkRRect::MakeRectXY({0, 0, 10, 10}, 2, 2), paint););
  RUN_TESTS(canvas.DrawDRRect(SkRRect::MakeRectXY({0, 0, 10, 10}, 2, 2),
                              SkRRect::MakeRectXY({2, 2, 8, 8}, 2, 2), paint););
  RUN_TESTS(canvas.DrawPath(
      SkPath().addOval({0, 0, 10, 10}).addOval({5, 5, 15, 15}), paint););
  RUN_TESTS(canvas.DrawArc(SkRect{0, 0, 10, 10}, 0, math::kPi, true, paint););
  RUN_TESTS2(
      canvas.DrawPoints(PointMode::kPoints, TestPointCount, kTestPoints, paint);
      , false);
  RUN_TESTS2(canvas.DrawVertices(kTestVertices1, DlBlendMode::kSrc, paint);
             , false);
  RUN_TESTS(
      canvas.DrawImage(TestImage1, SkPoint{0, 0}, kLinearSampling, &paint););
  RUN_TESTS2(
      canvas.DrawImage(TestImage1, SkPoint{0, 0}, kLinearSampling, nullptr);
      , true);
  RUN_TESTS(canvas.DrawImageRect(TestImage1, SkIRect{10, 10, 20, 20},
                                 {0, 0, 10, 10}, kNearestSampling, &paint,
                                 DlCanvas::SrcRectConstraint::kFast););
  RUN_TESTS2(canvas.DrawImageRect(TestImage1, SkIRect{10, 10, 20, 20},
                                  {0, 0, 10, 10}, kNearestSampling, nullptr,
                                  DlCanvas::SrcRectConstraint::kFast);
             , true);
  RUN_TESTS(canvas.DrawImageNine(TestImage2, SkIRect{20, 20, 30, 30},
                                 SkRect{0, 0, 20, 20}, DlFilterMode::kLinear,
                                 &paint););
  RUN_TESTS2(canvas.DrawImageNine(TestImage2, SkIRect{20, 20, 30, 30},
                                  SkRect{0, 0, 20, 20}, DlFilterMode::kLinear,
                                  nullptr);
             , true);
  static SkRSXform xforms[] = {{1, 0, 0, 0}, {0, 1, 0, 0}};
  static SkRect texs[] = {{10, 10, 20, 20}, {20, 20, 30, 30}};
  RUN_TESTS2(
      canvas.DrawAtlas(TestImage1, xforms, texs, nullptr, 2,
                       DlBlendMode::kSrcIn, kNearestSampling, nullptr, &paint);
      , false);
  RUN_TESTS2(
      canvas.DrawAtlas(TestImage1, xforms, texs, nullptr, 2,
                       DlBlendMode::kSrcIn, kNearestSampling, nullptr, nullptr);
      , false);
  EXPECT_TRUE(TestDisplayList1->can_apply_group_opacity());
  RUN_TESTS2(canvas.DrawDisplayList(TestDisplayList1);, true);
  {
    static DisplayListBuilder builder;
    builder.DrawRect(SkRect{0, 0, 10, 10}, DlPaint());
    builder.DrawRect(SkRect{5, 5, 15, 15}, DlPaint());
    static auto display_list = builder.Build();
    RUN_TESTS2(canvas.DrawDisplayList(display_list);, false);
  }
  RUN_TESTS2(canvas.DrawTextBlob(GetTestTextBlob(1), 0, 0, paint);, false);
  RUN_TESTS2(
      canvas.DrawShadow(kTestPath1, DlColor(SK_ColorBLACK), 1.0, false, 1.0);
      , false);

#undef RUN_TESTS2
#undef RUN_TESTS
}

TEST_F(DisplayListTest, OverlappingOpsDoNotSupportGroupOpacity) {
  DisplayListBuilder builder;
  for (int i = 0; i < 10; i++) {
    builder.DrawRect(SkRect::MakeXYWH(i * 10, 0, 30, 30), DlPaint());
  }
  auto display_list = builder.Build();
  EXPECT_FALSE(display_list->can_apply_group_opacity());
}

TEST_F(DisplayListTest, LineOfNonOverlappingOpsSupportGroupOpacity) {
  DisplayListBuilder builder;
  for (int i = 0; i < 10; i++) {
    builder.DrawRect(SkRect::MakeXYWH(i * 30, 0, 30, 30), DlPaint());
  }
  auto display_list = builder.Build();
  EXPECT_TRUE(display_list->can_apply_group_opacity());
}

TEST_F(DisplayListTest, CrossOfNonOverlappingOpsSupportGroupOpacity) {
  DisplayListBuilder builder;
  builder.DrawRect(SkRect::MakeLTRB(200, 200, 300, 300), DlPaint());  // center
  builder.DrawRect(SkRect::MakeLTRB(100, 200, 200, 300), DlPaint());  // left
  builder.DrawRect(SkRect::MakeLTRB(200, 100, 300, 200), DlPaint());  // above
  builder.DrawRect(SkRect::MakeLTRB(300, 200, 400, 300), DlPaint());  // right
  builder.DrawRect(SkRect::MakeLTRB(200, 300, 300, 400), DlPaint());  // below
  auto display_list = builder.Build();
  EXPECT_TRUE(display_list->can_apply_group_opacity());
}

TEST_F(DisplayListTest, SaveLayerFalseSupportsGroupOpacityOverlappingChidren) {
  DisplayListBuilder builder;
  builder.SaveLayer(nullptr, nullptr);
  for (int i = 0; i < 10; i++) {
    builder.DrawRect(SkRect::MakeXYWH(i * 10, 0, 30, 30), DlPaint());
  }
  builder.Restore();
  auto display_list = builder.Build();
  EXPECT_TRUE(display_list->can_apply_group_opacity());
}

TEST_F(DisplayListTest, SaveLayerTrueSupportsGroupOpacityOverlappingChidren) {
  DisplayListBuilder builder;
  DlPaint save_paint;
  builder.SaveLayer(nullptr, &save_paint);
  for (int i = 0; i < 10; i++) {
    builder.DrawRect(SkRect::MakeXYWH(i * 10, 0, 30, 30), DlPaint());
  }
  builder.Restore();
  auto display_list = builder.Build();
  EXPECT_TRUE(display_list->can_apply_group_opacity());
}

TEST_F(DisplayListTest, SaveLayerFalseWithSrcBlendSupportsGroupOpacity) {
  DisplayListBuilder builder;
  // This empty draw rect will not actually be inserted into the stream,
  // but the Src blend mode will be synchronized as an attribute. The
  // SaveLayer following it should not use that attribute to base its
  // decisions about group opacity and the draw rect after that comes
  // with its own compatible blend mode.
  builder.DrawRect(SkRect{0, 0, 0, 0},
                   DlPaint().setBlendMode(DlBlendMode::kSrc));
  builder.SaveLayer(nullptr, nullptr);
  builder.DrawRect(SkRect{0, 0, 10, 10}, DlPaint());
  builder.Restore();
  auto display_list = builder.Build();
  EXPECT_TRUE(display_list->can_apply_group_opacity());
}

TEST_F(DisplayListTest, SaveLayerTrueWithSrcBlendDoesNotSupportGroupOpacity) {
  DisplayListBuilder builder;
  DlPaint save_paint;
  save_paint.setBlendMode(DlBlendMode::kSrc);
  builder.SaveLayer(nullptr, &save_paint);
  builder.DrawRect(SkRect{0, 0, 10, 10}, DlPaint());
  builder.Restore();
  auto display_list = builder.Build();
  EXPECT_FALSE(display_list->can_apply_group_opacity());
}

TEST_F(DisplayListTest, SaveLayerFalseSupportsGroupOpacityWithChildSrcBlend) {
  DisplayListBuilder builder;
  builder.SaveLayer(nullptr, nullptr);
  builder.DrawRect(SkRect{0, 0, 10, 10},
                   DlPaint().setBlendMode(DlBlendMode::kSrc));
  builder.Restore();
  auto display_list = builder.Build();
  EXPECT_TRUE(display_list->can_apply_group_opacity());
}

TEST_F(DisplayListTest, SaveLayerTrueSupportsGroupOpacityWithChildSrcBlend) {
  DisplayListBuilder builder;
  DlPaint save_paint;
  builder.SaveLayer(nullptr, &save_paint);
  builder.DrawRect(SkRect{0, 0, 10, 10},
                   DlPaint().setBlendMode(DlBlendMode::kSrc));
  builder.Restore();
  auto display_list = builder.Build();
  EXPECT_TRUE(display_list->can_apply_group_opacity());
}

TEST_F(DisplayListTest, SaveLayerBoundsSnapshotsImageFilter) {
  DisplayListBuilder builder;
  DlPaint save_paint;
  builder.SaveLayer(nullptr, &save_paint);
  builder.DrawRect(SkRect{50, 50, 100, 100}, DlPaint());
  // This image filter should be ignored since it was not set before SaveLayer
  // And the rect drawn with it will not contribute any more area to the bounds
  DlPaint draw_paint;
  draw_paint.setImageFilter(&kTestBlurImageFilter1);
  builder.DrawRect(SkRect{70, 70, 80, 80}, draw_paint);
  builder.Restore();
  SkRect bounds = builder.Build()->bounds();
  EXPECT_EQ(bounds, SkRect::MakeLTRB(50, 50, 100, 100));
}

#define SAVE_LAYER_EXPECTOR(name) SaveLayerExpector name(__FILE__, __LINE__)

using SaveLayerOptionsTester =
    std::function<bool(const SaveLayerOptions& options)>;

struct SaveLayerExpectations {
  SaveLayerExpectations() {}

  // NOLINTNEXTLINE(google-explicit-constructor)
  SaveLayerExpectations(const SaveLayerOptions& o) : options(o) {}
  // NOLINTNEXTLINE(google-explicit-constructor)
  SaveLayerExpectations(const SaveLayerOptionsTester& t) : tester(t) {}
  // NOLINTNEXTLINE(google-explicit-constructor)
  SaveLayerExpectations(DlBlendMode mode) : max_blend_mode(mode) {}

  std::optional<SaveLayerOptions> options;
  std::optional<SaveLayerOptionsTester> tester;
  std::optional<DlBlendMode> max_blend_mode;
};

::std::ostream& operator<<(::std::ostream& os,
                           const SaveLayerExpectations& expect) {
  os << "SaveLayerExpectation(";
  if (expect.options.has_value()) {
    os << "options: " << expect.options.value();
  }
  if (expect.tester.has_value()) {
    os << "option tester: " << &expect.tester.value();
  }
  if (expect.max_blend_mode.has_value()) {
    os << "max_blend: " << expect.max_blend_mode.value();
  }
  os << ")";
  return os;
}

class SaveLayerExpector : public virtual DlOpReceiver,
                          public IgnoreAttributeDispatchHelper,
                          public IgnoreClipDispatchHelper,
                          public IgnoreTransformDispatchHelper,
                          public IgnoreDrawDispatchHelper {
 public:
  SaveLayerExpector(const std::string& file, int line)
      : file_(file), line_(line), detail_("") {}

  ~SaveLayerExpector() {  //
    EXPECT_EQ(save_layer_count_, expected_.size()) << label();
    while (save_layer_count_ < expected_.size()) {
      auto expect = expected_[save_layer_count_];
      FML_LOG(ERROR) << "leftover expectation[" << save_layer_count_
                     << "] = " << expect;
      save_layer_count_++;
    }
  }

  SaveLayerExpector& addDetail(const std::string& detail) {
    detail_ = detail;
    return *this;
  }

  SaveLayerExpector& addExpectation(const SaveLayerExpectations& expected) {
    expected_.push_back(expected);
    return *this;
  }

  SaveLayerExpector& addExpectation(const SaveLayerOptionsTester& tester) {
    expected_.push_back(SaveLayerExpectations(tester));
    return *this;
  }

  SaveLayerExpector& addOpenExpectation() {
    expected_.push_back(SaveLayerExpectations());
    return *this;
  }

  void saveLayer(const DlRect& bounds,
                 const SaveLayerOptions options,
                 const DlImageFilter* backdrop,
                 std::optional<int64_t> backdrop_id) override {
    FML_UNREACHABLE();
  }

  virtual void saveLayer(const DlRect& bounds,
                         const SaveLayerOptions& options,
                         uint32_t total_content_depth,
                         DlBlendMode max_content_blend_mode,
                         const DlImageFilter* backdrop = nullptr,
                         std::optional<int64_t> backdrop_id = std::nullopt) {
    ASSERT_LT(save_layer_count_, expected_.size()) << label();
    auto expect = expected_[save_layer_count_];
    if (expect.options.has_value()) {
      EXPECT_EQ(options, expect.options.value()) << label();
    }
    if (expect.tester.has_value()) {
      EXPECT_TRUE(expect.tester.value()(options)) << label();
    }
    if (expect.max_blend_mode.has_value()) {
      EXPECT_EQ(max_content_blend_mode, expect.max_blend_mode.value())
          << label();
    }
    save_layer_count_++;
  }

  bool all_expectations_checked() const {
    return save_layer_count_ == expected_.size();
  }

 private:
  // mutable allows the copy constructor to leave no expectations behind
  mutable std::vector<SaveLayerExpectations> expected_;
  size_t save_layer_count_ = 0;

  const std::string file_;
  const int line_;
  std::string detail_;

  std::string label() {
    std::string label = "at index " + std::to_string(save_layer_count_) +  //
                        ", from " + file_ +                                //
                        ":" + std::to_string(line_);
    if (detail_.length() > 0) {
      label = label + " (" + detail_ + ")";
    }
    return label;
  }
};

TEST_F(DisplayListTest, SaveLayerOneSimpleOpInheritsOpacity) {
  SAVE_LAYER_EXPECTOR(expector);
  expector.addExpectation(
      SaveLayerOptions::kWithAttributes.with_can_distribute_opacity());

  DisplayListBuilder builder;
  DlPaint save_paint;
  save_paint.setColor(DlColor(SkColorSetARGB(127, 255, 255, 255)));
  builder.SaveLayer(nullptr, &save_paint);
  builder.DrawRect(SkRect{10, 10, 20, 20}, DlPaint());
  builder.Restore();

  builder.Build()->Dispatch(expector);
  EXPECT_TRUE(expector.all_expectations_checked());
}

TEST_F(DisplayListTest, SaveLayerNoAttributesInheritsOpacity) {
  SAVE_LAYER_EXPECTOR(expector);
  expector.addExpectation(
      SaveLayerOptions::kNoAttributes.with_can_distribute_opacity());

  DisplayListBuilder builder;
  builder.SaveLayer(nullptr, nullptr);
  builder.DrawRect(SkRect{10, 10, 20, 20}, DlPaint());
  builder.Restore();

  builder.Build()->Dispatch(expector);
  EXPECT_TRUE(expector.all_expectations_checked());
}

TEST_F(DisplayListTest, SaveLayerTwoOverlappingOpsDoesNotInheritOpacity) {
  SAVE_LAYER_EXPECTOR(expector);
  expector.addExpectation(SaveLayerOptions::kWithAttributes);

  DisplayListBuilder builder;
  DlPaint save_paint;
  save_paint.setColor(DlColor(SkColorSetARGB(127, 255, 255, 255)));
  builder.SaveLayer(nullptr, &save_paint);
  builder.DrawRect(SkRect{10, 10, 20, 20}, DlPaint());
  builder.DrawRect(SkRect{15, 15, 25, 25}, DlPaint());
  builder.Restore();

  builder.Build()->Dispatch(expector);
  EXPECT_TRUE(expector.all_expectations_checked());
}

TEST_F(DisplayListTest, NestedSaveLayersMightInheritOpacity) {
  SAVE_LAYER_EXPECTOR(expector);
  expector  //
      .addExpectation(
          SaveLayerOptions::kWithAttributes.with_can_distribute_opacity())
      .addExpectation(SaveLayerOptions::kWithAttributes)
      .addExpectation(
          SaveLayerOptions::kWithAttributes.with_can_distribute_opacity());

  DisplayListBuilder builder;
  DlPaint save_paint;
  save_paint.setColor(DlColor(SkColorSetARGB(127, 255, 255, 255)));
  builder.SaveLayer(nullptr, &save_paint);
  builder.SaveLayer(nullptr, &save_paint);
  builder.DrawRect(SkRect{10, 10, 20, 20}, DlPaint());
  builder.SaveLayer(nullptr, &save_paint);
  builder.DrawRect(SkRect{15, 15, 25, 25}, DlPaint());
  builder.Restore();
  builder.Restore();
  builder.Restore();

  builder.Build()->Dispatch(expector);
  EXPECT_TRUE(expector.all_expectations_checked());
}

TEST_F(DisplayListTest, NestedSaveLayersCanBothSupportOpacityOptimization) {
  SAVE_LAYER_EXPECTOR(expector);
  expector  //
      .addExpectation(
          SaveLayerOptions::kWithAttributes.with_can_distribute_opacity())
      .addExpectation(
          SaveLayerOptions::kNoAttributes.with_can_distribute_opacity());

  DisplayListBuilder builder;
  DlPaint save_paint;
  save_paint.setColor(DlColor(SkColorSetARGB(127, 255, 255, 255)));
  builder.SaveLayer(nullptr, &save_paint);
  builder.SaveLayer(nullptr, nullptr);
  builder.DrawRect(SkRect{10, 10, 20, 20}, DlPaint());
  builder.Restore();
  builder.Restore();

  builder.Build()->Dispatch(expector);
  EXPECT_TRUE(expector.all_expectations_checked());
}

TEST_F(DisplayListTest, SaveLayerImageFilterDoesNotInheritOpacity) {
  SAVE_LAYER_EXPECTOR(expector);
  expector.addExpectation(SaveLayerOptions::kWithAttributes);

  DisplayListBuilder builder;
  DlPaint save_paint;
  save_paint.setColor(DlColor(SkColorSetARGB(127, 255, 255, 255)));
  save_paint.setImageFilter(&kTestBlurImageFilter1);
  builder.SaveLayer(nullptr, &save_paint);
  builder.DrawRect(SkRect{10, 10, 20, 20}, DlPaint());
  builder.Restore();

  builder.Build()->Dispatch(expector);
  EXPECT_TRUE(expector.all_expectations_checked());
}

TEST_F(DisplayListTest, SaveLayerColorFilterDoesNotInheritOpacity) {
  SAVE_LAYER_EXPECTOR(expector);
  expector.addExpectation(SaveLayerOptions::kWithAttributes);

  DisplayListBuilder builder;
  DlPaint save_paint;
  save_paint.setColor(DlColor(SkColorSetARGB(127, 255, 255, 255)));
  save_paint.setColorFilter(kTestMatrixColorFilter1);
  builder.SaveLayer(nullptr, &save_paint);
  builder.DrawRect(SkRect{10, 10, 20, 20}, DlPaint());
  builder.Restore();

  builder.Build()->Dispatch(expector);
  EXPECT_TRUE(expector.all_expectations_checked());
}

TEST_F(DisplayListTest, SaveLayerSrcBlendDoesNotInheritOpacity) {
  SAVE_LAYER_EXPECTOR(expector);
  expector.addExpectation(SaveLayerOptions::kWithAttributes);

  DisplayListBuilder builder;
  DlPaint save_paint;
  save_paint.setColor(DlColor(SkColorSetARGB(127, 255, 255, 255)));
  save_paint.setBlendMode(DlBlendMode::kSrc);
  builder.SaveLayer(nullptr, &save_paint);
  builder.DrawRect(SkRect{10, 10, 20, 20}, DlPaint());
  builder.Restore();

  builder.Build()->Dispatch(expector);
  EXPECT_TRUE(expector.all_expectations_checked());
}

TEST_F(DisplayListTest, SaveLayerImageFilterOnChildInheritsOpacity) {
  SAVE_LAYER_EXPECTOR(expector);
  expector.addExpectation(
      SaveLayerOptions::kWithAttributes.with_can_distribute_opacity());

  DisplayListBuilder builder;
  DlPaint save_paint;
  save_paint.setColor(DlColor(SkColorSetARGB(127, 255, 255, 255)));
  builder.SaveLayer(nullptr, &save_paint);
  DlPaint draw_paint = save_paint;
  draw_paint.setImageFilter(&kTestBlurImageFilter1);
  builder.DrawRect(SkRect{10, 10, 20, 20}, draw_paint);
  builder.Restore();

  builder.Build()->Dispatch(expector);
  EXPECT_TRUE(expector.all_expectations_checked());
}

TEST_F(DisplayListTest, SaveLayerColorFilterOnChildDoesNotInheritOpacity) {
  SAVE_LAYER_EXPECTOR(expector);
  expector.addExpectation(SaveLayerOptions::kWithAttributes);

  DisplayListBuilder builder;
  DlPaint save_paint;
  save_paint.setColor(DlColor(SkColorSetARGB(127, 255, 255, 255)));
  builder.SaveLayer(nullptr, &save_paint);
  DlPaint draw_paint = save_paint;
  draw_paint.setColorFilter(kTestMatrixColorFilter1);
  builder.DrawRect(SkRect{10, 10, 20, 20}, draw_paint);
  builder.Restore();

  builder.Build()->Dispatch(expector);
  EXPECT_TRUE(expector.all_expectations_checked());
}

TEST_F(DisplayListTest, SaveLayerSrcBlendOnChildDoesNotInheritOpacity) {
  SAVE_LAYER_EXPECTOR(expector);
  expector.addExpectation(SaveLayerOptions::kWithAttributes);

  DisplayListBuilder builder;
  DlPaint save_paint;
  save_paint.setColor(DlColor(SkColorSetARGB(127, 255, 255, 255)));
  builder.SaveLayer(nullptr, &save_paint);
  DlPaint draw_paint = save_paint;
  draw_paint.setBlendMode(DlBlendMode::kSrc);
  builder.DrawRect(SkRect{10, 10, 20, 20}, draw_paint);
  builder.Restore();

  builder.Build()->Dispatch(expector);
  EXPECT_TRUE(expector.all_expectations_checked());
}

TEST_F(DisplayListTest, FlutterSvgIssue661BoundsWereEmpty) {
  // See https://github.com/dnfield/flutter_svg/issues/661

  SkPath path1;
  path1.setFillType(SkPathFillType::kWinding);
  path1.moveTo(25.54f, 37.52f);
  path1.cubicTo(20.91f, 37.52f, 16.54f, 33.39f, 13.62f, 30.58f);
  path1.lineTo(13, 30);
  path1.lineTo(12.45f, 29.42f);
  path1.cubicTo(8.39f, 25.15f, 1.61f, 18, 8.37f, 11.27f);
  path1.cubicTo(10.18f, 9.46f, 12.37f, 9.58f, 14.49f, 11.58f);
  path1.cubicTo(15.67f, 12.71f, 17.05f, 14.69f, 17.07f, 16.58f);
  path1.cubicTo(17.0968f, 17.458f, 16.7603f, 18.3081f, 16.14f, 18.93f);
  path1.cubicTo(15.8168f, 19.239f, 15.4653f, 19.5169f, 15.09f, 19.76f);
  path1.cubicTo(14.27f, 20.33f, 14.21f, 20.44f, 14.27f, 20.62f);
  path1.cubicTo(15.1672f, 22.3493f, 16.3239f, 23.9309f, 17.7f, 25.31f);
  path1.cubicTo(19.0791f, 26.6861f, 20.6607f, 27.8428f, 22.39f, 28.74f);
  path1.cubicTo(22.57f, 28.8f, 22.69f, 28.74f, 23.25f, 27.92f);
  path1.cubicTo(23.5f, 27.566f, 23.778f, 27.231f, 24.08f, 26.92f);
  path1.cubicTo(24.7045f, 26.3048f, 25.5538f, 25.9723f, 26.43f, 26);
  path1.cubicTo(28.29f, 26, 30.27f, 27.4f, 31.43f, 28.58f);
  path1.cubicTo(33.43f, 30.67f, 33.55f, 32.9f, 31.74f, 34.7f);
  path1.cubicTo(30.1477f, 36.4508f, 27.906f, 37.4704f, 25.54f, 37.52f);
  path1.close();
  path1.moveTo(11.17f, 12.23f);
  path1.cubicTo(10.6946f, 12.2571f, 10.2522f, 12.4819f, 9.95f, 12.85f);
  path1.cubicTo(5.12f, 17.67f, 8.95f, 22.5f, 14.05f, 27.85f);
  path1.lineTo(14.62f, 28.45f);
  path1.lineTo(15.16f, 28.96f);
  path1.cubicTo(20.52f, 34.06f, 25.35f, 37.89f, 30.16f, 33.06f);
  path1.cubicTo(30.83f, 32.39f, 31.25f, 31.56f, 29.81f, 30.06f);
  path1.cubicTo(28.9247f, 29.07f, 27.7359f, 28.4018f, 26.43f, 28.16f);
  path1.cubicTo(26.1476f, 28.1284f, 25.8676f, 28.2367f, 25.68f, 28.45f);
  path1.cubicTo(25.4633f, 28.6774f, 25.269f, 28.9252f, 25.1f, 29.19f);
  path1.cubicTo(24.53f, 30.01f, 23.47f, 31.54f, 21.54f, 30.79f);
  path1.lineTo(21.41f, 30.72f);
  path1.cubicTo(19.4601f, 29.7156f, 17.6787f, 28.4133f, 16.13f, 26.86f);
  path1.cubicTo(14.5748f, 25.3106f, 13.2693f, 23.5295f, 12.26f, 21.58f);
  path1.lineTo(12.2f, 21.44f);
  path1.cubicTo(11.45f, 19.51f, 12.97f, 18.44f, 13.8f, 17.88f);
  path1.cubicTo(14.061f, 17.706f, 14.308f, 17.512f, 14.54f, 17.3f);
  path1.cubicTo(14.7379f, 17.1067f, 14.8404f, 16.8359f, 14.82f, 16.56f);
  path1.cubicTo(14.5978f, 15.268f, 13.9585f, 14.0843f, 13, 13.19f);
  path1.cubicTo(12.5398f, 12.642f, 11.8824f, 12.2971f, 11.17f, 12.23f);
  path1.lineTo(11.17f, 12.23f);
  path1.close();
  path1.moveTo(27, 19.34f);
  path1.lineTo(24.74f, 19.34f);
  path1.cubicTo(24.7319f, 18.758f, 24.262f, 18.2881f, 23.68f, 18.28f);
  path1.lineTo(23.68f, 16.05f);
  path1.lineTo(23.7f, 16.05f);
  path1.cubicTo(25.5153f, 16.0582f, 26.9863f, 17.5248f, 27, 19.34f);
  path1.lineTo(27, 19.34f);
  path1.close();
  path1.moveTo(32.3f, 19.34f);
  path1.lineTo(30.07f, 19.34f);
  path1.cubicTo(30.037f, 15.859f, 27.171f, 13.011f, 23.69f, 13);
  path1.lineTo(23.69f, 10.72f);
  path1.cubicTo(28.415f, 10.725f, 32.3f, 14.615f, 32.3f, 19.34f);
  path1.close();

  SkPath path2;
  path2.setFillType(SkPathFillType::kWinding);
  path2.moveTo(37.5f, 19.33f);
  path2.lineTo(35.27f, 19.33f);
  path2.cubicTo(35.265f, 12.979f, 30.041f, 7.755f, 23.69f, 7.75f);
  path2.lineTo(23.69f, 5.52f);
  path2.cubicTo(31.264f, 5.525f, 37.495f, 11.756f, 37.5f, 19.33f);
  path2.close();

  DisplayListBuilder builder;
  DlPaint paint = DlPaint(DlColor::kWhite()).setAntiAlias(true);
  {
    builder.Save();
    builder.ClipRect(SkRect{0, 0, 100, 100}, ClipOp::kIntersect, true);
    {
      builder.Save();
      builder.Transform2DAffine(2.17391, 0, -2547.83,  //
                                0, 2.04082, -500);
      {
        builder.Save();
        builder.ClipRect(SkRect{1172, 245, 1218, 294}, ClipOp::kIntersect,
                         true);
        {
          builder.SaveLayer(nullptr, nullptr, nullptr);
          {
            builder.Save();
            builder.Transform2DAffine(1.4375, 0, 1164.09,  //
                                      0, 1.53125, 236.548);
            builder.DrawPath(path1, paint);
            builder.Restore();
          }
          {
            builder.Save();
            builder.Transform2DAffine(1.4375, 0, 1164.09,  //
                                      0, 1.53125, 236.548);
            builder.DrawPath(path2, paint);
            builder.Restore();
          }
          builder.Restore();
        }
        builder.Restore();
      }
      builder.Restore();
    }
    builder.Restore();
  }
  sk_sp<DisplayList> display_list = builder.Build();
  // Prior to the fix, the bounds were empty.
  EXPECT_FALSE(display_list->bounds().isEmpty());
  // These are just inside and outside of the expected bounds, but
  // testing float values can be flaky wrt minor changes in the bounds
  // calculations. If these lines have to be revised too often as the DL
  // implementation is improved and maintained, then we can eliminate
  // this test and just rely on the "rounded out" bounds test that follows.
  SkRect min_bounds = SkRect::MakeLTRB(0, 0.00191, 99.983, 100);
  SkRect max_bounds = SkRect::MakeLTRB(0, 0.00189, 99.985, 100);
  ASSERT_TRUE(max_bounds.contains(min_bounds));
  EXPECT_TRUE(max_bounds.contains(display_list->bounds()));
  EXPECT_TRUE(display_list->bounds().contains(min_bounds));

  // This is the more practical result. The bounds are "almost" 0,0,100x100
  EXPECT_EQ(display_list->bounds().roundOut(), SkIRect::MakeWH(100, 100));
  EXPECT_EQ(display_list->op_count(), 19u);
  EXPECT_EQ(display_list->bytes(), sizeof(DisplayList) + 392u);
  EXPECT_EQ(display_list->total_depth(), 3u);
}

TEST_F(DisplayListTest, TranslateAffectsCurrentTransform) {
  DisplayListBuilder builder;
  builder.Translate(12.3, 14.5);
  SkMatrix matrix = SkMatrix::Translate(12.3, 14.5);
  SkM44 m44 = SkM44(matrix);
  SkM44 cur_m44 = builder.GetTransformFullPerspective();
  SkMatrix cur_matrix = builder.GetTransform();
  ASSERT_EQ(cur_m44, m44);
  ASSERT_EQ(cur_matrix, matrix);
  builder.Translate(10, 10);
  // CurrentTransform has changed
  ASSERT_NE(builder.GetTransformFullPerspective(), m44);
  ASSERT_NE(builder.GetTransform(), cur_matrix);
  // Previous return values have not
  ASSERT_EQ(cur_m44, m44);
  ASSERT_EQ(cur_matrix, matrix);
}

TEST_F(DisplayListTest, ScaleAffectsCurrentTransform) {
  DisplayListBuilder builder;
  builder.Scale(12.3, 14.5);
  SkMatrix matrix = SkMatrix::Scale(12.3, 14.5);
  SkM44 m44 = SkM44(matrix);
  SkM44 cur_m44 = builder.GetTransformFullPerspective();
  SkMatrix cur_matrix = builder.GetTransform();
  ASSERT_EQ(cur_m44, m44);
  ASSERT_EQ(cur_matrix, matrix);
  builder.Translate(10, 10);
  // CurrentTransform has changed
  ASSERT_NE(builder.GetTransformFullPerspective(), m44);
  ASSERT_NE(builder.GetTransform(), cur_matrix);
  // Previous return values have not
  ASSERT_EQ(cur_m44, m44);
  ASSERT_EQ(cur_matrix, matrix);
}

TEST_F(DisplayListTest, RotateAffectsCurrentTransform) {
  DisplayListBuilder builder;
  builder.Rotate(12.3);
  SkMatrix matrix = SkMatrix::RotateDeg(12.3);
  SkM44 m44 = SkM44(matrix);
  SkM44 cur_m44 = builder.GetTransformFullPerspective();
  SkMatrix cur_matrix = builder.GetTransform();
  ASSERT_EQ(cur_m44, m44);
  ASSERT_EQ(cur_matrix, matrix);
  builder.Translate(10, 10);
  // CurrentTransform has changed
  ASSERT_NE(builder.GetTransformFullPerspective(), m44);
  ASSERT_NE(builder.GetTransform(), cur_matrix);
  // Previous return values have not
  ASSERT_EQ(cur_m44, m44);
  ASSERT_EQ(cur_matrix, matrix);
}

TEST_F(DisplayListTest, SkewAffectsCurrentTransform) {
  DisplayListBuilder builder;
  builder.Skew(12.3, 14.5);
  SkMatrix matrix = SkMatrix::Skew(12.3, 14.5);
  SkM44 m44 = SkM44(matrix);
  SkM44 cur_m44 = builder.GetTransformFullPerspective();
  SkMatrix cur_matrix = builder.GetTransform();
  ASSERT_EQ(cur_m44, m44);
  ASSERT_EQ(cur_matrix, matrix);
  builder.Translate(10, 10);
  // CurrentTransform has changed
  ASSERT_NE(builder.GetTransformFullPerspective(), m44);
  ASSERT_NE(builder.GetTransform(), cur_matrix);
  // Previous return values have not
  ASSERT_EQ(cur_m44, m44);
  ASSERT_EQ(cur_matrix, matrix);
}

TEST_F(DisplayListTest, TransformAffectsCurrentTransform) {
  DisplayListBuilder builder;
  builder.Transform2DAffine(3, 0, 12.3,  //
                            1, 5, 14.5);
  SkMatrix matrix = SkMatrix::MakeAll(3, 0, 12.3,  //
                                      1, 5, 14.5,  //
                                      0, 0, 1);
  SkM44 m44 = SkM44(matrix);
  SkM44 cur_m44 = builder.GetTransformFullPerspective();
  SkMatrix cur_matrix = builder.GetTransform();
  ASSERT_EQ(cur_m44, m44);
  ASSERT_EQ(cur_matrix, matrix);
  builder.Translate(10, 10);
  // CurrentTransform has changed
  ASSERT_NE(builder.GetTransformFullPerspective(), m44);
  ASSERT_NE(builder.GetTransform(), cur_matrix);
  // Previous return values have not
  ASSERT_EQ(cur_m44, m44);
  ASSERT_EQ(cur_matrix, matrix);
}

TEST_F(DisplayListTest, FullTransformAffectsCurrentTransform) {
  DisplayListBuilder builder;
  builder.TransformFullPerspective(3, 0, 4, 12.3,  //
                                   1, 5, 3, 14.5,  //
                                   0, 0, 7, 16.2,  //
                                   0, 0, 0, 1);
  SkMatrix matrix = SkMatrix::MakeAll(3, 0, 12.3,  //
                                      1, 5, 14.5,  //
                                      0, 0, 1);
  SkM44 m44 = SkM44(3, 0, 4, 12.3,  //
                    1, 5, 3, 14.5,  //
                    0, 0, 7, 16.2,  //
                    0, 0, 0, 1);
  SkM44 cur_m44 = builder.GetTransformFullPerspective();
  SkMatrix cur_matrix = builder.GetTransform();
  ASSERT_EQ(cur_m44, m44);
  ASSERT_EQ(cur_matrix, matrix);
  builder.Translate(10, 10);
  // CurrentTransform has changed
  ASSERT_NE(builder.GetTransformFullPerspective(), m44);
  ASSERT_NE(builder.GetTransform(), cur_matrix);
  // Previous return values have not
  ASSERT_EQ(cur_m44, m44);
  ASSERT_EQ(cur_matrix, matrix);
}

TEST_F(DisplayListTest, ClipRectAffectsClipBounds) {
  DisplayListBuilder builder;
  SkRect clip_bounds = SkRect::MakeLTRB(10.2, 11.3, 20.4, 25.7);
  builder.ClipRect(clip_bounds, ClipOp::kIntersect, false);

  // Save initial return values for testing restored values
  SkRect initial_local_bounds = builder.GetLocalClipBounds();
  SkRect initial_destination_bounds = builder.GetDestinationClipBounds();
  ASSERT_EQ(initial_local_bounds, clip_bounds);
  ASSERT_EQ(initial_destination_bounds, clip_bounds);

  builder.Save();
  builder.ClipRect(SkRect{0, 0, 15, 15}, ClipOp::kIntersect, false);
  // Both clip bounds have changed
  ASSERT_NE(builder.GetLocalClipBounds(), clip_bounds);
  ASSERT_NE(builder.GetDestinationClipBounds(), clip_bounds);
  // Previous return values have not changed
  ASSERT_EQ(initial_local_bounds, clip_bounds);
  ASSERT_EQ(initial_destination_bounds, clip_bounds);
  builder.Restore();

  // save/restore returned the values to their original values
  ASSERT_EQ(builder.GetLocalClipBounds(), initial_local_bounds);
  ASSERT_EQ(builder.GetDestinationClipBounds(), initial_destination_bounds);

  builder.Save();
  builder.Scale(2, 2);
  SkRect scaled_clip_bounds = SkRect::MakeLTRB(5.1, 5.65, 10.2, 12.85);
  ASSERT_EQ(builder.GetLocalClipBounds(), scaled_clip_bounds);
  // Destination bounds are unaffected by transform
  ASSERT_EQ(builder.GetDestinationClipBounds(), clip_bounds);
  builder.Restore();

  // save/restore returned the values to their original values
  ASSERT_EQ(builder.GetLocalClipBounds(), initial_local_bounds);
  ASSERT_EQ(builder.GetDestinationClipBounds(), initial_destination_bounds);
}

TEST_F(DisplayListTest, ClipRectDoAAAffectsClipBounds) {
  DisplayListBuilder builder;
  SkRect clip_bounds = SkRect::MakeLTRB(10.2, 11.3, 20.4, 25.7);
  SkRect clip_expanded_bounds = SkRect::MakeLTRB(10, 11, 21, 26);
  builder.ClipRect(clip_bounds, ClipOp::kIntersect, true);

  // Save initial return values for testing restored values
  SkRect initial_local_bounds = builder.GetLocalClipBounds();
  SkRect initial_destination_bounds = builder.GetDestinationClipBounds();
  ASSERT_EQ(initial_local_bounds, clip_expanded_bounds);
  ASSERT_EQ(initial_destination_bounds, clip_expanded_bounds);

  builder.Save();
  builder.ClipRect(SkRect{0, 0, 15, 15}, ClipOp::kIntersect, true);
  // Both clip bounds have changed
  ASSERT_NE(builder.GetLocalClipBounds(), clip_expanded_bounds);
  ASSERT_NE(builder.GetDestinationClipBounds(), clip_expanded_bounds);
  // Previous return values have not changed
  ASSERT_EQ(initial_local_bounds, clip_expanded_bounds);
  ASSERT_EQ(initial_destination_bounds, clip_expanded_bounds);
  builder.Restore();

  // save/restore returned the values to their original values
  ASSERT_EQ(builder.GetLocalClipBounds(), initial_local_bounds);
  ASSERT_EQ(builder.GetDestinationClipBounds(), initial_destination_bounds);

  builder.Save();
  builder.Scale(2, 2);
  SkRect scaled_expanded_bounds = SkRect::MakeLTRB(5, 5.5, 10.5, 13);
  ASSERT_EQ(builder.GetLocalClipBounds(), scaled_expanded_bounds);
  // Destination bounds are unaffected by transform
  ASSERT_EQ(builder.GetDestinationClipBounds(), clip_expanded_bounds);
  builder.Restore();

  // save/restore returned the values to their original values
  ASSERT_EQ(builder.GetLocalClipBounds(), initial_local_bounds);
  ASSERT_EQ(builder.GetDestinationClipBounds(), initial_destination_bounds);
}

TEST_F(DisplayListTest, ClipRectAffectsClipBoundsWithMatrix) {
  DisplayListBuilder builder;
  SkRect clip_bounds_1 = SkRect::MakeLTRB(0, 0, 10, 10);
  SkRect clip_bounds_2 = SkRect::MakeLTRB(10, 10, 20, 20);
  builder.Save();
  builder.ClipRect(clip_bounds_1, ClipOp::kIntersect, false);
  builder.Translate(10, 0);
  builder.ClipRect(clip_bounds_1, ClipOp::kIntersect, false);
  ASSERT_TRUE(builder.GetDestinationClipBounds().isEmpty());
  builder.Restore();

  builder.Save();
  builder.ClipRect(clip_bounds_1, ClipOp::kIntersect, false);
  builder.Translate(-10, -10);
  builder.ClipRect(clip_bounds_2, ClipOp::kIntersect, false);
  ASSERT_EQ(builder.GetDestinationClipBounds(), clip_bounds_1);
  builder.Restore();
}

TEST_F(DisplayListTest, ClipRRectAffectsClipBounds) {
  DisplayListBuilder builder;
  SkRect clip_bounds = SkRect::MakeLTRB(10.2, 11.3, 20.4, 25.7);
  SkRRect clip = SkRRect::MakeRectXY(clip_bounds, 3, 2);
  builder.ClipRRect(clip, ClipOp::kIntersect, false);

  // Save initial return values for testing restored values
  SkRect initial_local_bounds = builder.GetLocalClipBounds();
  SkRect initial_destination_bounds = builder.GetDestinationClipBounds();
  ASSERT_EQ(initial_local_bounds, clip_bounds);
  ASSERT_EQ(initial_destination_bounds, clip_bounds);

  builder.Save();
  builder.ClipRect(SkRect{0, 0, 15, 15}, ClipOp::kIntersect, false);
  // Both clip bounds have changed
  ASSERT_NE(builder.GetLocalClipBounds(), clip_bounds);
  ASSERT_NE(builder.GetDestinationClipBounds(), clip_bounds);
  // Previous return values have not changed
  ASSERT_EQ(initial_local_bounds, clip_bounds);
  ASSERT_EQ(initial_destination_bounds, clip_bounds);
  builder.Restore();

  // save/restore returned the values to their original values
  ASSERT_EQ(builder.GetLocalClipBounds(), initial_local_bounds);
  ASSERT_EQ(builder.GetDestinationClipBounds(), initial_destination_bounds);

  builder.Save();
  builder.Scale(2, 2);
  SkRect scaled_clip_bounds = SkRect::MakeLTRB(5.1, 5.65, 10.2, 12.85);
  ASSERT_EQ(builder.GetLocalClipBounds(), scaled_clip_bounds);
  // Destination bounds are unaffected by transform
  ASSERT_EQ(builder.GetDestinationClipBounds(), clip_bounds);
  builder.Restore();

  // save/restore returned the values to their original values
  ASSERT_EQ(builder.GetLocalClipBounds(), initial_local_bounds);
  ASSERT_EQ(builder.GetDestinationClipBounds(), initial_destination_bounds);
}

TEST_F(DisplayListTest, ClipRRectDoAAAffectsClipBounds) {
  DisplayListBuilder builder;
  SkRect clip_bounds = SkRect::MakeLTRB(10.2, 11.3, 20.4, 25.7);
  SkRect clip_expanded_bounds = SkRect::MakeLTRB(10, 11, 21, 26);
  SkRRect clip = SkRRect::MakeRectXY(clip_bounds, 3, 2);
  builder.ClipRRect(clip, ClipOp::kIntersect, true);

  // Save initial return values for testing restored values
  SkRect initial_local_bounds = builder.GetLocalClipBounds();
  SkRect initial_destination_bounds = builder.GetDestinationClipBounds();
  ASSERT_EQ(initial_local_bounds, clip_expanded_bounds);
  ASSERT_EQ(initial_destination_bounds, clip_expanded_bounds);

  builder.Save();
  builder.ClipRect(SkRect{0, 0, 15, 15}, ClipOp::kIntersect, true);
  // Both clip bounds have changed
  ASSERT_NE(builder.GetLocalClipBounds(), clip_expanded_bounds);
  ASSERT_NE(builder.GetDestinationClipBounds(), clip_expanded_bounds);
  // Previous return values have not changed
  ASSERT_EQ(initial_local_bounds, clip_expanded_bounds);
  ASSERT_EQ(initial_destination_bounds, clip_expanded_bounds);
  builder.Restore();

  // save/restore returned the values to their original values
  ASSERT_EQ(builder.GetLocalClipBounds(), initial_local_bounds);
  ASSERT_EQ(builder.GetDestinationClipBounds(), initial_destination_bounds);

  builder.Save();
  builder.Scale(2, 2);
  SkRect scaled_expanded_bounds = SkRect::MakeLTRB(5, 5.5, 10.5, 13);
  ASSERT_EQ(builder.GetLocalClipBounds(), scaled_expanded_bounds);
  // Destination bounds are unaffected by transform
  ASSERT_EQ(builder.GetDestinationClipBounds(), clip_expanded_bounds);
  builder.Restore();

  // save/restore returned the values to their original values
  ASSERT_EQ(builder.GetLocalClipBounds(), initial_local_bounds);
  ASSERT_EQ(builder.GetDestinationClipBounds(), initial_destination_bounds);
}

TEST_F(DisplayListTest, ClipRRectAffectsClipBoundsWithMatrix) {
  DisplayListBuilder builder;
  SkRect clip_bounds_1 = SkRect::MakeLTRB(0, 0, 10, 10);
  SkRect clip_bounds_2 = SkRect::MakeLTRB(10, 10, 20, 20);
  SkRRect clip1 = SkRRect::MakeRectXY(clip_bounds_1, 3, 2);
  SkRRect clip2 = SkRRect::MakeRectXY(clip_bounds_2, 3, 2);

  builder.Save();
  builder.ClipRRect(clip1, ClipOp::kIntersect, false);
  builder.Translate(10, 0);
  builder.ClipRRect(clip1, ClipOp::kIntersect, false);
  ASSERT_TRUE(builder.GetDestinationClipBounds().isEmpty());
  builder.Restore();

  builder.Save();
  builder.ClipRRect(clip1, ClipOp::kIntersect, false);
  builder.Translate(-10, -10);
  builder.ClipRRect(clip2, ClipOp::kIntersect, false);
  ASSERT_EQ(builder.GetDestinationClipBounds(), clip_bounds_1);
  builder.Restore();
}

TEST_F(DisplayListTest, ClipPathAffectsClipBounds) {
  DisplayListBuilder builder;
  SkPath clip = SkPath().addCircle(10.2, 11.3, 2).addCircle(20.4, 25.7, 2);
  SkRect clip_bounds = SkRect::MakeLTRB(8.2, 9.3, 22.4, 27.7);
  builder.ClipPath(clip, ClipOp::kIntersect, false);

  // Save initial return values for testing restored values
  SkRect initial_local_bounds = builder.GetLocalClipBounds();
  SkRect initial_destination_bounds = builder.GetDestinationClipBounds();
  ASSERT_EQ(initial_local_bounds, clip_bounds);
  ASSERT_EQ(initial_destination_bounds, clip_bounds);

  builder.Save();
  builder.ClipRect(SkRect{0, 0, 15, 15}, ClipOp::kIntersect, false);
  // Both clip bounds have changed
  ASSERT_NE(builder.GetLocalClipBounds(), clip_bounds);
  ASSERT_NE(builder.GetDestinationClipBounds(), clip_bounds);
  // Previous return values have not changed
  ASSERT_EQ(initial_local_bounds, clip_bounds);
  ASSERT_EQ(initial_destination_bounds, clip_bounds);
  builder.Restore();

  // save/restore returned the values to their original values
  ASSERT_EQ(builder.GetLocalClipBounds(), initial_local_bounds);
  ASSERT_EQ(builder.GetDestinationClipBounds(), initial_destination_bounds);

  builder.Save();
  builder.Scale(2, 2);
  SkRect scaled_clip_bounds = SkRect::MakeLTRB(4.1, 4.65, 11.2, 13.85);
  ASSERT_EQ(builder.GetLocalClipBounds(), scaled_clip_bounds);
  // Destination bounds are unaffected by transform
  ASSERT_EQ(builder.GetDestinationClipBounds(), clip_bounds);
  builder.Restore();

  // save/restore returned the values to their original values
  ASSERT_EQ(builder.GetLocalClipBounds(), initial_local_bounds);
  ASSERT_EQ(builder.GetDestinationClipBounds(), initial_destination_bounds);
}

TEST_F(DisplayListTest, ClipPathDoAAAffectsClipBounds) {
  DisplayListBuilder builder;
  SkPath clip = SkPath().addCircle(10.2, 11.3, 2).addCircle(20.4, 25.7, 2);
  SkRect clip_expanded_bounds = SkRect::MakeLTRB(8, 9, 23, 28);
  builder.ClipPath(clip, ClipOp::kIntersect, true);

  // Save initial return values for testing restored values
  SkRect initial_local_bounds = builder.GetLocalClipBounds();
  SkRect initial_destination_bounds = builder.GetDestinationClipBounds();
  ASSERT_EQ(initial_local_bounds, clip_expanded_bounds);
  ASSERT_EQ(initial_destination_bounds, clip_expanded_bounds);

  builder.Save();
  builder.ClipRect(SkRect{0, 0, 15, 15}, ClipOp::kIntersect, true);
  // Both clip bounds have changed
  ASSERT_NE(builder.GetLocalClipBounds(), clip_expanded_bounds);
  ASSERT_NE(builder.GetDestinationClipBounds(), clip_expanded_bounds);
  // Previous return values have not changed
  ASSERT_EQ(initial_local_bounds, clip_expanded_bounds);
  ASSERT_EQ(initial_destination_bounds, clip_expanded_bounds);
  builder.Restore();

  // save/restore returned the values to their original values
  ASSERT_EQ(builder.GetLocalClipBounds(), initial_local_bounds);
  ASSERT_EQ(builder.GetDestinationClipBounds(), initial_destination_bounds);

  builder.Save();
  builder.Scale(2, 2);
  SkRect scaled_expanded_bounds = SkRect::MakeLTRB(4, 4.5, 11.5, 14);
  ASSERT_EQ(builder.GetLocalClipBounds(), scaled_expanded_bounds);
  // Destination bounds are unaffected by transform
  ASSERT_EQ(builder.GetDestinationClipBounds(), clip_expanded_bounds);
  builder.Restore();

  // save/restore returned the values to their original values
  ASSERT_EQ(builder.GetLocalClipBounds(), initial_local_bounds);
  ASSERT_EQ(builder.GetDestinationClipBounds(), initial_destination_bounds);
}

TEST_F(DisplayListTest, ClipPathAffectsClipBoundsWithMatrix) {
  DisplayListBuilder builder;
  SkRect clip_bounds = SkRect::MakeLTRB(0, 0, 10, 10);
  SkPath clip1 = SkPath().addCircle(2.5, 2.5, 2.5).addCircle(7.5, 7.5, 2.5);
  SkPath clip2 = SkPath().addCircle(12.5, 12.5, 2.5).addCircle(17.5, 17.5, 2.5);

  builder.Save();
  builder.ClipPath(clip1, ClipOp::kIntersect, false);
  builder.Translate(10, 0);
  builder.ClipPath(clip1, ClipOp::kIntersect, false);
  ASSERT_TRUE(builder.GetDestinationClipBounds().isEmpty());
  builder.Restore();

  builder.Save();
  builder.ClipPath(clip1, ClipOp::kIntersect, false);
  builder.Translate(-10, -10);
  builder.ClipPath(clip2, ClipOp::kIntersect, false);
  ASSERT_EQ(builder.GetDestinationClipBounds(), clip_bounds);
  builder.Restore();
}

TEST_F(DisplayListTest, DiffClipRectDoesNotAffectClipBounds) {
  DisplayListBuilder builder;
  SkRect diff_clip = SkRect::MakeLTRB(0, 0, 15, 15);
  SkRect clip_bounds = SkRect::MakeLTRB(10.2, 11.3, 20.4, 25.7);
  builder.ClipRect(clip_bounds, ClipOp::kIntersect, false);

  // Save initial return values for testing after kDifference clip
  SkRect initial_local_bounds = builder.GetLocalClipBounds();
  SkRect initial_destination_bounds = builder.GetDestinationClipBounds();
  ASSERT_EQ(initial_local_bounds, clip_bounds);
  ASSERT_EQ(initial_destination_bounds, clip_bounds);

  builder.ClipRect(diff_clip, ClipOp::kDifference, false);
  ASSERT_EQ(builder.GetLocalClipBounds(), initial_local_bounds);
  ASSERT_EQ(builder.GetDestinationClipBounds(), initial_destination_bounds);
}

TEST_F(DisplayListTest, DiffClipRRectDoesNotAffectClipBounds) {
  DisplayListBuilder builder;
  SkRRect diff_clip = SkRRect::MakeRectXY({0, 0, 15, 15}, 1, 1);
  SkRect clip_bounds = SkRect::MakeLTRB(10.2, 11.3, 20.4, 25.7);
  SkRRect clip = SkRRect::MakeRectXY({10.2, 11.3, 20.4, 25.7}, 3, 2);
  builder.ClipRRect(clip, ClipOp::kIntersect, false);

  // Save initial return values for testing after kDifference clip
  SkRect initial_local_bounds = builder.GetLocalClipBounds();
  SkRect initial_destination_bounds = builder.GetDestinationClipBounds();
  ASSERT_EQ(initial_local_bounds, clip_bounds);
  ASSERT_EQ(initial_destination_bounds, clip_bounds);

  builder.ClipRRect(diff_clip, ClipOp::kDifference, false);
  ASSERT_EQ(builder.GetLocalClipBounds(), initial_local_bounds);
  ASSERT_EQ(builder.GetDestinationClipBounds(), initial_destination_bounds);
}

TEST_F(DisplayListTest, DiffClipPathDoesNotAffectClipBounds) {
  DisplayListBuilder builder;
  SkPath diff_clip = SkPath().addRect({0, 0, 15, 15});
  SkPath clip = SkPath().addCircle(10.2, 11.3, 2).addCircle(20.4, 25.7, 2);
  SkRect clip_bounds = SkRect::MakeLTRB(8.2, 9.3, 22.4, 27.7);
  builder.ClipPath(clip, ClipOp::kIntersect, false);

  // Save initial return values for testing after kDifference clip
  SkRect initial_local_bounds = builder.GetLocalClipBounds();
  SkRect initial_destination_bounds = builder.GetDestinationClipBounds();
  ASSERT_EQ(initial_local_bounds, clip_bounds);
  ASSERT_EQ(initial_destination_bounds, clip_bounds);

  builder.ClipPath(diff_clip, ClipOp::kDifference, false);
  ASSERT_EQ(builder.GetLocalClipBounds(), initial_local_bounds);
  ASSERT_EQ(builder.GetDestinationClipBounds(), initial_destination_bounds);
}

TEST_F(DisplayListTest, ClipPathWithInvertFillTypeDoesNotAffectClipBounds) {
  SkRect cull_rect = SkRect::MakeLTRB(0, 0, 100.0, 100.0);
  DisplayListBuilder builder(cull_rect);
  SkPath clip = SkPath().addCircle(10.2, 11.3, 2).addCircle(20.4, 25.7, 2);
  clip.setFillType(SkPathFillType::kInverseWinding);
  builder.ClipPath(clip, ClipOp::kIntersect, false);

  ASSERT_EQ(builder.GetLocalClipBounds(), cull_rect);
  ASSERT_EQ(builder.GetDestinationClipBounds(), cull_rect);
}

TEST_F(DisplayListTest, DiffClipPathWithInvertFillTypeAffectsClipBounds) {
  SkRect cull_rect = SkRect::MakeLTRB(0, 0, 100.0, 100.0);
  DisplayListBuilder builder(cull_rect);
  SkPath clip = SkPath().addCircle(10.2, 11.3, 2).addCircle(20.4, 25.7, 2);
  clip.setFillType(SkPathFillType::kInverseWinding);
  SkRect clip_bounds = SkRect::MakeLTRB(8.2, 9.3, 22.4, 27.7);
  builder.ClipPath(clip, ClipOp::kDifference, false);

  ASSERT_EQ(builder.GetLocalClipBounds(), clip_bounds);
  ASSERT_EQ(builder.GetDestinationClipBounds(), clip_bounds);
}

TEST_F(DisplayListTest, FlatDrawPointsProducesBounds) {
  SkPoint horizontal_points[2] = {{10, 10}, {20, 10}};
  SkPoint vertical_points[2] = {{10, 10}, {10, 20}};
  {
    DisplayListBuilder builder;
    builder.DrawPoints(PointMode::kPolygon, 2, horizontal_points, DlPaint());
    SkRect bounds = builder.Build()->bounds();
    EXPECT_TRUE(bounds.contains(10, 10));
    EXPECT_TRUE(bounds.contains(20, 10));
    EXPECT_GE(bounds.width(), 10);
  }
  {
    DisplayListBuilder builder;
    builder.DrawPoints(PointMode::kPolygon, 2, vertical_points, DlPaint());
    SkRect bounds = builder.Build()->bounds();
    EXPECT_TRUE(bounds.contains(10, 10));
    EXPECT_TRUE(bounds.contains(10, 20));
    EXPECT_GE(bounds.height(), 10);
  }
  {
    DisplayListBuilder builder;
    builder.DrawPoints(PointMode::kPoints, 1, horizontal_points, DlPaint());
    SkRect bounds = builder.Build()->bounds();
    EXPECT_TRUE(bounds.contains(10, 10));
  }
  {
    DisplayListBuilder builder;
    DlPaint paint;
    paint.setStrokeWidth(2);
    builder.DrawPoints(PointMode::kPolygon, 2, horizontal_points, paint);
    SkRect bounds = builder.Build()->bounds();
    EXPECT_TRUE(bounds.contains(10, 10));
    EXPECT_TRUE(bounds.contains(20, 10));
    EXPECT_EQ(bounds, SkRect::MakeLTRB(9, 9, 21, 11));
  }
  {
    DisplayListBuilder builder;
    DlPaint paint;
    paint.setStrokeWidth(2);
    builder.DrawPoints(PointMode::kPolygon, 2, vertical_points, paint);
    SkRect bounds = builder.Build()->bounds();
    EXPECT_TRUE(bounds.contains(10, 10));
    EXPECT_TRUE(bounds.contains(10, 20));
    EXPECT_EQ(bounds, SkRect::MakeLTRB(9, 9, 11, 21));
  }
  {
    DisplayListBuilder builder;
    DlPaint paint;
    paint.setStrokeWidth(2);
    builder.DrawPoints(PointMode::kPoints, 1, horizontal_points, paint);
    SkRect bounds = builder.Build()->bounds();
    EXPECT_TRUE(bounds.contains(10, 10));
    EXPECT_EQ(bounds, SkRect::MakeLTRB(9, 9, 11, 11));
  }
}

#define TEST_RTREE(rtree, query, expected_rects, expected_indices) \
  test_rtree(rtree, query, expected_rects, expected_indices, __FILE__, __LINE__)

static void test_rtree(const sk_sp<const DlRTree>& rtree,
                       const DlRect& query,
                       std::vector<DlRect> expected_rects,
                       const std::vector<int>& expected_indices,
                       const std::string& file,
                       int line) {
  std::vector<int> indices;
  auto label = "from " + file + ":" + std::to_string(line);
  rtree->search(query, &indices);
  EXPECT_EQ(indices, expected_indices) << label;
  EXPECT_EQ(indices.size(), expected_indices.size()) << label;
  std::list<DlRect> rects = rtree->searchAndConsolidateRects(query, false);
  // ASSERT_EQ(rects.size(), expected_indices.size());
  auto iterator = rects.cbegin();
  for (int i : expected_indices) {
    ASSERT_TRUE(iterator != rects.cend()) << label;
    EXPECT_EQ(*iterator++, expected_rects[i]) << label;
  }
}

TEST_F(DisplayListTest, RTreeOfSimpleScene) {
  DisplayListBuilder builder(/*prepare_rtree=*/true);
  std::vector<DlRect> rects = {
      DlRect::MakeLTRB(10, 10, 20, 20),
      DlRect::MakeLTRB(50, 50, 60, 60),
  };
  builder.DrawRect(rects[0], DlPaint());
  builder.DrawRect(rects[1], DlPaint());
  auto display_list = builder.Build();
  auto rtree = display_list->rtree();

  // Missing all drawRect calls
  TEST_RTREE(rtree, DlRect::MakeLTRB(5, 5, 10, 10), rects, {});
  TEST_RTREE(rtree, DlRect::MakeLTRB(20, 20, 25, 25), rects, {});
  TEST_RTREE(rtree, DlRect::MakeLTRB(45, 45, 50, 50), rects, {});
  TEST_RTREE(rtree, DlRect::MakeLTRB(60, 60, 65, 65), rects, {});

  // Hitting just 1 of the drawRects
  TEST_RTREE(rtree, DlRect::MakeLTRB(5, 5, 11, 11), rects, {0});
  TEST_RTREE(rtree, DlRect::MakeLTRB(19, 19, 25, 25), rects, {0});
  TEST_RTREE(rtree, DlRect::MakeLTRB(45, 45, 51, 51), rects, {1});
  TEST_RTREE(rtree, DlRect::MakeLTRB(59, 59, 65, 65), rects, {1});

  // Hitting both drawRect calls
  TEST_RTREE(rtree, DlRect::MakeLTRB(19, 19, 51, 51), rects,
             std::vector<int>({0, 1}));
}

TEST_F(DisplayListTest, RTreeOfSaveRestoreScene) {
  DisplayListBuilder builder(/*prepare_rtree=*/true);
  builder.DrawRect(DlRect::MakeLTRB(10, 10, 20, 20), DlPaint());
  builder.Save();
  builder.DrawRect(DlRect::MakeLTRB(50, 50, 60, 60), DlPaint());
  builder.Restore();
  auto display_list = builder.Build();
  auto rtree = display_list->rtree();
  std::vector<DlRect> rects = {
      DlRect::MakeLTRB(10, 10, 20, 20),
      DlRect::MakeLTRB(50, 50, 60, 60),
  };

  // Missing all drawRect calls
  TEST_RTREE(rtree, DlRect::MakeLTRB(5, 5, 10, 10), rects, {});
  TEST_RTREE(rtree, DlRect::MakeLTRB(20, 20, 25, 25), rects, {});
  TEST_RTREE(rtree, DlRect::MakeLTRB(45, 45, 50, 50), rects, {});
  TEST_RTREE(rtree, DlRect::MakeLTRB(60, 60, 65, 65), rects, {});

  // Hitting just 1 of the drawRects
  TEST_RTREE(rtree, DlRect::MakeLTRB(5, 5, 11, 11), rects, {0});
  TEST_RTREE(rtree, DlRect::MakeLTRB(19, 19, 25, 25), rects, {0});
  TEST_RTREE(rtree, DlRect::MakeLTRB(45, 45, 51, 51), rects, {1});
  TEST_RTREE(rtree, DlRect::MakeLTRB(59, 59, 65, 65), rects, {1});

  // Hitting both drawRect calls
  TEST_RTREE(rtree, DlRect::MakeLTRB(19, 19, 51, 51), rects,
             std::vector<int>({0, 1}));
}

TEST_F(DisplayListTest, RTreeOfSaveLayerFilterScene) {
  DisplayListBuilder builder(/*prepare_rtree=*/true);
  // blur filter with sigma=1 expands by 3 on all sides
  auto filter = DlBlurImageFilter(1.0, 1.0, DlTileMode::kClamp);
  DlPaint default_paint = DlPaint();
  DlPaint filter_paint = DlPaint().setImageFilter(&filter);
  builder.DrawRect(DlRect::MakeLTRB(10, 10, 20, 20), default_paint);
  builder.SaveLayer(nullptr, &filter_paint);
  // the following rectangle will be expanded to 50,50,60,60
  // by the SaveLayer filter during the restore operation
  builder.DrawRect(DlRect::MakeLTRB(53, 53, 57, 57), default_paint);
  builder.Restore();
  auto display_list = builder.Build();
  auto rtree = display_list->rtree();
  std::vector<DlRect> rects = {
      DlRect::MakeLTRB(10, 10, 20, 20),
      DlRect::MakeLTRB(50, 50, 60, 60),
  };

  // Missing all drawRect calls
  TEST_RTREE(rtree, DlRect::MakeLTRB(5, 5, 10, 10), rects, {});
  TEST_RTREE(rtree, DlRect::MakeLTRB(20, 20, 25, 25), rects, {});
  TEST_RTREE(rtree, DlRect::MakeLTRB(45, 45, 50, 50), rects, {});
  TEST_RTREE(rtree, DlRect::MakeLTRB(60, 60, 65, 65), rects, {});

  // Hitting just 1 of the drawRects
  TEST_RTREE(rtree, DlRect::MakeLTRB(5, 5, 11, 11), rects, {0});
  TEST_RTREE(rtree, DlRect::MakeLTRB(19, 19, 25, 25), rects, {0});
  TEST_RTREE(rtree, DlRect::MakeLTRB(45, 45, 51, 51), rects, {1});
  TEST_RTREE(rtree, DlRect::MakeLTRB(59, 59, 65, 65), rects, {1});

  // Hitting both drawRect calls
  auto expected_indices = std::vector<int>{0, 1};
  TEST_RTREE(rtree, DlRect::MakeLTRB(19, 19, 51, 51), rects, expected_indices);
}

TEST_F(DisplayListTest, NestedDisplayListRTreesAreSparse) {
  DisplayListBuilder nested_dl_builder(/**prepare_rtree=*/true);
  nested_dl_builder.DrawRect(DlRect::MakeLTRB(10, 10, 20, 20), DlPaint());
  nested_dl_builder.DrawRect(DlRect::MakeLTRB(50, 50, 60, 60), DlPaint());
  auto nested_display_list = nested_dl_builder.Build();

  DisplayListBuilder builder(/**prepare_rtree=*/true);
  builder.DrawDisplayList(nested_display_list);
  auto display_list = builder.Build();

  auto rtree = display_list->rtree();
  std::vector<DlRect> rects = {
      DlRect::MakeLTRB(10, 10, 20, 20),
      DlRect::MakeLTRB(50, 50, 60, 60),
  };

  // Hitting both sub-dl drawRect calls
  TEST_RTREE(rtree, DlRect::MakeLTRB(19, 19, 51, 51), rects,
             std::vector<int>({0, 1}));
}

TEST_F(DisplayListTest, RemoveUnnecessarySaveRestorePairs) {
  {
    DisplayListBuilder builder;
    builder.DrawRect(SkRect{10, 10, 20, 20}, DlPaint());
    builder.Save();  // This save op is unnecessary
    builder.DrawRect(SkRect{50, 50, 60, 60}, DlPaint());
    builder.Restore();

    DisplayListBuilder builder2;
    builder2.DrawRect(SkRect{10, 10, 20, 20}, DlPaint());
    builder2.DrawRect(SkRect{50, 50, 60, 60}, DlPaint());
    ASSERT_TRUE(DisplayListsEQ_Verbose(builder.Build(), builder2.Build()));
  }

  {
    DisplayListBuilder builder;
    builder.DrawRect(SkRect{10, 10, 20, 20}, DlPaint());
    builder.Save();
    {
      builder.Translate(1.0, 1.0);
      builder.Save();
      {  //
        builder.DrawRect(SkRect{50, 50, 60, 60}, DlPaint());
      }
      builder.Restore();
    }
    builder.Restore();

    DisplayListBuilder builder2;
    builder2.DrawRect(SkRect{10, 10, 20, 20}, DlPaint());
    builder2.Save();
    {  //
      builder2.Translate(1.0, 1.0);
      {  //
        builder2.DrawRect(SkRect{50, 50, 60, 60}, DlPaint());
      }
    }
    builder2.Restore();
    ASSERT_TRUE(DisplayListsEQ_Verbose(builder.Build(), builder2.Build()));
  }
}

TEST_F(DisplayListTest, CollapseMultipleNestedSaveRestore) {
  DisplayListBuilder builder1;
  builder1.Save();
  {
    builder1.Save();
    {
      builder1.Save();
      {
        builder1.Translate(10, 10);
        builder1.Scale(2, 2);
        builder1.ClipRect(SkRect{10, 10, 20, 20}, ClipOp::kIntersect, false);
        builder1.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
      }
      builder1.Restore();
    }
    builder1.Restore();
  }
  builder1.Restore();
  auto display_list1 = builder1.Build();

  DisplayListBuilder builder2;
  builder2.Save();
  {
    builder2.Translate(10, 10);
    builder2.Scale(2, 2);
    builder2.ClipRect(SkRect{10, 10, 20, 20}, ClipOp::kIntersect, false);
    builder2.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
  }
  builder2.Restore();
  auto display_list2 = builder2.Build();

  ASSERT_TRUE(DisplayListsEQ_Verbose(display_list1, display_list2));
}

TEST_F(DisplayListTest, CollapseNestedSaveAndSaveLayerRestore) {
  DisplayListBuilder builder1;
  builder1.Save();
  {
    builder1.SaveLayer(nullptr, nullptr);
    {
      builder1.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
      builder1.Scale(2, 2);
    }
    builder1.Restore();
  }
  builder1.Restore();
  auto display_list1 = builder1.Build();

  DisplayListBuilder builder2;
  builder2.SaveLayer(nullptr, nullptr);
  {
    builder2.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
    builder2.Scale(2, 2);
  }
  builder2.Restore();
  auto display_list2 = builder2.Build();

  ASSERT_TRUE(DisplayListsEQ_Verbose(display_list1, display_list2));
}

TEST_F(DisplayListTest, RemoveUnnecessarySaveRestorePairsInSetPaint) {
  SkRect build_bounds = SkRect::MakeLTRB(-100, -100, 200, 200);
  SkRect rect = SkRect::MakeLTRB(30, 30, 70, 70);
  // clang-format off
  const float alpha_matrix[] = {
      0, 0, 0, 0, 0,
      0, 1, 0, 0, 0,
      0, 0, 1, 0, 0,
      0, 0, 0, 0, 1,
  };
  // clang-format on
  auto alpha_color_filter = DlColorFilter::MakeMatrix(alpha_matrix);
  // Making sure hiding a problematic ColorFilter as an ImageFilter
  // will generate the same behavior as setting it as a ColorFilter

  DlColorFilterImageFilter color_filter_image_filter(alpha_color_filter);
  {
    DisplayListBuilder builder(build_bounds);
    builder.Save();
    DlPaint paint;
    paint.setImageFilter(&color_filter_image_filter);
    builder.DrawRect(rect, paint);
    builder.Restore();
    sk_sp<DisplayList> display_list1 = builder.Build();

    DisplayListBuilder builder2(build_bounds);
    DlPaint paint2;
    paint2.setImageFilter(&color_filter_image_filter);
    builder2.DrawRect(rect, paint2);
    sk_sp<DisplayList> display_list2 = builder2.Build();
    ASSERT_TRUE(DisplayListsEQ_Verbose(display_list1, display_list2));
  }

  {
    DisplayListBuilder builder(build_bounds);
    builder.Save();
    builder.SaveLayer(&build_bounds);
    DlPaint paint;
    paint.setImageFilter(&color_filter_image_filter);
    builder.DrawRect(rect, paint);
    builder.Restore();
    builder.Restore();
    sk_sp<DisplayList> display_list1 = builder.Build();

    DisplayListBuilder builder2(build_bounds);
    builder2.SaveLayer(&build_bounds);
    DlPaint paint2;
    paint2.setImageFilter(&color_filter_image_filter);
    builder2.DrawRect(rect, paint2);
    builder2.Restore();
    sk_sp<DisplayList> display_list2 = builder2.Build();
    ASSERT_TRUE(DisplayListsEQ_Verbose(display_list1, display_list2));
  }
}

TEST_F(DisplayListTest, TransformTriggersDeferredSave) {
  DisplayListBuilder builder1;
  builder1.Save();
  {
    builder1.Save();
    {
      builder1.TransformFullPerspective(1, 0, 0, 10,   //
                                        0, 1, 0, 100,  //
                                        0, 0, 1, 0,    //
                                        0, 0, 0, 1);
      builder1.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
    }
    builder1.Restore();
    builder1.TransformFullPerspective(1, 0, 0, 10,   //
                                      0, 1, 0, 100,  //
                                      0, 0, 1, 0,    //
                                      0, 0, 0, 1);
    builder1.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
  }
  builder1.Restore();
  auto display_list1 = builder1.Build();

  DisplayListBuilder builder2;
  builder2.Save();
  {
    builder2.TransformFullPerspective(1, 0, 0, 10,   //
                                      0, 1, 0, 100,  //
                                      0, 0, 1, 0,    //
                                      0, 0, 0, 1);
    builder2.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
  }
  builder2.Restore();
  builder2.Save();
  {
    builder2.TransformFullPerspective(1, 0, 0, 10,   //
                                      0, 1, 0, 100,  //
                                      0, 0, 1, 0,    //
                                      0, 0, 0, 1);
    builder2.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
  }
  builder2.Restore();
  auto display_list2 = builder2.Build();

  ASSERT_TRUE(DisplayListsEQ_Verbose(display_list1, display_list2));
}

TEST_F(DisplayListTest, Transform2DTriggersDeferredSave) {
  DisplayListBuilder builder1;
  builder1.Save();
  {
    builder1.Save();
    {
      builder1.Transform2DAffine(0, 1, 12, 1, 0, 33);
      builder1.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
    }
    builder1.Restore();
  }
  builder1.Restore();
  auto display_list1 = builder1.Build();

  DisplayListBuilder builder2;
  builder2.Save();
  {
    builder2.Transform2DAffine(0, 1, 12, 1, 0, 33);
    builder2.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
  }
  builder2.Restore();
  auto display_list2 = builder2.Build();

  ASSERT_TRUE(DisplayListsEQ_Verbose(display_list1, display_list2));
}

TEST_F(DisplayListTest, TransformPerspectiveTriggersDeferredSave) {
  DisplayListBuilder builder1;
  builder1.Save();
  {
    builder1.Save();
    {
      builder1.TransformFullPerspective(0, 1, 0, 12,  //
                                        1, 0, 0, 33,  //
                                        3, 2, 5, 29,  //
                                        0, 0, 0, 12);
      builder1.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
    }
    builder1.Restore();
  }
  builder1.Restore();
  auto display_list1 = builder1.Build();

  DisplayListBuilder builder2;
  builder2.Save();
  {
    builder2.TransformFullPerspective(0, 1, 0, 12,  //
                                      1, 0, 0, 33,  //
                                      3, 2, 5, 29,  //
                                      0, 0, 0, 12);
    builder2.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
  }
  builder2.Restore();
  auto display_list2 = builder2.Build();

  ASSERT_TRUE(DisplayListsEQ_Verbose(display_list1, display_list2));
}

TEST_F(DisplayListTest, ResetTransformTriggersDeferredSave) {
  DisplayListBuilder builder1;
  builder1.Save();
  {
    builder1.Save();
    {
      builder1.TransformReset();
      builder1.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
    }
    builder1.Restore();
  }
  builder1.Restore();
  auto display_list1 = builder1.Build();

  DisplayListBuilder builder2;
  builder2.Save();
  {
    builder2.TransformReset();
    builder2.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
  }
  builder2.Restore();
  auto display_list2 = builder2.Build();

  ASSERT_TRUE(DisplayListsEQ_Verbose(display_list1, display_list2));
}

TEST_F(DisplayListTest, SkewTriggersDeferredSave) {
  DisplayListBuilder builder1;
  builder1.Save();
  {
    builder1.Save();
    {
      builder1.Skew(10, 10);
      builder1.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
    }
    builder1.Restore();
  }
  builder1.Restore();
  auto display_list1 = builder1.Build();

  DisplayListBuilder builder2;
  builder2.Save();
  {
    builder2.Skew(10, 10);
    builder2.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
  }
  builder2.Restore();
  auto display_list2 = builder2.Build();

  ASSERT_TRUE(DisplayListsEQ_Verbose(display_list1, display_list2));
}

TEST_F(DisplayListTest, TranslateTriggersDeferredSave) {
  DisplayListBuilder builder1;
  builder1.Save();
  {
    builder1.Save();
    {
      builder1.Translate(10, 10);
      builder1.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
    }
    builder1.Restore();
  }
  builder1.Restore();
  auto display_list1 = builder1.Build();

  DisplayListBuilder builder2;
  builder2.Save();
  {
    builder2.Translate(10, 10);
    builder2.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
  }
  builder2.Restore();
  auto display_list2 = builder2.Build();

  ASSERT_TRUE(DisplayListsEQ_Verbose(display_list1, display_list2));
}

TEST_F(DisplayListTest, ScaleTriggersDeferredSave) {
  DisplayListBuilder builder1;
  builder1.Save();
  {
    builder1.Save();
    {
      builder1.Scale(0.5, 0.5);
      builder1.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
    }
    builder1.Restore();
  }
  builder1.Restore();
  auto display_list1 = builder1.Build();

  DisplayListBuilder builder2;
  builder2.Save();
  {
    builder2.Scale(0.5, 0.5);
    builder2.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
  }
  builder2.Restore();
  auto display_list2 = builder2.Build();

  ASSERT_TRUE(DisplayListsEQ_Verbose(display_list1, display_list2));
}

TEST_F(DisplayListTest, ClipRectTriggersDeferredSave) {
  DisplayListBuilder builder1;
  builder1.Save();
  {
    builder1.Save();
    {
      builder1.ClipRect(SkRect::MakeLTRB(0, 0, 100, 100), ClipOp::kIntersect,
                        true);
      builder1.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
    }
    builder1.Restore();
    builder1.TransformFullPerspective(1, 0, 0, 0,  //
                                      0, 1, 0, 0,  //
                                      0, 0, 1, 0,  //
                                      0, 0, 0, 1);
    builder1.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
  }
  builder1.Restore();
  auto display_list1 = builder1.Build();

  DisplayListBuilder builder2;
  builder2.Save();
  {
    builder2.ClipRect(SkRect::MakeLTRB(0, 0, 100, 100), ClipOp::kIntersect,
                      true);
    builder2.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
  }
  builder2.Restore();
  builder2.TransformFullPerspective(1, 0, 0, 0,  //
                                    0, 1, 0, 0,  //
                                    0, 0, 1, 0,  //
                                    0, 0, 0, 1);
  builder2.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
  auto display_list2 = builder2.Build();

  ASSERT_TRUE(DisplayListsEQ_Verbose(display_list1, display_list2));
}

TEST_F(DisplayListTest, ClipRRectTriggersDeferredSave) {
  DisplayListBuilder builder1;
  builder1.Save();
  {
    builder1.Save();
    {
      builder1.ClipRRect(kTestSkRRect, ClipOp::kIntersect, true);

      builder1.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
    }
    builder1.Restore();
    builder1.TransformFullPerspective(1, 0, 0, 0,  //
                                      0, 1, 0, 0,  //
                                      0, 0, 1, 0,  //
                                      0, 0, 0, 1);
    builder1.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
  }
  builder1.Restore();
  auto display_list1 = builder1.Build();

  DisplayListBuilder builder2;
  builder2.Save();
  builder2.ClipRRect(kTestSkRRect, ClipOp::kIntersect, true);

  builder2.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
  builder2.Restore();
  builder2.TransformFullPerspective(1, 0, 0, 0,  //
                                    0, 1, 0, 0,  //
                                    0, 0, 1, 0,  //
                                    0, 0, 0, 1);
  builder2.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
  auto display_list2 = builder2.Build();

  ASSERT_TRUE(DisplayListsEQ_Verbose(display_list1, display_list2));
}

TEST_F(DisplayListTest, ClipPathTriggersDeferredSave) {
  DisplayListBuilder builder1;
  builder1.Save();
  {
    builder1.Save();
    {
      builder1.ClipPath(kTestPath1, ClipOp::kIntersect, true);
      builder1.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
    }
    builder1.Restore();
    builder1.TransformFullPerspective(1, 0, 0, 0,  //
                                      0, 1, 0, 0,  //
                                      0, 0, 1, 0,  //
                                      0, 0, 0, 1);
    builder1.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
  }
  builder1.Restore();
  auto display_list1 = builder1.Build();

  DisplayListBuilder builder2;
  builder2.Save();
  {
    builder2.ClipPath(kTestPath1, ClipOp::kIntersect, true);
    builder2.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
  }
  builder2.Restore();
  builder2.TransformFullPerspective(1, 0, 0, 0,  //
                                    0, 1, 0, 0,  //
                                    0, 0, 1, 0,  //
                                    0, 0, 0, 1);
  builder2.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
  auto display_list2 = builder2.Build();

  ASSERT_TRUE(DisplayListsEQ_Verbose(display_list1, display_list2));
}

TEST_F(DisplayListTest, NOPTranslateDoesNotTriggerDeferredSave) {
  DisplayListBuilder builder1;
  builder1.Save();
  {
    builder1.Save();
    {
      builder1.Translate(0, 0);
      builder1.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
    }
    builder1.Restore();
    builder1.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
  }
  builder1.Restore();
  auto display_list1 = builder1.Build();

  DisplayListBuilder builder2;
  builder2.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
  builder2.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
  auto display_list2 = builder2.Build();

  ASSERT_TRUE(DisplayListsEQ_Verbose(display_list1, display_list2));
}

TEST_F(DisplayListTest, NOPScaleDoesNotTriggerDeferredSave) {
  DisplayListBuilder builder1;
  builder1.Save();
  {
    builder1.Save();
    {
      builder1.Scale(1.0, 1.0);
      builder1.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
    }
    builder1.Restore();
    builder1.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
  }
  builder1.Restore();
  auto display_list1 = builder1.Build();

  DisplayListBuilder builder2;
  builder2.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
  builder2.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
  auto display_list2 = builder2.Build();

  ASSERT_TRUE(DisplayListsEQ_Verbose(display_list1, display_list2));
}

TEST_F(DisplayListTest, NOPRotationDoesNotTriggerDeferredSave) {
  DisplayListBuilder builder1;
  builder1.Save();
  {
    builder1.Save();
    {
      builder1.Rotate(360);
      builder1.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
    }
    builder1.Restore();
    builder1.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
  }
  builder1.Restore();
  auto display_list1 = builder1.Build();

  DisplayListBuilder builder2;
  builder2.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
  builder2.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
  auto display_list2 = builder2.Build();

  ASSERT_TRUE(DisplayListsEQ_Verbose(display_list1, display_list2));
}

TEST_F(DisplayListTest, NOPSkewDoesNotTriggerDeferredSave) {
  DisplayListBuilder builder1;
  builder1.Save();
  {
    builder1.Save();
    {
      builder1.Skew(0, 0);
      builder1.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
    }
    builder1.Restore();
    builder1.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
  }
  builder1.Restore();
  auto display_list1 = builder1.Build();

  DisplayListBuilder builder2;
  builder2.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
  builder2.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
  auto display_list2 = builder2.Build();

  ASSERT_TRUE(DisplayListsEQ_Verbose(display_list1, display_list2));
}

TEST_F(DisplayListTest, NOPTransformDoesNotTriggerDeferredSave) {
  DisplayListBuilder builder1;
  builder1.Save();
  {
    builder1.Save();
    {
      builder1.TransformFullPerspective(1, 0, 0, 0,  //
                                        0, 1, 0, 0,  //
                                        0, 0, 1, 0,  //
                                        0, 0, 0, 1);
      builder1.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
    }
    builder1.Restore();
    builder1.TransformFullPerspective(1, 0, 0, 0,  //
                                      0, 1, 0, 0,  //
                                      0, 0, 1, 0,  //
                                      0, 0, 0, 1);
    builder1.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
  }
  builder1.Restore();
  auto display_list1 = builder1.Build();

  DisplayListBuilder builder2;
  builder2.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
  builder2.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
  auto display_list2 = builder2.Build();

  ASSERT_TRUE(DisplayListsEQ_Verbose(display_list1, display_list2));
}

TEST_F(DisplayListTest, NOPTransform2DDoesNotTriggerDeferredSave) {
  DisplayListBuilder builder1;
  builder1.Save();
  {
    builder1.Save();
    {
      builder1.Transform2DAffine(1, 0, 0, 0, 1, 0);
      builder1.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
    }
    builder1.Restore();
    builder1.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
  }
  builder1.Restore();
  auto display_list1 = builder1.Build();

  DisplayListBuilder builder2;
  builder2.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
  builder2.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
  auto display_list2 = builder2.Build();

  ASSERT_TRUE(DisplayListsEQ_Verbose(display_list1, display_list2));
}

TEST_F(DisplayListTest, NOPTransformFullPerspectiveDoesNotTriggerDeferredSave) {
  {
    DisplayListBuilder builder1;
    builder1.Save();
    {
      builder1.Save();
      {
        builder1.TransformFullPerspective(1, 0, 0, 0,  //
                                          0, 1, 0, 0,  //
                                          0, 0, 1, 0,  //
                                          0, 0, 0, 1);
        builder1.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
      }
      builder1.Restore();
      builder1.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
    }
    builder1.Restore();
    auto display_list1 = builder1.Build();

    DisplayListBuilder builder2;
    builder2.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
    builder2.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
    auto display_list2 = builder2.Build();

    ASSERT_TRUE(DisplayListsEQ_Verbose(display_list1, display_list2));
  }

  {
    DisplayListBuilder builder1;
    builder1.Save();
    {
      builder1.Save();
      {
        builder1.TransformFullPerspective(1, 0, 0, 0,  //
                                          0, 1, 0, 0,  //
                                          0, 0, 1, 0,  //
                                          0, 0, 0, 1);
        builder1.TransformReset();
        builder1.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
      }
      builder1.Restore();
      builder1.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
    }
    builder1.Restore();
    auto display_list1 = builder1.Build();

    DisplayListBuilder builder2;
    builder2.Save();
    {
      builder2.TransformReset();
      builder2.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
    }
    builder2.Restore();
    builder2.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
    auto display_list2 = builder2.Build();

    ASSERT_TRUE(DisplayListsEQ_Verbose(display_list1, display_list2));
  }
}

TEST_F(DisplayListTest, NOPClipDoesNotTriggerDeferredSave) {
  DisplayListBuilder builder1;
  builder1.Save();
  {
    builder1.Save();
    {
      builder1.ClipRect(SkRect::MakeLTRB(0, SK_ScalarNaN, SK_ScalarNaN, 0),
                        ClipOp::kIntersect, true);
      builder1.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
    }
    builder1.Restore();
    builder1.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
  }
  builder1.Restore();
  auto display_list1 = builder1.Build();

  DisplayListBuilder builder2;
  builder2.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
  builder2.DrawRect(SkRect{0, 0, 100, 100}, DlPaint());
  auto display_list2 = builder2.Build();

  ASSERT_TRUE(DisplayListsEQ_Verbose(display_list1, display_list2));
}

TEST_F(DisplayListTest, RTreeOfClippedSaveLayerFilterScene) {
  DisplayListBuilder builder(/*prepare_rtree=*/true);
  // blur filter with sigma=1 expands by 30 on all sides
  auto filter = DlBlurImageFilter(10.0, 10.0, DlTileMode::kClamp);
  DlPaint default_paint = DlPaint();
  DlPaint filter_paint = DlPaint().setImageFilter(&filter);
  builder.DrawRect(DlRect::MakeLTRB(10, 10, 20, 20), default_paint);
  builder.ClipRect(DlRect::MakeLTRB(50, 50, 60, 60), ClipOp::kIntersect, false);
  builder.SaveLayer(nullptr, &filter_paint);
  // the following rectangle will be expanded to 23,23,87,87
  // by the SaveLayer filter during the restore operation
  // but it will then be clipped to 50,50,60,60
  builder.DrawRect(DlRect::MakeLTRB(53, 53, 57, 57), default_paint);
  builder.Restore();
  auto display_list = builder.Build();
  auto rtree = display_list->rtree();
  std::vector<DlRect> rects = {
      DlRect::MakeLTRB(10, 10, 20, 20),
      DlRect::MakeLTRB(50, 50, 60, 60),
  };

  // Missing all drawRect calls
  TEST_RTREE(rtree, DlRect::MakeLTRB(5, 5, 10, 10), rects, {});
  TEST_RTREE(rtree, DlRect::MakeLTRB(20, 20, 25, 25), rects, {});
  TEST_RTREE(rtree, DlRect::MakeLTRB(45, 45, 50, 50), rects, {});
  TEST_RTREE(rtree, DlRect::MakeLTRB(60, 60, 65, 65), rects, {});

  // Hitting just 1 of the drawRects
  TEST_RTREE(rtree, DlRect::MakeLTRB(5, 5, 11, 11), rects, {0});
  TEST_RTREE(rtree, DlRect::MakeLTRB(19, 19, 25, 25), rects, {0});
  TEST_RTREE(rtree, DlRect::MakeLTRB(45, 45, 51, 51), rects, {1});
  TEST_RTREE(rtree, DlRect::MakeLTRB(59, 59, 65, 65), rects, {1});

  // Hitting both drawRect calls
  TEST_RTREE(rtree, DlRect::MakeLTRB(19, 19, 51, 51), rects,
             std::vector<int>({0, 1}));
}

TEST_F(DisplayListTest, RTreeRenderCulling) {
  DlRect rect1 = DlRect::MakeLTRB(0, 0, 10, 10);
  DlRect rect2 = DlRect::MakeLTRB(20, 0, 30, 10);
  DlRect rect3 = DlRect::MakeLTRB(0, 20, 10, 30);
  DlRect rect4 = DlRect::MakeLTRB(20, 20, 30, 30);
  DlPaint paint1 = DlPaint().setColor(DlColor::kRed());
  DlPaint paint2 = DlPaint().setColor(DlColor::kGreen());
  DlPaint paint3 = DlPaint().setColor(DlColor::kBlue());
  DlPaint paint4 = DlPaint().setColor(DlColor::kMagenta());

  DisplayListBuilder main_builder(true);
  main_builder.DrawRect(rect1, paint1);
  main_builder.DrawRect(rect2, paint2);
  main_builder.DrawRect(rect3, paint3);
  main_builder.DrawRect(rect4, paint4);
  auto main = main_builder.Build();

  auto test = [main](DlIRect cull_rect, const sk_sp<DisplayList>& expected,
                     const std::string& label) {
    DlRect cull_rectf = DlRect::Make(cull_rect);

    {  // Test DlIRect culling
      DisplayListBuilder culling_builder;
      main->Dispatch(ToReceiver(culling_builder), cull_rect);

      EXPECT_TRUE(DisplayListsEQ_Verbose(culling_builder.Build(), expected))
          << "using cull rect " << cull_rect  //
          << " where " << label;
    }

    {  // Test DlRect culling
      DisplayListBuilder culling_builder;
      main->Dispatch(ToReceiver(culling_builder), cull_rectf);

      EXPECT_TRUE(DisplayListsEQ_Verbose(culling_builder.Build(), expected))
          << "using cull rect " << cull_rectf  //
          << " where " << label;
    }

    {  // Test using vector of culled indices
      DisplayListBuilder culling_builder;
      DlOpReceiver& receiver = ToReceiver(culling_builder);
      auto indices = main->GetCulledIndices(cull_rectf);
      for (DlIndex i : indices) {
        EXPECT_TRUE(main->Dispatch(receiver, i));
      }

      EXPECT_TRUE(DisplayListsEQ_Verbose(culling_builder.Build(), expected))
          << "using culled indices on cull rect " << cull_rectf  //
          << " where " << label;
    }
  };

  {  // No rects
    DlIRect cull_rect = DlIRect::MakeLTRB(11, 11, 19, 19);

    DisplayListBuilder expected_builder;
    auto expected = expected_builder.Build();

    test(cull_rect, expected, "no rects intersect");
  }

  {  // Rect 1
    DlIRect cull_rect = DlIRect::MakeLTRB(9, 9, 19, 19);

    DisplayListBuilder expected_builder;
    expected_builder.DrawRect(rect1, paint1);
    auto expected = expected_builder.Build();

    test(cull_rect, expected, "rect 1 intersects");
  }

  {  // Rect 2
    DlIRect cull_rect = DlIRect::MakeLTRB(11, 9, 21, 19);

    DisplayListBuilder expected_builder;
    // Unfortunately we don't cull attribute records (yet?), so we forcibly
    // record all attributes for the un-culled operations
    ToReceiver(expected_builder).setColor(paint1.getColor());
    expected_builder.DrawRect(rect2, paint2);
    auto expected = expected_builder.Build();

    test(cull_rect, expected, "rect 2 intersects");
  }

  {  // Rect 3
    DlIRect cull_rect = DlIRect::MakeLTRB(9, 11, 19, 21);

    DisplayListBuilder expected_builder;
    // Unfortunately we don't cull attribute records (yet?), so we forcibly
    // record all attributes for the un-culled operations
    ToReceiver(expected_builder).setColor(paint1.getColor());
    ToReceiver(expected_builder).setColor(paint2.getColor());
    expected_builder.DrawRect(rect3, paint3);
    auto expected = expected_builder.Build();

    test(cull_rect, expected, "rect 3 intersects");
  }

  {  // Rect 4
    DlIRect cull_rect = DlIRect::MakeLTRB(11, 11, 21, 21);

    DisplayListBuilder expected_builder;
    // Unfortunately we don't cull attribute records (yet?), so we forcibly
    // record all attributes for the un-culled operations
    ToReceiver(expected_builder).setColor(paint1.getColor());
    ToReceiver(expected_builder).setColor(paint2.getColor());
    ToReceiver(expected_builder).setColor(paint3.getColor());
    expected_builder.DrawRect(rect4, paint4);
    auto expected = expected_builder.Build();

    test(cull_rect, expected, "rect 4 intersects");
  }

  {  // All 4 rects
    DlIRect cull_rect = DlIRect::MakeLTRB(9, 9, 21, 21);

    test(cull_rect, main, "all rects intersect");
  }
}

TEST_F(DisplayListTest, DrawSaveDrawCannotInheritOpacity) {
  DisplayListBuilder builder;
  builder.DrawCircle(SkPoint{10, 10}, 5, DlPaint());
  builder.Save();
  builder.ClipRect(SkRect{0, 0, 20, 20}, DlCanvas::ClipOp::kIntersect, false);
  builder.DrawRect(SkRect{5, 5, 15, 15}, DlPaint());
  builder.Restore();
  auto display_list = builder.Build();

  ASSERT_FALSE(display_list->can_apply_group_opacity());
}

TEST_F(DisplayListTest, DrawUnorderedRect) {
  auto renderer = [](DlCanvas& canvas, DlPaint& paint, SkRect& rect) {
    canvas.DrawRect(rect, paint);
  };
  check_inverted_bounds(renderer, "DrawRect");
}

TEST_F(DisplayListTest, DrawUnorderedRoundRect) {
  auto renderer = [](DlCanvas& canvas, DlPaint& paint, SkRect& rect) {
    canvas.DrawRRect(SkRRect::MakeRectXY(rect, 2.0f, 2.0f), paint);
  };
  check_inverted_bounds(renderer, "DrawRoundRect");
}

TEST_F(DisplayListTest, DrawUnorderedOval) {
  auto renderer = [](DlCanvas& canvas, DlPaint& paint, SkRect& rect) {
    canvas.DrawOval(rect, paint);
  };
  check_inverted_bounds(renderer, "DrawOval");
}

TEST_F(DisplayListTest, DrawUnorderedRectangularPath) {
  auto renderer = [](DlCanvas& canvas, DlPaint& paint, SkRect& rect) {
    canvas.DrawPath(SkPath().addRect(rect), paint);
  };
  check_inverted_bounds(renderer, "DrawRectangularPath");
}

TEST_F(DisplayListTest, DrawUnorderedOvalPath) {
  auto renderer = [](DlCanvas& canvas, DlPaint& paint, SkRect& rect) {
    canvas.DrawPath(SkPath().addOval(rect), paint);
  };
  check_inverted_bounds(renderer, "DrawOvalPath");
}

TEST_F(DisplayListTest, DrawUnorderedRoundRectPathCW) {
  auto renderer = [](DlCanvas& canvas, DlPaint& paint, SkRect& rect) {
    SkPath path = SkPath()  //
                      .addRoundRect(rect, 2.0f, 2.0f, SkPathDirection::kCW);
    canvas.DrawPath(path, paint);
  };
  check_inverted_bounds(renderer, "DrawRoundRectPath Clockwise");
}

TEST_F(DisplayListTest, DrawUnorderedRoundRectPathCCW) {
  auto renderer = [](DlCanvas& canvas, DlPaint& paint, SkRect& rect) {
    SkPath path = SkPath()  //
                      .addRoundRect(rect, 2.0f, 2.0f, SkPathDirection::kCCW);
    canvas.DrawPath(path, paint);
  };
  check_inverted_bounds(renderer, "DrawRoundRectPath Counter-Clockwise");
}

TEST_F(DisplayListTest, NopOperationsOmittedFromRecords) {
  auto run_tests = [](const std::string& name,
                      void init(DisplayListBuilder & builder, DlPaint & paint),
                      uint32_t expected_op_count = 0u,
                      uint32_t expected_total_depth = 0u) {
    auto run_one_test =
        [init](const std::string& name,
               void build(DisplayListBuilder & builder, DlPaint & paint),
               uint32_t expected_op_count = 0u,
               uint32_t expected_total_depth = 0u) {
          DisplayListBuilder builder;
          DlPaint paint;
          init(builder, paint);
          build(builder, paint);
          auto list = builder.Build();
          if (list->op_count() != expected_op_count) {
            FML_LOG(ERROR) << *list;
          }
          ASSERT_EQ(list->op_count(), expected_op_count) << name;
          EXPECT_EQ(list->total_depth(), expected_total_depth) << name;
          ASSERT_TRUE(list->bounds().isEmpty()) << name;
        };
    run_one_test(
        name + " DrawColor",
        [](DisplayListBuilder& builder, DlPaint& paint) {
          builder.DrawColor(paint.getColor(), paint.getBlendMode());
        },
        expected_op_count, expected_total_depth);
    run_one_test(
        name + " DrawPaint",
        [](DisplayListBuilder& builder, DlPaint& paint) {
          builder.DrawPaint(paint);
        },
        expected_op_count, expected_total_depth);
    run_one_test(
        name + " DrawRect",
        [](DisplayListBuilder& builder, DlPaint& paint) {
          builder.DrawRect(SkRect{10, 10, 20, 20}, paint);
        },
        expected_op_count, expected_total_depth);
    run_one_test(
        name + " Other Draw Ops",
        [](DisplayListBuilder& builder, DlPaint& paint) {
          builder.DrawLine(SkPoint{10, 10}, SkPoint{20, 20}, paint);
          builder.DrawOval(SkRect{10, 10, 20, 20}, paint);
          builder.DrawCircle(SkPoint{50, 50}, 20, paint);
          builder.DrawRRect(SkRRect::MakeRectXY({10, 10, 20, 20}, 5, 5), paint);
          builder.DrawDRRect(SkRRect::MakeRectXY({5, 5, 100, 100}, 5, 5),
                             SkRRect::MakeRectXY({10, 10, 20, 20}, 5, 5),
                             paint);
          builder.DrawPath(kTestPath1, paint);
          builder.DrawArc(SkRect{10, 10, 20, 20}, 45, 90, true, paint);
          SkPoint pts[] = {{10, 10}, {20, 20}};
          builder.DrawPoints(PointMode::kLines, 2, pts, paint);
          builder.DrawVertices(kTestVertices1, DlBlendMode::kSrcOver, paint);
          builder.DrawImage(TestImage1, SkPoint{10, 10},
                            DlImageSampling::kLinear, &paint);
          builder.DrawImageRect(TestImage1, SkRect{0.0f, 0.0f, 10.0f, 10.0f},
                                SkRect{10.0f, 10.0f, 25.0f, 25.0f},
                                DlImageSampling::kLinear, &paint);
          builder.DrawImageNine(TestImage1, SkIRect{10, 10, 20, 20},
                                SkRect{10, 10, 100, 100}, DlFilterMode::kLinear,
                                &paint);
          SkRSXform xforms[] = {{1, 0, 10, 10}, {0, 1, 10, 10}};
          SkRect rects[] = {{10, 10, 20, 20}, {10, 20, 30, 20}};
          builder.DrawAtlas(TestImage1, xforms, rects, nullptr, 2,
                            DlBlendMode::kSrcOver, DlImageSampling::kLinear,
                            nullptr, &paint);
          builder.DrawTextBlob(GetTestTextBlob(1), 10, 10, paint);

          // Dst mode eliminates most rendering ops except for
          // the following two, so we'll prune those manually...
          if (paint.getBlendMode() != DlBlendMode::kDst) {
            builder.DrawDisplayList(TestDisplayList1, paint.getOpacity());
            builder.DrawShadow(kTestPath1, paint.getColor(), 1, true, 1);
          }
        },
        expected_op_count, expected_total_depth);
    run_one_test(
        name + " SaveLayer",
        [](DisplayListBuilder& builder, DlPaint& paint) {
          builder.SaveLayer(nullptr, &paint, nullptr);
          builder.DrawRect(SkRect{10, 10, 20, 20}, DlPaint());
          builder.Restore();
        },
        expected_op_count, expected_total_depth);
    run_one_test(
        name + " inside Save",
        [](DisplayListBuilder& builder, DlPaint& paint) {
          builder.Save();
          builder.DrawRect(SkRect{10, 10, 20, 20}, paint);
          builder.Restore();
        },
        expected_op_count, expected_total_depth);
  };
  run_tests("transparent color",  //
            [](DisplayListBuilder& builder, DlPaint& paint) {
              paint.setColor(DlColor::kTransparent());
            });
  run_tests("0 alpha",  //
            [](DisplayListBuilder& builder, DlPaint& paint) {
              // The transparent test above already tested transparent
              // black (all 0s), we set White color here so we can test
              // the case of all 1s with a 0 alpha
              paint.setColor(DlColor::kWhite());
              paint.setAlpha(0);
            });
  run_tests("BlendMode::kDst",  //
            [](DisplayListBuilder& builder, DlPaint& paint) {
              paint.setBlendMode(DlBlendMode::kDst);
            });
  run_tests("Empty rect clip",  //
            [](DisplayListBuilder& builder, DlPaint& paint) {
              builder.ClipRect(SkRect::MakeEmpty(), ClipOp::kIntersect, false);
            });
  run_tests("Empty rrect clip",  //
            [](DisplayListBuilder& builder, DlPaint& paint) {
              builder.ClipRRect(SkRRect::MakeEmpty(), ClipOp::kIntersect,
                                false);
            });
  run_tests("Empty path clip",  //
            [](DisplayListBuilder& builder, DlPaint& paint) {
              builder.ClipPath(SkPath(), ClipOp::kIntersect, false);
            });
  run_tests("Transparent SaveLayer",  //
            [](DisplayListBuilder& builder, DlPaint& paint) {
              DlPaint save_paint;
              save_paint.setColor(DlColor::kTransparent());
              builder.SaveLayer(nullptr, &save_paint);
            });
  run_tests("0 alpha SaveLayer",  //
            [](DisplayListBuilder& builder, DlPaint& paint) {
              DlPaint save_paint;
              // The transparent test above already tested transparent
              // black (all 0s), we set White color here so we can test
              // the case of all 1s with a 0 alpha
              save_paint.setColor(DlColor::kWhite());
              save_paint.setAlpha(0);
              builder.SaveLayer(nullptr, &save_paint);
            });
  run_tests("Dst blended SaveLayer",  //
            [](DisplayListBuilder& builder, DlPaint& paint) {
              DlPaint save_paint;
              save_paint.setBlendMode(DlBlendMode::kDst);
              builder.SaveLayer(nullptr, &save_paint);
            });
  run_tests(
      "Nop inside SaveLayer",
      [](DisplayListBuilder& builder, DlPaint& paint) {
        builder.SaveLayer(nullptr, nullptr);
        paint.setBlendMode(DlBlendMode::kDst);
      },
      2u, 1u);
  run_tests("DrawImage inside Culled SaveLayer",  //
            [](DisplayListBuilder& builder, DlPaint& paint) {
              DlPaint save_paint;
              save_paint.setColor(DlColor::kTransparent());
              builder.SaveLayer(nullptr, &save_paint);
              builder.DrawImage(TestImage1, SkPoint{10, 10},
                                DlImageSampling::kLinear);
            });
}

class SaveLayerBoundsExpector : public virtual DlOpReceiver,
                                public IgnoreAttributeDispatchHelper,
                                public IgnoreClipDispatchHelper,
                                public IgnoreTransformDispatchHelper,
                                public IgnoreDrawDispatchHelper {
 public:
  explicit SaveLayerBoundsExpector() {}

  SaveLayerBoundsExpector& addComputedExpectation(const DlRect& bounds) {
    expected_.emplace_back(BoundsExpectation{
        .bounds = bounds,
        .options = SaveLayerOptions(),
    });
    return *this;
  }
  SaveLayerBoundsExpector& addComputedExpectation(const SkRect& bounds) {
    return addComputedExpectation(ToDlRect(bounds));
  }

  SaveLayerBoundsExpector& addSuppliedExpectation(const DlRect& bounds,
                                                  bool clipped = false) {
    SaveLayerOptions options;
    options = options.with_bounds_from_caller();
    if (clipped) {
      options = options.with_content_is_clipped();
    }
    expected_.emplace_back(BoundsExpectation{
        .bounds = bounds,
        .options = options,
    });
    return *this;
  }
  SaveLayerBoundsExpector& addSuppliedExpectation(const SkRect& bounds,
                                                  bool clipped = false) {
    return addSuppliedExpectation(ToDlRect(bounds), clipped);
  }

  void saveLayer(const DlRect& bounds,
                 const SaveLayerOptions options,
                 const DlImageFilter* backdrop,
                 std::optional<int64_t> backdrop_id) override {
    ASSERT_LT(save_layer_count_, expected_.size());
    auto expected = expected_[save_layer_count_];
    EXPECT_EQ(options.bounds_from_caller(),
              expected.options.bounds_from_caller())
        << "expected bounds index " << save_layer_count_;
    EXPECT_EQ(options.content_is_clipped(),
              expected.options.content_is_clipped())
        << "expected bounds index " << save_layer_count_;
    if (!SkScalarNearlyEqual(bounds.GetLeft(), expected.bounds.GetLeft()) ||
        !SkScalarNearlyEqual(bounds.GetTop(), expected.bounds.GetTop()) ||
        !SkScalarNearlyEqual(bounds.GetRight(), expected.bounds.GetRight()) ||
        !SkScalarNearlyEqual(bounds.GetBottom(), expected.bounds.GetBottom())) {
      EXPECT_EQ(bounds, expected.bounds)
          << "expected bounds index " << save_layer_count_;
    }
    save_layer_count_++;
  }

  bool all_bounds_checked() const {
    return save_layer_count_ == expected_.size();
  }

 private:
  struct BoundsExpectation {
    const DlRect bounds;
    const SaveLayerOptions options;
  };

  std::vector<BoundsExpectation> expected_;
  size_t save_layer_count_ = 0;
};

TEST_F(DisplayListTest, SaveLayerBoundsComputationOfSimpleRect) {
  SkRect rect = SkRect::MakeLTRB(100.0f, 100.0f, 200.0f, 200.0f);

  DisplayListBuilder builder;
  builder.SaveLayer(nullptr, nullptr);
  {  //
    builder.DrawRect(rect, DlPaint());
  }
  builder.Restore();
  auto display_list = builder.Build();

  SaveLayerBoundsExpector expector;
  expector.addComputedExpectation(rect);
  display_list->Dispatch(expector);
  EXPECT_TRUE(expector.all_bounds_checked());
}

TEST_F(DisplayListTest, SaveLayerBoundsComputationOfMaskBlurredRect) {
  SkRect rect = SkRect::MakeLTRB(100.0f, 100.0f, 200.0f, 200.0f);
  DlPaint draw_paint;
  auto mask_filter = DlBlurMaskFilter::Make(DlBlurStyle::kNormal, 2.0f);
  draw_paint.setMaskFilter(mask_filter);

  DisplayListBuilder builder;
  builder.SaveLayer(nullptr, nullptr);
  {  //
    builder.DrawRect(rect, draw_paint);
  }
  builder.Restore();
  auto display_list = builder.Build();

  SaveLayerBoundsExpector expector;
  expector.addComputedExpectation(rect.makeOutset(6.0f, 6.0f));
  display_list->Dispatch(expector);
  EXPECT_TRUE(expector.all_bounds_checked());
}

TEST_F(DisplayListTest, SaveLayerBoundsComputationOfImageBlurredRect) {
  SkRect rect = SkRect::MakeLTRB(100.0f, 100.0f, 200.0f, 200.0f);
  DlPaint draw_paint;
  auto image_filter = DlImageFilter::MakeBlur(2.0f, 3.0f, DlTileMode::kDecal);
  draw_paint.setImageFilter(image_filter);

  DisplayListBuilder builder;
  builder.SaveLayer(nullptr, nullptr);
  {  //
    builder.DrawRect(rect, draw_paint);
  }
  builder.Restore();
  auto display_list = builder.Build();

  SaveLayerBoundsExpector expector;
  expector.addComputedExpectation(rect.makeOutset(6.0f, 9.0f));
  display_list->Dispatch(expector);
  EXPECT_TRUE(expector.all_bounds_checked());
}

TEST_F(DisplayListTest, SaveLayerBoundsComputationOfStrokedRect) {
  SkRect rect = SkRect::MakeLTRB(100.0f, 100.0f, 200.0f, 200.0f);
  DlPaint draw_paint;
  draw_paint.setStrokeWidth(5.0f);
  draw_paint.setDrawStyle(DlDrawStyle::kStroke);

  DisplayListBuilder builder;
  builder.SaveLayer(nullptr, nullptr);
  {  //
    builder.DrawRect(rect, draw_paint);
  }
  builder.Restore();
  auto display_list = builder.Build();

  SaveLayerBoundsExpector expector;
  expector.addComputedExpectation(rect.makeOutset(2.5f, 2.5f));
  display_list->Dispatch(expector);
  EXPECT_TRUE(expector.all_bounds_checked());
}

TEST_F(DisplayListTest, TranslatedSaveLayerBoundsComputationOfSimpleRect) {
  SkRect rect = SkRect::MakeLTRB(100.0f, 100.0f, 200.0f, 200.0f);

  DisplayListBuilder builder;
  builder.Translate(10.0f, 10.0f);
  builder.SaveLayer(nullptr, nullptr);
  {  //
    builder.DrawRect(rect, DlPaint());
  }
  builder.Restore();
  auto display_list = builder.Build();

  SaveLayerBoundsExpector expector;
  expector.addComputedExpectation(rect);
  display_list->Dispatch(expector);
  EXPECT_TRUE(expector.all_bounds_checked());
}

TEST_F(DisplayListTest, ScaledSaveLayerBoundsComputationOfSimpleRect) {
  SkRect rect = SkRect::MakeLTRB(100.0f, 100.0f, 200.0f, 200.0f);

  DisplayListBuilder builder;
  builder.Scale(10.0f, 10.0f);
  builder.SaveLayer(nullptr, nullptr);
  {  //
    builder.DrawRect(rect, DlPaint());
  }
  builder.Restore();
  auto display_list = builder.Build();

  SaveLayerBoundsExpector expector;
  expector.addComputedExpectation(rect);
  display_list->Dispatch(expector);
  EXPECT_TRUE(expector.all_bounds_checked());
}

TEST_F(DisplayListTest, RotatedSaveLayerBoundsComputationOfSimpleRect) {
  SkRect rect = SkRect::MakeLTRB(100.0f, 100.0f, 200.0f, 200.0f);

  DisplayListBuilder builder;
  builder.Rotate(45.0f);
  builder.SaveLayer(nullptr, nullptr);
  {  //
    builder.DrawRect(rect, DlPaint());
  }
  builder.Restore();
  auto display_list = builder.Build();

  SaveLayerBoundsExpector expector;
  expector.addComputedExpectation(rect);
  display_list->Dispatch(expector);
  EXPECT_TRUE(expector.all_bounds_checked());
}

TEST_F(DisplayListTest, TransformResetSaveLayerBoundsComputationOfSimpleRect) {
  SkRect rect = SkRect::MakeLTRB(100.0f, 100.0f, 200.0f, 200.0f);
  SkRect rect_doubled = SkMatrix::Scale(2.0f, 2.0f).mapRect(rect);

  DisplayListBuilder builder;
  builder.Scale(10.0f, 10.0f);
  builder.SaveLayer(nullptr, nullptr);
  builder.TransformReset();
  builder.Scale(20.0f, 20.0f);
  // Net local transform for SaveLayer is Scale(2, 2)
  {  //
    builder.DrawRect(rect, DlPaint());
  }
  builder.Restore();
  auto display_list = builder.Build();

  SaveLayerBoundsExpector expector;
  expector.addComputedExpectation(rect_doubled);
  display_list->Dispatch(expector);
  EXPECT_TRUE(expector.all_bounds_checked());
}

TEST_F(DisplayListTest, SaveLayerBoundsComputationOfTranslatedSimpleRect) {
  SkRect rect = SkRect::MakeLTRB(100.0f, 100.0f, 200.0f, 200.0f);

  DisplayListBuilder builder;
  builder.SaveLayer(nullptr, nullptr);
  {  //
    builder.Translate(10.0f, 10.0f);
    builder.DrawRect(rect, DlPaint());
  }
  builder.Restore();
  auto display_list = builder.Build();

  SaveLayerBoundsExpector expector;
  expector.addComputedExpectation(rect.makeOffset(10.0f, 10.0f));
  display_list->Dispatch(expector);
  EXPECT_TRUE(expector.all_bounds_checked());
}

TEST_F(DisplayListTest, SaveLayerBoundsComputationOfScaledSimpleRect) {
  SkRect rect = SkRect::MakeLTRB(100.0f, 100.0f, 200.0f, 200.0f);

  DisplayListBuilder builder;
  builder.SaveLayer(nullptr, nullptr);
  {  //
    builder.Scale(10.0f, 10.0f);
    builder.DrawRect(rect, DlPaint());
  }
  builder.Restore();
  auto display_list = builder.Build();

  SaveLayerBoundsExpector expector;
  expector.addComputedExpectation(
      SkRect::MakeLTRB(1000.0f, 1000.0f, 2000.0f, 2000.0f));
  display_list->Dispatch(expector);
  EXPECT_TRUE(expector.all_bounds_checked());
}

TEST_F(DisplayListTest, SaveLayerBoundsComputationOfRotatedSimpleRect) {
  SkRect rect = SkRect::MakeLTRB(100.0f, 100.0f, 200.0f, 200.0f);

  DisplayListBuilder builder;
  builder.SaveLayer(nullptr, nullptr);
  {  //
    builder.Rotate(45.0f);
    builder.DrawRect(rect, DlPaint());
  }
  builder.Restore();
  auto display_list = builder.Build();

  SkMatrix matrix = SkMatrix::RotateDeg(45.0f);
  SaveLayerBoundsExpector expector;
  expector.addComputedExpectation(matrix.mapRect(rect));
  display_list->Dispatch(expector);
  EXPECT_TRUE(expector.all_bounds_checked());
}

TEST_F(DisplayListTest, SaveLayerBoundsComputationOfNestedSimpleRect) {
  SkRect rect = SkRect::MakeLTRB(100.0f, 100.0f, 200.0f, 200.0f);

  DisplayListBuilder builder;
  builder.SaveLayer(nullptr, nullptr);
  {  //
    builder.SaveLayer(nullptr, nullptr);
    {  //
      builder.DrawRect(rect, DlPaint());
    }
    builder.Restore();
  }
  builder.Restore();
  auto display_list = builder.Build();

  SaveLayerBoundsExpector expector;
  expector.addComputedExpectation(rect);
  expector.addComputedExpectation(rect);
  display_list->Dispatch(expector);
  EXPECT_TRUE(expector.all_bounds_checked());
}

TEST_F(DisplayListTest, FloodingSaveLayerBoundsComputationOfSimpleRect) {
  SkRect rect = SkRect::MakeLTRB(100.0f, 100.0f, 200.0f, 200.0f);
  DlPaint save_paint;
  auto color_filter =
      DlColorFilter::MakeBlend(DlColor::kRed(), DlBlendMode::kSrc);
  ASSERT_TRUE(color_filter->modifies_transparent_black());
  save_paint.setColorFilter(color_filter);
  SkRect clip_rect = rect.makeOutset(100.0f, 100.0f);
  ASSERT_NE(clip_rect, rect);
  ASSERT_TRUE(clip_rect.contains(rect));

  DisplayListBuilder builder;
  builder.ClipRect(clip_rect);
  builder.SaveLayer(nullptr, &save_paint);
  {  //
    builder.DrawRect(rect, DlPaint());
  }
  builder.Restore();
  auto display_list = builder.Build();

  SaveLayerBoundsExpector expector;
  expector.addComputedExpectation(rect);
  display_list->Dispatch(expector);
  EXPECT_TRUE(expector.all_bounds_checked());
}

TEST_F(DisplayListTest, NestedFloodingSaveLayerBoundsComputationOfSimpleRect) {
  SkRect rect = SkRect::MakeLTRB(100.0f, 100.0f, 200.0f, 200.0f);
  DlPaint save_paint;
  auto color_filter =
      DlColorFilter::MakeBlend(DlColor::kRed(), DlBlendMode::kSrc);
  ASSERT_TRUE(color_filter->modifies_transparent_black());
  save_paint.setColorFilter(color_filter);
  SkRect clip_rect = rect.makeOutset(100.0f, 100.0f);
  ASSERT_NE(clip_rect, rect);
  ASSERT_TRUE(clip_rect.contains(rect));

  DisplayListBuilder builder;
  builder.ClipRect(clip_rect);
  builder.SaveLayer(nullptr, nullptr);
  {
    builder.SaveLayer(nullptr, &save_paint);
    {  //
      builder.DrawRect(rect, DlPaint());
    }
    builder.Restore();
  }
  builder.Restore();
  auto display_list = builder.Build();

  EXPECT_EQ(display_list->bounds(), clip_rect);

  SaveLayerBoundsExpector expector;
  expector.addComputedExpectation(clip_rect);
  expector.addComputedExpectation(rect);
  display_list->Dispatch(expector);
  EXPECT_TRUE(expector.all_bounds_checked());
}

TEST_F(DisplayListTest, SaveLayerBoundsComputationOfFloodingImageFilter) {
  SkRect rect = SkRect::MakeLTRB(100.0f, 100.0f, 200.0f, 200.0f);
  DlPaint draw_paint;
  auto color_filter =
      DlColorFilter::MakeBlend(DlColor::kRed(), DlBlendMode::kSrc);
  ASSERT_TRUE(color_filter->modifies_transparent_black());
  auto image_filter = DlImageFilter::MakeColorFilter(color_filter);
  draw_paint.setImageFilter(image_filter);
  SkRect clip_rect = rect.makeOutset(100.0f, 100.0f);
  ASSERT_NE(clip_rect, rect);
  ASSERT_TRUE(clip_rect.contains(rect));

  DisplayListBuilder builder;
  builder.ClipRect(clip_rect);
  builder.SaveLayer(nullptr, nullptr);
  {  //
    builder.DrawRect(rect, draw_paint);
  }
  builder.Restore();
  auto display_list = builder.Build();

  SaveLayerBoundsExpector expector;
  expector.addComputedExpectation(clip_rect);
  display_list->Dispatch(expector);
  EXPECT_TRUE(expector.all_bounds_checked());
}

TEST_F(DisplayListTest, SaveLayerBoundsComputationOfFloodingColorFilter) {
  SkRect rect = SkRect::MakeLTRB(100.0f, 100.0f, 200.0f, 200.0f);
  DlPaint draw_paint;
  auto color_filter =
      DlColorFilter::MakeBlend(DlColor::kRed(), DlBlendMode::kSrc);
  ASSERT_TRUE(color_filter->modifies_transparent_black());
  draw_paint.setColorFilter(color_filter);
  SkRect clip_rect = rect.makeOutset(100.0f, 100.0f);
  ASSERT_NE(clip_rect, rect);
  ASSERT_TRUE(clip_rect.contains(rect));

  DisplayListBuilder builder;
  builder.ClipRect(clip_rect);
  builder.SaveLayer(nullptr, nullptr);
  {  //
    builder.DrawRect(rect, draw_paint);
  }
  builder.Restore();
  auto display_list = builder.Build();

  // A color filter is implicitly clipped to the draw bounds so the layer
  // bounds will be the same as the draw bounds.
  SaveLayerBoundsExpector expector;
  expector.addComputedExpectation(rect);
  display_list->Dispatch(expector);
  EXPECT_TRUE(expector.all_bounds_checked());
}

TEST_F(DisplayListTest, SaveLayerBoundsClipDetectionSimpleUnclippedRect) {
  SkRect rect = SkRect::MakeLTRB(100.0f, 100.0f, 200.0f, 200.0f);
  SkRect save_rect = SkRect::MakeLTRB(50.0f, 50.0f, 250.0f, 250.0f);

  DisplayListBuilder builder;
  builder.SaveLayer(&save_rect, nullptr);
  {  //
    builder.DrawRect(rect, DlPaint());
  }
  builder.Restore();
  auto display_list = builder.Build();

  SaveLayerBoundsExpector expector;
  expector.addSuppliedExpectation(rect);
  display_list->Dispatch(expector);
  EXPECT_TRUE(expector.all_bounds_checked());
}

TEST_F(DisplayListTest, SaveLayerBoundsClipDetectionSimpleClippedRect) {
  SkRect rect = SkRect::MakeLTRB(100.0f, 100.0f, 200.0f, 200.0f);
  SkRect save_rect = SkRect::MakeLTRB(50.0f, 50.0f, 110.0f, 110.0f);
  SkRect content_rect = SkRect::MakeLTRB(100.0f, 100.0f, 110.0f, 110.0f);

  DisplayListBuilder builder;
  builder.SaveLayer(&save_rect, nullptr);
  {  //
    builder.DrawRect(rect, DlPaint());
  }
  builder.Restore();
  auto display_list = builder.Build();

  SaveLayerBoundsExpector expector;
  expector.addSuppliedExpectation(content_rect, true);
  display_list->Dispatch(expector);
  EXPECT_TRUE(expector.all_bounds_checked());
}

TEST_F(DisplayListTest, DisjointSaveLayerBoundsProduceEmptySuppliedBounds) {
  // This test was added when we fixed the Builder code to check the
  // return value of the SkRect::intersect method, but it turns out
  // that the indicated case never happens in practice due to the
  // internal culling during the recording process. It actually passes
  // both before and after the fix, but is here to ensure the right
  // behavior does not regress.

  SkRect layer_bounds = SkRect::MakeLTRB(10.0f, 10.0f, 20.0f, 20.0f);
  SkRect draw_rect = SkRect::MakeLTRB(50.0f, 50.0f, 100.0f, 100.0f);
  ASSERT_FALSE(layer_bounds.intersects(draw_rect));
  ASSERT_FALSE(layer_bounds.isEmpty());
  ASSERT_FALSE(draw_rect.isEmpty());

  DisplayListBuilder builder;
  builder.SaveLayer(&layer_bounds, nullptr);
  builder.DrawRect(draw_rect, DlPaint());
  builder.Restore();
  auto display_list = builder.Build();

  SaveLayerBoundsExpector expector;
  expector.addSuppliedExpectation(SkRect::MakeEmpty(), false);
  display_list->Dispatch(expector);
  EXPECT_TRUE(expector.all_bounds_checked());
}

class DepthExpector : public virtual DlOpReceiver,
                      virtual IgnoreAttributeDispatchHelper,
                      virtual IgnoreTransformDispatchHelper,
                      virtual IgnoreClipDispatchHelper,
                      virtual IgnoreDrawDispatchHelper {
 public:
  explicit DepthExpector(std::vector<uint32_t> expectations)
      : depth_expectations_(std::move(expectations)) {}

  void save() override {
    // This method should not be called since we override the variant with
    // the total_content_depth parameter.
    FAIL() << "save(no depth parameter) method should not be called";
  }

  void save(uint32_t total_content_depth) override {
    ASSERT_LT(index_, depth_expectations_.size());
    EXPECT_EQ(depth_expectations_[index_], total_content_depth)
        << "at index " << index_;
    index_++;
  }

  void saveLayer(const DlRect& bounds,
                 SaveLayerOptions options,
                 const DlImageFilter* backdrop,
                 std::optional<int64_t> backdrop_id) override {
    // This method should not be called since we override the variant with
    // the total_content_depth parameter.
    FAIL() << "saveLayer(no depth parameter) method should not be called";
  }

  void saveLayer(const DlRect& bounds,
                 const SaveLayerOptions& options,
                 uint32_t total_content_depth,
                 DlBlendMode max_content_mode,
                 const DlImageFilter* backdrop,
                 std::optional<int64_t> backdrop_id) override {
    ASSERT_LT(index_, depth_expectations_.size());
    EXPECT_EQ(depth_expectations_[index_], total_content_depth)
        << "at index " << index_;
    index_++;
  }

  bool all_depths_checked() const {
    return index_ == depth_expectations_.size();
  }

 private:
  size_t index_ = 0;
  std::vector<uint32_t> depth_expectations_;
};

TEST_F(DisplayListTest, SaveContentDepthTest) {
  DisplayListBuilder child_builder;
  child_builder.DrawRect(SkRect{10, 10, 20, 20}, DlPaint());  // depth 1
  auto child = child_builder.Build();

  DisplayListBuilder builder;
  builder.DrawRect(SkRect{10, 10, 20, 20}, DlPaint());  // depth 1

  builder.Save();  // covers depth 1->9
  {
    builder.Translate(5, 5);  // triggers deferred save at depth 1
    builder.DrawRect(SkRect{10, 10, 20, 20}, DlPaint());  // depth 2

    builder.DrawDisplayList(child, 1.0f);  // depth 3 (content) + 4 (self)

    builder.SaveLayer(nullptr, nullptr);  // covers depth 5->6
    {
      builder.DrawRect(SkRect{12, 12, 22, 22}, DlPaint());  // depth 5
      builder.DrawRect(SkRect{14, 14, 24, 24}, DlPaint());  // depth 6
    }
    builder.Restore();  // layer is restored with depth 6

    builder.DrawRect(SkRect{16, 16, 26, 26}, DlPaint());  // depth 8
    builder.DrawRect(SkRect{18, 18, 28, 28}, DlPaint());  // depth 9
  }
  builder.Restore();  // save is restored with depth 9

  builder.DrawRect(SkRect{16, 16, 26, 26}, DlPaint());  // depth 10
  builder.DrawRect(SkRect{18, 18, 28, 28}, DlPaint());  // depth 11
  auto display_list = builder.Build();

  EXPECT_EQ(display_list->total_depth(), 11u);

  DepthExpector expector({8, 2});
  display_list->Dispatch(expector);
}

TEST_F(DisplayListTest, FloodingFilteredLayerPushesRestoreOpIndex) {
  DisplayListBuilder builder(true);
  builder.ClipRect(DlRect::MakeLTRB(100.0f, 100.0f, 200.0f, 200.0f));
  // ClipRect does not contribute to rtree rects, no id needed

  DlPaint save_paint;
  // clang-format off
  const float matrix[] = {
    0.5f, 0.0f, 0.0f, 0.0f, 0.5f,
    0.5f, 0.0f, 0.0f, 0.0f, 0.5f,
    0.5f, 0.0f, 0.0f, 0.0f, 0.5f,
    0.5f, 0.0f, 0.0f, 0.0f, 0.5f
  };
  // clang-format on
  auto color_filter = DlColorFilter::MakeMatrix(matrix);
  save_paint.setImageFilter(DlImageFilter::MakeColorFilter(color_filter));
  builder.SaveLayer(nullptr, &save_paint);
  int save_layer_id = DisplayListBuilderTestingLastOpIndex(builder);

  builder.DrawRect(DlRect::MakeLTRB(120.0f, 120.0f, 125.0f, 125.0f), DlPaint());
  int draw_rect_id = DisplayListBuilderTestingLastOpIndex(builder);

  builder.Restore();
  int restore_id = DisplayListBuilderTestingLastOpIndex(builder);

  auto dl = builder.Build();
  std::vector<int> indices;
  dl->rtree()->search(DlRect::MakeLTRB(0.0f, 0.0f, 500.0f, 500.0f), &indices);
  ASSERT_EQ(indices.size(), 3u);
  EXPECT_EQ(dl->rtree()->id(indices[0]), save_layer_id);
  EXPECT_EQ(dl->rtree()->id(indices[1]), draw_rect_id);
  EXPECT_EQ(dl->rtree()->id(indices[2]), restore_id);
}

TEST_F(DisplayListTest, TransformingFilterSaveLayerSimpleContentBounds) {
  DisplayListBuilder builder;
  builder.ClipRect(SkRect::MakeLTRB(100.0f, 100.0f, 200.0f, 200.0f));

  DlPaint save_paint;
  auto image_filter =
      DlImageFilter::MakeMatrix(DlMatrix::MakeTranslation({100.0f, 100.0f}),
                                DlImageSampling::kNearestNeighbor);
  save_paint.setImageFilter(image_filter);
  builder.SaveLayer(nullptr, &save_paint);

  builder.DrawRect(SkRect::MakeLTRB(20.0f, 20.0f, 25.0f, 25.0f), DlPaint());

  builder.Restore();

  auto dl = builder.Build();
  EXPECT_EQ(dl->bounds(), SkRect::MakeLTRB(120.0f, 120.0f, 125.0f, 125.0f));
}

TEST_F(DisplayListTest, TransformingFilterSaveLayerFloodedContentBounds) {
  DisplayListBuilder builder;
  builder.ClipRect(SkRect::MakeLTRB(100.0f, 100.0f, 200.0f, 200.0f));

  DlPaint save_paint;
  auto image_filter =
      DlImageFilter::MakeMatrix(DlMatrix::MakeTranslation({100.0f, 100.0f}),
                                DlImageSampling::kNearestNeighbor);
  save_paint.setImageFilter(image_filter);
  builder.SaveLayer(nullptr, &save_paint);

  builder.DrawColor(DlColor::kBlue(), DlBlendMode::kSrcOver);

  builder.Restore();

  auto dl = builder.Build();
  EXPECT_EQ(dl->bounds(), SkRect::MakeLTRB(100.0f, 100.0f, 200.0f, 200.0f));
}

TEST_F(DisplayListTest, OpacityIncompatibleRenderOpInsideDeferredSave) {
  {
    // Without deferred save
    DisplayListBuilder builder;
    builder.DrawRect(SkRect::MakeLTRB(0, 0, 10, 10),
                     DlPaint().setBlendMode(DlBlendMode::kClear));
    EXPECT_FALSE(builder.Build()->can_apply_group_opacity());
  }

  {
    // With deferred save
    DisplayListBuilder builder;
    builder.Save();
    {
      // Nothing to trigger the deferred save...
      builder.DrawRect(SkRect::MakeLTRB(0, 0, 10, 10),
                       DlPaint().setBlendMode(DlBlendMode::kClear));
    }
    // Deferred save was not triggered, did it forward the incompatibility
    // flags?
    builder.Restore();
    EXPECT_FALSE(builder.Build()->can_apply_group_opacity());
  }
}

TEST_F(DisplayListTest, MaxBlendModeEmptyDisplayList) {
  DisplayListBuilder builder;
  EXPECT_EQ(builder.Build()->max_root_blend_mode(), DlBlendMode::kClear);
}

TEST_F(DisplayListTest, MaxBlendModeSimpleRect) {
  auto test = [](DlBlendMode mode) {
    DisplayListBuilder builder;
    builder.DrawRect(SkRect::MakeLTRB(0, 0, 10, 10),
                     DlPaint().setAlpha(0x7f).setBlendMode(mode));
    DlBlendMode expect =
        (mode == DlBlendMode::kDst) ? DlBlendMode::kClear : mode;
    EXPECT_EQ(builder.Build()->max_root_blend_mode(), expect)  //
        << "testing " << mode;
  };

  for (int i = 0; i < static_cast<int>(DlBlendMode::kLastMode); i++) {
    test(static_cast<DlBlendMode>(i));
  }
}

TEST_F(DisplayListTest, MaxBlendModeInsideNonDeferredSave) {
  DisplayListBuilder builder;
  builder.Save();
  {
    // Trigger the deferred save
    builder.Scale(2.0f, 2.0f);
    builder.DrawRect(SkRect::MakeLTRB(0, 0, 10, 10),
                     DlPaint().setBlendMode(DlBlendMode::kModulate));
  }
  // Save was triggered, did it forward the max blend mode?
  builder.Restore();
  EXPECT_EQ(builder.Build()->max_root_blend_mode(), DlBlendMode::kModulate);
}

TEST_F(DisplayListTest, MaxBlendModeInsideDeferredSave) {
  DisplayListBuilder builder;
  builder.Save();
  {
    // Nothing to trigger the deferred save...
    builder.DrawRect(SkRect::MakeLTRB(0, 0, 10, 10),
                     DlPaint().setBlendMode(DlBlendMode::kModulate));
  }
  // Deferred save was not triggered, did it forward the max blend mode?
  builder.Restore();
  EXPECT_EQ(builder.Build()->max_root_blend_mode(), DlBlendMode::kModulate);
}

TEST_F(DisplayListTest, MaxBlendModeInsideSaveLayer) {
  DisplayListBuilder builder;
  builder.SaveLayer(nullptr, nullptr);
  {
    builder.DrawRect(SkRect::MakeLTRB(0, 0, 10, 10),
                     DlPaint().setBlendMode(DlBlendMode::kModulate));
  }
  builder.Restore();
  auto dl = builder.Build();
  EXPECT_EQ(dl->max_root_blend_mode(), DlBlendMode::kSrcOver);
  SAVE_LAYER_EXPECTOR(expector);
  expector.addExpectation(DlBlendMode::kModulate);
  dl->Dispatch(expector);
  EXPECT_TRUE(expector.all_expectations_checked());
}

TEST_F(DisplayListTest, MaxBlendModeInsideNonDefaultBlendedSaveLayer) {
  DisplayListBuilder builder;
  DlPaint save_paint;
  save_paint.setBlendMode(DlBlendMode::kScreen);
  builder.SaveLayer(nullptr, &save_paint);
  {
    builder.DrawRect(SkRect::MakeLTRB(0, 0, 10, 10),
                     DlPaint().setBlendMode(DlBlendMode::kModulate));
  }
  builder.Restore();
  auto dl = builder.Build();
  EXPECT_EQ(dl->max_root_blend_mode(), DlBlendMode::kScreen);
  SAVE_LAYER_EXPECTOR(expector);
  expector.addExpectation(DlBlendMode::kModulate);
  dl->Dispatch(expector);
  EXPECT_TRUE(expector.all_expectations_checked());
}

TEST_F(DisplayListTest, MaxBlendModeInsideComplexDeferredSaves) {
  DisplayListBuilder builder;
  builder.Save();
  {
    builder.DrawRect(SkRect::MakeLTRB(0, 0, 10, 10),
                     DlPaint().setBlendMode(DlBlendMode::kModulate));
    builder.Save();
    {
      // We want to use a blend mode that is greater than modulate here
      ASSERT_GT(DlBlendMode::kScreen, DlBlendMode::kModulate);
      builder.DrawRect(SkRect::MakeLTRB(0, 0, 10, 10),
                       DlPaint().setBlendMode(DlBlendMode::kScreen));
    }
    builder.Restore();

    // We want to use a blend mode that is smaller than modulate here
    ASSERT_LT(DlBlendMode::kSrc, DlBlendMode::kModulate);
    builder.DrawRect(SkRect::MakeLTRB(0, 0, 10, 10),
                     DlPaint().setBlendMode(DlBlendMode::kSrc));
  }
  builder.Restore();

  // Double check that kScreen is the max blend mode
  auto expect = std::max(DlBlendMode::kModulate, DlBlendMode::kScreen);
  expect = std::max(expect, DlBlendMode::kSrc);
  ASSERT_EQ(expect, DlBlendMode::kScreen);

  EXPECT_EQ(builder.Build()->max_root_blend_mode(), DlBlendMode::kScreen);
}

TEST_F(DisplayListTest, MaxBlendModeInsideComplexSaveLayers) {
  DisplayListBuilder builder;
  builder.SaveLayer(nullptr, nullptr);
  {
    // outer save layer has Modulate now and Src later - Modulate is larger
    builder.DrawRect(SkRect::MakeLTRB(0, 0, 10, 10),
                     DlPaint().setBlendMode(DlBlendMode::kModulate));
    builder.SaveLayer(nullptr, nullptr);
    {
      // inner save layer only has a Screen blend
      builder.DrawRect(SkRect::MakeLTRB(0, 0, 10, 10),
                       DlPaint().setBlendMode(DlBlendMode::kScreen));
    }
    builder.Restore();

    // We want to use a blend mode that is smaller than modulate here
    ASSERT_LT(DlBlendMode::kSrc, DlBlendMode::kModulate);
    builder.DrawRect(SkRect::MakeLTRB(0, 0, 10, 10),
                     DlPaint().setBlendMode(DlBlendMode::kSrc));
  }
  builder.Restore();

  // Double check that kModulate is the max blend mode for the first
  // SaveLayer operations
  auto expect = std::max(DlBlendMode::kModulate, DlBlendMode::kSrc);
  ASSERT_EQ(expect, DlBlendMode::kModulate);

  auto dl = builder.Build();
  EXPECT_EQ(dl->max_root_blend_mode(), DlBlendMode::kSrcOver);
  SAVE_LAYER_EXPECTOR(expector);
  expector  //
      .addExpectation(DlBlendMode::kModulate)
      .addExpectation(DlBlendMode::kScreen);
  dl->Dispatch(expector);
  EXPECT_TRUE(expector.all_expectations_checked());
}

TEST_F(DisplayListTest, BackdropDetectionEmptyDisplayList) {
  DisplayListBuilder builder;
  EXPECT_FALSE(builder.Build()->root_has_backdrop_filter());
}

TEST_F(DisplayListTest, BackdropDetectionSimpleRect) {
  DisplayListBuilder builder;
  builder.DrawRect(SkRect::MakeLTRB(0, 0, 10, 10), DlPaint());
  EXPECT_FALSE(builder.Build()->root_has_backdrop_filter());
}

TEST_F(DisplayListTest, BackdropDetectionSimpleSaveLayer) {
  DisplayListBuilder builder;
  builder.SaveLayer(nullptr, nullptr, &kTestBlurImageFilter1);
  {
    // inner content has no backdrop filter
    builder.DrawRect(SkRect::MakeLTRB(0, 0, 10, 10), DlPaint());
  }
  builder.Restore();
  auto dl = builder.Build();

  EXPECT_TRUE(dl->root_has_backdrop_filter());
  // The SaveLayer itself, though, does not have the contains backdrop
  // flag set because its content does not contain a SaveLayer with backdrop
  SAVE_LAYER_EXPECTOR(expector);
  expector.addExpectation(
      SaveLayerOptions::kNoAttributes.with_can_distribute_opacity());
  dl->Dispatch(expector);
  EXPECT_TRUE(expector.all_expectations_checked());
}

TEST_F(DisplayListTest, BackdropDetectionNestedSaveLayer) {
  DisplayListBuilder builder;
  builder.SaveLayer(nullptr, nullptr);
  {
    // first inner content does have backdrop filter
    builder.DrawRect(SkRect::MakeLTRB(0, 0, 10, 10), DlPaint());
    builder.SaveLayer(nullptr, nullptr, &kTestBlurImageFilter1);
    {
      // second inner content has no backdrop filter
      builder.DrawRect(SkRect::MakeLTRB(10, 10, 20, 20), DlPaint());
    }
    builder.Restore();
  }
  builder.Restore();
  auto dl = builder.Build();

  EXPECT_FALSE(dl->root_has_backdrop_filter());
  SAVE_LAYER_EXPECTOR(expector);
  expector                                             //
      .addExpectation(SaveLayerOptions::kNoAttributes  //
                          .with_contains_backdrop_filter()
                          .with_content_is_unbounded())
      .addExpectation(
          SaveLayerOptions::kNoAttributes.with_can_distribute_opacity());
  dl->Dispatch(expector);
  EXPECT_TRUE(expector.all_expectations_checked());
}

TEST_F(DisplayListTest, DrawDisplayListForwardsMaxBlend) {
  DisplayListBuilder child_builder;
  child_builder.DrawRect(SkRect::MakeLTRB(0, 0, 10, 10),
                         DlPaint().setBlendMode(DlBlendMode::kMultiply));
  auto child_dl = child_builder.Build();
  EXPECT_EQ(child_dl->max_root_blend_mode(), DlBlendMode::kMultiply);
  EXPECT_FALSE(child_dl->root_has_backdrop_filter());

  DisplayListBuilder parent_builder;
  parent_builder.DrawDisplayList(child_dl);
  auto parent_dl = parent_builder.Build();
  EXPECT_EQ(parent_dl->max_root_blend_mode(), DlBlendMode::kMultiply);
  EXPECT_FALSE(parent_dl->root_has_backdrop_filter());
}

TEST_F(DisplayListTest, DrawDisplayListForwardsBackdropFlag) {
  DisplayListBuilder child_builder;
  DlBlurImageFilter backdrop(2.0f, 2.0f, DlTileMode::kDecal);
  child_builder.SaveLayer(nullptr, nullptr, &backdrop);
  child_builder.DrawRect(SkRect::MakeLTRB(0, 0, 10, 10),
                         DlPaint().setBlendMode(DlBlendMode::kMultiply));
  child_builder.Restore();
  auto child_dl = child_builder.Build();
  EXPECT_EQ(child_dl->max_root_blend_mode(), DlBlendMode::kSrcOver);
  EXPECT_TRUE(child_dl->root_has_backdrop_filter());

  DisplayListBuilder parent_builder;
  parent_builder.DrawDisplayList(child_dl);
  auto parent_dl = parent_builder.Build();
  EXPECT_EQ(parent_dl->max_root_blend_mode(), DlBlendMode::kSrcOver);
  EXPECT_TRUE(parent_dl->root_has_backdrop_filter());
}

#define CLIP_EXPECTOR(name) ClipExpector name(__FILE__, __LINE__)

struct ClipExpectation {
  std::variant<DlRect, DlRoundRect, DlPath> shape;
  bool is_oval;
  ClipOp clip_op;
  bool is_aa;

  std::string shape_name() {
    switch (shape.index()) {
      case 0:
        return is_oval ? "SkOval" : "SkRect";
      case 1:
        return "DlRoundRect";
      case 2:
        return "DlPath";
      default:
        return "Unknown";
    }
  }
};

::std::ostream& operator<<(::std::ostream& os, const ClipExpectation& expect) {
  os << "Expectation(";
  switch (expect.shape.index()) {
    case 0:
      os << std::get<DlRect>(expect.shape);
      if (expect.is_oval) {
        os << " (oval)";
      }
      break;
    case 1:
      os << std::get<DlRoundRect>(expect.shape);
      break;
    case 2:
      os << std::get<DlPath>(expect.shape).GetSkPath();
      break;
    case 3:
      os << "Unknown";
  }
  os << ", " << expect.clip_op;
  os << ", " << expect.is_aa;
  os << ")";
  return os;
}

class ClipExpector : public virtual DlOpReceiver,
                     virtual IgnoreAttributeDispatchHelper,
                     virtual IgnoreTransformDispatchHelper,
                     virtual IgnoreDrawDispatchHelper {
 public:
  // file and line supplied automatically from CLIP_EXPECTOR macro
  explicit ClipExpector(const std::string& file, int line)
      : file_(file), line_(line) {}

  ~ClipExpector() {  //
    EXPECT_EQ(index_, clip_expectations_.size()) << label();
    while (index_ < clip_expectations_.size()) {
      auto expect = clip_expectations_[index_];
      FML_LOG(ERROR) << "leftover clip shape[" << index_ << "] = " << expect;
      index_++;
    }
  }

  ClipExpector& addExpectation(const SkRect& rect,
                               ClipOp clip_op = ClipOp::kIntersect,
                               bool is_aa = false) {
    clip_expectations_.push_back({
        .shape = ToDlRect(rect),
        .is_oval = false,
        .clip_op = clip_op,
        .is_aa = is_aa,
    });
    return *this;
  }

  ClipExpector& addOvalExpectation(const SkRect& rect,
                                   ClipOp clip_op = ClipOp::kIntersect,
                                   bool is_aa = false) {
    clip_expectations_.push_back({
        .shape = ToDlRect(rect),
        .is_oval = true,
        .clip_op = clip_op,
        .is_aa = is_aa,
    });
    return *this;
  }

  ClipExpector& addExpectation(const SkRRect& rrect,
                               ClipOp clip_op = ClipOp::kIntersect,
                               bool is_aa = false) {
    auto dl_rrect = ToDlRoundRect(rrect);
    EXPECT_EQ(ToSkRRect(dl_rrect), rrect);
    clip_expectations_.push_back({
        .shape = dl_rrect,
        .is_oval = false,
        .clip_op = clip_op,
        .is_aa = is_aa,
    });
    return *this;
  }

  ClipExpector& addExpectation(const DlPath& path,
                               ClipOp clip_op = ClipOp::kIntersect,
                               bool is_aa = false) {
    clip_expectations_.push_back({
        .shape = path,
        .is_oval = false,
        .clip_op = clip_op,
        .is_aa = is_aa,
    });
    return *this;
  }

  ClipExpector& addExpectation(const SkPath& path,
                               ClipOp clip_op = ClipOp::kIntersect,
                               bool is_aa = false) {
    return addExpectation(DlPath(path), clip_op, is_aa);
  }

  void clipRect(const DlRect& rect,
                DlCanvas::ClipOp clip_op,
                bool is_aa) override {
    check(rect, clip_op, is_aa);
  }
  void clipOval(const DlRect& bounds,
                DlCanvas::ClipOp clip_op,
                bool is_aa) override {
    check(bounds, clip_op, is_aa, true);
  }
  void clipRoundRect(const DlRoundRect& rrect,
                     DlCanvas::ClipOp clip_op,
                     bool is_aa) override {
    check(rrect, clip_op, is_aa);
  }
  void clipPath(const DlPath& path,
                DlCanvas::ClipOp clip_op,
                bool is_aa) override {
    check(path, clip_op, is_aa);
  }

 private:
  size_t index_ = 0;
  std::vector<ClipExpectation> clip_expectations_;

  template <typename T>
  void check(T shape, ClipOp clip_op, bool is_aa, bool is_oval = false) {
    ASSERT_LT(index_, clip_expectations_.size())
        << label() << std::endl
        << "extra clip shape = " << shape << (is_oval ? " (oval)" : "");
    auto expected = clip_expectations_[index_];
    if (!std::holds_alternative<T>(expected.shape)) {
      EXPECT_TRUE(std::holds_alternative<T>(expected.shape))
          << label() << ", expected type: " << expected.shape_name();
    } else {
      EXPECT_EQ(std::get<T>(expected.shape), shape) << label();
    }
    EXPECT_EQ(expected.is_oval, is_oval) << label();
    EXPECT_EQ(expected.clip_op, clip_op) << label();
    EXPECT_EQ(expected.is_aa, is_aa) << label();
    index_++;
  }

  const std::string file_;
  const int line_;

  std::string label() {
    return "at index " + std::to_string(index_) +  //
           ", from " + file_ +                     //
           ":" + std::to_string(line_);
  }
};

TEST_F(DisplayListTest, ClipRectCullingPixel6a) {
  // These particular values create bit errors if we use the path that
  // tests for inclusion in local space, but work OK if we use a forward
  // path that tests for inclusion in device space, due to the fact that
  // the extra matrix inversion is just enough math to cause the transform
  // to place the local space cull corners just outside the original rect.
  // The test in device space only works under a simple scale, such as we
  // use for DPR adjustments (and which are not always inversion friendly).

  auto frame = SkRect::MakeLTRB(0.0f, 0.0f, 1080.0f, 2400.0f);
  DlScalar DPR = 2.625f;
  auto clip = SkRect::MakeLTRB(0.0f, 0.0f, 1080.0f / DPR, 2400.0f / DPR);

  DisplayListBuilder cull_builder;
  cull_builder.ClipRect(frame, ClipOp::kIntersect, false);
  cull_builder.Scale(DPR, DPR);
  cull_builder.ClipRect(clip, ClipOp::kIntersect, false);
  auto cull_dl = cull_builder.Build();

  CLIP_EXPECTOR(expector);
  expector.addExpectation(frame, ClipOp::kIntersect, false);
  cull_dl->Dispatch(expector);
}

TEST_F(DisplayListTest, ClipRectCulling) {
  auto clip = SkRect::MakeLTRB(10.0f, 10.0f, 20.0f, 20.0f);

  DisplayListBuilder cull_builder;
  cull_builder.ClipRect(clip, ClipOp::kIntersect, false);
  cull_builder.ClipRect(clip.makeOutset(1.0f, 1.0f), ClipOp::kIntersect, false);
  auto cull_dl = cull_builder.Build();

  CLIP_EXPECTOR(expector);
  expector.addExpectation(clip, ClipOp::kIntersect, false);
  cull_dl->Dispatch(expector);
}

TEST_F(DisplayListTest, ClipRectNonCulling) {
  auto clip = SkRect::MakeLTRB(10.0f, 10.0f, 20.0f, 20.0f);
  auto smaller_clip = clip.makeInset(1.0f, 1.0f);

  DisplayListBuilder cull_builder;
  cull_builder.ClipRect(clip, ClipOp::kIntersect, false);
  cull_builder.ClipRect(smaller_clip, ClipOp::kIntersect, false);
  auto cull_dl = cull_builder.Build();

  CLIP_EXPECTOR(expector);
  expector.addExpectation(clip, ClipOp::kIntersect, false);
  expector.addExpectation(smaller_clip, ClipOp::kIntersect, false);
  cull_dl->Dispatch(expector);
}

TEST_F(DisplayListTest, ClipRectNestedCulling) {
  auto clip = SkRect::MakeLTRB(10.0f, 10.0f, 20.0f, 20.0f);
  auto larger_clip = clip.makeOutset(1.0f, 1.0f);

  DisplayListBuilder cull_builder;
  cull_builder.ClipRect(clip, ClipOp::kIntersect, false);
  cull_builder.Save();
  cull_builder.ClipRect(larger_clip, ClipOp::kIntersect, false);
  cull_builder.Restore();
  auto cull_dl = cull_builder.Build();

  CLIP_EXPECTOR(expector);
  expector.addExpectation(clip, ClipOp::kIntersect, false);
  cull_dl->Dispatch(expector);
}

TEST_F(DisplayListTest, ClipRectNestedNonCulling) {
  auto clip = SkRect::MakeLTRB(10.0f, 10.0f, 20.0f, 20.0f);
  auto larger_clip = clip.makeOutset(1.0f, 1.0f);

  DisplayListBuilder cull_builder;
  cull_builder.Save();
  cull_builder.ClipRect(clip, ClipOp::kIntersect, false);
  cull_builder.Restore();
  // Should not be culled because we have restored the prior clip
  cull_builder.ClipRect(larger_clip, ClipOp::kIntersect, false);
  auto cull_dl = cull_builder.Build();

  CLIP_EXPECTOR(expector);
  expector.addExpectation(clip, ClipOp::kIntersect, false);
  expector.addExpectation(larger_clip, ClipOp::kIntersect, false);
  cull_dl->Dispatch(expector);
}

TEST_F(DisplayListTest, ClipRectNestedCullingComplex) {
  auto clip = SkRect::MakeLTRB(10.0f, 10.0f, 20.0f, 20.0f);
  auto smaller_clip = clip.makeInset(1.0f, 1.0f);
  auto smallest_clip = clip.makeInset(2.0f, 2.0f);

  DisplayListBuilder cull_builder;
  cull_builder.ClipRect(clip, ClipOp::kIntersect, false);
  cull_builder.Save();
  cull_builder.ClipRect(smallest_clip, ClipOp::kIntersect, false);
  cull_builder.ClipRect(smaller_clip, ClipOp::kIntersect, false);
  cull_builder.Restore();
  auto cull_dl = cull_builder.Build();

  CLIP_EXPECTOR(expector);
  expector.addExpectation(clip, ClipOp::kIntersect, false);
  expector.addExpectation(smallest_clip, ClipOp::kIntersect, false);
  cull_dl->Dispatch(expector);
}

TEST_F(DisplayListTest, ClipRectNestedNonCullingComplex) {
  auto clip = SkRect::MakeLTRB(10.0f, 10.0f, 20.0f, 20.0f);
  auto smaller_clip = clip.makeInset(1.0f, 1.0f);
  auto smallest_clip = clip.makeInset(2.0f, 2.0f);

  DisplayListBuilder cull_builder;
  cull_builder.ClipRect(clip, ClipOp::kIntersect, false);
  cull_builder.Save();
  cull_builder.ClipRect(smallest_clip, ClipOp::kIntersect, false);
  cull_builder.Restore();
  // Would not be culled if it was inside the clip
  cull_builder.ClipRect(smaller_clip, ClipOp::kIntersect, false);
  auto cull_dl = cull_builder.Build();

  CLIP_EXPECTOR(expector);
  expector.addExpectation(clip, ClipOp::kIntersect, false);
  expector.addExpectation(smallest_clip, ClipOp::kIntersect, false);
  expector.addExpectation(smaller_clip, ClipOp::kIntersect, false);
  cull_dl->Dispatch(expector);
}

TEST_F(DisplayListTest, ClipOvalCulling) {
  auto clip = SkRect::MakeLTRB(10.0f, 10.0f, 20.0f, 20.0f);
  // A 10x10 rectangle extends 5x5 from the center to each corner. To have
  // an oval that encompasses that rectangle, the radius must be at least
  // length(5, 5), or 7.071+ so we expand the radius 5 square clip by 2.072
  // on each side to barely contain the corners of the square.
  auto encompassing_oval = clip.makeOutset(2.072f, 2.072f);

  DisplayListBuilder cull_builder;
  cull_builder.ClipRect(clip, ClipOp::kIntersect, false);
  cull_builder.ClipOval(encompassing_oval, ClipOp::kIntersect, false);
  auto cull_dl = cull_builder.Build();

  CLIP_EXPECTOR(expector);
  expector.addExpectation(clip, ClipOp::kIntersect, false);
  cull_dl->Dispatch(expector);
}

TEST_F(DisplayListTest, ClipOvalNonCulling) {
  auto clip = SkRect::MakeLTRB(10.0f, 10.0f, 20.0f, 20.0f);
  // A 10x10 rectangle extends 5x5 from the center to each corner. To have
  // an oval that encompasses that rectangle, the radius must be at least
  // length(5, 5), or 7.071+ so we expand the radius 5 square clip by 2.072
  // on each side to barely exclude the corners of the square.
  auto non_encompassing_oval = clip.makeOutset(2.071f, 2.071f);

  DisplayListBuilder cull_builder;
  cull_builder.ClipRect(clip, ClipOp::kIntersect, false);
  cull_builder.ClipOval(non_encompassing_oval, ClipOp::kIntersect, false);
  auto cull_dl = cull_builder.Build();

  CLIP_EXPECTOR(expector);
  expector.addExpectation(clip, ClipOp::kIntersect, false);
  expector.addOvalExpectation(non_encompassing_oval, ClipOp::kIntersect, false);
  cull_dl->Dispatch(expector);
}

TEST_F(DisplayListTest, ClipRRectCulling) {
  auto clip = SkRect::MakeLTRB(10.0f, 10.0f, 20.0f, 20.0f);
  auto rrect = SkRRect::MakeRectXY(clip.makeOutset(2.0f, 2.0f), 2.0f, 2.0f);
  ASSERT_FALSE(rrect.isOval());

  DisplayListBuilder cull_builder;
  cull_builder.ClipRect(clip, ClipOp::kIntersect, false);
  cull_builder.ClipRRect(rrect, ClipOp::kIntersect, false);
  auto cull_dl = cull_builder.Build();

  CLIP_EXPECTOR(expector);
  expector.addExpectation(clip, ClipOp::kIntersect, false);
  cull_dl->Dispatch(expector);
}

TEST_F(DisplayListTest, ClipRRectNonCulling) {
  auto clip = SkRect::MakeLTRB(10.0f, 10.0f, 20.0f, 20.0f);
  auto rrect = SkRRect::MakeRectXY(clip.makeOutset(1.0f, 1.0f), 4.0f, 4.0f);
  ASSERT_FALSE(rrect.isOval());

  DisplayListBuilder cull_builder;
  cull_builder.ClipRect(clip, ClipOp::kIntersect, false);
  cull_builder.ClipRRect(rrect, ClipOp::kIntersect, false);
  auto cull_dl = cull_builder.Build();

  CLIP_EXPECTOR(expector);
  expector.addExpectation(clip, ClipOp::kIntersect, false);
  expector.addExpectation(rrect, ClipOp::kIntersect, false);
  cull_dl->Dispatch(expector);
}

TEST_F(DisplayListTest, ClipPathNonCulling) {
  auto clip = SkRect::MakeLTRB(10.0f, 10.0f, 20.0f, 20.0f);
  SkPath path;
  path.moveTo(0.0f, 0.0f);
  path.lineTo(1000.0f, 0.0f);
  path.lineTo(0.0f, 1000.0f);
  path.close();

  // Double checking that the path does indeed contain the clip. But,
  // sadly, the Builder will not check paths for coverage to this level
  // of detail. (In particular, path containment of the corners is not
  // authoritative of true containment, but we know in this case that
  // a triangle contains a rect if it contains all 4 corners...)
  ASSERT_TRUE(path.contains(clip.fLeft, clip.fTop));
  ASSERT_TRUE(path.contains(clip.fRight, clip.fTop));
  ASSERT_TRUE(path.contains(clip.fRight, clip.fBottom));
  ASSERT_TRUE(path.contains(clip.fLeft, clip.fBottom));

  DisplayListBuilder cull_builder;
  cull_builder.ClipRect(clip, ClipOp::kIntersect, false);
  cull_builder.ClipPath(path, ClipOp::kIntersect, false);
  auto cull_dl = cull_builder.Build();

  CLIP_EXPECTOR(expector);
  expector.addExpectation(clip, ClipOp::kIntersect, false);
  expector.addExpectation(path, ClipOp::kIntersect, false);
  cull_dl->Dispatch(expector);
}

TEST_F(DisplayListTest, ClipPathRectCulling) {
  auto clip = SkRect::MakeLTRB(10.0f, 10.0f, 20.0f, 20.0f);
  SkPath path;
  path.addRect(clip.makeOutset(1.0f, 1.0f));

  DisplayListBuilder cull_builder;
  cull_builder.ClipRect(clip, ClipOp::kIntersect, false);
  cull_builder.ClipPath(path, ClipOp::kIntersect, false);
  auto cull_dl = cull_builder.Build();

  CLIP_EXPECTOR(expector);
  expector.addExpectation(clip, ClipOp::kIntersect, false);
  cull_dl->Dispatch(expector);
}

TEST_F(DisplayListTest, ClipPathRectNonCulling) {
  auto clip = SkRect::MakeLTRB(10.0f, 10.0f, 20.0f, 20.0f);
  auto smaller_clip = clip.makeInset(1.0f, 1.0f);
  SkPath path;
  path.addRect(smaller_clip);

  DisplayListBuilder cull_builder;
  cull_builder.ClipRect(clip, ClipOp::kIntersect, false);
  cull_builder.ClipPath(path, ClipOp::kIntersect, false);
  auto cull_dl = cull_builder.Build();

  CLIP_EXPECTOR(expector);
  expector.addExpectation(clip, ClipOp::kIntersect, false);
  // Builder will not cull this clip, but it will turn it into a ClipRect
  expector.addExpectation(smaller_clip, ClipOp::kIntersect, false);
  cull_dl->Dispatch(expector);
}

TEST_F(DisplayListTest, ClipPathOvalCulling) {
  auto clip = SkRect::MakeLTRB(10.0f, 10.0f, 20.0f, 20.0f);
  // A 10x10 rectangle extends 5x5 from the center to each corner. To have
  // an oval that encompasses that rectangle, the radius must be at least
  // length(5, 5), or 7.071+ so we expand the radius 5 square clip by 2.072
  // on each side to barely contain the corners of the square.
  auto encompassing_oval = clip.makeOutset(2.072f, 2.072f);
  SkPath path;
  path.addOval(encompassing_oval);

  DisplayListBuilder cull_builder;
  cull_builder.ClipRect(clip, ClipOp::kIntersect, false);
  cull_builder.ClipPath(path, ClipOp::kIntersect, false);
  auto cull_dl = cull_builder.Build();

  CLIP_EXPECTOR(expector);
  expector.addExpectation(clip, ClipOp::kIntersect, false);
  cull_dl->Dispatch(expector);
}

TEST_F(DisplayListTest, ClipPathOvalNonCulling) {
  auto clip = SkRect::MakeLTRB(10.0f, 10.0f, 20.0f, 20.0f);
  // A 10x10 rectangle extends 5x5 from the center to each corner. To have
  // an oval that encompasses that rectangle, the radius must be at least
  // length(5, 5), or 7.071+ so we expand the radius 5 square clip by 2.072
  // on each side to barely exclude the corners of the square.
  auto non_encompassing_oval = clip.makeOutset(2.071f, 2.071f);
  SkPath path;
  path.addOval(non_encompassing_oval);

  DisplayListBuilder cull_builder;
  cull_builder.ClipRect(clip, ClipOp::kIntersect, false);
  cull_builder.ClipPath(path, ClipOp::kIntersect, false);
  auto cull_dl = cull_builder.Build();

  CLIP_EXPECTOR(expector);
  expector.addExpectation(clip, ClipOp::kIntersect, false);
  // Builder will not cull this clip, but it will turn it into a ClipOval
  expector.addOvalExpectation(non_encompassing_oval, ClipOp::kIntersect, false);
  cull_dl->Dispatch(expector);
}

TEST_F(DisplayListTest, ClipPathRRectCulling) {
  auto clip = SkRect::MakeLTRB(10.0f, 10.0f, 20.0f, 20.0f);
  auto rrect = SkRRect::MakeRectXY(clip.makeOutset(2.0f, 2.0f), 2.0f, 2.0f);
  ASSERT_FALSE(rrect.isOval());
  SkPath path;
  path.addRRect(rrect);

  DisplayListBuilder cull_builder;
  cull_builder.ClipRect(clip, ClipOp::kIntersect, false);
  cull_builder.ClipPath(path, ClipOp::kIntersect, false);
  auto cull_dl = cull_builder.Build();

  CLIP_EXPECTOR(expector);
  expector.addExpectation(clip, ClipOp::kIntersect, false);
  cull_dl->Dispatch(expector);
}

TEST_F(DisplayListTest, ClipPathRRectNonCulling) {
  auto clip = SkRect::MakeLTRB(10.0f, 10.0f, 20.0f, 20.0f);
  auto rrect = SkRRect::MakeRectXY(clip.makeOutset(1.0f, 1.0f), 4.0f, 4.0f);
  ASSERT_FALSE(rrect.isOval());
  SkPath path;
  path.addRRect(rrect);

  DisplayListBuilder cull_builder;
  cull_builder.ClipRect(clip, ClipOp::kIntersect, false);
  cull_builder.ClipPath(path, ClipOp::kIntersect, false);
  auto cull_dl = cull_builder.Build();

  CLIP_EXPECTOR(expector);
  expector.addExpectation(clip, ClipOp::kIntersect, false);
  // Builder will not cull this clip, but it will turn it into a ClipRRect
  expector.addExpectation(rrect, ClipOp::kIntersect, false);
  cull_dl->Dispatch(expector);
}

TEST_F(DisplayListTest, RecordLargeVertices) {
  constexpr size_t vertex_count = 2000000;
  auto points = std::vector<DlPoint>();
  points.reserve(vertex_count);
  auto colors = std::vector<DlColor>();
  colors.reserve(vertex_count);
  for (size_t i = 0; i < vertex_count; i++) {
    colors.emplace_back(DlColor(-i));
    points.emplace_back(((i & 1) == 0) ? DlPoint(-i, i) : DlPoint(i, i));
  }
  ASSERT_EQ(points.size(), vertex_count);
  ASSERT_EQ(colors.size(), vertex_count);
  auto vertices = DlVertices::Make(DlVertexMode::kTriangleStrip, vertex_count,
                                   points.data(), points.data(), colors.data());
  ASSERT_GT(vertices->size(), 1u << 24);
  auto backdrop = DlImageFilter::MakeBlur(5.0f, 5.0f, DlTileMode::kDecal);

  for (int i = 0; i < 1000; i++) {
    DisplayListBuilder builder;
    for (int j = 0; j < 16; j++) {
      builder.SaveLayer(nullptr, nullptr, backdrop.get());
      builder.DrawVertices(vertices, DlBlendMode::kSrcOver, DlPaint());
      builder.Restore();
    }
    auto dl = builder.Build();
  }
}

TEST_F(DisplayListTest, DrawRectRRectPromoteToDrawRect) {
  SkRect rect = SkRect::MakeLTRB(10.0f, 10.0f, 20.0f, 20.0f);

  DisplayListBuilder builder;
  builder.DrawRRect(SkRRect::MakeRect(rect), DlPaint());
  auto dl = builder.Build();

  DisplayListBuilder expected;
  expected.DrawRect(rect, DlPaint());
  auto expect_dl = expected.Build();

  ASSERT_TRUE(DisplayListsEQ_Verbose(dl, expect_dl));
}

TEST_F(DisplayListTest, DrawOvalRRectPromoteToDrawOval) {
  SkRect rect = SkRect::MakeLTRB(10.0f, 10.0f, 20.0f, 20.0f);

  DisplayListBuilder builder;
  builder.DrawRRect(SkRRect::MakeOval(rect), DlPaint());
  auto dl = builder.Build();

  DisplayListBuilder expected;
  expected.DrawOval(rect, DlPaint());
  auto expect_dl = expected.Build();

  ASSERT_TRUE(DisplayListsEQ_Verbose(dl, expect_dl));
}

TEST_F(DisplayListTest, DrawRectPathPromoteToDrawRect) {
  SkRect rect = SkRect::MakeLTRB(10.0f, 10.0f, 20.0f, 20.0f);

  DisplayListBuilder builder;
  builder.DrawPath(SkPath::Rect(rect), DlPaint());
  auto dl = builder.Build();

  DisplayListBuilder expected;
  expected.DrawRect(rect, DlPaint());
  auto expect_dl = expected.Build();

  // Support for this will be re-added soon, until then verify that we
  // do not promote.
  ASSERT_TRUE(DisplayListsNE_Verbose(dl, expect_dl));
}

TEST_F(DisplayListTest, DrawOvalPathPromoteToDrawOval) {
  SkRect rect = SkRect::MakeLTRB(10.0f, 10.0f, 20.0f, 20.0f);

  DisplayListBuilder builder;
  builder.DrawPath(SkPath::Oval(rect), DlPaint());
  auto dl = builder.Build();

  DisplayListBuilder expected;
  expected.DrawOval(rect, DlPaint());
  auto expect_dl = expected.Build();

  // Support for this will be re-added soon, until then verify that we
  // do not promote.
  ASSERT_TRUE(DisplayListsNE_Verbose(dl, expect_dl));
}

TEST_F(DisplayListTest, DrawRRectPathPromoteToDrawRRect) {
  SkRect rect = SkRect::MakeLTRB(10.0f, 10.0f, 20.0f, 20.0f);
  SkRRect rrect = SkRRect::MakeRectXY(rect, 2.0f, 2.0f);

  DisplayListBuilder builder;
  builder.DrawPath(SkPath::RRect(rrect), DlPaint());
  auto dl = builder.Build();

  DisplayListBuilder expected;
  expected.DrawRRect(rrect, DlPaint());
  auto expect_dl = expected.Build();

  // Support for this will be re-added soon, until then verify that we
  // do not promote.
  ASSERT_TRUE(DisplayListsNE_Verbose(dl, expect_dl));
}

TEST_F(DisplayListTest, DrawRectRRectPathPromoteToDrawRect) {
  SkRect rect = SkRect::MakeLTRB(10.0f, 10.0f, 20.0f, 20.0f);
  SkRRect rrect = SkRRect::MakeRect(rect);

  DisplayListBuilder builder;
  builder.DrawPath(SkPath::RRect(rrect), DlPaint());
  auto dl = builder.Build();

  DisplayListBuilder expected;
  expected.DrawRect(rect, DlPaint());
  auto expect_dl = expected.Build();

  // Support for this will be re-added soon, until then verify that we
  // do not promote.
  ASSERT_TRUE(DisplayListsNE_Verbose(dl, expect_dl));
}

TEST_F(DisplayListTest, DrawOvalRRectPathPromoteToDrawOval) {
  SkRect rect = SkRect::MakeLTRB(10.0f, 10.0f, 20.0f, 20.0f);
  SkRRect rrect = SkRRect::MakeOval(rect);

  DisplayListBuilder builder;
  builder.DrawPath(SkPath::RRect(rrect), DlPaint());
  auto dl = builder.Build();

  DisplayListBuilder expected;
  expected.DrawOval(rect, DlPaint());
  auto expect_dl = expected.Build();

  // Support for this will be re-added soon, until then verify that we
  // do not promote.
  ASSERT_TRUE(DisplayListsNE_Verbose(dl, expect_dl));
}

TEST_F(DisplayListTest, ClipRectRRectPromoteToClipRect) {
  SkRect clip_rect = SkRect::MakeLTRB(10.0f, 10.0f, 20.0f, 20.0f);
  SkRect draw_rect = clip_rect.makeOutset(2.0f, 2.0f);

  DisplayListBuilder builder;
  builder.ClipRRect(SkRRect::MakeRect(clip_rect), ClipOp::kIntersect, false);
  // Include a rendering op in case DlBuilder ever removes unneeded clips
  builder.DrawRect(draw_rect, DlPaint());
  auto dl = builder.Build();

  DisplayListBuilder expected;
  expected.ClipRect(clip_rect, ClipOp::kIntersect, false);
  expected.DrawRect(draw_rect, DlPaint());
  auto expect_dl = expected.Build();

  ASSERT_TRUE(DisplayListsEQ_Verbose(dl, expect_dl));
}

TEST_F(DisplayListTest, ClipOvalRRectPromoteToClipOval) {
  SkRect clip_rect = SkRect::MakeLTRB(10.0f, 10.0f, 20.0f, 20.0f);
  SkRect draw_rect = clip_rect.makeOutset(2.0f, 2.0f);

  DisplayListBuilder builder;
  builder.ClipRRect(SkRRect::MakeOval(clip_rect), ClipOp::kIntersect, false);
  // Include a rendering op in case DlBuilder ever removes unneeded clips
  builder.DrawRect(draw_rect, DlPaint());
  auto dl = builder.Build();

  DisplayListBuilder expected;
  expected.ClipOval(clip_rect, ClipOp::kIntersect, false);
  expected.DrawRect(draw_rect, DlPaint());
  auto expect_dl = expected.Build();

  ASSERT_TRUE(DisplayListsEQ_Verbose(dl, expect_dl));
}

TEST_F(DisplayListTest, ClipRectPathPromoteToClipRect) {
  SkRect clip_rect = SkRect::MakeLTRB(10.0f, 10.0f, 20.0f, 20.0f);
  SkRect draw_rect = clip_rect.makeOutset(2.0f, 2.0f);
  SkPath clip_path = SkPath::Rect(clip_rect);
  ASSERT_TRUE(clip_path.isRect(nullptr));
  ASSERT_FALSE(clip_path.isInverseFillType());

  DisplayListBuilder builder;
  builder.ClipPath(clip_path, ClipOp::kIntersect, false);
  // Include a rendering op in case DlBuilder ever removes unneeded clips
  builder.DrawRect(draw_rect, DlPaint());
  auto dl = builder.Build();

  DisplayListBuilder expected;
  expected.ClipRect(clip_rect, ClipOp::kIntersect, false);
  expected.DrawRect(draw_rect, DlPaint());
  auto expect_dl = expected.Build();

  ASSERT_TRUE(DisplayListsEQ_Verbose(dl, expect_dl));
}

TEST_F(DisplayListTest, ClipOvalPathPromoteToClipOval) {
  SkRect clip_rect = SkRect::MakeLTRB(10.0f, 10.0f, 20.0f, 20.0f);
  SkRect draw_rect = clip_rect.makeOutset(2.0f, 2.0f);
  SkPath clip_path = SkPath::Oval(clip_rect);
  ASSERT_TRUE(clip_path.isOval(nullptr));
  ASSERT_FALSE(clip_path.isInverseFillType());

  DisplayListBuilder builder;
  builder.ClipPath(clip_path, ClipOp::kIntersect, false);
  // Include a rendering op in case DlBuilder ever removes unneeded clips
  builder.DrawRect(draw_rect, DlPaint());
  auto dl = builder.Build();

  DisplayListBuilder expected;
  expected.ClipOval(clip_rect, ClipOp::kIntersect, false);
  expected.DrawRect(draw_rect, DlPaint());
  auto expect_dl = expected.Build();

  ASSERT_TRUE(DisplayListsEQ_Verbose(dl, expect_dl));
}

TEST_F(DisplayListTest, ClipRRectPathPromoteToClipRRect) {
  SkRect clip_rect = SkRect::MakeLTRB(10.0f, 10.0f, 20.0f, 20.0f);
  SkRRect clip_rrect = SkRRect::MakeRectXY(clip_rect, 2.0f, 2.0f);
  SkRect draw_rect = clip_rect.makeOutset(2.0f, 2.0f);
  SkPath clip_path = SkPath::RRect(clip_rrect);
  ASSERT_TRUE(clip_path.isRRect(nullptr));
  ASSERT_FALSE(clip_path.isInverseFillType());

  DisplayListBuilder builder;
  builder.ClipPath(clip_path, ClipOp::kIntersect, false);
  // Include a rendering op in case DlBuilder ever removes unneeded clips
  builder.DrawRect(draw_rect, DlPaint());
  auto dl = builder.Build();

  DisplayListBuilder expected;
  expected.ClipRRect(clip_rrect, ClipOp::kIntersect, false);
  expected.DrawRect(draw_rect, DlPaint());
  auto expect_dl = expected.Build();

  ASSERT_TRUE(DisplayListsEQ_Verbose(dl, expect_dl));
}

TEST_F(DisplayListTest, ClipRectInversePathNoPromoteToClipRect) {
  SkRect clip_rect = SkRect::MakeLTRB(10.0f, 10.0f, 20.0f, 20.0f);
  SkRect draw_rect = clip_rect.makeOutset(2.0f, 2.0f);
  SkPath clip_path = SkPath::Rect(clip_rect);
  clip_path.toggleInverseFillType();
  ASSERT_TRUE(clip_path.isRect(nullptr));
  ASSERT_TRUE(clip_path.isInverseFillType());

  DisplayListBuilder builder;
  builder.ClipPath(clip_path, ClipOp::kIntersect, false);
  // Include a rendering op in case DlBuilder ever removes unneeded clips
  builder.DrawRect(draw_rect, DlPaint());
  auto dl = builder.Build();

  // Non-promoting tests can't use DL comparisons to verify that the
  // promotion isn't happening because the test and expectation builders
  // would both apply or not apply the same optimization. For this case
  // we use the CLIP_EXPECTOR instead to see exactly which type of
  // clip operation was recorded.
  CLIP_EXPECTOR(expector);
  expector.addExpectation(clip_path, ClipOp::kIntersect, false);
  dl->Dispatch(expector);
}

TEST_F(DisplayListTest, ClipOvalInversePathNoPromoteToClipOval) {
  SkRect clip_rect = SkRect::MakeLTRB(10.0f, 10.0f, 20.0f, 20.0f);
  SkRect draw_rect = clip_rect.makeOutset(2.0f, 2.0f);
  SkPath clip_path = SkPath::Oval(clip_rect);
  clip_path.toggleInverseFillType();
  ASSERT_TRUE(clip_path.isOval(nullptr));
  ASSERT_TRUE(clip_path.isInverseFillType());

  DisplayListBuilder builder;
  builder.ClipPath(clip_path, ClipOp::kIntersect, false);
  // Include a rendering op in case DlBuilder ever removes unneeded clips
  builder.DrawRect(draw_rect, DlPaint());
  auto dl = builder.Build();

  // Non-promoting tests can't use DL comparisons to verify that the
  // promotion isn't happening because the test and expectation builders
  // would both apply or not apply the same optimization. For this case
  // we use the CLIP_EXPECTOR instead to see exactly which type of
  // clip operation was recorded.
  CLIP_EXPECTOR(expector);
  expector.addExpectation(clip_path, ClipOp::kIntersect, false);
  dl->Dispatch(expector);
}

TEST_F(DisplayListTest, ClipRRectInversePathNoPromoteToClipRRect) {
  SkRect clip_rect = SkRect::MakeLTRB(10.0f, 10.0f, 20.0f, 20.0f);
  SkRRect clip_rrect = SkRRect::MakeRectXY(clip_rect, 2.0f, 2.0f);
  SkRect draw_rect = clip_rect.makeOutset(2.0f, 2.0f);
  SkPath clip_path = SkPath::RRect(clip_rrect);
  clip_path.toggleInverseFillType();
  ASSERT_TRUE(clip_path.isRRect(nullptr));
  ASSERT_TRUE(clip_path.isInverseFillType());

  DisplayListBuilder builder;
  builder.ClipPath(clip_path, ClipOp::kIntersect, false);
  // Include a rendering op in case DlBuilder ever removes unneeded clips
  builder.DrawRect(draw_rect, DlPaint());
  auto dl = builder.Build();

  // Non-promoting tests can't use DL comparisons to verify that the
  // promotion isn't happening because the test and expectation builders
  // would both apply or not apply the same optimization. For this case
  // we use the CLIP_EXPECTOR instead to see exactly which type of
  // clip operation was recorded.
  CLIP_EXPECTOR(expector);
  expector.addExpectation(clip_path, ClipOp::kIntersect, false);
  dl->Dispatch(expector);
}

TEST_F(DisplayListTest, ClipRectRRectPathPromoteToClipRect) {
  SkRect clip_rect = SkRect::MakeLTRB(10.0f, 10.0f, 20.0f, 20.0f);
  SkRRect clip_rrect = SkRRect::MakeRect(clip_rect);
  SkRect draw_rect = clip_rect.makeOutset(2.0f, 2.0f);
  SkPath clip_path = SkPath::RRect(clip_rrect);
  ASSERT_TRUE(clip_path.isRRect(nullptr));
  ASSERT_FALSE(clip_path.isInverseFillType());

  DisplayListBuilder builder;
  builder.ClipPath(clip_path, ClipOp::kIntersect, false);
  // Include a rendering op in case DlBuilder ever removes unneeded clips
  builder.DrawRect(draw_rect, DlPaint());
  auto dl = builder.Build();

  DisplayListBuilder expected;
  expected.ClipRect(clip_rect, ClipOp::kIntersect, false);
  expected.DrawRect(draw_rect, DlPaint());
  auto expect_dl = expected.Build();

  ASSERT_TRUE(DisplayListsEQ_Verbose(dl, expect_dl));
}

TEST_F(DisplayListTest, ClipOvalRRectPathPromoteToClipOval) {
  SkRect clip_rect = SkRect::MakeLTRB(10.0f, 10.0f, 20.0f, 20.0f);
  SkRRect clip_rrect = SkRRect::MakeOval(clip_rect);
  SkRect draw_rect = clip_rect.makeOutset(2.0f, 2.0f);
  SkPath clip_path = SkPath::RRect(clip_rrect);
  ASSERT_TRUE(clip_path.isRRect(nullptr));
  ASSERT_FALSE(clip_path.isInverseFillType());

  DisplayListBuilder builder;
  builder.ClipPath(clip_path, ClipOp::kIntersect, false);
  // Include a rendering op in case DlBuilder ever removes unneeded clips
  builder.DrawRect(draw_rect, DlPaint());
  auto dl = builder.Build();

  DisplayListBuilder expected;
  expected.ClipOval(clip_rect, ClipOp::kIntersect, false);
  expected.DrawRect(draw_rect, DlPaint());
  auto expect_dl = expected.Build();

  ASSERT_TRUE(DisplayListsEQ_Verbose(dl, expect_dl));
}

TEST_F(DisplayListTest, BoundedRenderOpsDoNotReportUnbounded) {
  static const SkRect root_cull = SkRect::MakeLTRB(100, 100, 200, 200);
  static const SkRect draw_rect = SkRect::MakeLTRB(110, 110, 190, 190);

  using Renderer = const std::function<void(DlCanvas&)>;
  auto test_bounded = [](const std::string& label, const Renderer& renderer) {
    {
      DisplayListBuilder builder(root_cull);
      renderer(builder);
      auto display_list = builder.Build();

      EXPECT_EQ(display_list->bounds(), draw_rect) << label;
      EXPECT_FALSE(display_list->root_is_unbounded()) << label;
    }

    {
      DisplayListBuilder builder(root_cull);
      builder.SaveLayer(nullptr, nullptr);
      renderer(builder);
      builder.Restore();
      auto display_list = builder.Build();

      EXPECT_EQ(display_list->bounds(), draw_rect) << label;
      EXPECT_FALSE(display_list->root_is_unbounded()) << label;

      SAVE_LAYER_EXPECTOR(expector);
      expector  //
          .addDetail(label)
          .addExpectation([](const SaveLayerOptions& options) {
            return !options.content_is_unbounded();
          });
      display_list->Dispatch(expector);
    }
  };

  test_bounded("DrawLine", [](DlCanvas& builder) {
    builder.DrawLine(
        SkPoint{draw_rect.left() + 1.0f, draw_rect.top() + 1.0f},
        SkPoint{draw_rect.right() - 1.0f, draw_rect.top() + 1.0f},
        DlPaint().setStrokeWidth(2.0f).setStrokeCap(DlStrokeCap::kSquare));
    builder.DrawLine(
        SkPoint{draw_rect.left() + 1.0f, draw_rect.bottom() - 1.0f},
        SkPoint{draw_rect.right() - 1.0f, draw_rect.bottom() - 1.0f},
        DlPaint().setStrokeWidth(2.0f).setStrokeCap(DlStrokeCap::kSquare));
  });

  test_bounded("DrawDashedLine", [](DlCanvas& builder) {
    builder.DrawDashedLine(
        {draw_rect.left() + 1.0f, draw_rect.top() + 1.0f},
        {draw_rect.right() - 1.0f, draw_rect.top() + 1.0f},
        // must fill 80 x 80 square with on dashes at both
        // ends - 40 + 25 + 40 == 105 so it will be on
        // at both ends
        40.0f, 25.0f,
        DlPaint().setStrokeWidth(2.0f).setStrokeCap(DlStrokeCap::kSquare));
    builder.DrawDashedLine(
        {draw_rect.left() + 1.0f, draw_rect.bottom() - 1.0f},
        {draw_rect.right() - 1.0f, draw_rect.bottom() - 1.0f},
        // must fill 80 x 80 square with on dashes at both
        // ends - 40 + 25 + 40 == 105 so it will be on
        // at both ends
        40.0f, 25.0f,
        DlPaint().setStrokeWidth(2.0f).setStrokeCap(DlStrokeCap::kSquare));
  });

  test_bounded("DrawRect", [](DlCanvas& builder) {
    builder.DrawRect(draw_rect, DlPaint());
  });

  test_bounded("DrawOval", [](DlCanvas& builder) {
    builder.DrawOval(draw_rect, DlPaint());
  });

  test_bounded("DrawCircle", [](DlCanvas& builder) {
    builder.DrawCircle(draw_rect.center(), draw_rect.width() * 0.5f, DlPaint());
  });

  test_bounded("DrawRRect", [](DlCanvas& builder) {
    builder.DrawRRect(SkRRect::MakeRectXY(draw_rect, 5.0f, 5.0f), DlPaint());
  });

  test_bounded("DrawDRRect", [](DlCanvas& builder) {
    builder.DrawDRRect(
        SkRRect::MakeRectXY(draw_rect, 5.0f, 5.0f),
        SkRRect::MakeRectXY(draw_rect.makeInset(10.0f, 10.0f), 5.0f, 5.0f),
        DlPaint());
  });

  test_bounded("DrawArc", [](DlCanvas& builder) {
    builder.DrawArc(draw_rect, 45.0f, 355.0f, false, DlPaint());
  });

  test_bounded("DrawPathEvenOdd", [](DlCanvas& builder) {
    SkPath path = SkPath::Rect(draw_rect);
    path.setFillType(SkPathFillType::kEvenOdd);
    builder.DrawPath(path, DlPaint());
  });

  test_bounded("DrawPathWinding", [](DlCanvas& builder) {
    SkPath path = SkPath::Rect(draw_rect);
    path.setFillType(SkPathFillType::kWinding);
    builder.DrawPath(path, DlPaint());
  });

  auto test_draw_points = [&test_bounded](PointMode mode) {
    std::stringstream ss;
    ss << "DrawPoints(" << mode << ")";
    test_bounded(ss.str(), [mode](DlCanvas& builder) {
      SkPoint points[4] = {
          {draw_rect.left() + 1.0f, draw_rect.top() + 1.0f},
          {draw_rect.right() - 1.0f, draw_rect.top() + 1.0f},
          {draw_rect.right() - 1.0f, draw_rect.bottom() - 1.0f},
          {draw_rect.left() + 1.0f, draw_rect.bottom() - 1.0f},
      };
      DlPaint paint;
      paint.setStrokeWidth(2.0f);
      paint.setDrawStyle(DlDrawStyle::kStroke);
      // bounds accumulation doesn't examine the points to see if they
      // have diagonals so Square caps may have their corners accumulated
      // but Round caps will always pad by only half the stroke width.
      paint.setStrokeCap(DlStrokeCap::kRound);

      builder.DrawPoints(mode, 4, points, paint);
    });
  };

  test_draw_points(PointMode::kPoints);
  test_draw_points(PointMode::kLines);
  test_draw_points(PointMode::kPolygon);

  test_bounded("DrawVerticesTriangles", [](DlCanvas& builder) {
    DlPoint points[6] = {
        {draw_rect.left(), draw_rect.top()},
        {draw_rect.right(), draw_rect.top()},
        {draw_rect.right(), draw_rect.bottom()},
        {draw_rect.right(), draw_rect.bottom()},
        {draw_rect.left(), draw_rect.bottom()},
        {draw_rect.left(), draw_rect.top()},
    };
    DlVertices::Builder vertices(DlVertexMode::kTriangles, 6,
                                 DlVertices::Builder::kNone, 0);
    vertices.store_vertices(points);
    builder.DrawVertices(vertices.build(), DlBlendMode::kSrcOver, DlPaint());
  });

  test_bounded("DrawVerticesTriangleStrip", [](DlCanvas& builder) {
    DlPoint points[6] = {
        {draw_rect.left(), draw_rect.top()},
        {draw_rect.right(), draw_rect.top()},
        {draw_rect.right(), draw_rect.bottom()},
        {draw_rect.left(), draw_rect.bottom()},
        {draw_rect.left(), draw_rect.top()},
        {draw_rect.right(), draw_rect.top()},
    };
    DlVertices::Builder vertices(DlVertexMode::kTriangleStrip, 6,
                                 DlVertices::Builder::kNone, 0);
    vertices.store_vertices(points);
    builder.DrawVertices(vertices.build(), DlBlendMode::kSrcOver, DlPaint());
  });

  test_bounded("DrawVerticesTriangleFan", [](DlCanvas& builder) {
    DlPoint points[6] = {
        ToDlPoint(draw_rect.center()),
        {draw_rect.left(), draw_rect.top()},
        {draw_rect.right(), draw_rect.top()},
        {draw_rect.right(), draw_rect.bottom()},
        {draw_rect.left(), draw_rect.bottom()},
    };
    DlVertices::Builder vertices(DlVertexMode::kTriangleFan, 5,
                                 DlVertices::Builder::kNone, 0);
    vertices.store_vertices(points);
    builder.DrawVertices(vertices.build(), DlBlendMode::kSrcOver, DlPaint());
  });

  test_bounded("DrawImage", [](DlCanvas& builder) {
    auto image = MakeTestImage(draw_rect.width(), draw_rect.height(), 5);
    builder.DrawImage(image, SkPoint{draw_rect.left(), draw_rect.top()},
                      DlImageSampling::kLinear);
  });

  test_bounded("DrawImageRect", [](DlCanvas& builder) {
    auto image = MakeTestImage(root_cull.width(), root_cull.height(), 5);
    builder.DrawImageRect(image, draw_rect, DlImageSampling::kLinear);
  });

  test_bounded("DrawImageNine", [](DlCanvas& builder) {
    auto image = MakeTestImage(root_cull.width(), root_cull.height(), 5);
    SkIRect center = image->bounds().makeInset(10, 10);
    builder.DrawImageNine(image, center, draw_rect, DlFilterMode::kLinear);
  });

  test_bounded("DrawTextBlob", [](DlCanvas& builder) {
    SkFont font = CreateTestFontOfSize(20.0f);
    sk_sp<SkTypeface> face = font.refTypeface();
    ASSERT_TRUE(face);
    ASSERT_TRUE(face->countGlyphs() > 0) << "No glyphs in font";
    auto blob = SkTextBlob::MakeFromText("Hello", 5, font);

    // Make sure the blob fits within the draw_rect bounds.
    ASSERT_LT(blob->bounds().width(), draw_rect.width());
    ASSERT_LT(blob->bounds().height(), draw_rect.height());

    // Draw once at upper left and again at lower right to fill the bounds.
    builder.DrawTextBlob(blob, draw_rect.left() - blob->bounds().left(),
                         draw_rect.top() - blob->bounds().top(), DlPaint());
    builder.DrawTextBlob(blob, draw_rect.right() - blob->bounds().right(),
                         draw_rect.bottom() - blob->bounds().bottom(),
                         DlPaint());
  });

  test_bounded("DrawTextFrame", [](DlCanvas& builder) {
    SkFont font = CreateTestFontOfSize(20.0f);
    sk_sp<SkTypeface> face = font.refTypeface();
    ASSERT_TRUE(face);
    ASSERT_TRUE(face->countGlyphs() > 0) << "No glyphs in font";
    auto blob = SkTextBlob::MakeFromText("Hello", 5, font);
    auto frame = impeller::MakeTextFrameFromTextBlobSkia(blob);

    // Make sure the blob fits within the draw_rect bounds.
    ASSERT_LT(blob->bounds().width(), draw_rect.width());
    ASSERT_LT(blob->bounds().height(), draw_rect.height());

    // Draw once at upper left and again at lower right to fill the bounds.
    builder.DrawTextFrame(frame, draw_rect.left() - blob->bounds().left(),
                          draw_rect.top() - blob->bounds().top(), DlPaint());
    builder.DrawTextFrame(frame, draw_rect.right() - blob->bounds().right(),
                          draw_rect.bottom() - blob->bounds().bottom(),
                          DlPaint());
  });

  test_bounded("DrawBoundedDisplayList", [](DlCanvas& builder) {
    DisplayListBuilder nested_builder(root_cull);
    nested_builder.DrawRect(draw_rect, DlPaint());
    auto nested_display_list = nested_builder.Build();

    EXPECT_EQ(nested_display_list->bounds(), draw_rect);
    ASSERT_FALSE(nested_display_list->root_is_unbounded());

    builder.DrawDisplayList(nested_display_list);
  });

  test_bounded("DrawShadow", [](DlCanvas& builder) {
    SkPath path = SkPath::Rect(draw_rect.makeInset(20, 20));
    DlScalar elevation = 2.0f;
    DlScalar dpr = 1.0f;
    auto shadow_bounds =
        DlCanvas::ComputeShadowBounds(path, elevation, dpr, SkMatrix::I());

    // Make sure the shadow fits within the draw_rect bounds.
    ASSERT_LT(shadow_bounds.width(), draw_rect.width());
    ASSERT_LT(shadow_bounds.height(), draw_rect.height());

    // Draw once at upper left and again at lower right to fill the bounds.
    SkPath pathUL =  //
        SkPath(path).offset(draw_rect.left() - shadow_bounds.left(),
                            draw_rect.top() - shadow_bounds.top());
    builder.DrawShadow(pathUL, DlColor::kMagenta(), elevation, true, dpr);
    SkPath pathLR =
        SkPath(path).offset(draw_rect.right() - shadow_bounds.right(),
                            draw_rect.bottom() - shadow_bounds.bottom());
    builder.DrawShadow(pathLR, DlColor::kMagenta(), elevation, true, dpr);
  });

  for (int i = 0; i <= static_cast<int>(DlBlendMode::kLastMode); i++) {
    DlBlendMode mode = static_cast<DlBlendMode>(i);
    if (mode == DlBlendMode::kDst) {
      // No way to make kDst a non-nop
      continue;
    }
    std::stringstream ss;
    ss << "DrawRectWith" << mode;
    test_bounded(ss.str(), [mode](DlCanvas& builder) {
      // alpha of 0x7f prevents kDstIn from being a nop
      builder.DrawRect(draw_rect, DlPaint().setBlendMode(mode).setAlpha(0x7f));
    });
  }
}

TEST_F(DisplayListTest, UnboundedRenderOpsAreReportedUnlessClipped) {
  static const DlRect root_cull = DlRect::MakeLTRB(100, 100, 200, 200);
  static const DlRect clip_rect = DlRect::MakeLTRB(120, 120, 180, 180);
  static const DlRect draw_rect = DlRect::MakeLTRB(110, 110, 190, 190);

  using Renderer = const std::function<void(DlCanvas&)>;
  auto test_unbounded = [](const std::string& label,  //
                           const Renderer& renderer,
                           int extra_save_layers = 0) {
    {
      DisplayListBuilder builder(root_cull);
      renderer(builder);
      auto display_list = builder.Build();

      EXPECT_EQ(display_list->GetBounds(), root_cull) << label;
      EXPECT_TRUE(display_list->root_is_unbounded()) << label;
    }

    {
      DisplayListBuilder builder(root_cull);
      builder.ClipRect(clip_rect);
      renderer(builder);
      auto display_list = builder.Build();

      EXPECT_EQ(display_list->GetBounds(), clip_rect) << label;
      EXPECT_FALSE(display_list->root_is_unbounded()) << label;
    }

    {
      DisplayListBuilder builder(root_cull);
      builder.SaveLayer(nullptr, nullptr);
      renderer(builder);
      builder.Restore();
      auto display_list = builder.Build();

      EXPECT_EQ(display_list->GetBounds(), root_cull) << label;
      EXPECT_FALSE(display_list->root_is_unbounded()) << label;

      SAVE_LAYER_EXPECTOR(expector);
      expector  //
          .addDetail(label)
          .addExpectation([](const SaveLayerOptions& options) {
            return options.content_is_unbounded();
          });
      for (int i = 0; i < extra_save_layers; i++) {
        expector.addOpenExpectation();
      }
      display_list->Dispatch(expector);
    }

    {
      DisplayListBuilder builder(root_cull);
      builder.SaveLayer(nullptr, nullptr);
      builder.ClipRect(clip_rect);
      renderer(builder);
      builder.Restore();
      auto display_list = builder.Build();

      EXPECT_EQ(display_list->GetBounds(), clip_rect) << label;
      EXPECT_FALSE(display_list->root_is_unbounded()) << label;

      SAVE_LAYER_EXPECTOR(expector);
      expector  //
          .addDetail(label)
          .addExpectation([](const SaveLayerOptions& options) {
            return !options.content_is_unbounded();
          });
      for (int i = 0; i < extra_save_layers; i++) {
        expector.addOpenExpectation();
      }
      display_list->Dispatch(expector);
    }
  };

  test_unbounded("DrawPaint", [](DlCanvas& builder) {  //
    builder.DrawPaint(DlPaint());
  });

  test_unbounded("DrawColor", [](DlCanvas& builder) {
    builder.DrawColor(DlColor::kMagenta(), DlBlendMode::kSrc);
  });

  test_unbounded("Clear", [](DlCanvas& builder) {  //
    builder.Clear(DlColor::kMagenta());
  });

  test_unbounded("DrawPathEvenOddInverted", [](DlCanvas& builder) {
    SkPath path = SkPath::Rect(ToSkRect(draw_rect));
    path.setFillType(SkPathFillType::kInverseEvenOdd);
    builder.DrawPath(path, DlPaint());
  });

  test_unbounded("DrawPathWindingInverted", [](DlCanvas& builder) {
    SkPath path = SkPath::Rect(ToSkRect(draw_rect));
    path.setFillType(SkPathFillType::kInverseWinding);
    builder.DrawPath(path, DlPaint());
  });

  test_unbounded("DrawUnboundedDisplayList", [](DlCanvas& builder) {
    DisplayListBuilder nested_builder(root_cull);
    nested_builder.DrawPaint(DlPaint());
    auto nested_display_list = nested_builder.Build();

    EXPECT_EQ(nested_display_list->GetBounds(), root_cull);
    ASSERT_TRUE(nested_display_list->root_is_unbounded());

    builder.DrawDisplayList(nested_display_list);
  });

  test_unbounded("DrawRectWithUnboundedImageFilter", [](DlCanvas& builder) {
    // clang-format off
    const DlScalar matrix[20] = {
      0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
      0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
      0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
      0.0f, 0.0f, 0.0f, 0.0f, 1.0f,
    };
    // clang-format on
    auto unbounded_cf = DlColorFilter::MakeMatrix(matrix);
    // ColorFilter must modify transparent black to be "unbounded"
    ASSERT_TRUE(unbounded_cf->modifies_transparent_black());
    auto unbounded_if = DlImageFilter::MakeColorFilter(unbounded_cf);
    DlRect output_bounds;
    // ImageFilter returns null from bounds queries if it is "unbounded"
    ASSERT_EQ(unbounded_if->map_local_bounds(draw_rect, output_bounds),
              nullptr);

    builder.DrawRect(draw_rect, DlPaint().setImageFilter(unbounded_if));
  });

  test_unbounded(
      "SaveLayerWithBackdropFilter",
      [](DlCanvas& builder) {
        auto filter = DlImageFilter::MakeBlur(3.0f, 3.0f, DlTileMode::kMirror);
        builder.SaveLayer(nullptr, nullptr, filter.get());
        builder.Restore();
      },
      1);
}

TEST_F(DisplayListTest, BackdropFilterCulledAlongsideClipAndTransform) {
  SkRect frame_bounds = SkRect::MakeWH(100.0f, 100.0f);
  SkRect frame_clip = frame_bounds.makeInset(0.5f, 0.5f);

  SkRect clip_rect = SkRect::MakeLTRB(40.0f, 40.0f, 60.0f, 60.0f);
  SkRect draw_rect1 = SkRect::MakeLTRB(10.0f, 10.0f, 20.0f, 20.0f);
  SkRect draw_rect2 = SkRect::MakeLTRB(45.0f, 20.0f, 55.0f, 55.0f);
  SkRect cull_rect = SkRect::MakeLTRB(1.0f, 1.0f, 99.0f, 30.0f);
  auto bdf_filter = DlImageFilter::MakeBlur(5.0f, 5.0f, DlTileMode::kClamp);

  ASSERT_TRUE(frame_bounds.contains(clip_rect));
  ASSERT_TRUE(frame_bounds.contains(draw_rect1));
  ASSERT_TRUE(frame_bounds.contains(draw_rect2));
  ASSERT_TRUE(frame_bounds.contains(cull_rect));

  ASSERT_TRUE(frame_clip.contains(clip_rect));
  ASSERT_TRUE(frame_clip.contains(draw_rect1));
  ASSERT_TRUE(frame_clip.contains(draw_rect2));
  ASSERT_TRUE(frame_clip.contains(cull_rect));

  ASSERT_FALSE(clip_rect.intersects(draw_rect1));
  ASSERT_TRUE(clip_rect.intersects(draw_rect2));

  ASSERT_FALSE(cull_rect.intersects(clip_rect));
  ASSERT_TRUE(cull_rect.intersects(draw_rect1));
  ASSERT_TRUE(cull_rect.intersects(draw_rect2));

  DisplayListBuilder builder(frame_bounds, true);
  builder.Save();
  {
    builder.Translate(0.1f, 0.1f);
    builder.ClipRect(frame_clip);
    builder.DrawRect(draw_rect1, DlPaint());
    // Should all be culled below
    builder.ClipRect(clip_rect);
    builder.Translate(0.1f, 0.1f);
    builder.SaveLayer(nullptr, nullptr, bdf_filter.get());
    {  //
      builder.DrawRect(clip_rect, DlPaint());
    }
    builder.Restore();
    // End of culling
  }
  builder.Restore();
  builder.DrawRect(draw_rect2, DlPaint());
  auto display_list = builder.Build();

  {
    DisplayListBuilder unculled(frame_bounds);
    display_list->Dispatch(ToReceiver(unculled), frame_bounds);
    auto unculled_dl = unculled.Build();

    EXPECT_TRUE(DisplayListsEQ_Verbose(display_list, unculled_dl));
  }

  {
    DisplayListBuilder culled(frame_bounds);
    display_list->Dispatch(ToReceiver(culled), cull_rect);
    auto culled_dl = culled.Build();

    EXPECT_TRUE(DisplayListsNE_Verbose(display_list, culled_dl));

    DisplayListBuilder expected(frame_bounds);
    expected.Save();
    {
      expected.Translate(0.1f, 0.1f);
      expected.ClipRect(frame_clip);
      expected.DrawRect(draw_rect1, DlPaint());
    }
    expected.Restore();
    expected.DrawRect(draw_rect2, DlPaint());
    auto expected_dl = expected.Build();

    EXPECT_TRUE(DisplayListsEQ_Verbose(culled_dl, expected_dl));
  }
}

TEST_F(DisplayListTest, RecordManyLargeDisplayListOperations) {
  DisplayListBuilder builder;

  // 2050 points is sizeof(DlPoint) * 2050 = 16400 bytes, this is more
  // than the page size of 16384 bytes.
  std::vector<DlPoint> points(2050);
  builder.DrawPoints(PointMode::kPoints, points.size(), points.data(),
                     DlPaint{});
  builder.DrawPoints(PointMode::kPoints, points.size(), points.data(),
                     DlPaint{});
  builder.DrawPoints(PointMode::kPoints, points.size(), points.data(),
                     DlPaint{});
  builder.DrawPoints(PointMode::kPoints, points.size(), points.data(),
                     DlPaint{});
  builder.DrawPoints(PointMode::kPoints, points.size(), points.data(),
                     DlPaint{});
  builder.DrawPoints(PointMode::kPoints, points.size(), points.data(),
                     DlPaint{});

  EXPECT_TRUE(!!builder.Build());
}

TEST_F(DisplayListTest, RecordSingleLargeDisplayListOperation) {
  DisplayListBuilder builder;

  std::vector<DlPoint> points(40000);
  builder.DrawPoints(PointMode::kPoints, points.size(), points.data(),
                     DlPaint{});

  EXPECT_TRUE(!!builder.Build());
}

TEST_F(DisplayListTest, DisplayListDetectsRuntimeEffect) {
  const auto runtime_effect = DlRuntimeEffect::MakeSkia(
      SkRuntimeEffect::MakeForShader(
          SkString("vec4 main(vec2 p) { return vec4(0); }"))
          .effect);
  auto color_source = DlColorSource::MakeRuntimeEffect(
      runtime_effect, {}, std::make_shared<std::vector<uint8_t>>());
  auto image_filter = DlImageFilter::MakeRuntimeEffect(
      runtime_effect, {}, std::make_shared<std::vector<uint8_t>>());

  {
    // Default - no runtime effects, supports group opacity
    DisplayListBuilder builder;
    DlPaint paint;

    builder.DrawRect(DlRect::MakeLTRB(0, 0, 50, 50), paint);
    EXPECT_TRUE(builder.Build()->can_apply_group_opacity());
  }

  {
    // Draw with RTE color source does not support group opacity
    DisplayListBuilder builder;
    DlPaint paint;

    paint.setColorSource(color_source);
    builder.DrawRect(DlRect::MakeLTRB(0, 0, 50, 50), paint);

    EXPECT_FALSE(builder.Build()->can_apply_group_opacity());
  }

  {
    // Draw with RTE image filter does not support group opacity
    DisplayListBuilder builder;
    DlPaint paint;

    paint.setImageFilter(image_filter);
    builder.DrawRect(DlRect::MakeLTRB(0, 0, 50, 50), paint);

    EXPECT_FALSE(builder.Build()->can_apply_group_opacity());
  }

  {
    // Draw with RTE color source inside SaveLayer does not support group
    // opacity on the SaveLayer, but does support it on the DisplayList
    DisplayListBuilder builder;
    DlPaint paint;

    builder.SaveLayer(nullptr, nullptr);
    paint.setColorSource(color_source);
    builder.DrawRect(DlRect::MakeLTRB(0, 0, 50, 50), paint);
    builder.Restore();

    auto display_list = builder.Build();
    EXPECT_TRUE(display_list->can_apply_group_opacity());

    SAVE_LAYER_EXPECTOR(expector);
    expector.addExpectation([](const SaveLayerOptions& options) {
      return !options.can_distribute_opacity();
    });
    display_list->Dispatch(expector);
  }

  {
    // Draw with RTE image filter inside SaveLayer does not support group
    // opacity on the SaveLayer, but does support it on the DisplayList
    DisplayListBuilder builder;
    DlPaint paint;

    builder.SaveLayer(nullptr, nullptr);
    paint.setImageFilter(image_filter);
    builder.DrawRect(DlRect::MakeLTRB(0, 0, 50, 50), paint);
    builder.Restore();

    auto display_list = builder.Build();
    EXPECT_TRUE(display_list->can_apply_group_opacity());

    SAVE_LAYER_EXPECTOR(expector);
    expector.addExpectation([](const SaveLayerOptions& options) {
      return !options.can_distribute_opacity();
    });
    display_list->Dispatch(expector);
  }

  {
    // Draw with RTE color source inside nested saveLayers does not support
    // group opacity on the inner SaveLayer, but does support it on the
    // outer SaveLayer and the DisplayList
    DisplayListBuilder builder;
    DlPaint paint;

    builder.SaveLayer(nullptr, nullptr);

    builder.SaveLayer(nullptr, nullptr);
    paint.setColorSource(color_source);
    builder.DrawRect(DlRect::MakeLTRB(0, 0, 50, 50), paint);
    paint.setColorSource(nullptr);
    builder.Restore();

    builder.SaveLayer(nullptr, nullptr);
    paint.setImageFilter(image_filter);
    // Make sure these DrawRects are non-overlapping otherwise the outer
    // SaveLayer and DisplayList will be incompatible due to overlaps
    builder.DrawRect(DlRect::MakeLTRB(60, 60, 100, 100), paint);
    paint.setImageFilter(nullptr);
    builder.Restore();

    builder.Restore();
    auto display_list = builder.Build();
    EXPECT_TRUE(display_list->can_apply_group_opacity());

    SAVE_LAYER_EXPECTOR(expector);
    expector.addExpectation([](const SaveLayerOptions& options) {
      // outer SaveLayer supports group opacity
      return options.can_distribute_opacity();
    });
    expector.addExpectation([](const SaveLayerOptions& options) {
      // first inner SaveLayer does not support group opacity
      return !options.can_distribute_opacity();
    });
    expector.addExpectation([](const SaveLayerOptions& options) {
      // second inner SaveLayer does not support group opacity
      return !options.can_distribute_opacity();
    });
    display_list->Dispatch(expector);
  }
}

}  // namespace testing
}  // namespace flutter
