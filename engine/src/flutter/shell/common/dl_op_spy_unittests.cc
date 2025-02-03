// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/display_list/display_list.h"
#include "flutter/display_list/dl_builder.h"
#include "flutter/display_list/testing/dl_test_snippets.h"
#include "flutter/shell/common/dl_op_spy.h"
#include "flutter/testing/testing.h"

namespace flutter {
namespace testing {

// The following macros demonstrate that the DlOpSpy class is equivalent
// to DisplayList::affects_transparent_surface() now that DisplayListBuilder
// implements operation culling.
// See https://github.com/flutter/flutter/issues/125403
#define ASSERT_DID_DRAW(spy, dl)                   \
  do {                                             \
    ASSERT_TRUE(spy.did_draw());                   \
    ASSERT_TRUE(dl->modifies_transparent_black()); \
  } while (0)

#define ASSERT_NO_DRAW(spy, dl)                     \
  do {                                              \
    ASSERT_FALSE(spy.did_draw());                   \
    ASSERT_FALSE(dl->modifies_transparent_black()); \
  } while (0)

TEST(DlOpSpy, DidDrawIsFalseByDefault) {
  DlOpSpy dl_op_spy;
  ASSERT_FALSE(dl_op_spy.did_draw());
}

TEST(DlOpSpy, EmptyDisplayList) {
  DisplayListBuilder builder;
  sk_sp<DisplayList> dl = builder.Build();
  DlOpSpy dl_op_spy;
  dl->Dispatch(dl_op_spy);
  ASSERT_NO_DRAW(dl_op_spy, dl);
}

TEST(DlOpSpy, SetColor) {
  {  // No Color set.
    DisplayListBuilder builder;
    DlPaint paint;
    builder.DrawRect(DlRect::MakeWH(5, 5), paint);
    sk_sp<DisplayList> dl = builder.Build();
    DlOpSpy dl_op_spy;
    dl->Dispatch(dl_op_spy);
    ASSERT_DID_DRAW(dl_op_spy, dl);
  }
  {  // Set transparent color.
    DisplayListBuilder builder;
    DlPaint paint(DlColor::kTransparent());
    builder.DrawRect(DlRect::MakeWH(5, 5), paint);
    sk_sp<DisplayList> dl = builder.Build();
    DlOpSpy dl_op_spy;
    dl->Dispatch(dl_op_spy);
    ASSERT_NO_DRAW(dl_op_spy, dl);
  }
  {  // Set black color.
    DisplayListBuilder builder;
    DlPaint paint(DlColor::kBlack());
    builder.DrawRect(DlRect::MakeWH(5, 5), paint);
    sk_sp<DisplayList> dl = builder.Build();
    DlOpSpy dl_op_spy;
    dl->Dispatch(dl_op_spy);
    ASSERT_DID_DRAW(dl_op_spy, dl);
  }
}

TEST(DlOpSpy, SetColorSource) {
  {  // Set null source
    DisplayListBuilder builder;
    DlPaint paint;
    paint.setColorSource(nullptr);
    builder.DrawRect(DlRect::MakeWH(5, 5), paint);
    sk_sp<DisplayList> dl = builder.Build();
    DlOpSpy dl_op_spy;
    dl->Dispatch(dl_op_spy);
    ASSERT_DID_DRAW(dl_op_spy, dl);
  }
  {  // setColorSource(null) restores previous color visibility
    DlOpSpy dl_op_spy;
    DlOpReceiver* receiver = &dl_op_spy;
    receiver->setColor(DlColor::kTransparent());
    receiver->drawRect(DlRect::MakeWH(5, 5));
    ASSERT_FALSE(dl_op_spy.did_draw());
    DlColor colors[2] = {
        DlColor::kGreen(),
        DlColor::kBlue(),
    };
    float stops[2] = {
        0.0f,
        1.0f,
    };
    auto color_source = DlColorSource::MakeLinear({0, 0}, {10, 10}, 2, colors,
                                                  stops, DlTileMode::kClamp);
    receiver->setColorSource(color_source.get());
    receiver->setColorSource(nullptr);
    receiver->drawRect(DlRect::MakeWH(5, 5));
    ASSERT_FALSE(dl_op_spy.did_draw());
  }
}

TEST(DlOpSpy, DrawColor) {
  {  // Black color source.
    DisplayListBuilder builder;
    auto color = DlColor::kBlack();
    builder.DrawColor(color, DlBlendMode::kSrc);
    sk_sp<DisplayList> dl = builder.Build();
    DlOpSpy dl_op_spy;
    dl->Dispatch(dl_op_spy);
    ASSERT_DID_DRAW(dl_op_spy, dl);
  }
  {  // Transparent color with kSrc.
    DisplayListBuilder builder;
    auto color = DlColor::kTransparent();
    builder.DrawColor(color, DlBlendMode::kSrc);
    sk_sp<DisplayList> dl = builder.Build();
    DlOpSpy dl_op_spy;
    dl->Dispatch(dl_op_spy);
    ASSERT_NO_DRAW(dl_op_spy, dl);
  }
  {  // Transparent color with kSrcOver.
    DisplayListBuilder builder;
    auto color = DlColor::kTransparent();
    builder.DrawColor(color, DlBlendMode::kSrcOver);
    sk_sp<DisplayList> dl = builder.Build();
    DlOpSpy dl_op_spy;
    dl->Dispatch(dl_op_spy);
    ASSERT_NO_DRAW(dl_op_spy, dl);
  }
}

TEST(DlOpSpy, DrawPaint) {
  {  // Transparent color in paint.
    DisplayListBuilder builder;
    DlPaint paint(DlColor::kTransparent());
    builder.DrawPaint(paint);
    sk_sp<DisplayList> dl = builder.Build();
    DlOpSpy dl_op_spy;
    dl->Dispatch(dl_op_spy);
    ASSERT_NO_DRAW(dl_op_spy, dl);
  }
  {  // black color in paint.
    DisplayListBuilder builder;
    DlPaint paint(DlColor::kBlack());
    builder.DrawPaint(paint);
    sk_sp<DisplayList> dl = builder.Build();
    DlOpSpy dl_op_spy;
    dl->Dispatch(dl_op_spy);
    ASSERT_DID_DRAW(dl_op_spy, dl);
  }
}

TEST(DlOpSpy, DrawLine) {
  {  // black
    DisplayListBuilder builder;
    DlPaint paint(DlColor::kBlack());
    builder.DrawLine(DlPoint(0, 1), DlPoint(1, 2), paint);
    sk_sp<DisplayList> dl = builder.Build();
    DlOpSpy dl_op_spy;
    dl->Dispatch(dl_op_spy);
    ASSERT_DID_DRAW(dl_op_spy, dl);
  }
  {  // transparent
    DisplayListBuilder builder;
    DlPaint paint(DlColor::kTransparent());
    builder.DrawLine(DlPoint(0, 1), DlPoint(1, 2), paint);
    sk_sp<DisplayList> dl = builder.Build();
    DlOpSpy dl_op_spy;
    dl->Dispatch(dl_op_spy);
    ASSERT_NO_DRAW(dl_op_spy, dl);
  }
}

TEST(DlOpSpy, DrawDashedLine) {
  {  // black
    DisplayListBuilder builder;
    DlPaint paint(DlColor::kBlack());
    builder.DrawDashedLine(DlPoint(0, 1), DlPoint(1, 2), 1.0f, 1.0f, paint);
    sk_sp<DisplayList> dl = builder.Build();
    DlOpSpy dl_op_spy;
    dl->Dispatch(dl_op_spy);
    ASSERT_DID_DRAW(dl_op_spy, dl);
  }
  {  // transparent
    DisplayListBuilder builder;
    DlPaint paint(DlColor::kTransparent());
    builder.DrawDashedLine(DlPoint(0, 1), DlPoint(1, 2), 1.0f, 1.0f, paint);
    sk_sp<DisplayList> dl = builder.Build();
    DlOpSpy dl_op_spy;
    dl->Dispatch(dl_op_spy);
    ASSERT_NO_DRAW(dl_op_spy, dl);
  }
}

TEST(DlOpSpy, DrawRect) {
  {  // black
    DisplayListBuilder builder;
    DlPaint paint(DlColor::kBlack());
    builder.DrawRect(DlRect::MakeWH(5, 5), paint);
    sk_sp<DisplayList> dl = builder.Build();
    DlOpSpy dl_op_spy;
    dl->Dispatch(dl_op_spy);
    ASSERT_DID_DRAW(dl_op_spy, dl);
  }
  {  // transparent
    DisplayListBuilder builder;
    DlPaint paint(DlColor::kTransparent());
    builder.DrawRect(DlRect::MakeWH(5, 5), paint);
    sk_sp<DisplayList> dl = builder.Build();
    DlOpSpy dl_op_spy;
    dl->Dispatch(dl_op_spy);
    ASSERT_NO_DRAW(dl_op_spy, dl);
  }
}

TEST(DlOpSpy, DrawOval) {
  {  // black
    DisplayListBuilder builder;
    DlPaint paint(DlColor::kBlack());
    builder.DrawOval(DlRect::MakeWH(5, 5), paint);
    sk_sp<DisplayList> dl = builder.Build();
    DlOpSpy dl_op_spy;
    dl->Dispatch(dl_op_spy);
    ASSERT_DID_DRAW(dl_op_spy, dl);
  }
  {  // transparent
    DisplayListBuilder builder;
    DlPaint paint(DlColor::kTransparent());
    builder.DrawOval(DlRect::MakeWH(5, 5), paint);
    sk_sp<DisplayList> dl = builder.Build();
    DlOpSpy dl_op_spy;
    dl->Dispatch(dl_op_spy);
    ASSERT_NO_DRAW(dl_op_spy, dl);
  }
}

TEST(DlOpSpy, DrawCircle) {
  {  // black
    DisplayListBuilder builder;
    DlPaint paint(DlColor::kBlack());
    builder.DrawCircle(DlPoint(5, 5), 1.0, paint);
    sk_sp<DisplayList> dl = builder.Build();
    DlOpSpy dl_op_spy;
    dl->Dispatch(dl_op_spy);
    ASSERT_DID_DRAW(dl_op_spy, dl);
  }
  {  // transparent
    DisplayListBuilder builder;
    DlPaint paint(DlColor::kTransparent());
    builder.DrawCircle(DlPoint(5, 5), 1.0, paint);
    sk_sp<DisplayList> dl = builder.Build();
    DlOpSpy dl_op_spy;
    dl->Dispatch(dl_op_spy);
    ASSERT_NO_DRAW(dl_op_spy, dl);
  }
}

TEST(DlOpSpy, DrawRRect) {
  {  // black
    DisplayListBuilder builder;
    DlPaint paint(DlColor::kBlack());
    builder.DrawRoundRect(DlRoundRect::MakeRect(DlRect::MakeWH(5, 5)), paint);
    sk_sp<DisplayList> dl = builder.Build();
    DlOpSpy dl_op_spy;
    dl->Dispatch(dl_op_spy);
    ASSERT_DID_DRAW(dl_op_spy, dl);
  }
  {  // transparent
    DisplayListBuilder builder;
    DlPaint paint(DlColor::kTransparent());
    builder.DrawRoundRect(DlRoundRect::MakeRect(DlRect::MakeWH(5, 5)), paint);
    sk_sp<DisplayList> dl = builder.Build();
    DlOpSpy dl_op_spy;
    dl->Dispatch(dl_op_spy);
    ASSERT_NO_DRAW(dl_op_spy, dl);
  }
}

TEST(DlOpSpy, DrawPath) {
  {  // black line
    DisplayListBuilder builder;
    DlPaint paint(DlColor::kBlack());
    paint.setDrawStyle(DlDrawStyle::kStroke);
    DlPathBuilder path_builder;
    path_builder.MoveTo({0, 1});
    path_builder.LineTo({1, 1});
    builder.DrawPath(DlPath(path_builder), paint);
    sk_sp<DisplayList> dl = builder.Build();
    DlOpSpy dl_op_spy;
    dl->Dispatch(dl_op_spy);
    ASSERT_DID_DRAW(dl_op_spy, dl);
  }
  {  // triangle
    DisplayListBuilder builder;
    DlPaint paint(DlColor::kBlack());
    DlPathBuilder path_builder;
    path_builder.MoveTo({0, 0});
    path_builder.LineTo({1, 0});
    path_builder.LineTo({0, 1});
    builder.DrawPath(DlPath(path_builder), paint);
    sk_sp<DisplayList> dl = builder.Build();
    DlOpSpy dl_op_spy;
    dl->Dispatch(dl_op_spy);
    ASSERT_DID_DRAW(dl_op_spy, dl);
  }
  {  // transparent line
    DisplayListBuilder builder;
    DlPaint paint(DlColor::kTransparent());
    paint.setDrawStyle(DlDrawStyle::kStroke);
    DlPathBuilder path_builder;
    path_builder.MoveTo({0, 1});
    path_builder.LineTo({1, 1});
    builder.DrawPath(DlPath(path_builder), paint);
    sk_sp<DisplayList> dl = builder.Build();
    DlOpSpy dl_op_spy;
    dl->Dispatch(dl_op_spy);
    ASSERT_NO_DRAW(dl_op_spy, dl);
  }
}

TEST(DlOpSpy, DrawArc) {
  {  // black
    DisplayListBuilder builder;
    DlPaint paint(DlColor::kBlack());
    builder.DrawArc(DlRect::MakeWH(5, 5), 0, 1, true, paint);
    sk_sp<DisplayList> dl = builder.Build();
    DlOpSpy dl_op_spy;
    dl->Dispatch(dl_op_spy);
    ASSERT_DID_DRAW(dl_op_spy, dl);
  }
  {  // transparent
    DisplayListBuilder builder;
    DlPaint paint(DlColor::kTransparent());
    builder.DrawArc(DlRect::MakeWH(5, 5), 0, 1, true, paint);
    sk_sp<DisplayList> dl = builder.Build();
    DlOpSpy dl_op_spy;
    dl->Dispatch(dl_op_spy);
    ASSERT_NO_DRAW(dl_op_spy, dl);
  }
}

TEST(DlOpSpy, DrawPoints) {
  {  // black
    DisplayListBuilder builder;
    DlPaint paint(DlColor::kBlack());
    const DlPoint points[] = {DlPoint(5, 4)};
    builder.DrawPoints(DlPointMode::kPoints, 1, points, paint);
    sk_sp<DisplayList> dl = builder.Build();
    DlOpSpy dl_op_spy;
    dl->Dispatch(dl_op_spy);
    ASSERT_DID_DRAW(dl_op_spy, dl);
  }
  {  // transparent
    DisplayListBuilder builder;
    DlPaint paint(DlColor::kTransparent());
    const DlPoint points[] = {DlPoint(5, 4)};
    builder.DrawPoints(DlPointMode::kPoints, 1, points, paint);
    sk_sp<DisplayList> dl = builder.Build();
    DlOpSpy dl_op_spy;
    dl->Dispatch(dl_op_spy);
    ASSERT_NO_DRAW(dl_op_spy, dl);
  }
}

TEST(DlOpSpy, DrawVertices) {
  {  // black
    DisplayListBuilder builder;
    DlPaint paint(DlColor::kBlack());
    const DlPoint vertices[] = {
        DlPoint(5, 5),
        DlPoint(5, 15),
        DlPoint(15, 5),
    };
    const DlPoint texture_coordinates[] = {
        DlPoint(5, 5),
        DlPoint(15, 5),
        DlPoint(5, 15),
    };
    const DlColor colors[] = {
        DlColor::kBlack(),
        DlColor::kRed(),
        DlColor::kGreen(),
    };
    auto dl_vertices = DlVertices::Make(DlVertexMode::kTriangles, 3, vertices,
                                        texture_coordinates, colors, 0);
    builder.DrawVertices(dl_vertices, DlBlendMode::kSrc, paint);
    sk_sp<DisplayList> dl = builder.Build();
    DlOpSpy dl_op_spy;
    dl->Dispatch(dl_op_spy);
    ASSERT_DID_DRAW(dl_op_spy, dl);
  }
  {  // transparent
    DisplayListBuilder builder;
    DlPaint paint(DlColor::kTransparent());
    const DlPoint vertices[] = {
        DlPoint(5, 5),
        DlPoint(5, 15),
        DlPoint(15, 5),
    };
    const DlPoint texture_coordinates[] = {
        DlPoint(5, 5),
        DlPoint(15, 5),
        DlPoint(5, 15),
    };
    const DlColor colors[] = {
        DlColor::kBlack(),
        DlColor::kRed(),
        DlColor::kGreen(),
    };
    auto dl_vertices = DlVertices::Make(DlVertexMode::kTriangles, 3, vertices,
                                        texture_coordinates, colors, 0);
    builder.DrawVertices(dl_vertices, DlBlendMode::kSrc, paint);
    sk_sp<DisplayList> dl = builder.Build();
    DlOpSpy dl_op_spy;
    dl->Dispatch(dl_op_spy);
    ASSERT_NO_DRAW(dl_op_spy, dl);
  }
}

TEST(DlOpSpy, Images) {
  {  // DrawImage
    DisplayListBuilder builder;
    DlPaint paint(DlColor::kBlack());
    builder.DrawImage(kTestImage1, DlPoint(5, 5), DlImageSampling::kLinear);
    sk_sp<DisplayList> dl = builder.Build();
    DlOpSpy dl_op_spy;
    dl->Dispatch(dl_op_spy);
    ASSERT_DID_DRAW(dl_op_spy, dl);
  }
  {  // DrawImageRect
    DisplayListBuilder builder;
    DlPaint paint(DlColor::kBlack());
    builder.DrawImageRect(kTestImage1, DlRect::MakeWH(5, 5),
                          DlRect::MakeWH(5, 5), DlImageSampling::kLinear);
    sk_sp<DisplayList> dl = builder.Build();
    DlOpSpy dl_op_spy;
    dl->Dispatch(dl_op_spy);
    ASSERT_DID_DRAW(dl_op_spy, dl);
  }
  {  // DrawImageNine
    DisplayListBuilder builder;
    DlPaint paint(DlColor::kBlack());
    builder.DrawImageNine(kTestImage1, DlIRect::MakeWH(5, 5),
                          DlRect::MakeWH(5, 5), DlFilterMode::kLinear);
    sk_sp<DisplayList> dl = builder.Build();
    DlOpSpy dl_op_spy;
    dl->Dispatch(dl_op_spy);
    ASSERT_DID_DRAW(dl_op_spy, dl);
  }
  {  // DrawAtlas
    DisplayListBuilder builder;
    DlPaint paint(DlColor::kBlack());
    const DlRSTransform xform[] = {
        DlRSTransform::Make({0, 0}, 1.0f, DlDegrees(0)),
    };
    const DlRect tex[] = {DlRect::MakeXYWH(10, 10, 10, 10)};
    DlRect cull_rect = DlRect::MakeWH(5, 5);
    builder.DrawAtlas(kTestImage1, xform, tex, nullptr, 1, DlBlendMode::kSrc,
                      DlImageSampling::kLinear, &cull_rect);
    sk_sp<DisplayList> dl = builder.Build();
    DlOpSpy dl_op_spy;
    dl->Dispatch(dl_op_spy);
    ASSERT_DID_DRAW(dl_op_spy, dl);
  }
}

TEST(DlOpSpy, DrawDisplayList) {
  {  // Recursive Transparent DisplayList
    DisplayListBuilder builder;
    DlPaint paint(DlColor::kTransparent());
    builder.DrawPaint(paint);
    sk_sp<DisplayList> dl = builder.Build();

    DisplayListBuilder builder_parent;
    DlPaint paint_parent(DlColor::kTransparent());
    builder_parent.DrawPaint(paint_parent);
    builder_parent.DrawDisplayList(dl, 1);
    sk_sp<DisplayList> dl2 = builder_parent.Build();

    DlOpSpy dl_op_spy;
    dl2->Dispatch(dl_op_spy);
    ASSERT_NO_DRAW(dl_op_spy, dl2);
  }
  {  // Sub non-transparent DisplayList,
    DisplayListBuilder builder;
    DlPaint paint(DlColor::kBlack());
    builder.DrawPaint(paint);
    sk_sp<DisplayList> dl = builder.Build();

    DisplayListBuilder builder_parent;
    DlPaint paint_parent(DlColor::kTransparent());
    builder_parent.DrawPaint(paint_parent);
    builder_parent.DrawDisplayList(dl, 1);
    sk_sp<DisplayList> dl2 = builder_parent.Build();

    DlOpSpy dl_op_spy;
    dl2->Dispatch(dl_op_spy);
    ASSERT_DID_DRAW(dl_op_spy, dl2);
  }

  {  // Sub non-transparent DisplayList, 0 opacity
    DisplayListBuilder builder;
    DlPaint paint(DlColor::kBlack());
    builder.DrawPaint(paint);
    sk_sp<DisplayList> dl = builder.Build();

    DisplayListBuilder builder_parent;
    DlPaint paint_parent(DlColor::kTransparent());
    builder_parent.DrawPaint(paint_parent);
    builder_parent.DrawDisplayList(dl, 0);
    sk_sp<DisplayList> dl2 = builder_parent.Build();

    DlOpSpy dl_op_spy;
    dl2->Dispatch(dl_op_spy);
    ASSERT_NO_DRAW(dl_op_spy, dl2);
  }

  {  // Parent non-transparent DisplayList
    DisplayListBuilder builder;
    DlPaint paint(DlColor::kTransparent());
    builder.DrawPaint(paint);
    sk_sp<DisplayList> dl = builder.Build();

    DisplayListBuilder builder_parent;
    DlPaint paint_parent(DlColor::kBlack());
    builder_parent.DrawPaint(paint_parent);
    builder_parent.DrawDisplayList(dl, 0);
    sk_sp<DisplayList> dl2 = builder_parent.Build();

    DlOpSpy dl_op_spy;
    dl2->Dispatch(dl_op_spy);
    ASSERT_DID_DRAW(dl_op_spy, dl2);
  }
}

TEST(DlOpSpy, DrawTextBlob) {
  {  // Non-transparent color.
    DisplayListBuilder builder;
    DlPaint paint(DlColor::kBlack());
    std::string string = "xx";
    builder.DrawTextBlob(GetTestTextBlob(42), 1, 1, paint);
    sk_sp<DisplayList> dl = builder.Build();
    DlOpSpy dl_op_spy;
    dl->Dispatch(dl_op_spy);
    ASSERT_DID_DRAW(dl_op_spy, dl);
  }
  {  // transparent color.
    DisplayListBuilder builder;
    DlPaint paint(DlColor::kTransparent());
    std::string string = "xx";
    builder.DrawTextBlob(GetTestTextBlob(43), 1, 1, paint);
    sk_sp<DisplayList> dl = builder.Build();
    DlOpSpy dl_op_spy;
    dl->Dispatch(dl_op_spy);
    ASSERT_NO_DRAW(dl_op_spy, dl);
  }
}

TEST(DlOpSpy, DrawShadow) {
  {  // valid shadow
    DisplayListBuilder builder;
    DlPaint paint;
    DlColor color = DlColor::kBlack();
    builder.DrawShadow(kTestPath1, color, 1, false, 1);
    sk_sp<DisplayList> dl = builder.Build();
    DlOpSpy dl_op_spy;
    dl->Dispatch(dl_op_spy);
    ASSERT_DID_DRAW(dl_op_spy, dl);
  }
  {  // transparent color
    DisplayListBuilder builder;
    DlPaint paint;
    DlColor color = DlColor::kTransparent();
    builder.DrawShadow(kTestPath1, color, 1, false, 1);
    sk_sp<DisplayList> dl = builder.Build();
    DlOpSpy dl_op_spy;
    dl->Dispatch(dl_op_spy);
    ASSERT_NO_DRAW(dl_op_spy, dl);
  }
}

}  // namespace testing
}  // namespace flutter
