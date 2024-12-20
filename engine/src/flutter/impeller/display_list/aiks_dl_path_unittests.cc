// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "display_list/dl_sampling_options.h"
#include "display_list/dl_tile_mode.h"
#include "display_list/effects/dl_color_source.h"
#include "display_list/effects/dl_mask_filter.h"
#include "flutter/impeller/display_list/aiks_unittests.h"

#include "flutter/display_list/dl_blend_mode.h"
#include "flutter/display_list/dl_builder.h"
#include "flutter/display_list/dl_color.h"
#include "flutter/display_list/dl_paint.h"
#include "flutter/display_list/effects/dl_color_filter.h"
#include "flutter/testing/testing.h"
#include "impeller/display_list/dl_image_impeller.h"
#include "impeller/playground/widgets.h"

#include "include/core/SkMatrix.h"
#include "include/core/SkPath.h"
#include "include/core/SkPathTypes.h"
#include "include/core/SkRRect.h"

namespace impeller {
namespace testing {

using namespace flutter;

TEST_P(AiksTest, RotateColorFilteredPath) {
  DisplayListBuilder builder;
  builder.Transform(SkMatrix::Translate(300, 300) * SkMatrix::RotateDeg(90));

  SkPath arrow_stem;
  SkPath arrow_head;

  arrow_stem.moveTo({120, 190}).lineTo({120, 50});
  arrow_head.moveTo({50, 120}).lineTo({120, 190}).lineTo({190, 120});

  auto filter =
      DlColorFilter::MakeBlend(DlColor::kAliceBlue(), DlBlendMode::kSrcIn);

  DlPaint paint;
  paint.setStrokeWidth(15.0);
  paint.setStrokeCap(DlStrokeCap::kRound);
  paint.setStrokeJoin(DlStrokeJoin::kRound);
  paint.setDrawStyle(DlDrawStyle::kStroke);
  paint.setColorFilter(filter);
  paint.setColor(DlColor::kBlack());

  builder.DrawPath(arrow_stem, paint);
  builder.DrawPath(arrow_head, paint);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanRenderStrokes) {
  DisplayListBuilder builder;
  DlPaint paint;
  paint.setColor(DlColor::kRed());
  paint.setStrokeWidth(20);
  paint.setDrawStyle(DlDrawStyle::kStroke);

  builder.DrawPath(SkPath::Line({200, 100}, {800, 100}), paint);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanRenderCurvedStrokes) {
  DisplayListBuilder builder;
  DlPaint paint;
  paint.setColor(DlColor::kRed());
  paint.setStrokeWidth(25);
  paint.setDrawStyle(DlDrawStyle::kStroke);

  builder.DrawPath(SkPath::Circle(500, 500, 250), paint);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanRenderThickCurvedStrokes) {
  DisplayListBuilder builder;
  DlPaint paint;
  paint.setColor(DlColor::kRed());
  paint.setStrokeWidth(100);
  paint.setDrawStyle(DlDrawStyle::kStroke);

  builder.DrawPath(SkPath::Circle(100, 100, 50), paint);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanRenderThinCurvedStrokes) {
  DisplayListBuilder builder;
  DlPaint paint;
  paint.setColor(DlColor::kRed());
  paint.setStrokeWidth(0.01);
  paint.setDrawStyle(DlDrawStyle::kStroke);

  builder.DrawPath(SkPath::Circle(100, 100, 50), paint);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanRenderStrokePathThatEndsAtSharpTurn) {
  DisplayListBuilder builder;
  DlPaint paint;
  paint.setColor(DlColor::kRed());
  paint.setStrokeWidth(200);
  paint.setDrawStyle(DlDrawStyle::kStroke);

  SkPath path;
  path.arcTo(SkRect::MakeXYWH(100, 100, 200, 200), 0, 90, false);

  builder.DrawPath(path, paint);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanRenderStrokePathWithCubicLine) {
  DisplayListBuilder builder;

  DlPaint paint;
  paint.setColor(DlColor::kRed());
  paint.setStrokeWidth(20);
  paint.setDrawStyle(DlDrawStyle::kStroke);

  SkPath path;
  path.moveTo(0, 200);
  path.cubicTo(50, 400, 350, 0, 400, 200);

  builder.DrawPath(path, paint);
  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanRenderQuadraticStrokeWithInstantTurn) {
  DisplayListBuilder builder;

  DlPaint paint;
  paint.setColor(DlColor::kRed());
  paint.setStrokeWidth(50);
  paint.setDrawStyle(DlDrawStyle::kStroke);
  paint.setStrokeCap(DlStrokeCap::kRound);

  // Should draw a diagonal pill shape. If flat on either end, the stroke is
  // rendering wrong.
  SkPath path;
  path.moveTo(250, 250);
  path.quadTo(100, 100, 250, 250);

  builder.DrawPath(path, paint);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanRenderDifferencePaths) {
  DisplayListBuilder builder;

  DlPaint paint;
  paint.setColor(DlColor::kRed());

  SkPoint radii[4] = {{50, 25}, {25, 50}, {50, 25}, {25, 50}};
  SkPath path;
  SkRRect rrect;
  rrect.setRectRadii(SkRect::MakeXYWH(100, 100, 200, 200), radii);
  path.addRRect(rrect);
  path.addCircle(200, 200, 50);
  path.setFillType(SkPathFillType::kEvenOdd);

  builder.DrawImage(
      DlImageImpeller::Make(CreateTextureForFixture("boston.jpg")),
      SkPoint{10, 10}, {});
  builder.DrawPath(path, paint);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

// Regression test for https://github.com/flutter/flutter/issues/134816.
//
// It should be possible to draw 3 lines, and not have an implicit close path.
TEST_P(AiksTest, CanDrawAnOpenPath) {
  DisplayListBuilder builder;

  // Starting at (50, 50), draw lines from:
  // 1. (50, height)
  // 2. (width, height)
  // 3. (width, 50)
  SkPath path;
  path.moveTo(50, 50);
  path.lineTo(50, 100);
  path.lineTo(100, 100);
  path.lineTo(100, 50);

  DlPaint paint;
  paint.setColor(DlColor::kRed());
  paint.setDrawStyle(DlDrawStyle::kStroke);
  paint.setStrokeWidth(10);

  builder.DrawPath(path, paint);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanDrawAnOpenPathThatIsntARect) {
  DisplayListBuilder builder;

  // Draw a stroked path that is explicitly closed to verify
  // It doesn't become a rectangle.
  SkPath path;
  // PathBuilder builder;
  path.moveTo(50, 50);
  path.lineTo(520, 120);
  path.lineTo(300, 310);
  path.lineTo(100, 50);
  path.close();

  DlPaint paint;
  paint.setColor(DlColor::kRed());
  paint.setDrawStyle(DlDrawStyle::kStroke);
  paint.setStrokeWidth(10);

  builder.DrawPath(path, paint);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, SolidStrokesRenderCorrectly) {
  // Compare with https://fiddle.skia.org/c/027392122bec8ac2b5d5de00a4b9bbe2
  auto callback = [&]() -> sk_sp<DisplayList> {
    static Color color = Color::Black().WithAlpha(0.5);
    static float scale = 3;
    static bool add_circle_clip = true;

    if (AiksTest::ImGuiBegin("Controls", nullptr,
                             ImGuiWindowFlags_AlwaysAutoResize)) {
      ImGui::ColorEdit4("Color", reinterpret_cast<float*>(&color));
      ImGui::SliderFloat("Scale", &scale, 0, 6);
      ImGui::Checkbox("Circle clip", &add_circle_clip);
      ImGui::End();
    }

    DisplayListBuilder builder;
    builder.Scale(GetContentScale().x, GetContentScale().y);
    DlPaint paint;

    paint.setColor(DlColor::kWhite());
    builder.DrawPaint(paint);

    paint.setColor(
        DlColor::ARGB(color.alpha, color.red, color.green, color.blue));
    paint.setDrawStyle(DlDrawStyle::kStroke);
    paint.setStrokeWidth(10);

    SkPath path;
    path.moveTo({20, 20});
    path.quadTo({60, 20}, {60, 60});
    path.close();
    path.moveTo({60, 20});
    path.quadTo({60, 60}, {20, 60});

    builder.Scale(scale, scale);

    if (add_circle_clip) {
      static PlaygroundPoint circle_clip_point_a(Point(60, 300), 20,
                                                 Color::Red());
      static PlaygroundPoint circle_clip_point_b(Point(600, 300), 20,
                                                 Color::Red());
      auto [handle_a, handle_b] =
          DrawPlaygroundLine(circle_clip_point_a, circle_clip_point_b);

      SkMatrix screen_to_canvas = SkMatrix::I();
      if (!builder.GetTransform().invert(&screen_to_canvas)) {
        return nullptr;
      }

      SkPoint point_a =
          screen_to_canvas.mapPoint(SkPoint::Make(handle_a.x, handle_a.y));
      SkPoint point_b =
          screen_to_canvas.mapPoint(SkPoint::Make(handle_b.x, handle_b.y));

      SkPoint middle = point_a + point_b;
      middle.scale(GetContentScale().x / 2);

      auto radius = SkPoint::Distance(point_a, middle);

      builder.ClipPath(SkPath::Circle(middle.x(), middle.y(), radius));
    }

    for (auto join :
         {DlStrokeJoin::kBevel, DlStrokeJoin::kRound, DlStrokeJoin::kMiter}) {
      paint.setStrokeJoin(join);
      for (auto cap :
           {DlStrokeCap::kButt, DlStrokeCap::kSquare, DlStrokeCap::kRound}) {
        paint.setStrokeCap(cap);
        builder.DrawPath(path, paint);
        builder.Translate(80, 0);
      }
      builder.Translate(-240, 60);
    }

    return builder.Build();
  };

  ASSERT_TRUE(OpenPlaygroundHere(callback));
}

TEST_P(AiksTest, DrawLinesRenderCorrectly) {
  DisplayListBuilder builder;
  builder.Scale(GetContentScale().x, GetContentScale().y);

  DlPaint paint;
  paint.setColor(DlColor::kBlue());
  paint.setStrokeWidth(10);

  auto draw = [&builder](DlPaint& paint) {
    for (auto cap :
         {DlStrokeCap::kButt, DlStrokeCap::kSquare, DlStrokeCap::kRound}) {
      paint.setStrokeCap(cap);
      SkPoint origin = {100, 100};
      builder.DrawLine(SkPoint{150, 100}, SkPoint{250, 100}, paint);
      for (int d = 15; d < 90; d += 15) {
        Matrix m = Matrix::MakeRotationZ(Degrees(d));
        Point origin = {100, 100};
        Point p0 = {50, 0};
        Point p1 = {150, 0};
        auto a = origin + m * p0;
        auto b = origin + m * p1;

        builder.DrawLine(SkPoint::Make(a.x, a.y), SkPoint::Make(b.x, b.y),
                         paint);
      }
      builder.DrawLine(SkPoint{100, 150}, SkPoint{100, 250}, paint);
      builder.DrawCircle({origin}, 35, paint);

      builder.DrawLine(SkPoint{250, 250}, SkPoint{250, 250}, paint);

      builder.Translate(250, 0);
    }
    builder.Translate(-750, 250);
  };

  std::vector<DlColor> colors = {
      DlColor::ARGB(1, 0x1f / 255.0, 0.0, 0x5c / 255.0),
      DlColor::ARGB(1, 0x5b / 255.0, 0.0, 0x60 / 255.0),
      DlColor::ARGB(1, 0x87 / 255.0, 0x01 / 255.0, 0x60 / 255.0),
      DlColor::ARGB(1, 0xac / 255.0, 0x25 / 255.0, 0x53 / 255.0),
      DlColor::ARGB(1, 0xe1 / 255.0, 0x6b / 255.0, 0x5c / 255.0),
      DlColor::ARGB(1, 0xf3 / 255.0, 0x90 / 255.0, 0x60 / 255.0),
      DlColor::ARGB(1, 0xff / 255.0, 0xb5 / 255.0, 0x6b / 250.0)};
  std::vector<Scalar> stops = {
      0.0,
      (1.0 / 6.0) * 1,
      (1.0 / 6.0) * 2,
      (1.0 / 6.0) * 3,
      (1.0 / 6.0) * 4,
      (1.0 / 6.0) * 5,
      1.0,
  };

  auto texture = DlImageImpeller::Make(
      CreateTextureForFixture("airplane.jpg",
                              /*enable_mipmapping=*/true));

  draw(paint);

  paint.setColorSource(DlColorSource::MakeRadial({100, 100}, 200, stops.size(),
                                                 colors.data(), stops.data(),
                                                 DlTileMode::kMirror));
  draw(paint);

  DlMatrix matrix = DlMatrix::MakeTranslation({-150, 75});
  paint.setColorSource(DlColorSource::MakeImage(
      texture, DlTileMode::kRepeat, DlTileMode::kRepeat,
      DlImageSampling::kMipmapLinear, &matrix));
  draw(paint);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, DrawRectStrokesRenderCorrectly) {
  DisplayListBuilder builder;
  DlPaint paint;
  paint.setColor(DlColor::kRed());
  paint.setDrawStyle(DlDrawStyle::kStroke);
  paint.setStrokeWidth(10);

  builder.Translate(100, 100);
  builder.DrawPath(SkPath::Rect(SkRect::MakeSize(SkSize{100, 100})), {paint});

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, DrawRectStrokesWithBevelJoinRenderCorrectly) {
  DisplayListBuilder builder;
  DlPaint paint;
  paint.setColor(DlColor::kRed());
  paint.setDrawStyle(DlDrawStyle::kStroke);
  paint.setStrokeWidth(10);
  paint.setStrokeJoin(DlStrokeJoin::kBevel);

  builder.Translate(100, 100);
  builder.DrawPath(SkPath::Rect(SkRect::MakeSize(SkSize{100, 100})), paint);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanDrawMultiContourConvexPath) {
  SkPath path;
  for (auto i = 0; i < 10; i++) {
    if (i % 2 == 0) {
      path.addCircle(100 + 50 * i, 100 + 50 * i, 100);
      path.close();
    } else {
      path.moveTo({100.f + 50.f * i - 100, 100.f + 50.f * i});
      path.lineTo({100.f + 50.f * i, 100.f + 50.f * i - 100});
      path.lineTo({100.f + 50.f * i - 100, 100.f + 50.f * i - 100});
      path.close();
    }
  }

  DisplayListBuilder builder;
  DlPaint paint;
  paint.setColor(DlColor::kRed().withAlpha(102));
  builder.DrawPath(path, paint);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, ArcWithZeroSweepAndBlur) {
  DisplayListBuilder builder;
  builder.Scale(GetContentScale().x, GetContentScale().y);

  DlPaint paint;
  paint.setColor(DlColor::kRed());

  std::vector<DlColor> colors = {DlColor::RGBA(1.0, 0.0, 0.0, 1.0),
                                 DlColor::RGBA(0.0, 0.0, 0.0, 1.0)};
  std::vector<Scalar> stops = {0.0, 1.0};

  paint.setColorSource(
      DlColorSource::MakeSweep({100, 100}, 45, 135, stops.size(), colors.data(),
                               stops.data(), DlTileMode::kMirror));
  paint.setMaskFilter(DlBlurMaskFilter::Make(DlBlurStyle::kNormal, 20));

  SkPath path;
  path.addArc(SkRect::MakeXYWH(10, 10, 100, 100), 0, 0);
  builder.DrawPath(path, paint);

  // Check that this empty picture can be created without crashing.
  builder.Build();
}

TEST_P(AiksTest, CanRenderClips) {
  DisplayListBuilder builder;
  DlPaint paint;
  paint.setColor(DlColor::kFuchsia());

  builder.ClipPath(SkPath::Rect(SkRect::MakeXYWH(0, 0, 500, 500)));
  builder.DrawPath(SkPath::Circle(500, 500, 250), paint);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanRenderOverlappingMultiContourPath) {
  DisplayListBuilder builder;

  DlPaint paint;
  paint.setColor(DlColor::kRed());

  SkPoint radii[4] = {{50, 50}, {50, 50}, {50, 50}, {50, 50}};

  const Scalar kTriangleHeight = 100;
  SkRRect rrect;
  rrect.setRectRadii(
      SkRect::MakeXYWH(-kTriangleHeight / 2.0f, -kTriangleHeight / 2.0f,
                       kTriangleHeight, kTriangleHeight),
      radii  //
  );

  builder.Translate(200, 200);
  // Form a path similar to the Material drop slider value indicator. Both
  // shapes should render identically side-by-side.
  {
    SkPath path;
    path.moveTo(0, kTriangleHeight);
    path.lineTo(-kTriangleHeight / 2.0f, 0);
    path.lineTo(kTriangleHeight / 2.0f, 0);
    path.close();
    path.addRRect(rrect);

    builder.DrawPath(path, paint);
  }
  builder.Translate(100, 0);

  {
    SkPath path;
    path.moveTo(0, kTriangleHeight);
    path.lineTo(-kTriangleHeight / 2.0f, 0);
    path.lineTo(0, -10);
    path.lineTo(kTriangleHeight / 2.0f, 0);
    path.close();
    path.addRRect(rrect);

    builder.DrawPath(path, paint);
  }

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

}  // namespace testing
}  // namespace impeller
