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

namespace impeller {
namespace testing {

using namespace flutter;

TEST_P(AiksTest, RotateColorFilteredPath) {
  DisplayListBuilder builder;
  builder.Transform(DlMatrix::MakeTranslation(DlPoint(300, 300)) *
                    DlMatrix::MakeRotationZ(DlDegrees(90)));

  DlPathBuilder arrow_stem;
  DlPathBuilder arrow_head;

  arrow_stem.MoveTo(DlPoint(120, 190)).LineTo(DlPoint(120, 50));
  arrow_head.MoveTo(DlPoint(50, 120))
      .LineTo(DlPoint(120, 190))
      .LineTo(DlPoint(190, 120));

  auto filter =
      DlColorFilter::MakeBlend(DlColor::kAliceBlue(), DlBlendMode::kSrcIn);

  DlPaint paint;
  paint.setStrokeWidth(15.0);
  paint.setStrokeCap(DlStrokeCap::kRound);
  paint.setStrokeJoin(DlStrokeJoin::kRound);
  paint.setDrawStyle(DlDrawStyle::kStroke);
  paint.setColorFilter(filter);
  paint.setColor(DlColor::kBlack());

  builder.DrawPath(DlPath(arrow_stem), paint);
  builder.DrawPath(DlPath(arrow_head), paint);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanRenderStrokes) {
  DisplayListBuilder builder;
  DlPaint paint;
  paint.setColor(DlColor::kRed());
  paint.setStrokeWidth(20);
  paint.setDrawStyle(DlDrawStyle::kStroke);

  builder.DrawPath(DlPath::MakeLine(DlPoint(200, 100), DlPoint(800, 100)),
                   paint);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanRenderCurvedStrokes) {
  DisplayListBuilder builder;
  DlPaint paint;
  paint.setColor(DlColor::kRed());
  paint.setStrokeWidth(25);
  paint.setDrawStyle(DlDrawStyle::kStroke);

  builder.DrawPath(DlPath::MakeCircle(DlPoint(500, 500), 250), paint);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanRenderThickCurvedStrokes) {
  DisplayListBuilder builder;
  DlPaint paint;
  paint.setColor(DlColor::kRed());
  paint.setStrokeWidth(100);
  paint.setDrawStyle(DlDrawStyle::kStroke);

  builder.DrawPath(DlPath::MakeCircle(DlPoint(100, 100), 50), paint);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanRenderThinCurvedStrokes) {
  DisplayListBuilder builder;
  DlPaint paint;
  paint.setColor(DlColor::kRed());
  paint.setStrokeWidth(0.01);
  paint.setDrawStyle(DlDrawStyle::kStroke);

  builder.DrawPath(DlPath::MakeCircle(DlPoint(100, 100), 50), paint);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanRenderStrokePathThatEndsAtSharpTurn) {
  DisplayListBuilder builder;
  DlPaint paint;
  paint.setColor(DlColor::kRed());
  paint.setStrokeWidth(200);
  paint.setDrawStyle(DlDrawStyle::kStroke);

  DlPath path = DlPath::MakeArc(DlRect::MakeXYWH(100, 100, 200, 200),  //
                                DlDegrees(0), DlDegrees(90), false);

  builder.DrawPath(path, paint);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanRenderStrokePathWithCubicLine) {
  DisplayListBuilder builder;

  DlPaint paint;
  paint.setColor(DlColor::kRed());
  paint.setStrokeWidth(20);
  paint.setDrawStyle(DlDrawStyle::kStroke);

  DlPathBuilder path_builder;
  path_builder.MoveTo(DlPoint(0, 200));
  path_builder.CubicCurveTo(DlPoint(50, 400), DlPoint(350, 0),
                            DlPoint(400, 200));

  builder.DrawPath(DlPath(path_builder), paint);
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
  DlPathBuilder path_builder;
  path_builder.MoveTo(DlPoint(250, 250));
  path_builder.QuadraticCurveTo(DlPoint(100, 100), DlPoint(250, 250));

  builder.DrawPath(DlPath(path_builder), paint);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanRenderDifferencePaths) {
  DisplayListBuilder builder;

  DlPaint paint;
  paint.setColor(DlColor::kRed());

  RoundingRadii radii = {
      .top_left = {50, 25},
      .top_right = {25, 50},
      .bottom_left = {25, 50},
      .bottom_right = {50, 25},
  };
  PathBuilder path_builder;
  DlRoundRect rrect =
      DlRoundRect::MakeRectRadii(DlRect::MakeXYWH(100, 100, 200, 200), radii);
  // We use the factory method to convert the rrect and circle to a path so
  // that they use the legacy conics for legacy golden output.
  path_builder.AddPath(DlPath::MakeRoundRect(rrect).GetPath());
  path_builder.AddPath(DlPath::MakeCircle(DlPoint(200, 200), 50).GetPath());
  DlPath path(path_builder, DlPathFillType::kOdd);

  builder.DrawImage(
      DlImageImpeller::Make(CreateTextureForFixture("boston.jpg")),
      DlPoint{10, 10}, {});
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
  DlPathBuilder path_builder;
  path_builder.MoveTo(DlPoint(50, 50));
  path_builder.LineTo(DlPoint(50, 100));
  path_builder.LineTo(DlPoint(100, 100));
  path_builder.LineTo(DlPoint(100, 50));

  DlPaint paint;
  paint.setColor(DlColor::kRed());
  paint.setDrawStyle(DlDrawStyle::kStroke);
  paint.setStrokeWidth(10);

  builder.DrawPath(DlPath(path_builder), paint);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanDrawAnOpenPathThatIsntARect) {
  DisplayListBuilder builder;

  // Draw a stroked path that is explicitly closed to verify
  // It doesn't become a rectangle.
  DlPathBuilder path_builder;
  path_builder.MoveTo(DlPoint(50, 50));
  path_builder.LineTo(DlPoint(520, 120));
  path_builder.LineTo(DlPoint(300, 310));
  path_builder.LineTo(DlPoint(100, 50));
  path_builder.Close();

  DlPaint paint;
  paint.setColor(DlColor::kRed());
  paint.setDrawStyle(DlDrawStyle::kStroke);
  paint.setStrokeWidth(10);

  builder.DrawPath(DlPath(path_builder), paint);

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

    DlPathBuilder path_builder;
    path_builder.MoveTo(DlPoint(20, 20));
    path_builder.QuadraticCurveTo(DlPoint(60, 20), DlPoint(60, 60));
    path_builder.Close();
    path_builder.MoveTo(DlPoint(60, 20));
    path_builder.QuadraticCurveTo(DlPoint(60, 60), DlPoint(20, 60));
    DlPath path(path_builder);

    builder.Scale(scale, scale);

    if (add_circle_clip) {
      static PlaygroundPoint circle_clip_point_a(Point(60, 300), 20,
                                                 Color::Red());
      static PlaygroundPoint circle_clip_point_b(Point(600, 300), 20,
                                                 Color::Red());
      auto [handle_a, handle_b] =
          DrawPlaygroundLine(circle_clip_point_a, circle_clip_point_b);

      Matrix screen_to_canvas = builder.GetMatrix();
      if (!screen_to_canvas.IsInvertible()) {
        return nullptr;
      }
      screen_to_canvas = screen_to_canvas.Invert();

      Point point_a = screen_to_canvas * handle_a;
      Point point_b = screen_to_canvas * handle_b;

      Point middle = point_a + point_b;
      middle *= GetContentScale().x / 2;

      auto radius = point_a.GetDistance(middle);

      builder.ClipPath(DlPath::MakeCircle(middle, radius));
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
      DlPoint origin = {100, 100};
      builder.DrawLine(DlPoint(150, 100), DlPoint(250, 100), paint);
      for (int d = 15; d < 90; d += 15) {
        Matrix m = Matrix::MakeRotationZ(Degrees(d));
        Point origin = {100, 100};
        Point p0 = {50, 0};
        Point p1 = {150, 0};
        auto a = origin + m * p0;
        auto b = origin + m * p1;

        builder.DrawLine(a, b, paint);
      }
      builder.DrawLine(DlPoint(100, 150), DlPoint(100, 250), paint);
      builder.DrawCircle(origin, 35, paint);

      builder.DrawLine(DlPoint(250, 250), DlPoint(250, 250), paint);

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
  builder.DrawPath(DlPath::MakeRect(DlRect::MakeSize(DlSize(100, 100))), paint);

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
  builder.DrawPath(DlPath::MakeRect(DlRect::MakeSize(DlSize(100, 100))), paint);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanDrawMultiContourConvexPath) {
  DlPathBuilder path_builder;
  for (auto i = 0; i < 10; i++) {
    if (i % 2 == 0) {
      // We use the factory method to convert the circle to a path so that it
      // uses the legacy conics for legacy golden output.
      DlPath circle =
          DlPath::MakeCircle(DlPoint(100 + 50 * i, 100 + 50 * i), 100);
      path_builder.AddPath(circle.GetPath());
      path_builder.Close();
    } else {
      path_builder.MoveTo(DlPoint(100.f + 50.f * i - 100, 100.f + 50.f * i));
      path_builder.LineTo(DlPoint(100.f + 50.f * i, 100.f + 50.f * i - 100));
      path_builder.LineTo(DlPoint(100.f + 50.f * i - 100,  //
                                  100.f + 50.f * i - 100));
      path_builder.Close();
    }
  }
  DlPath path(path_builder);

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

  DlPathBuilder path_builder;
  path_builder.AddArc(DlRect::MakeXYWH(10, 10, 100, 100),  //
                      DlDegrees(0), DlDegrees(0));
  builder.DrawPath(DlPath(path_builder), paint);

  // Check that this empty picture can be created without crashing.
  builder.Build();
}

TEST_P(AiksTest, CanRenderClips) {
  DisplayListBuilder builder;
  DlPaint paint;
  paint.setColor(DlColor::kFuchsia());

  builder.ClipPath(DlPath::MakeRect(DlRect::MakeXYWH(0, 0, 500, 500)));
  builder.DrawPath(DlPath::MakeCircle(DlPoint(500, 500), 250), paint);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, FatStrokeArc) {
  DlScalar stroke_width = 300;
  DlScalar aspect = 1.0;
  DlScalar start_angle = 0;
  DlScalar end_angle = 90;
  auto callback = [&]() -> sk_sp<DisplayList> {
    if (AiksTest::ImGuiBegin("Controls", nullptr,
                             ImGuiWindowFlags_AlwaysAutoResize)) {
      ImGui::SliderFloat("Stroke Width", &stroke_width, 1, 300);
      ImGui::SliderFloat("Aspect", &aspect, 0.5, 2.0);
      ImGui::SliderFloat("Start Angle", &start_angle, 0, 360);
      ImGui::SliderFloat("End Angle", &end_angle, 0, 360);
      ImGui::End();
    }

    DisplayListBuilder builder;
    DlPaint grey_paint;
    grey_paint.setColor(DlColor(0xff111111));
    builder.DrawPaint(grey_paint);

    DlPaint white_paint;
    white_paint.setColor(DlColor::kWhite());
    white_paint.setStrokeWidth(stroke_width);
    white_paint.setDrawStyle(DlDrawStyle::kStroke);
    DlPaint red_paint;
    red_paint.setColor(DlColor::kRed());

    Rect rect = Rect::MakeXYWH(100, 100, 100, aspect * 100);
    builder.DrawRect(rect, red_paint);
    builder.DrawArc(rect, start_angle, end_angle,
                    /*useCenter=*/false, white_paint);
    DlScalar frontier = rect.GetRight() + stroke_width / 2.0;
    builder.DrawLine(Point(frontier, 0), Point(frontier, 150), red_paint);

    return builder.Build();
  };
  ASSERT_TRUE(OpenPlaygroundHere(callback));
}

TEST_P(AiksTest, CanRenderOverlappingMultiContourPath) {
  DisplayListBuilder builder;

  DlPaint paint;
  paint.setColor(DlColor::kRed());

  RoundingRadii radii = {
      .top_left = DlSize(50, 50),
      .top_right = DlSize(50, 50),
      .bottom_left = DlSize(50, 50),
      .bottom_right = DlSize(50, 50),
  };

  const Scalar kTriangleHeight = 100;
  DlRoundRect rrect = DlRoundRect::MakeRectRadii(
      DlRect::MakeXYWH(-kTriangleHeight / 2.0f, -kTriangleHeight / 2.0f,
                       kTriangleHeight, kTriangleHeight),
      radii  //
  );
  // We use the factory method to convert the rrect to a path so that it
  // uses the legacy conics for legacy golden output.
  DlPath rrect_path = DlPath::MakeRoundRect(rrect);

  builder.Translate(200, 200);
  // Form a path similar to the Material drop slider value indicator. Both
  // shapes should render identically side-by-side.
  {
    DlPathBuilder path_builder;
    path_builder.MoveTo(DlPoint(0, kTriangleHeight));
    path_builder.LineTo(DlPoint(-kTriangleHeight / 2.0f, 0));
    path_builder.LineTo(DlPoint(kTriangleHeight / 2.0f, 0));
    path_builder.Close();
    path_builder.AddPath(rrect_path.GetPath());

    builder.DrawPath(DlPath(path_builder), paint);
  }
  builder.Translate(100, 0);

  {
    DlPathBuilder path_builder;
    path_builder.MoveTo(DlPoint(0, kTriangleHeight));
    path_builder.LineTo(DlPoint(-kTriangleHeight / 2.0f, 0));
    path_builder.LineTo(DlPoint(0, -10));
    path_builder.LineTo(DlPoint(kTriangleHeight / 2.0f, 0));
    path_builder.Close();
    path_builder.AddPath(rrect_path.GetPath());

    builder.DrawPath(DlPath(path_builder), paint);
  }

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

}  // namespace testing
}  // namespace impeller
