// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <array>
#include <cmath>
#include <memory>
#include <vector>

#include "flutter/display_list/dl_blend_mode.h"
#include "flutter/display_list/dl_builder.h"
#include "flutter/display_list/dl_color.h"
#include "flutter/display_list/dl_paint.h"
#include "flutter/display_list/dl_tile_mode.h"
#include "flutter/display_list/effects/dl_color_filter.h"
#include "flutter/display_list/effects/dl_color_source.h"
#include "flutter/display_list/effects/dl_image_filters.h"
#include "flutter/display_list/effects/dl_mask_filter.h"
#include "flutter/testing/testing.h"
#include "gtest/gtest.h"
#include "impeller/display_list/aiks_context.h"
#include "impeller/display_list/dl_dispatcher.h"
#include "impeller/display_list/dl_image_impeller.h"
#include "impeller/display_list/dl_playground.h"
#include "impeller/entity/contents/clip_contents.h"
#include "impeller/entity/contents/solid_color_contents.h"
#include "impeller/entity/contents/solid_rrect_blur_contents.h"
#include "impeller/geometry/constants.h"
#include "impeller/geometry/point.h"
#include "impeller/geometry/scalar.h"
#include "impeller/playground/widgets.h"
#include "third_party/imgui/imgui.h"
#include "third_party/skia/include/core/SkBlurTypes.h"
#include "third_party/skia/include/core/SkClipOp.h"
#include "third_party/skia/include/core/SkPathBuilder.h"
#include "third_party/skia/include/core/SkRRect.h"

namespace impeller {
namespace testing {

flutter::DlColor toColor(const float* components) {
  return flutter::DlColor(Color::ToIColor(
      Color(components[0], components[1], components[2], components[3])));
}

using DisplayListTest = DlPlayground;
INSTANTIATE_PLAYGROUND_SUITE(DisplayListTest);

TEST_P(DisplayListTest, CanDrawRect) {
  flutter::DisplayListBuilder builder;
  builder.DrawRect(SkRect::MakeXYWH(10, 10, 100, 100),
                   flutter::DlPaint(flutter::DlColor::kBlue()));
  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(DisplayListTest, CanDrawTextBlob) {
  flutter::DisplayListBuilder builder;
  builder.DrawTextBlob(SkTextBlob::MakeFromString("Hello", CreateTestFont()),
                       100, 100, flutter::DlPaint(flutter::DlColor::kBlue()));
  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(DisplayListTest, CanDrawTextBlobWithGradient) {
  flutter::DisplayListBuilder builder;

  std::vector<flutter::DlColor> colors = {flutter::DlColor::kBlue(),
                                          flutter::DlColor::kRed()};
  const float stops[2] = {0.0, 1.0};

  auto linear = flutter::DlColorSource::MakeLinear({0.0, 0.0}, {300.0, 300.0},
                                                   2, colors.data(), stops,
                                                   flutter::DlTileMode::kClamp);
  flutter::DlPaint paint;
  paint.setColorSource(linear);

  builder.DrawTextBlob(
      SkTextBlob::MakeFromString("Hello World", CreateTestFont()), 100, 100,
      paint);
  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(DisplayListTest, CanDrawTextWithSaveLayer) {
  flutter::DisplayListBuilder builder;
  builder.DrawTextBlob(SkTextBlob::MakeFromString("Hello", CreateTestFont()),
                       100, 100, flutter::DlPaint(flutter::DlColor::kRed()));

  flutter::DlPaint save_paint;
  float alpha = 0.5;
  save_paint.setAlpha(static_cast<uint8_t>(255 * alpha));
  builder.SaveLayer(nullptr, &save_paint);
  builder.DrawTextBlob(SkTextBlob::MakeFromString("Hello with half alpha",
                                                  CreateTestFontOfSize(100)),
                       100, 300, flutter::DlPaint(flutter::DlColor::kRed()));
  builder.Restore();
  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(DisplayListTest, CanDrawImage) {
  auto texture = CreateTextureForFixture("embarcadero.jpg");
  flutter::DisplayListBuilder builder;
  builder.DrawImage(DlImageImpeller::Make(texture), SkPoint::Make(100, 100),
                    flutter::DlImageSampling::kNearestNeighbor, nullptr);
  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(DisplayListTest, CanDrawCapsAndJoins) {
  flutter::DisplayListBuilder builder;
  flutter::DlPaint paint;

  paint.setDrawStyle(flutter::DlDrawStyle::kStroke);
  paint.setStrokeWidth(30);
  paint.setColor(flutter::DlColor::kRed());

  auto path =
      SkPathBuilder{}.moveTo(-50, 0).lineTo(0, -50).lineTo(50, 0).snapshot();

  builder.Translate(100, 100);
  {
    paint.setStrokeCap(flutter::DlStrokeCap::kButt);
    paint.setStrokeJoin(flutter::DlStrokeJoin::kMiter);
    paint.setStrokeMiter(4);
    builder.DrawPath(path, paint);
  }

  {
    builder.Save();
    builder.Translate(0, 100);
    // The joint in the path is 45 degrees. A miter length of 1 convert to a
    // bevel in this case.
    paint.setStrokeMiter(1);
    builder.DrawPath(path, paint);
    builder.Restore();
  }

  builder.Translate(150, 0);
  {
    paint.setStrokeCap(flutter::DlStrokeCap::kSquare);
    paint.setStrokeJoin(flutter::DlStrokeJoin::kBevel);
    builder.DrawPath(path, paint);
  }

  builder.Translate(150, 0);
  {
    paint.setStrokeCap(flutter::DlStrokeCap::kRound);
    paint.setStrokeJoin(flutter::DlStrokeJoin::kRound);
    builder.DrawPath(path, paint);
  }

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(DisplayListTest, CanDrawArc) {
  auto callback = [&]() {
    static float start_angle = 45;
    static float sweep_angle = 270;
    static float stroke_width = 10;
    static bool use_center = true;

    static int selected_cap = 0;
    const char* cap_names[] = {"Butt", "Round", "Square"};
    flutter::DlStrokeCap cap;

    ImGui::Begin("Controls", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    ImGui::SliderFloat("Start angle", &start_angle, -360, 360);
    ImGui::SliderFloat("Sweep angle", &sweep_angle, -360, 360);
    ImGui::SliderFloat("Stroke width", &stroke_width, 0, 300);
    ImGui::Combo("Cap", &selected_cap, cap_names,
                 sizeof(cap_names) / sizeof(char*));
    ImGui::Checkbox("Use center", &use_center);
    ImGui::End();

    switch (selected_cap) {
      case 0:
        cap = flutter::DlStrokeCap::kButt;
        break;
      case 1:
        cap = flutter::DlStrokeCap::kRound;
        break;
      case 2:
        cap = flutter::DlStrokeCap::kSquare;
        break;
      default:
        cap = flutter::DlStrokeCap::kButt;
        break;
    }

    static PlaygroundPoint point_a(Point(200, 200), 20, Color::White());
    static PlaygroundPoint point_b(Point(400, 400), 20, Color::White());
    auto [p1, p2] = DrawPlaygroundLine(point_a, point_b);

    flutter::DisplayListBuilder builder;
    flutter::DlPaint paint;

    Vector2 scale = GetContentScale();
    builder.Scale(scale.x, scale.y);
    paint.setDrawStyle(flutter::DlDrawStyle::kStroke);
    paint.setStrokeCap(cap);
    paint.setStrokeJoin(flutter::DlStrokeJoin::kMiter);
    paint.setStrokeMiter(10);
    auto rect = SkRect::MakeLTRB(p1.x, p1.y, p2.x, p2.y);
    paint.setColor(flutter::DlColor::kGreen());
    paint.setStrokeWidth(2);
    builder.DrawRect(rect, paint);
    paint.setColor(flutter::DlColor::kRed());
    paint.setStrokeWidth(stroke_width);
    builder.DrawArc(rect, start_angle, sweep_angle, use_center, paint);

    return builder.Build();
  };
  ASSERT_TRUE(OpenPlaygroundHere(callback));
}

TEST_P(DisplayListTest, StrokedPathsDrawCorrectly) {
  auto callback = [&]() {
    flutter::DisplayListBuilder builder;
    flutter::DlPaint paint;

    paint.setColor(flutter::DlColor::kRed());
    paint.setDrawStyle(flutter::DlDrawStyle::kStroke);

    static float stroke_width = 10.0f;
    static int selected_stroke_type = 0;
    static int selected_join_type = 0;
    const char* stroke_types[] = {"Butte", "Round", "Square"};
    const char* join_type[] = {"kMiter", "Round", "kBevel"};

    ImGui::Begin("Controls", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    ImGui::Combo("Cap", &selected_stroke_type, stroke_types,
                 sizeof(stroke_types) / sizeof(char*));
    ImGui::Combo("Join", &selected_join_type, join_type,
                 sizeof(join_type) / sizeof(char*));
    ImGui::SliderFloat("Stroke Width", &stroke_width, 10.0f, 50.0f);
    ImGui::End();

    flutter::DlStrokeCap cap;
    flutter::DlStrokeJoin join;
    switch (selected_stroke_type) {
      case 0:
        cap = flutter::DlStrokeCap::kButt;
        break;
      case 1:
        cap = flutter::DlStrokeCap::kRound;
        break;
      case 2:
        cap = flutter::DlStrokeCap::kSquare;
        break;
      default:
        cap = flutter::DlStrokeCap::kButt;
        break;
    }
    switch (selected_join_type) {
      case 0:
        join = flutter::DlStrokeJoin::kMiter;
        break;
      case 1:
        join = flutter::DlStrokeJoin::kRound;
        break;
      case 2:
        join = flutter::DlStrokeJoin::kBevel;
        break;
      default:
        join = flutter::DlStrokeJoin::kMiter;
        break;
    }
    paint.setStrokeCap(cap);
    paint.setStrokeJoin(join);
    paint.setStrokeWidth(stroke_width);

    // Make rendering better to watch.
    builder.Scale(1.5f, 1.5f);

    // Rectangle
    builder.Translate(100, 100);
    builder.DrawRect(SkRect::MakeSize({100, 100}), paint);

    // Rounded rectangle
    builder.Translate(150, 0);
    builder.DrawRRect(SkRRect::MakeRectXY(SkRect::MakeSize({100, 50}), 10, 10),
                      paint);

    // Double rounded rectangle
    builder.Translate(150, 0);
    builder.DrawDRRect(
        SkRRect::MakeRectXY(SkRect::MakeSize({100, 50}), 10, 10),
        SkRRect::MakeRectXY(SkRect::MakeXYWH(10, 10, 80, 30), 10, 10), paint);

    // Contour with duplicate join points
    {
      builder.Translate(150, 0);
      SkPath path;
      path.moveTo(0, 0);
      path.lineTo(0, 0);
      path.lineTo({100, 0});
      path.lineTo({100, 0});
      path.lineTo({100, 100});
      builder.DrawPath(path, paint);
    }

    // Contour with duplicate start and end points

    // Line.
    builder.Translate(200, 0);
    {
      builder.Save();

      SkPath line_path;
      line_path.moveTo(0, 0);
      line_path.moveTo(0, 0);
      line_path.lineTo({0, 0});
      line_path.lineTo({0, 0});
      line_path.lineTo({50, 50});
      line_path.lineTo({50, 50});
      line_path.lineTo({100, 0});
      line_path.lineTo({100, 0});
      builder.DrawPath(line_path, paint);

      builder.Translate(0, 100);
      builder.DrawPath(line_path, paint);

      builder.Translate(0, 100);
      SkPath line_path2;
      line_path2.moveTo(0, 0);
      line_path2.lineTo(0, 0);
      line_path2.lineTo(0, 0);
      builder.DrawPath(line_path2, paint);

      builder.Restore();
    }

    // Cubic.
    builder.Translate(150, 0);
    {
      builder.Save();

      SkPath cubic_path;
      cubic_path.moveTo({0, 0});
      cubic_path.cubicTo(0, 0, 140.0, 100.0, 140, 20);
      builder.DrawPath(cubic_path, paint);

      builder.Translate(0, 100);
      SkPath cubic_path2;
      cubic_path2.moveTo({0, 0});
      cubic_path2.cubicTo(0, 0, 0, 0, 150, 150);
      builder.DrawPath(cubic_path2, paint);

      builder.Translate(0, 100);
      SkPath cubic_path3;
      cubic_path3.moveTo({0, 0});
      cubic_path3.cubicTo(0, 0, 0, 0, 0, 0);
      builder.DrawPath(cubic_path3, paint);

      builder.Restore();
    }

    // Quad.
    builder.Translate(200, 0);
    {
      builder.Save();

      SkPath quad_path;
      quad_path.moveTo(0, 0);
      quad_path.moveTo(0, 0);
      quad_path.quadTo({100, 40}, {50, 80});
      builder.DrawPath(quad_path, paint);

      builder.Translate(0, 150);
      SkPath quad_path2;
      quad_path2.moveTo(0, 0);
      quad_path2.moveTo(0, 0);
      quad_path2.quadTo({0, 0}, {100, 100});
      builder.DrawPath(quad_path2, paint);

      builder.Translate(0, 100);
      SkPath quad_path3;
      quad_path3.moveTo(0, 0);
      quad_path3.quadTo({0, 0}, {0, 0});
      builder.DrawPath(quad_path3, paint);

      builder.Restore();
    }
    return builder.Build();
  };
  ASSERT_TRUE(OpenPlaygroundHere(callback));
}

TEST_P(DisplayListTest, CanDrawWithOddPathWinding) {
  flutter::DisplayListBuilder builder;
  flutter::DlPaint paint;

  paint.setColor(flutter::DlColor::kRed());
  paint.setDrawStyle(flutter::DlDrawStyle::kFill);

  builder.Translate(300, 300);
  SkPath path;
  path.setFillType(SkPathFillType::kEvenOdd);
  path.addCircle(0, 0, 100);
  path.addCircle(0, 0, 50);
  builder.DrawPath(path, paint);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

// Regression test for https://github.com/flutter/flutter/issues/134816.
//
// It should be possible to draw 3 lines, and not have an implicit close path.
TEST_P(DisplayListTest, CanDrawAnOpenPath) {
  flutter::DisplayListBuilder builder;
  flutter::DlPaint paint;

  paint.setColor(flutter::DlColor::kRed());
  paint.setDrawStyle(flutter::DlDrawStyle::kStroke);
  paint.setStrokeWidth(10);

  builder.Translate(300, 300);

  // Move to (50, 50) and draw lines from:
  // 1. (50, height)
  // 2. (width, height)
  // 3. (width, 50)
  SkPath path;
  path.moveTo(50, 50);
  path.lineTo(50, 100);
  path.lineTo(100, 100);
  path.lineTo(100, 50);
  builder.DrawPath(path, paint);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(DisplayListTest, CanDrawWithMaskBlur) {
  auto texture = CreateTextureForFixture("embarcadero.jpg");
  flutter::DisplayListBuilder builder;
  flutter::DlPaint paint;

  // Mask blurred image.
  {
    auto filter =
        flutter::DlBlurMaskFilter(flutter::DlBlurStyle::kNormal, 10.0f);
    paint.setMaskFilter(&filter);
    builder.DrawImage(DlImageImpeller::Make(texture), SkPoint::Make(100, 100),
                      flutter::DlImageSampling::kNearestNeighbor, &paint);
  }

  // Mask blurred filled path.
  {
    paint.setColor(flutter::DlColor::kYellow());
    auto filter =
        flutter::DlBlurMaskFilter(flutter::DlBlurStyle::kOuter, 10.0f);
    paint.setMaskFilter(&filter);
    builder.DrawArc(SkRect::MakeXYWH(410, 110, 100, 100), 45, 270, true, paint);
  }

  // Mask blurred text.
  {
    auto filter =
        flutter::DlBlurMaskFilter(flutter::DlBlurStyle::kSolid, 10.0f);
    paint.setMaskFilter(&filter);
    builder.DrawTextBlob(
        SkTextBlob::MakeFromString("Testing", CreateTestFont()), 220, 170,
        paint);
  }

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(DisplayListTest, CanDrawStrokedText) {
  flutter::DisplayListBuilder builder;
  flutter::DlPaint paint;

  paint.setDrawStyle(flutter::DlDrawStyle::kStroke);
  paint.setColor(flutter::DlColor::kRed());
  builder.DrawTextBlob(
      SkTextBlob::MakeFromString("stoked about stroked text", CreateTestFont()),
      250, 250, paint);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

// Regression test for https://github.com/flutter/flutter/issues/133157.
TEST_P(DisplayListTest, StrokedTextNotOffsetFromNormalText) {
  flutter::DisplayListBuilder builder;
  flutter::DlPaint paint;
  auto const& text_blob = SkTextBlob::MakeFromString("00000", CreateTestFont());

  // https://api.flutter.dev/flutter/material/Colors/blue-constant.html.
  auto const& mat_blue = flutter::DlColor(0xFF2196f3);

  // Draw a blue filled rectangle so the text is easier to see.
  paint.setDrawStyle(flutter::DlDrawStyle::kFill);
  paint.setColor(mat_blue);
  builder.DrawRect(SkRect::MakeXYWH(0, 0, 500, 500), paint);

  // Draw stacked text, with stroked text on top.
  paint.setDrawStyle(flutter::DlDrawStyle::kFill);
  paint.setColor(flutter::DlColor::kWhite());
  builder.DrawTextBlob(text_blob, 250, 250, paint);

  paint.setDrawStyle(flutter::DlDrawStyle::kStroke);
  paint.setColor(flutter::DlColor::kBlack());
  builder.DrawTextBlob(text_blob, 250, 250, paint);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(DisplayListTest, IgnoreMaskFilterWhenSavingLayer) {
  auto texture = CreateTextureForFixture("embarcadero.jpg");
  flutter::DisplayListBuilder builder;
  auto filter = flutter::DlBlurMaskFilter(flutter::DlBlurStyle::kNormal, 10.0f);
  flutter::DlPaint paint;
  paint.setMaskFilter(&filter);
  builder.SaveLayer(nullptr, &paint);
  builder.DrawImage(DlImageImpeller::Make(texture), SkPoint::Make(100, 100),
                    flutter::DlImageSampling::kNearestNeighbor);
  builder.Restore();
  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(DisplayListTest, CanDrawWithBlendColorFilter) {
  auto texture = CreateTextureForFixture("embarcadero.jpg");
  flutter::DisplayListBuilder builder;
  flutter::DlPaint paint;

  // Pipeline blended image.
  {
    auto filter = flutter::DlColorFilter::MakeBlend(
        flutter::DlColor::kYellow(), flutter::DlBlendMode::kModulate);
    paint.setColorFilter(filter);
    builder.DrawImage(DlImageImpeller::Make(texture), SkPoint::Make(100, 100),
                      flutter::DlImageSampling::kNearestNeighbor, &paint);
  }

  // Advanced blended image.
  {
    auto filter = flutter::DlColorFilter::MakeBlend(
        flutter::DlColor::kRed(), flutter::DlBlendMode::kScreen);
    paint.setColorFilter(filter);
    builder.DrawImage(DlImageImpeller::Make(texture), SkPoint::Make(250, 250),
                      flutter::DlImageSampling::kNearestNeighbor, &paint);
  }

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(DisplayListTest, CanDrawWithColorFilterImageFilter) {
  const float invert_color_matrix[20] = {
      -1, 0,  0,  0, 1,  //
      0,  -1, 0,  0, 1,  //
      0,  0,  -1, 0, 1,  //
      0,  0,  0,  1, 0,  //
  };
  auto texture = CreateTextureForFixture("boston.jpg");
  flutter::DisplayListBuilder builder;
  flutter::DlPaint paint;

  auto color_filter = flutter::DlColorFilter::MakeMatrix(invert_color_matrix);
  auto image_filter = flutter::DlImageFilter::MakeColorFilter(color_filter);

  paint.setImageFilter(image_filter);
  builder.DrawImage(DlImageImpeller::Make(texture), SkPoint::Make(100, 100),
                    flutter::DlImageSampling::kNearestNeighbor, &paint);

  builder.Translate(0, 700);
  paint.setColorFilter(color_filter);
  builder.DrawImage(DlImageImpeller::Make(texture), SkPoint::Make(100, 100),
                    flutter::DlImageSampling::kNearestNeighbor, &paint);
  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(DisplayListTest, CanDrawWithImageBlurFilter) {
  auto texture = CreateTextureForFixture("embarcadero.jpg");

  auto callback = [&]() {
    static float sigma[] = {10, 10};

    ImGui::Begin("Controls", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    ImGui::SliderFloat2("Sigma", sigma, 0, 100);
    ImGui::End();

    flutter::DisplayListBuilder builder;
    flutter::DlPaint paint;

    auto filter = flutter::DlBlurImageFilter(sigma[0], sigma[1],
                                             flutter::DlTileMode::kClamp);
    paint.setImageFilter(&filter);
    builder.DrawImage(DlImageImpeller::Make(texture), SkPoint::Make(200, 200),
                      flutter::DlImageSampling::kNearestNeighbor, &paint);

    return builder.Build();
  };

  ASSERT_TRUE(OpenPlaygroundHere(callback));
}

TEST_P(DisplayListTest, CanDrawWithComposeImageFilter) {
  auto texture = CreateTextureForFixture("boston.jpg");
  flutter::DisplayListBuilder builder;
  flutter::DlPaint paint;

  auto dilate = std::make_shared<flutter::DlDilateImageFilter>(10.0, 10.0);
  auto erode = std::make_shared<flutter::DlErodeImageFilter>(10.0, 10.0);
  auto open = std::make_shared<flutter::DlComposeImageFilter>(dilate, erode);
  auto close = std::make_shared<flutter::DlComposeImageFilter>(erode, dilate);

  paint.setImageFilter(open.get());
  builder.DrawImage(DlImageImpeller::Make(texture), SkPoint::Make(100, 100),
                    flutter::DlImageSampling::kNearestNeighbor, &paint);
  builder.Translate(0, 700);
  paint.setImageFilter(close.get());
  builder.DrawImage(DlImageImpeller::Make(texture), SkPoint::Make(100, 100),
                    flutter::DlImageSampling::kNearestNeighbor, &paint);
  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(DisplayListTest, CanClampTheResultingColorOfColorMatrixFilter) {
  auto texture = CreateTextureForFixture("boston.jpg");
  const float inner_color_matrix[20] = {
      1, 0, 0, 0, 0,  //
      0, 1, 0, 0, 0,  //
      0, 0, 1, 0, 0,  //
      0, 0, 0, 2, 0,  //
  };
  const float outer_color_matrix[20] = {
      1, 0, 0, 0,   0,  //
      0, 1, 0, 0,   0,  //
      0, 0, 1, 0,   0,  //
      0, 0, 0, 0.5, 0,  //
  };
  auto inner_color_filter =
      flutter::DlColorFilter::MakeMatrix(inner_color_matrix);
  auto outer_color_filter =
      flutter::DlColorFilter::MakeMatrix(outer_color_matrix);
  auto inner = flutter::DlImageFilter::MakeColorFilter(inner_color_filter);
  auto outer = flutter::DlImageFilter::MakeColorFilter(outer_color_filter);
  auto compose = std::make_shared<flutter::DlComposeImageFilter>(outer, inner);

  flutter::DisplayListBuilder builder;
  flutter::DlPaint paint;
  paint.setImageFilter(compose.get());
  builder.DrawImage(DlImageImpeller::Make(texture), SkPoint::Make(100, 100),
                    flutter::DlImageSampling::kNearestNeighbor, &paint);
  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(DisplayListTest, CanDrawBackdropFilter) {
  auto texture = CreateTextureForFixture("embarcadero.jpg");

  auto callback = [&]() {
    static float sigma[] = {10, 10};
    static float ctm_scale = 1;
    static bool use_bounds = true;
    static bool draw_circle = true;
    static bool add_clip = true;

    ImGui::Begin("Controls", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    ImGui::SliderFloat2("Sigma", sigma, 0, 100);
    ImGui::SliderFloat("Scale", &ctm_scale, 0, 10);
    ImGui::NewLine();
    ImGui::TextWrapped(
        "If everything is working correctly, none of the options below should "
        "impact the filter's appearance.");
    ImGui::Checkbox("Use SaveLayer bounds", &use_bounds);
    ImGui::Checkbox("Draw child element", &draw_circle);
    ImGui::Checkbox("Add pre-clip", &add_clip);
    ImGui::End();

    flutter::DisplayListBuilder builder;

    Vector2 scale = ctm_scale * GetContentScale();
    builder.Scale(scale.x, scale.y);

    auto filter = flutter::DlBlurImageFilter(sigma[0], sigma[1],
                                             flutter::DlTileMode::kClamp);

    std::optional<SkRect> bounds;
    if (use_bounds) {
      static PlaygroundPoint point_a(Point(350, 150), 20, Color::White());
      static PlaygroundPoint point_b(Point(800, 600), 20, Color::White());
      auto [p1, p2] = DrawPlaygroundLine(point_a, point_b);
      bounds = SkRect::MakeLTRB(p1.x, p1.y, p2.x, p2.y);
    }

    // Insert a clip to test that the backdrop filter handles stencil depths > 0
    // correctly.
    if (add_clip) {
      builder.ClipRect(SkRect::MakeLTRB(0, 0, 99999, 99999),
                       flutter::DlCanvas::ClipOp::kIntersect, true);
    }

    builder.DrawImage(DlImageImpeller::Make(texture), SkPoint::Make(200, 200),
                      flutter::DlImageSampling::kNearestNeighbor, nullptr);
    builder.SaveLayer(bounds.has_value() ? &bounds.value() : nullptr, nullptr,
                      &filter);

    if (draw_circle) {
      static PlaygroundPoint center_point(Point(500, 400), 20, Color::Red());
      auto circle_center = DrawPlaygroundPoint(center_point);

      flutter::DlPaint paint;
      paint.setDrawStyle(flutter::DlDrawStyle::kStroke);
      paint.setStrokeCap(flutter::DlStrokeCap::kButt);
      paint.setStrokeJoin(flutter::DlStrokeJoin::kBevel);
      paint.setStrokeWidth(10);
      paint.setColor(flutter::DlColor::kRed().withAlpha(100));
      builder.DrawCircle(SkPoint{circle_center.x, circle_center.y}, 100, paint);
    }

    return builder.Build();
  };

  ASSERT_TRUE(OpenPlaygroundHere(callback));
}

TEST_P(DisplayListTest, CanDrawNinePatchImage) {
  // Image is drawn with corners to scale and center pieces stretched to fit.
  auto texture = CreateTextureForFixture("embarcadero.jpg");
  flutter::DisplayListBuilder builder;
  auto size = texture->GetSize();
  builder.DrawImageNine(
      DlImageImpeller::Make(texture),
      SkIRect::MakeLTRB(size.width / 4, size.height / 4, size.width * 3 / 4,
                        size.height * 3 / 4),
      SkRect::MakeLTRB(0, 0, size.width * 2, size.height * 2),
      flutter::DlFilterMode::kNearest, nullptr);
  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(DisplayListTest, CanDrawNinePatchImageCenterWidthBiggerThanDest) {
  // Edge case, the width of the corners does not leave any room for the
  // center slice. The center (across the vertical axis) is folded out of the
  // resulting image.
  auto texture = CreateTextureForFixture("embarcadero.jpg");
  flutter::DisplayListBuilder builder;
  auto size = texture->GetSize();
  builder.DrawImageNine(
      DlImageImpeller::Make(texture),
      SkIRect::MakeLTRB(size.width / 4, size.height / 4, size.width * 3 / 4,
                        size.height * 3 / 4),
      SkRect::MakeLTRB(0, 0, size.width / 2, size.height),
      flutter::DlFilterMode::kNearest, nullptr);
  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(DisplayListTest, CanDrawNinePatchImageCenterHeightBiggerThanDest) {
  // Edge case, the height of the corners does not leave any room for the
  // center slice. The center (across the horizontal axis) is folded out of the
  // resulting image.
  auto texture = CreateTextureForFixture("embarcadero.jpg");
  flutter::DisplayListBuilder builder;
  auto size = texture->GetSize();
  builder.DrawImageNine(
      DlImageImpeller::Make(texture),
      SkIRect::MakeLTRB(size.width / 4, size.height / 4, size.width * 3 / 4,
                        size.height * 3 / 4),
      SkRect::MakeLTRB(0, 0, size.width, size.height / 2),
      flutter::DlFilterMode::kNearest, nullptr);
  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(DisplayListTest, CanDrawNinePatchImageCenterBiggerThanDest) {
  // Edge case, the width and height of the corners does not leave any
  // room for the center slices. Only the corners are displayed.
  auto texture = CreateTextureForFixture("embarcadero.jpg");
  flutter::DisplayListBuilder builder;
  auto size = texture->GetSize();
  builder.DrawImageNine(
      DlImageImpeller::Make(texture),
      SkIRect::MakeLTRB(size.width / 4, size.height / 4, size.width * 3 / 4,
                        size.height * 3 / 4),
      SkRect::MakeLTRB(0, 0, size.width / 2, size.height / 2),
      flutter::DlFilterMode::kNearest, nullptr);
  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(DisplayListTest, CanDrawNinePatchImageCornersScaledDown) {
  // Edge case, there is not enough room for the corners to be drawn
  // without scaling them down.
  auto texture = CreateTextureForFixture("embarcadero.jpg");
  flutter::DisplayListBuilder builder;
  auto size = texture->GetSize();
  builder.DrawImageNine(
      DlImageImpeller::Make(texture),
      SkIRect::MakeLTRB(size.width / 4, size.height / 4, size.width * 3 / 4,
                        size.height * 3 / 4),
      SkRect::MakeLTRB(0, 0, size.width / 4, size.height / 4),
      flutter::DlFilterMode::kNearest, nullptr);
  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(DisplayListTest, NinePatchImagePrecision) {
  // Draw a nine patch image with colored corners and verify that the corner
  // color does not leak outside the intended region.
  auto texture = CreateTextureForFixture("nine_patch_corners.png");
  flutter::DisplayListBuilder builder;
  builder.DrawImageNine(DlImageImpeller::Make(texture),
                        SkIRect::MakeXYWH(10, 10, 1, 1),
                        SkRect::MakeXYWH(0, 0, 200, 100),
                        flutter::DlFilterMode::kNearest, nullptr);
  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(DisplayListTest, CanDrawPoints) {
  flutter::DisplayListBuilder builder;
  SkPoint points[7] = {
      {0, 0},      //
      {100, 100},  //
      {100, 0},    //
      {0, 100},    //
      {0, 0},      //
      {48, 48},    //
      {52, 52},    //
  };
  std::vector<flutter::DlStrokeCap> caps = {
      flutter::DlStrokeCap::kButt,
      flutter::DlStrokeCap::kRound,
      flutter::DlStrokeCap::kSquare,
  };
  flutter::DlPaint paint =
      flutter::DlPaint()                                         //
          .setColor(flutter::DlColor::kYellow().withAlpha(127))  //
          .setStrokeWidth(20);
  builder.Translate(50, 50);
  for (auto cap : caps) {
    paint.setStrokeCap(cap);
    builder.Save();
    builder.DrawPoints(flutter::DlCanvas::PointMode::kPoints, 7, points, paint);
    builder.Translate(150, 0);
    builder.DrawPoints(flutter::DlCanvas::PointMode::kLines, 5, points, paint);
    builder.Translate(150, 0);
    builder.DrawPoints(flutter::DlCanvas::PointMode::kPolygon, 5, points,
                       paint);
    builder.Restore();
    builder.Translate(0, 150);
  }
  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(DisplayListTest, CanDrawZeroLengthLine) {
  flutter::DisplayListBuilder builder;
  std::vector<flutter::DlStrokeCap> caps = {
      flutter::DlStrokeCap::kButt,
      flutter::DlStrokeCap::kRound,
      flutter::DlStrokeCap::kSquare,
  };
  flutter::DlPaint paint =
      flutter::DlPaint()                                         //
          .setColor(flutter::DlColor::kYellow().withAlpha(127))  //
          .setDrawStyle(flutter::DlDrawStyle::kStroke)           //
          .setStrokeCap(flutter::DlStrokeCap::kButt)             //
          .setStrokeWidth(20);
  SkPath path = SkPath().addPoly({{150, 50}, {150, 50}}, false);
  for (auto cap : caps) {
    paint.setStrokeCap(cap);
    builder.DrawLine(SkPoint{50, 50}, SkPoint{50, 50}, paint);
    builder.DrawPath(path, paint);
    builder.Translate(0, 150);
  }
  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(DisplayListTest, CanDrawShadow) {
  flutter::DisplayListBuilder builder;
  flutter::DlPaint paint;

  auto content_scale = GetContentScale() * 0.8;
  builder.Scale(content_scale.x, content_scale.y);

  constexpr size_t star_spikes = 5;
  constexpr SkScalar half_spike_rotation = kPi / star_spikes;
  constexpr SkScalar radius = 40;
  constexpr SkScalar spike_size = 10;
  constexpr SkScalar outer_radius = radius + spike_size;
  constexpr SkScalar inner_radius = radius - spike_size;
  std::array<SkPoint, star_spikes * 2> star;
  for (size_t i = 0; i < star_spikes; i++) {
    const SkScalar rotation = half_spike_rotation * i * 2;
    star[i * 2] = SkPoint::Make(50 + std::sin(rotation) * outer_radius,
                                50 - std::cos(rotation) * outer_radius);
    star[i * 2 + 1] = SkPoint::Make(
        50 + std::sin(rotation + half_spike_rotation) * inner_radius,
        50 - std::cos(rotation + half_spike_rotation) * inner_radius);
  }

  std::array<SkPath, 4> paths = {
      SkPath{}.addRect(SkRect::MakeXYWH(0, 0, 200, 100)),
      SkPath{}.addRRect(
          SkRRect::MakeRectXY(SkRect::MakeXYWH(20, 0, 200, 100), 30, 30)),
      SkPath{}.addCircle(100, 50, 50),
      SkPath{}.addPoly(star.data(), star.size(), true),
  };
  paint.setColor(flutter::DlColor::kWhite());
  builder.DrawPaint(paint);
  paint.setColor(flutter::DlColor::kCyan());
  builder.Translate(100, 50);
  for (size_t x = 0; x < paths.size(); x++) {
    builder.Save();
    for (size_t y = 0; y < 6; y++) {
      builder.DrawShadow(paths[x], flutter::DlColor::kBlack(), 3 + y * 8, false,
                         1);
      builder.DrawPath(paths[x], paint);
      builder.Translate(0, 150);
    }
    builder.Restore();
    builder.Translate(250, 0);
  }

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(DisplayListTest, CanDrawZeroWidthLine) {
  flutter::DisplayListBuilder builder;
  std::vector<flutter::DlStrokeCap> caps = {
      flutter::DlStrokeCap::kButt,
      flutter::DlStrokeCap::kRound,
      flutter::DlStrokeCap::kSquare,
  };
  flutter::DlPaint paint =                              //
      flutter::DlPaint()                                //
          .setColor(flutter::DlColor::kWhite())         //
          .setDrawStyle(flutter::DlDrawStyle::kStroke)  //
          .setStrokeWidth(0);
  flutter::DlPaint outline_paint =                      //
      flutter::DlPaint()                                //
          .setColor(flutter::DlColor::kYellow())        //
          .setDrawStyle(flutter::DlDrawStyle::kStroke)  //
          .setStrokeCap(flutter::DlStrokeCap::kSquare)  //
          .setStrokeWidth(1);
  SkPath path = SkPath().addPoly({{150, 50}, {160, 50}}, false);
  for (auto cap : caps) {
    paint.setStrokeCap(cap);
    builder.DrawLine(SkPoint{50, 50}, SkPoint{60, 50}, paint);
    builder.DrawRect(SkRect{45, 45, 65, 55}, outline_paint);
    builder.DrawLine(SkPoint{100, 50}, SkPoint{100, 50}, paint);
    if (cap != flutter::DlStrokeCap::kButt) {
      builder.DrawRect(SkRect{95, 45, 105, 55}, outline_paint);
    }
    builder.DrawPath(path, paint);
    builder.DrawRect(path.getBounds().makeOutset(5, 5), outline_paint);
    builder.Translate(0, 150);
  }
  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(DisplayListTest, CanDrawWithMatrixFilter) {
  auto boston = CreateTextureForFixture("boston.jpg");

  auto callback = [&]() {
    static int selected_matrix_type = 0;
    const char* matrix_type_names[] = {"Matrix", "Local Matrix"};

    static float ctm_translation[2] = {200, 200};
    static float ctm_scale[2] = {0.65, 0.65};
    static float ctm_skew[2] = {0, 0};

    static bool enable = true;
    static float translation[2] = {100, 100};
    static float scale[2] = {0.8, 0.8};
    static float skew[2] = {0.2, 0.2};

    static bool enable_savelayer = true;

    ImGui::Begin("Controls", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    {
      ImGui::Combo("Filter type", &selected_matrix_type, matrix_type_names,
                   sizeof(matrix_type_names) / sizeof(char*));

      ImGui::TextWrapped("Current Transform");
      ImGui::SliderFloat2("CTM Translation", ctm_translation, 0, 1000);
      ImGui::SliderFloat2("CTM Scale", ctm_scale, 0, 3);
      ImGui::SliderFloat2("CTM Skew", ctm_skew, -3, 3);

      ImGui::TextWrapped(
          "MatrixFilter and LocalMatrixFilter modify the CTM in the same way. "
          "The only difference is that MatrixFilter doesn't affect the effect "
          "transform, whereas LocalMatrixFilter does.");
      // Note: See this behavior in:
      //       https://fiddle.skia.org/c/6cbb551ab36d06f163db8693972be954
      ImGui::Checkbox("Enable", &enable);
      ImGui::SliderFloat2("Filter Translation", translation, 0, 1000);
      ImGui::SliderFloat2("Filter Scale", scale, 0, 3);
      ImGui::SliderFloat2("Filter Skew", skew, -3, 3);

      ImGui::TextWrapped(
          "Rendering the filtered image within a layer can expose bounds "
          "issues. If the rendered image gets cut off when this setting is "
          "enabled, there's a coverage bug in the filter.");
      ImGui::Checkbox("Render in layer", &enable_savelayer);
    }
    ImGui::End();

    flutter::DisplayListBuilder builder;
    flutter::DlPaint paint;

    if (enable_savelayer) {
      builder.SaveLayer(nullptr, nullptr);
    }
    {
      auto content_scale = GetContentScale();
      builder.Scale(content_scale.x, content_scale.y);

      // Set the current transform
      auto ctm_matrix =
          SkMatrix::MakeAll(ctm_scale[0], ctm_skew[0], ctm_translation[0],  //
                            ctm_skew[1], ctm_scale[1], ctm_translation[1],  //
                            0, 0, 1);
      builder.Transform(ctm_matrix);

      // Set the matrix filter
      auto filter_matrix =
          Matrix::MakeRow(scale[0], skew[0], 0.0f, translation[0],  //
                          skew[1], scale[1], 0.0f, translation[1],  //
                          0.0f, 0.0f, 1.0f, 0.0f,                   //
                          0.0f, 0.0f, 0.0f, 1.0f);

      if (enable) {
        switch (selected_matrix_type) {
          case 0: {
            auto filter = flutter::DlMatrixImageFilter(
                filter_matrix, flutter::DlImageSampling::kLinear);
            paint.setImageFilter(&filter);
            break;
          }
          case 1: {
            auto internal_filter =
                flutter::DlBlurImageFilter(10, 10, flutter::DlTileMode::kDecal)
                    .shared();
            auto filter = flutter::DlLocalMatrixImageFilter(filter_matrix,
                                                            internal_filter);
            paint.setImageFilter(&filter);
            break;
          }
        }
      }

      builder.DrawImage(DlImageImpeller::Make(boston), SkPoint{},
                        flutter::DlImageSampling::kLinear, &paint);
    }
    if (enable_savelayer) {
      builder.Restore();
    }

    return builder.Build();
  };

  ASSERT_TRUE(OpenPlaygroundHere(callback));
}

TEST_P(DisplayListTest, CanDrawWithMatrixFilterWhenSavingLayer) {
  auto callback = [&]() {
    static float translation[2] = {0, 0};
    static bool enable_save_layer = true;

    ImGui::Begin("Controls", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    ImGui::SliderFloat2("Translation", translation, -130, 130);
    ImGui::Checkbox("Enable save layer", &enable_save_layer);
    ImGui::End();

    flutter::DisplayListBuilder builder;
    builder.Save();
    builder.Scale(2.0, 2.0);
    flutter::DlPaint paint;
    paint.setColor(flutter::DlColor::kYellow());
    builder.DrawRect(SkRect::MakeWH(300, 300), paint);
    paint.setStrokeWidth(1.0);
    paint.setDrawStyle(flutter::DlDrawStyle::kStroke);
    paint.setColor(flutter::DlColor::kBlack().withAlpha(0x80));
    builder.DrawLine(SkPoint::Make(150, 0), SkPoint::Make(150, 300), paint);
    builder.DrawLine(SkPoint::Make(0, 150), SkPoint::Make(300, 150), paint);

    flutter::DlPaint save_paint;
    SkRect bounds = SkRect::MakeXYWH(100, 100, 100, 100);
    Matrix translate_matrix =
        Matrix::MakeTranslation({translation[0], translation[1]});
    if (enable_save_layer) {
      auto filter = flutter::DlMatrixImageFilter(
          translate_matrix, flutter::DlImageSampling::kNearestNeighbor);
      save_paint.setImageFilter(filter.shared());
      builder.SaveLayer(&bounds, &save_paint);
    } else {
      builder.Save();
      builder.Transform(translate_matrix);
    }

    Matrix filter_matrix;
    filter_matrix.Translate({150, 150});
    filter_matrix.Scale({0.2f, 0.2f});
    filter_matrix.Translate({-150, -150});
    auto filter = flutter::DlMatrixImageFilter(
        filter_matrix, flutter::DlImageSampling::kNearestNeighbor);

    save_paint.setImageFilter(filter.shared());

    builder.SaveLayer(&bounds, &save_paint);
    flutter::DlPaint paint2;
    paint2.setColor(flutter::DlColor::kBlue());
    builder.DrawRect(bounds, paint2);
    builder.Restore();
    builder.Restore();
    return builder.Build();
  };

  ASSERT_TRUE(OpenPlaygroundHere(callback));
}

TEST_P(DisplayListTest, CanDrawRectWithLinearToSrgbColorFilter) {
  flutter::DlPaint paint;
  paint.setColor(flutter::DlColor(0xFF2196F3).withAlpha(128));
  flutter::DisplayListBuilder builder;
  paint.setColorFilter(flutter::DlColorFilter::MakeLinearToSrgbGamma());
  builder.DrawRect(SkRect::MakeXYWH(0, 0, 200, 200), paint);
  builder.Translate(0, 200);

  paint.setColorFilter(flutter::DlColorFilter::MakeSrgbToLinearGamma());
  builder.DrawRect(SkRect::MakeXYWH(0, 0, 200, 200), paint);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(DisplayListTest, CanDrawPaintWithColorSource) {
  const flutter::DlColor colors[2] = {
      flutter::DlColor(0xFFF44336),
      flutter::DlColor(0xFF2196F3),
  };
  const float stops[2] = {0.0, 1.0};
  flutter::DlPaint paint;
  flutter::DisplayListBuilder builder;
  auto clip_bounds = SkRect::MakeWH(300.0, 300.0);
  builder.Save();
  builder.Translate(100, 100);
  builder.ClipRect(clip_bounds, flutter::DlCanvas::ClipOp::kIntersect, false);
  auto linear =
      flutter::DlColorSource::MakeLinear({0.0, 0.0}, {100.0, 100.0}, 2, colors,
                                         stops, flutter::DlTileMode::kRepeat);
  paint.setColorSource(linear);
  builder.DrawPaint(paint);
  builder.Restore();

  builder.Save();
  builder.Translate(500, 100);
  builder.ClipRect(clip_bounds, flutter::DlCanvas::ClipOp::kIntersect, false);
  auto radial = flutter::DlColorSource::MakeRadial(
      {100.0, 100.0}, 100.0, 2, colors, stops, flutter::DlTileMode::kRepeat);
  paint.setColorSource(radial);
  builder.DrawPaint(paint);
  builder.Restore();

  builder.Save();
  builder.Translate(100, 500);
  builder.ClipRect(clip_bounds, flutter::DlCanvas::ClipOp::kIntersect, false);
  auto sweep =
      flutter::DlColorSource::MakeSweep({100.0, 100.0}, 180.0, 270.0, 2, colors,
                                        stops, flutter::DlTileMode::kRepeat);
  paint.setColorSource(sweep);
  builder.DrawPaint(paint);
  builder.Restore();

  builder.Save();
  builder.Translate(500, 500);
  builder.ClipRect(clip_bounds, flutter::DlCanvas::ClipOp::kIntersect, false);
  auto texture = CreateTextureForFixture("table_mountain_nx.png");
  auto image = flutter::DlColorSource::MakeImage(DlImageImpeller::Make(texture),
                                                 flutter::DlTileMode::kRepeat,
                                                 flutter::DlTileMode::kRepeat);
  paint.setColorSource(image);
  builder.DrawPaint(paint);
  builder.Restore();

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(DisplayListTest, CanBlendDstOverAndDstCorrectly) {
  flutter::DisplayListBuilder builder;

  {
    builder.SaveLayer(nullptr, nullptr);
    builder.Translate(100, 100);
    flutter::DlPaint paint;
    paint.setColor(flutter::DlColor::kRed());
    builder.DrawRect(SkRect::MakeSize({200, 200}), paint);
    paint.setColor(flutter::DlColor::kBlue().withAlpha(127));
    paint.setBlendMode(flutter::DlBlendMode::kSrcOver);
    builder.DrawRect(SkRect::MakeSize({200, 200}), paint);
    builder.Restore();
  }
  {
    builder.SaveLayer(nullptr, nullptr);
    builder.Translate(300, 100);
    flutter::DlPaint paint;
    paint.setColor(flutter::DlColor::kBlue().withAlpha(127));
    builder.DrawRect(SkRect::MakeSize({200, 200}), paint);
    paint.setColor(flutter::DlColor::kRed());
    paint.setBlendMode(flutter::DlBlendMode::kDstOver);
    builder.DrawRect(SkRect::MakeSize({200, 200}), paint);
    builder.Restore();
  }
  {
    builder.SaveLayer(nullptr, nullptr);
    builder.Translate(100, 300);
    flutter::DlPaint paint;
    paint.setColor(flutter::DlColor::kRed());
    builder.DrawRect(SkRect::MakeSize({200, 200}), paint);
    paint.setColor(flutter::DlColor::kBlue().withAlpha(127));
    paint.setBlendMode(flutter::DlBlendMode::kSrc);
    builder.DrawRect(SkRect::MakeSize({200, 200}), paint);
    builder.Restore();
  }
  {
    builder.SaveLayer(nullptr, nullptr);
    builder.Translate(300, 300);
    flutter::DlPaint paint;
    paint.setColor(flutter::DlColor::kBlue().withAlpha(127));
    builder.DrawRect(SkRect::MakeSize({200, 200}), paint);
    paint.setColor(flutter::DlColor::kRed());
    paint.setBlendMode(flutter::DlBlendMode::kDst);
    builder.DrawRect(SkRect::MakeSize({200, 200}), paint);
    builder.Restore();
  }

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(DisplayListTest, CanDrawCorrectlyWithColorFilterAndImageFilter) {
  flutter::DisplayListBuilder builder;
  const float green_color_matrix[20] = {
      0, 0, 0, 0, 0,  //
      0, 0, 0, 0, 1,  //
      0, 0, 0, 0, 0,  //
      0, 0, 0, 1, 0,  //
  };
  const float blue_color_matrix[20] = {
      0, 0, 0, 0, 0,  //
      0, 0, 0, 0, 0,  //
      0, 0, 0, 0, 1,  //
      0, 0, 0, 1, 0,  //
  };
  auto green_color_filter =
      flutter::DlColorFilter::MakeMatrix(green_color_matrix);
  auto blue_color_filter =
      flutter::DlColorFilter::MakeMatrix(blue_color_matrix);
  auto blue_image_filter =
      flutter::DlImageFilter::MakeColorFilter(blue_color_filter);

  flutter::DlPaint paint;
  paint.setColor(flutter::DlColor::kRed());
  paint.setColorFilter(green_color_filter);
  paint.setImageFilter(blue_image_filter);
  builder.DrawRect(SkRect::MakeLTRB(100, 100, 500, 500), paint);
  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(DisplayListTest, MaskBlursApplyCorrectlyToColorSources) {
  auto blur_filter = std::make_shared<flutter::DlBlurMaskFilter>(
      flutter::DlBlurStyle::kNormal, 10);

  flutter::DisplayListBuilder builder;

  std::array<flutter::DlColor, 2> colors = {flutter::DlColor::kBlue(),
                                            flutter::DlColor::kGreen()};
  std::array<float, 2> stops = {0, 1};
  auto texture = CreateTextureForFixture("airplane.jpg");
  auto matrix = flutter::DlMatrix::MakeTranslation({-300, -110});
  std::array<std::shared_ptr<flutter::DlColorSource>, 2> color_sources = {
      flutter::DlColorSource::MakeImage(
          DlImageImpeller::Make(texture), flutter::DlTileMode::kRepeat,
          flutter::DlTileMode::kRepeat, flutter::DlImageSampling::kLinear,
          &matrix),
      flutter::DlColorSource::MakeLinear(
          flutter::DlPoint(0, 0), flutter::DlPoint(100, 50), 2, colors.data(),
          stops.data(), flutter::DlTileMode::kClamp),
  };

  builder.Save();
  builder.Translate(0, 100);
  for (const auto& color_source : color_sources) {
    flutter::DlPaint paint;
    paint.setColorSource(color_source);
    paint.setMaskFilter(blur_filter);

    builder.Save();
    builder.Translate(100, 0);
    paint.setDrawStyle(flutter::DlDrawStyle::kFill);
    builder.DrawRRect(SkRRect::MakeRectXY(SkRect::MakeWH(100, 50), 30, 30),
                      paint);

    paint.setDrawStyle(flutter::DlDrawStyle::kStroke);
    paint.setStrokeWidth(10);
    builder.Translate(200, 0);
    builder.DrawRRect(SkRRect::MakeRectXY(SkRect::MakeWH(100, 50), 30, 30),
                      paint);

    builder.Restore();
    builder.Translate(0, 100);
  }
  builder.Restore();

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(DisplayListTest, DrawShapes) {
  flutter::DisplayListBuilder builder;
  std::vector<flutter::DlStrokeJoin> joins = {
      flutter::DlStrokeJoin::kBevel,
      flutter::DlStrokeJoin::kRound,
      flutter::DlStrokeJoin::kMiter,
  };
  flutter::DlPaint paint =                            //
      flutter::DlPaint()                              //
          .setColor(flutter::DlColor::kWhite())       //
          .setDrawStyle(flutter::DlDrawStyle::kFill)  //
          .setStrokeWidth(10);
  flutter::DlPaint stroke_paint =                       //
      flutter::DlPaint()                                //
          .setColor(flutter::DlColor::kWhite())         //
          .setDrawStyle(flutter::DlDrawStyle::kStroke)  //
          .setStrokeWidth(10);
  SkPath path = SkPath().addPoly({{150, 50}, {160, 50}}, false);

  builder.Translate(300, 50);
  builder.Scale(0.8, 0.8);
  for (auto join : joins) {
    paint.setStrokeJoin(join);
    stroke_paint.setStrokeJoin(join);
    builder.DrawRect(SkRect::MakeXYWH(0, 0, 100, 100), paint);
    builder.DrawRect(SkRect::MakeXYWH(0, 150, 100, 100), stroke_paint);
    builder.DrawRRect(
        SkRRect::MakeRectXY(SkRect::MakeXYWH(150, 0, 100, 100), 30, 30), paint);
    builder.DrawRRect(
        SkRRect::MakeRectXY(SkRect::MakeXYWH(150, 150, 100, 100), 30, 30),
        stroke_paint);
    builder.DrawCircle(SkPoint{350, 50}, 50, paint);
    builder.DrawCircle(SkPoint{350, 200}, 50, stroke_paint);
    builder.Translate(0, 300);
  }
  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(DisplayListTest, ClipDrawRRectWithNonCircularRadii) {
  flutter::DisplayListBuilder builder;

  flutter::DlPaint fill_paint =                       //
      flutter::DlPaint()                              //
          .setColor(flutter::DlColor::kBlue())        //
          .setDrawStyle(flutter::DlDrawStyle::kFill)  //
          .setStrokeWidth(10);
  flutter::DlPaint stroke_paint =                       //
      flutter::DlPaint()                                //
          .setColor(flutter::DlColor::kGreen())         //
          .setDrawStyle(flutter::DlDrawStyle::kStroke)  //
          .setStrokeWidth(10);

  builder.DrawRRect(
      SkRRect::MakeRectXY(SkRect::MakeXYWH(500, 100, 300, 300), 120, 40),
      fill_paint);
  builder.DrawRRect(
      SkRRect::MakeRectXY(SkRect::MakeXYWH(500, 100, 300, 300), 120, 40),
      stroke_paint);

  builder.DrawRRect(
      SkRRect::MakeRectXY(SkRect::MakeXYWH(100, 500, 300, 300), 40, 120),
      fill_paint);
  builder.DrawRRect(
      SkRRect::MakeRectXY(SkRect::MakeXYWH(100, 500, 300, 300), 40, 120),
      stroke_paint);

  flutter::DlPaint reference_paint =                  //
      flutter::DlPaint()                              //
          .setColor(flutter::DlColor::kMidGrey())     //
          .setDrawStyle(flutter::DlDrawStyle::kFill)  //
          .setStrokeWidth(10);

  builder.DrawRRect(
      SkRRect::MakeRectXY(SkRect::MakeXYWH(500, 500, 300, 300), 40, 40),
      reference_paint);
  builder.DrawRRect(
      SkRRect::MakeRectXY(SkRect::MakeXYWH(100, 100, 300, 300), 120, 120),
      reference_paint);

  flutter::DlPaint clip_fill_paint =                  //
      flutter::DlPaint()                              //
          .setColor(flutter::DlColor::kCyan())        //
          .setDrawStyle(flutter::DlDrawStyle::kFill)  //
          .setStrokeWidth(10);

  builder.Save();
  builder.ClipRRect(
      SkRRect::MakeRectXY(SkRect::MakeXYWH(900, 100, 300, 300), 120, 40));
  builder.DrawPaint(clip_fill_paint);
  builder.Restore();

  builder.Save();
  builder.ClipRRect(
      SkRRect::MakeRectXY(SkRect::MakeXYWH(100, 900, 300, 300), 40, 120));
  builder.DrawPaint(clip_fill_paint);
  builder.Restore();

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(DisplayListTest, DrawVerticesBlendModes) {
  std::vector<const char*> blend_mode_names;
  std::vector<flutter::DlBlendMode> blend_mode_values;
  {
    const std::vector<std::tuple<const char*, flutter::DlBlendMode>> blends = {
        // Pipeline blends (Porter-Duff alpha compositing)
        {"Clear", flutter::DlBlendMode::kClear},
        {"Source", flutter::DlBlendMode::kSrc},
        {"Destination", flutter::DlBlendMode::kDst},
        {"SourceOver", flutter::DlBlendMode::kSrcOver},
        {"DestinationOver", flutter::DlBlendMode::kDstOver},
        {"SourceIn", flutter::DlBlendMode::kSrcIn},
        {"DestinationIn", flutter::DlBlendMode::kDstIn},
        {"SourceOut", flutter::DlBlendMode::kSrcOut},
        {"DestinationOut", flutter::DlBlendMode::kDstOut},
        {"SourceATop", flutter::DlBlendMode::kSrcATop},
        {"DestinationATop", flutter::DlBlendMode::kDstATop},
        {"Xor", flutter::DlBlendMode::kXor},
        {"Plus", flutter::DlBlendMode::kPlus},
        {"Modulate", flutter::DlBlendMode::kModulate},
        // Advanced blends (color component blends)
        {"Screen", flutter::DlBlendMode::kScreen},
        {"Overlay", flutter::DlBlendMode::kOverlay},
        {"Darken", flutter::DlBlendMode::kDarken},
        {"Lighten", flutter::DlBlendMode::kLighten},
        {"ColorDodge", flutter::DlBlendMode::kColorDodge},
        {"ColorBurn", flutter::DlBlendMode::kColorBurn},
        {"HardLight", flutter::DlBlendMode::kHardLight},
        {"SoftLight", flutter::DlBlendMode::kSoftLight},
        {"Difference", flutter::DlBlendMode::kDifference},
        {"Exclusion", flutter::DlBlendMode::kExclusion},
        {"Multiply", flutter::DlBlendMode::kMultiply},
        {"Hue", flutter::DlBlendMode::kHue},
        {"Saturation", flutter::DlBlendMode::kSaturation},
        {"Color", flutter::DlBlendMode::kColor},
        {"Luminosity", flutter::DlBlendMode::kLuminosity},
    };
    assert(blends.size() ==
           static_cast<size_t>(flutter::DlBlendMode::kLastMode) + 1);
    for (const auto& [name, mode] : blends) {
      blend_mode_names.push_back(name);
      blend_mode_values.push_back(mode);
    }
  }

  auto callback = [&]() {
    static int current_blend_index = 3;
    static float dst_alpha = 1;
    static float src_alpha = 1;
    static float color0[4] = {1.0f, 0.0f, 0.0f, 1.0f};
    static float color1[4] = {0.0f, 1.0f, 0.0f, 1.0f};
    static float color2[4] = {0.0f, 0.0f, 1.0f, 1.0f};
    static float src_color[4] = {1.0f, 1.0f, 1.0f, 1.0f};

    ImGui::Begin("Controls", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    {
      ImGui::ListBox("Blending mode", &current_blend_index,
                     blend_mode_names.data(), blend_mode_names.size());
      ImGui::SliderFloat("Source alpha", &src_alpha, 0, 1);
      ImGui::ColorEdit4("Color A", color0);
      ImGui::ColorEdit4("Color B", color1);
      ImGui::ColorEdit4("Color C", color2);
      ImGui::ColorEdit4("Source Color", src_color);
      ImGui::SliderFloat("Destination alpha", &dst_alpha, 0, 1);
    }
    ImGui::End();

    std::vector<DlPoint> positions = {DlPoint(100, 300),  //
                                      DlPoint(200, 100),  //
                                      DlPoint(300, 300)};
    std::vector<flutter::DlColor> colors = {
        toColor(color0).modulateOpacity(dst_alpha),
        toColor(color1).modulateOpacity(dst_alpha),
        toColor(color2).modulateOpacity(dst_alpha)};

    auto vertices = flutter::DlVertices::Make(
        flutter::DlVertexMode::kTriangles, 3, positions.data(),
        /*texture_coordinates=*/nullptr, colors.data());

    flutter::DisplayListBuilder builder;
    flutter::DlPaint paint;

    paint.setColor(toColor(src_color).modulateOpacity(src_alpha));
    builder.DrawVertices(vertices, blend_mode_values[current_blend_index],
                         paint);
    return builder.Build();
  };

  ASSERT_TRUE(OpenPlaygroundHere(callback));
}

TEST_P(DisplayListTest, DrawPaintIgnoresMaskFilter) {
  flutter::DisplayListBuilder builder;
  builder.DrawPaint(flutter::DlPaint().setColor(flutter::DlColor::kWhite()));

  auto filter = flutter::DlBlurMaskFilter(flutter::DlBlurStyle::kNormal, 10.0f);
  builder.DrawCircle(SkPoint{300, 300}, 200,
                     flutter::DlPaint().setMaskFilter(&filter));

  std::vector<flutter::DlColor> colors = {flutter::DlColor::kGreen(),
                                          flutter::DlColor::kGreen()};
  const float stops[2] = {0.0, 1.0};
  auto linear = flutter::DlColorSource::MakeLinear(
      {100.0, 100.0}, {300.0, 300.0}, 2, colors.data(), stops,
      flutter::DlTileMode::kRepeat);
  flutter::DlPaint blend_paint =
      flutter::DlPaint()           //
          .setColorSource(linear)  //
          .setBlendMode(flutter::DlBlendMode::kScreen);
  builder.DrawPaint(blend_paint);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(DisplayListTest, DrawMaskBlursThatMightUseSaveLayers) {
  flutter::DisplayListBuilder builder;
  builder.DrawColor(flutter::DlColor::kWhite(), flutter::DlBlendMode::kSrc);
  Vector2 scale = GetContentScale();
  builder.Scale(scale.x, scale.y);

  builder.Save();
  // We need a small transform op to avoid a deferred save
  builder.Translate(1.0f, 1.0f);
  auto solid_filter =
      flutter::DlBlurMaskFilter::Make(flutter::DlBlurStyle::kSolid, 5.0f);
  flutter::DlPaint solid_alpha_paint =
      flutter::DlPaint()                        //
          .setMaskFilter(solid_filter)          //
          .setColor(flutter::DlColor::kBlue())  //
          .setAlpha(0x7f);
  for (int x = 1; x <= 4; x++) {
    for (int y = 1; y <= 4; y++) {
      builder.DrawRect(SkRect::MakeXYWH(x * 100, y * 100, 80, 80),
                       solid_alpha_paint);
    }
  }
  builder.Restore();

  builder.Save();
  builder.Translate(500.0f, 0.0f);
  auto normal_filter =
      flutter::DlBlurMaskFilter::Make(flutter::DlBlurStyle::kNormal, 5.0f);
  auto rotate_if = flutter::DlMatrixImageFilter::Make(
      Matrix::MakeRotationZ(Degrees(10)), flutter::DlImageSampling::kLinear);
  flutter::DlPaint normal_if_paint =
      flutter::DlPaint()                         //
          .setMaskFilter(solid_filter)           //
          .setImageFilter(rotate_if)             //
          .setColor(flutter::DlColor::kGreen())  //
          .setAlpha(0x7f);
  for (int x = 1; x <= 4; x++) {
    for (int y = 1; y <= 4; y++) {
      builder.DrawRect(SkRect::MakeXYWH(x * 100, y * 100, 80, 80),
                       normal_if_paint);
    }
  }
  builder.Restore();

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

}  // namespace testing
}  // namespace impeller
