// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "display_list/display_list.h"
#include "display_list/dl_blend_mode.h"
#include "display_list/dl_tile_mode.h"
#include "display_list/effects/dl_color_filter.h"
#include "display_list/effects/dl_color_source.h"
#include "display_list/effects/dl_mask_filter.h"
#include "flutter/impeller/display_list/aiks_unittests.h"

#include "flutter/display_list/dl_builder.h"
#include "flutter/display_list/dl_color.h"
#include "flutter/display_list/dl_paint.h"
#include "flutter/testing/testing.h"
#include "impeller/playground/widgets.h"
#include "include/core/SkPath.h"
#include "include/core/SkRRect.h"
#include "include/core/SkRect.h"

using namespace flutter;
////////////////////////////////////////////////////////////////////////////////
// This is for tests of Canvas that are interested the results of rendering
// gradients.
////////////////////////////////////////////////////////////////////////////////

namespace impeller {
namespace testing {

namespace {

/// Test body for linear gradient tile mode tests (ex.
/// CanRenderLinearGradientClamp).
void CanRenderLinearGradient(AiksTest* aiks_test, DlTileMode tile_mode) {
  DisplayListBuilder builder;
  Point scale = aiks_test->GetContentScale();
  builder.Scale(scale.x, scale.y);
  DlPaint paint;
  builder.Translate(100.0f, 0);

  std::vector<DlColor> colors = {
      DlColor(Color{0.9568, 0.2627, 0.2118, 1.0}.ToARGB()),
      DlColor(Color{0.1294, 0.5882, 0.9529, 0.0}.ToARGB())};
  std::vector<Scalar> stops = {0.0, 1.0};

  auto gradient = DlColorSource::MakeLinear(
      {0, 0}, {200, 200}, 2, colors.data(), stops.data(), tile_mode);
  paint.setColorSource(gradient);
  paint.setColor(DlColor::kWhite());
  builder.DrawRect(SkRect::MakeXYWH(0, 0, 600, 600), paint);
  ASSERT_TRUE(aiks_test->OpenPlaygroundHere(builder.Build()));
}

Matrix ToMatrix(const SkMatrix& m) {
  return Matrix{
      // clang-format off
      m[0], m[3], 0, m[6],
      m[1], m[4], 0, m[7],
      0,    0,    1, 0,
      m[2], m[5], 0, m[8],
      // clang-format on
  };
}
}  // namespace

TEST_P(AiksTest, CanRenderLinearGradientClamp) {
  CanRenderLinearGradient(this, DlTileMode::kClamp);
}
TEST_P(AiksTest, CanRenderLinearGradientRepeat) {
  CanRenderLinearGradient(this, DlTileMode::kRepeat);
}
TEST_P(AiksTest, CanRenderLinearGradientMirror) {
  CanRenderLinearGradient(this, DlTileMode::kMirror);
}
TEST_P(AiksTest, CanRenderLinearGradientDecal) {
  CanRenderLinearGradient(this, DlTileMode::kDecal);
}

TEST_P(AiksTest, CanRenderLinearGradientDecalWithColorFilter) {
  DisplayListBuilder builder;
  Point scale = GetContentScale();
  builder.Scale(scale.x, scale.y);
  DlPaint paint;
  builder.Translate(100.0f, 0);

  std::vector<DlColor> colors = {
      DlColor(Color{0.9568, 0.2627, 0.2118, 1.0}.ToARGB()),
      DlColor(Color{0.1294, 0.5882, 0.9529, 0.0}.ToARGB())};
  std::vector<Scalar> stops = {0.0, 1.0};

  paint.setColorSource(DlColorSource::MakeLinear(
      {0, 0}, {200, 200}, 2, colors.data(), stops.data(), DlTileMode::kDecal));
  // Overlay the gradient with 25% green. This should appear as the entire
  // rectangle being drawn with 25% green, including the border area outside the
  // decal gradient.
  paint.setColorFilter(DlColorFilter::MakeBlend(DlColor::kGreen().withAlpha(64),
                                                DlBlendMode::kSrcOver));
  paint.setColor(DlColor::kWhite());
  builder.DrawRect(SkRect::MakeXYWH(0, 0, 600, 600), paint);
  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

static void CanRenderLinearGradientWithDithering(AiksTest* aiks_test) {
  DisplayListBuilder builder;
  DlPaint paint;
  builder.Translate(100.0, 100.0);

  // 0xffcccccc --> 0xff333333, taken from
  // https://github.com/flutter/flutter/issues/118073#issue-1521699748
  std::vector<DlColor> colors = {DlColor(0xFFCCCCCC), DlColor(0xFF333333)};
  std::vector<Scalar> stops = {0.0, 1.0};

  paint.setColorSource(DlColorSource::MakeLinear(
      {0, 0}, {800, 500}, 2, colors.data(), stops.data(), DlTileMode::kClamp));
  builder.DrawRect(SkRect::MakeXYWH(0, 0, 800, 500), paint);
  ASSERT_TRUE(aiks_test->OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanRenderLinearGradientWithDitheringEnabled) {
  CanRenderLinearGradientWithDithering(this);
}

static void CanRenderRadialGradientWithDithering(AiksTest* aiks_test) {
  DisplayListBuilder builder;
  DlPaint paint;
  builder.Translate(100.0, 100.0);

  // #FFF -> #000
  std::vector<DlColor> colors = {DlColor(Color{1.0, 1.0, 1.0, 1.0}.ToARGB()),
                                 DlColor(Color{0.0, 0.0, 0.0, 1.0}.ToARGB())};
  std::vector<Scalar> stops = {0.0, 1.0};

  paint.setColorSource(DlColorSource::MakeRadial(
      {600, 600}, 600, 2, colors.data(), stops.data(), DlTileMode::kClamp));
  builder.DrawRect(SkRect::MakeXYWH(0, 0, 1200, 1200), paint);
  ASSERT_TRUE(aiks_test->OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanRenderRadialGradientWithDitheringEnabled) {
  CanRenderRadialGradientWithDithering(this);
}

static void CanRenderSweepGradientWithDithering(AiksTest* aiks_test) {
  DisplayListBuilder builder;
  builder.Scale(aiks_test->GetContentScale().x, aiks_test->GetContentScale().y);
  DlPaint paint;
  builder.Translate(100.0, 100.0);

  // #FFF -> #000
  std::vector<DlColor> colors = {DlColor(Color{1.0, 1.0, 1.0, 1.0}.ToARGB()),
                                 DlColor(Color{0.0, 0.0, 0.0, 1.0}.ToARGB())};
  std::vector<Scalar> stops = {0.0, 1.0};

  paint.setColorSource(DlColorSource::MakeSweep(
      {100, 100}, /*start=*/45, /*end=*/135, 2, colors.data(), stops.data(),
      DlTileMode::kMirror));

  builder.DrawRect(SkRect::MakeXYWH(0, 0, 600, 600), paint);
  ASSERT_TRUE(aiks_test->OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanRenderSweepGradientWithDitheringEnabled) {
  CanRenderSweepGradientWithDithering(this);
}

static void CanRenderConicalGradientWithDithering(AiksTest* aiks_test) {
  DisplayListBuilder builder;
  builder.Scale(aiks_test->GetContentScale().x, aiks_test->GetContentScale().y);
  DlPaint paint;
  builder.Translate(100.0, 100.0);

  // #FFF -> #000
  std::vector<DlColor> colors = {DlColor(Color{1.0, 1.0, 1.0, 1.0}.ToARGB()),
                                 DlColor(Color{0.0, 0.0, 0.0, 1.0}.ToARGB())};
  std::vector<Scalar> stops = {0.0, 1.0};

  paint.setColorSource(DlColorSource::MakeConical({0, 1}, 0, {100, 100}, 100, 2,
                                                  colors.data(), stops.data(),
                                                  DlTileMode::kMirror));

  builder.DrawRect(SkRect::MakeXYWH(0, 0, 600, 600), paint);
  ASSERT_TRUE(aiks_test->OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanRenderConicalGradientWithDitheringEnabled) {
  CanRenderConicalGradientWithDithering(this);
}

namespace {
void CanRenderLinearGradientWithOverlappingStops(AiksTest* aiks_test,
                                                 DlTileMode tile_mode) {
  DisplayListBuilder builder;
  DlPaint paint;
  builder.Translate(100.0, 100.0);

  std::vector<DlColor> colors = {
      DlColor(Color{0.9568, 0.2627, 0.2118, 1.0}.ToARGB()),
      DlColor(Color{0.9568, 0.2627, 0.2118, 1.0}.ToARGB()),
      DlColor(Color{0.1294, 0.5882, 0.9529, 1.0}.ToARGB()),
      DlColor(Color{0.1294, 0.5882, 0.9529, 1.0}.ToARGB())};
  std::vector<Scalar> stops = {0.0, 0.5, 0.5, 1.0};

  paint.setColorSource(DlColorSource::MakeLinear({0, 0}, {500, 500},
                                                 stops.size(), colors.data(),
                                                 stops.data(), tile_mode));

  paint.setColor(DlColor::kWhite());
  builder.DrawRect(SkRect::MakeXYWH(0, 0, 500, 500), paint);
  ASSERT_TRUE(aiks_test->OpenPlaygroundHere(builder.Build()));
}
}  // namespace

// Only clamp is necessary. All tile modes are the same output.
TEST_P(AiksTest, CanRenderLinearGradientWithOverlappingStopsClamp) {
  CanRenderLinearGradientWithOverlappingStops(this, DlTileMode::kClamp);
}

namespace {
void CanRenderGradientWithIncompleteStops(AiksTest* aiks_test,
                                          DlColorSourceType type) {
  const DlTileMode tile_modes[4] = {
      DlTileMode::kClamp,
      DlTileMode::kRepeat,
      DlTileMode::kMirror,
      DlTileMode::kDecal,
  };
  const DlScalar test_size = 250;
  const DlScalar test_border = 25;
  const DlScalar gradient_size = 50;
  const DlScalar quadrant_size = test_size + test_border * 2;

  DisplayListBuilder builder;
  builder.DrawRect(DlRect::MakeWH(quadrant_size * 2, quadrant_size * 2),
                   DlPaint().setColor(DlColor::kDarkGrey()));

  for (int quadrant = 0; quadrant < 4; quadrant++) {
    builder.Save();
    builder.Translate((quadrant & 1) * quadrant_size + test_border,
                      (quadrant >> 1) * quadrant_size + test_border);

    if (type == DlColorSourceType::kLinearGradient) {
      // Alignment lines for the gradient edges/repeats/mirrors/etc.
      // (rendered under the gradient so as not to obscure it)
      DlPoint center = DlPoint(test_size, test_size) * 0.5;
      DlScalar ten_percent = gradient_size * 0.1;
      for (int i = gradient_size / 2; i <= test_size / 2; i += gradient_size) {
        auto draw_at = [=](DlCanvas& canvas, DlScalar offset, DlColor color) {
          DlPaint line_paint;
          line_paint.setColor(color);
          // strokewidth of 2 straddles the dividing line
          line_paint.setStrokeWidth(2.0f);
          line_paint.setDrawStyle(DlDrawStyle::kStroke);

          DlPoint along(offset, offset);
          DlScalar across_distance = test_size / 2 + 10 - offset;
          DlPoint across(across_distance, -across_distance);

          canvas.DrawLine(center - along - across,  //
                          center - along + across,  //
                          line_paint);
          canvas.DrawLine(center + along - across,  //
                          center + along + across,  //
                          line_paint);
        };
        // White line is at the edge of the gradient
        // Grey lines are where the 0.1 and 0.9 color stops land
        draw_at(builder, i - ten_percent, DlColor::kMidGrey());
        draw_at(builder, i, DlColor::kWhite());
        draw_at(builder, i + ten_percent, DlColor::kMidGrey());
      }
    }

    std::vector<DlColor> colors = {
        DlColor::kGreen(),
        DlColor::kPurple(),
        DlColor::kOrange(),
        DlColor::kBlue(),
    };
    std::vector<Scalar> stops = {0.1, 0.3, 0.7, 0.9};

    DlPaint paint;
    switch (type) {
      case DlColorSourceType::kLinearGradient:
        paint.setColorSource(DlColorSource::MakeLinear(
            {test_size / 2 - gradient_size / 2,
             test_size / 2 - gradient_size / 2},
            {test_size / 2 + gradient_size / 2,
             test_size / 2 + gradient_size / 2},
            stops.size(), colors.data(), stops.data(), tile_modes[quadrant]));
        break;
      case DlColorSourceType::kRadialGradient:
        paint.setColorSource(DlColorSource::MakeRadial(
            {test_size / 2, test_size / 2}, gradient_size,  //
            stops.size(), colors.data(), stops.data(), tile_modes[quadrant]));
        break;
      case DlColorSourceType::kConicalGradient:
        paint.setColorSource(DlColorSource::MakeConical(
            {test_size / 2, test_size / 2}, 0,
            {test_size / 2 + 20, test_size / 2 - 10}, gradient_size,
            stops.size(), colors.data(), stops.data(), tile_modes[quadrant]));
        break;
      case DlColorSourceType::kSweepGradient:
        paint.setColorSource(DlColorSource::MakeSweep(
            {test_size / 2, test_size / 2}, 0, 45,  //
            stops.size(), colors.data(), stops.data(), tile_modes[quadrant]));
        break;
      default:
        FML_UNREACHABLE();
    }

    builder.DrawRect(SkRect::MakeXYWH(0, 0, test_size, test_size), paint);
    builder.Restore();
  }

  ASSERT_TRUE(aiks_test->OpenPlaygroundHere(builder.Build()));
}
}  // namespace

TEST_P(AiksTest, CanRenderLinearGradientWithIncompleteStops) {
  CanRenderGradientWithIncompleteStops(this,
                                       DlColorSourceType::kLinearGradient);
}
TEST_P(AiksTest, CanRenderRadialGradientWithIncompleteStops) {
  CanRenderGradientWithIncompleteStops(this,
                                       DlColorSourceType::kRadialGradient);
}
TEST_P(AiksTest, CanRenderConicalGradientWithIncompleteStops) {
  CanRenderGradientWithIncompleteStops(this,
                                       DlColorSourceType::kConicalGradient);
}
TEST_P(AiksTest, CanRenderSweepGradientWithIncompleteStops) {
  CanRenderGradientWithIncompleteStops(this, DlColorSourceType::kSweepGradient);
}

namespace {
void CanRenderLinearGradientManyColors(AiksTest* aiks_test,
                                       DlTileMode tile_mode) {
  DisplayListBuilder builder;
  builder.Scale(aiks_test->GetContentScale().x, aiks_test->GetContentScale().y);
  DlPaint paint;
  builder.Translate(100, 100);

  std::vector<DlColor> colors = {
      DlColor(Color{0x1f / 255.0, 0.0, 0x5c / 255.0, 1.0}.ToARGB()),
      DlColor(Color{0x5b / 255.0, 0.0, 0x60 / 255.0, 1.0}.ToARGB()),
      DlColor(Color{0x87 / 255.0, 0x01 / 255.0, 0x60 / 255.0, 1.0}.ToARGB()),
      DlColor(Color{0xac / 255.0, 0x25 / 255.0, 0x53 / 255.0, 1.0}.ToARGB()),
      DlColor(Color{0xe1 / 255.0, 0x6b / 255.0, 0x5c / 255.0, 1.0}.ToARGB()),
      DlColor(Color{0xf3 / 255.0, 0x90 / 255.0, 0x60 / 255.0, 1.0}.ToARGB()),
      DlColor(Color{0xff / 255.0, 0xb5 / 255.0, 0x6b / 250.0, 1.0}.ToARGB())};
  std::vector<Scalar> stops = {
      0.0,
      (1.0 / 6.0) * 1,
      (1.0 / 6.0) * 2,
      (1.0 / 6.0) * 3,
      (1.0 / 6.0) * 4,
      (1.0 / 6.0) * 5,
      1.0,
  };

  paint.setColorSource(DlColorSource::MakeLinear({0, 0}, {200, 200},
                                                 stops.size(), colors.data(),
                                                 stops.data(), tile_mode));

  paint.setColor(DlColor::kWhite());
  builder.DrawRect(SkRect::MakeXYWH(0, 0, 600, 600), paint);
  builder.Restore();
  ASSERT_TRUE(aiks_test->OpenPlaygroundHere(builder.Build()));
}
}  // namespace

TEST_P(AiksTest, CanRenderLinearGradientManyColorsClamp) {
  CanRenderLinearGradientManyColors(this, DlTileMode::kClamp);
}
TEST_P(AiksTest, CanRenderLinearGradientManyColorsRepeat) {
  CanRenderLinearGradientManyColors(this, DlTileMode::kRepeat);
}
TEST_P(AiksTest, CanRenderLinearGradientManyColorsMirror) {
  CanRenderLinearGradientManyColors(this, DlTileMode::kMirror);
}
TEST_P(AiksTest, CanRenderLinearGradientManyColorsDecal) {
  CanRenderLinearGradientManyColors(this, DlTileMode::kDecal);
}

namespace {
void CanRenderLinearGradientWayManyColors(AiksTest* aiks_test,
                                          DlTileMode tile_mode) {
  DisplayListBuilder builder;
  DlPaint paint;
  builder.Translate(100.0, 100.0);
  auto color = DlColor(Color{0x1f / 255.0, 0.0, 0x5c / 255.0, 1.0}.ToARGB());
  std::vector<DlColor> colors;
  std::vector<Scalar> stops;
  auto current_stop = 0.0;
  for (int i = 0; i < 2000; i++) {
    colors.push_back(color);
    stops.push_back(current_stop);
    current_stop += 1 / 2000.0;
  }
  stops[2000 - 1] = 1.0;

  paint.setColorSource(DlColorSource::MakeLinear({0, 0}, {200, 200},
                                                 stops.size(), colors.data(),
                                                 stops.data(), tile_mode));

  builder.DrawRect(SkRect::MakeXYWH(0, 0, 600, 600), paint);
  ASSERT_TRUE(aiks_test->OpenPlaygroundHere(builder.Build()));
}
}  // namespace

// Only test clamp on purpose since they all look the same.
TEST_P(AiksTest, CanRenderLinearGradientWayManyColorsClamp) {
  CanRenderLinearGradientWayManyColors(this, DlTileMode::kClamp);
}

TEST_P(AiksTest, CanRenderLinearGradientManyColorsUnevenStops) {
  auto callback = [&]() -> sk_sp<DisplayList> {
    const char* tile_mode_names[] = {"Clamp", "Repeat", "Mirror", "Decal"};
    const DlTileMode tile_modes[] = {DlTileMode::kClamp, DlTileMode::kRepeat,
                                     DlTileMode::kMirror, DlTileMode::kDecal};

    static int selected_tile_mode = 0;
    static Matrix matrix;
    if (AiksTest::ImGuiBegin("Controls", nullptr,
                             ImGuiWindowFlags_AlwaysAutoResize)) {
      ImGui::Combo("Tile mode", &selected_tile_mode, tile_mode_names,
                   sizeof(tile_mode_names) / sizeof(char*));
      std::string label = "##1";
      for (int i = 0; i < 4; i++) {
        ImGui::InputScalarN(label.c_str(), ImGuiDataType_Float,
                            &(matrix.vec[i]), 4, NULL, NULL, "%.2f", 0);
        label[2]++;
      }
      ImGui::End();
    }

    DisplayListBuilder builder;
    DlPaint paint;
    builder.Translate(100.0, 100.0);
    auto tile_mode = tile_modes[selected_tile_mode];

    std::vector<DlColor> colors = {
        DlColor(Color{0x1f / 255.0, 0.0, 0x5c / 255.0, 1.0}.ToARGB()),
        DlColor(Color{0x5b / 255.0, 0.0, 0x60 / 255.0, 1.0}.ToARGB()),
        DlColor(Color{0x87 / 255.0, 0x01 / 255.0, 0x60 / 255.0, 1.0}.ToARGB()),
        DlColor(Color{0xac / 255.0, 0x25 / 255.0, 0x53 / 255.0, 1.0}.ToARGB()),
        DlColor(Color{0xe1 / 255.0, 0x6b / 255.0, 0x5c / 255.0, 1.0}.ToARGB()),
        DlColor(Color{0xf3 / 255.0, 0x90 / 255.0, 0x60 / 255.0, 1.0}.ToARGB()),
        DlColor(Color{0xff / 255.0, 0xb5 / 255.0, 0x6b / 250.0, 1.0}.ToARGB())};
    std::vector<Scalar> stops = {
        0.0, 2.0 / 62.0, 4.0 / 62.0, 8.0 / 62.0, 16.0 / 62.0, 32.0 / 62.0, 1.0,
    };

    paint.setColorSource(DlColorSource::MakeLinear({0, 0}, {200, 200},
                                                   stops.size(), colors.data(),
                                                   stops.data(), tile_mode));

    builder.DrawRect(SkRect::MakeXYWH(0, 0, 600, 600), paint);
    return builder.Build();
  };
  ASSERT_TRUE(OpenPlaygroundHere(callback));
}

TEST_P(AiksTest, CanRenderLinearGradientMaskBlur) {
  DisplayListBuilder builder;

  std::vector<DlColor> colors = {
      DlColor::kRed(), DlColor::kWhite(), DlColor::kRed(), DlColor::kWhite(),
      DlColor::kRed(), DlColor::kWhite(), DlColor::kRed(), DlColor::kWhite(),
      DlColor::kRed(), DlColor::kWhite(), DlColor::kRed()};
  std::vector<Scalar> stops = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5,
                               0.6, 0.7, 0.8, 0.9, 1.0};

  DlPaint paint;
  paint.setColor(DlColor::kWhite());
  paint.setColorSource(DlColorSource::MakeLinear(
      {200, 200}, {400, 400}, stops.size(), colors.data(), stops.data(),
      DlTileMode::kClamp));
  paint.setMaskFilter(DlBlurMaskFilter::Make(DlBlurStyle::kNormal, 20));

  builder.DrawCircle(SkPoint{300, 300}, 200, paint);
  builder.DrawRect(SkRect::MakeLTRB(100, 300, 500, 600), paint);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanRenderRadialGradient) {
  auto callback = [&]() -> sk_sp<DisplayList> {
    const char* tile_mode_names[] = {"Clamp", "Repeat", "Mirror", "Decal"};
    const DlTileMode tile_modes[] = {DlTileMode::kClamp, DlTileMode::kRepeat,
                                     DlTileMode::kMirror, DlTileMode::kDecal};

    static int selected_tile_mode = 0;
    static Matrix matrix;
    if (AiksTest::ImGuiBegin("Controls", nullptr,
                             ImGuiWindowFlags_AlwaysAutoResize)) {
      ImGui::Combo("Tile mode", &selected_tile_mode, tile_mode_names,
                   sizeof(tile_mode_names) / sizeof(char*));
      std::string label = "##1";
      for (int i = 0; i < 4; i++) {
        ImGui::InputScalarN(label.c_str(), ImGuiDataType_Float,
                            &(matrix.vec[i]), 4, NULL, NULL, "%.2f", 0);
        label[2]++;
      }
      ImGui::End();
    }

    DisplayListBuilder builder;
    DlPaint paint;
    builder.Translate(100.0, 100.0);
    auto tile_mode = tile_modes[selected_tile_mode];

    std::vector<DlColor> colors = {
        DlColor(Color{0.9568, 0.2627, 0.2118, 1.0}.ToARGB()),
        DlColor(Color{0.1294, 0.5882, 0.9529, 1.0}.ToARGB())};
    std::vector<Scalar> stops = {0.0, 1.0};

    paint.setColorSource(DlColorSource::MakeRadial(
        {100, 100}, 100, 2, colors.data(), stops.data(), tile_mode));

    builder.DrawRect(SkRect::MakeXYWH(0, 0, 600, 600), paint);
    return builder.Build();
  };
  ASSERT_TRUE(OpenPlaygroundHere(callback));
}

TEST_P(AiksTest, CanRenderRadialGradientManyColors) {
  auto callback = [&]() -> sk_sp<DisplayList> {
    const char* tile_mode_names[] = {"Clamp", "Repeat", "Mirror", "Decal"};
    const DlTileMode tile_modes[] = {DlTileMode::kClamp, DlTileMode::kRepeat,
                                     DlTileMode::kMirror, DlTileMode::kDecal};

    static int selected_tile_mode = 0;
    static Matrix matrix = {
        1, 0, 0, 0,  //
        0, 1, 0, 0,  //
        0, 0, 1, 0,  //
        0, 0, 0, 1   //
    };
    if (AiksTest::ImGuiBegin("Controls", nullptr,
                             ImGuiWindowFlags_AlwaysAutoResize)) {
      ImGui::Combo("Tile mode", &selected_tile_mode, tile_mode_names,
                   sizeof(tile_mode_names) / sizeof(char*));
      std::string label = "##1";
      for (int i = 0; i < 4; i++) {
        ImGui::InputScalarN(label.c_str(), ImGuiDataType_Float,
                            &(matrix.vec[i]), 4, NULL, NULL, "%.2f", 0);
        label[2]++;
      }
      ImGui::End();
    }

    DisplayListBuilder builder;
    DlPaint paint;
    builder.Translate(100.0, 100.0);
    auto tile_mode = tile_modes[selected_tile_mode];

    std::vector<DlColor> colors = {
        DlColor(Color{0x1f / 255.0, 0.0, 0x5c / 255.0, 1.0}.ToARGB()),
        DlColor(Color{0x5b / 255.0, 0.0, 0x60 / 255.0, 1.0}.ToARGB()),
        DlColor(Color{0x87 / 255.0, 0x01 / 255.0, 0x60 / 255.0, 1.0}.ToARGB()),
        DlColor(Color{0xac / 255.0, 0x25 / 255.0, 0x53 / 255.0, 1.0}.ToARGB()),
        DlColor(Color{0xe1 / 255.0, 0x6b / 255.0, 0x5c / 255.0, 1.0}.ToARGB()),
        DlColor(Color{0xf3 / 255.0, 0x90 / 255.0, 0x60 / 255.0, 1.0}.ToARGB()),
        DlColor(Color{0xff / 255.0, 0xb5 / 255.0, 0x6b / 250.0, 1.0}.ToARGB())};
    std::vector<Scalar> stops = {
        0.0,
        (1.0 / 6.0) * 1,
        (1.0 / 6.0) * 2,
        (1.0 / 6.0) * 3,
        (1.0 / 6.0) * 4,
        (1.0 / 6.0) * 5,
        1.0,
    };

    paint.setColorSource(DlColorSource::MakeRadial(
        {100, 100}, 100, stops.size(), colors.data(), stops.data(), tile_mode));

    builder.DrawRect(SkRect::MakeXYWH(0, 0, 600, 600), paint);
    return builder.Build();
  };
  ASSERT_TRUE(OpenPlaygroundHere(callback));
}

namespace {
void CanRenderSweepGradient(AiksTest* aiks_test, DlTileMode tile_mode) {
  DisplayListBuilder builder;
  builder.Scale(aiks_test->GetContentScale().x, aiks_test->GetContentScale().y);
  DlPaint paint;
  builder.Translate(100, 100);

  std::vector<DlColor> colors = {
      DlColor(Color{0.9568, 0.2627, 0.2118, 1.0}.ToARGB()),
      DlColor(Color{0.1294, 0.5882, 0.9529, 1.0}.ToARGB())};
  std::vector<Scalar> stops = {0.0, 1.0};

  paint.setColorSource(DlColorSource::MakeSweep(
      {100, 100}, /*start=*/45, /*end=*/135, /*stop_count=*/2, colors.data(),
      stops.data(), tile_mode));

  builder.DrawRect(SkRect::MakeXYWH(0, 0, 600, 600), paint);
  ASSERT_TRUE(aiks_test->OpenPlaygroundHere(builder.Build()));
}
}  // namespace

TEST_P(AiksTest, CanRenderSweepGradientClamp) {
  CanRenderSweepGradient(this, DlTileMode::kClamp);
}
TEST_P(AiksTest, CanRenderSweepGradientRepeat) {
  CanRenderSweepGradient(this, DlTileMode::kRepeat);
}
TEST_P(AiksTest, CanRenderSweepGradientMirror) {
  CanRenderSweepGradient(this, DlTileMode::kMirror);
}
TEST_P(AiksTest, CanRenderSweepGradientDecal) {
  CanRenderSweepGradient(this, DlTileMode::kDecal);
}

namespace {
void CanRenderSweepGradientManyColors(AiksTest* aiks_test,
                                      DlTileMode tile_mode) {
  DisplayListBuilder builder;
  DlPaint paint;
  builder.Translate(100.0, 100.0);

  std::vector<DlColor> colors = {
      DlColor(Color{0x1f / 255.0, 0.0, 0x5c / 255.0, 1.0}.ToARGB()),
      DlColor(Color{0x5b / 255.0, 0.0, 0x60 / 255.0, 1.0}.ToARGB()),
      DlColor(Color{0x87 / 255.0, 0x01 / 255.0, 0x60 / 255.0, 1.0}.ToARGB()),
      DlColor(Color{0xac / 255.0, 0x25 / 255.0, 0x53 / 255.0, 1.0}.ToARGB()),
      DlColor(Color{0xe1 / 255.0, 0x6b / 255.0, 0x5c / 255.0, 1.0}.ToARGB()),
      DlColor(Color{0xf3 / 255.0, 0x90 / 255.0, 0x60 / 255.0, 1.0}.ToARGB()),
      DlColor(Color{0xff / 255.0, 0xb5 / 255.0, 0x6b / 250.0, 1.0}.ToARGB())};
  std::vector<Scalar> stops = {
      0.0,
      (1.0 / 6.0) * 1,
      (1.0 / 6.0) * 2,
      (1.0 / 6.0) * 3,
      (1.0 / 6.0) * 4,
      (1.0 / 6.0) * 5,
      1.0,
  };

  paint.setColorSource(DlColorSource::MakeSweep({100, 100}, 45, 135,
                                                stops.size(), colors.data(),
                                                stops.data(), tile_mode));

  builder.DrawRect(SkRect::MakeXYWH(0, 0, 600, 600), paint);
  ASSERT_TRUE(aiks_test->OpenPlaygroundHere(builder.Build()));
}
}  // namespace

TEST_P(AiksTest, CanRenderSweepGradientManyColorsClamp) {
  CanRenderSweepGradientManyColors(this, DlTileMode::kClamp);
}
TEST_P(AiksTest, CanRenderSweepGradientManyColorsRepeat) {
  CanRenderSweepGradientManyColors(this, DlTileMode::kRepeat);
}
TEST_P(AiksTest, CanRenderSweepGradientManyColorsMirror) {
  CanRenderSweepGradientManyColors(this, DlTileMode::kMirror);
}
TEST_P(AiksTest, CanRenderSweepGradientManyColorsDecal) {
  CanRenderSweepGradientManyColors(this, DlTileMode::kDecal);
}

TEST_P(AiksTest, CanRenderConicalGradient) {
  Scalar size = 256;
  DisplayListBuilder builder;
  DlPaint paint;
  paint.setColor(DlColor::kWhite());
  builder.DrawRect(SkRect::MakeXYWH(0, 0, size * 3, size * 3), paint);
  std::vector<DlColor> colors = {
      DlColor(Color::MakeRGBA8(0xF4, 0x43, 0x36, 0xFF).ToARGB()),
      DlColor(Color::MakeRGBA8(0xFF, 0xEB, 0x3B, 0xFF).ToARGB()),
      DlColor(Color::MakeRGBA8(0x4c, 0xAF, 0x50, 0xFF).ToARGB()),
      DlColor(Color::MakeRGBA8(0x21, 0x96, 0xF3, 0xFF).ToARGB())};
  std::vector<Scalar> stops = {0.0, 1.f / 3.f, 2.f / 3.f, 1.0};
  std::array<std::tuple<DlPoint, float, DlPoint, float>, 8> array{
      std::make_tuple(DlPoint(size / 2.f, size / 2.f), 0.f,
                      DlPoint(size / 2.f, size / 2.f), size / 2.f),
      std::make_tuple(DlPoint(size / 2.f, size / 2.f), size / 4.f,
                      DlPoint(size / 2.f, size / 2.f), size / 2.f),
      std::make_tuple(DlPoint(size / 4.f, size / 4.f), 0.f,
                      DlPoint(size / 2.f, size / 2.f), size / 2.f),
      std::make_tuple(DlPoint(size / 4.f, size / 4.f), size / 2.f,
                      DlPoint(size / 2.f, size / 2.f), 0),
      std::make_tuple(DlPoint(size / 4.f, size / 4.f), size / 4.f,
                      DlPoint(size / 2.f, size / 2.f), size / 2.f),
      std::make_tuple(DlPoint(size / 4.f, size / 4.f), size / 16.f,
                      DlPoint(size / 2.f, size / 2.f), size / 8.f),
      std::make_tuple(DlPoint(size / 4.f, size / 4.f), size / 8.f,
                      DlPoint(size / 2.f, size / 2.f), size / 16.f),
      std::make_tuple(DlPoint(size / 8.f, size / 8.f), size / 8.f,
                      DlPoint(size / 2.f, size / 2.f), size / 8.f),
  };
  for (int i = 0; i < 8; i++) {
    builder.Save();
    builder.Translate((i % 3) * size, i / 3 * size);
    paint.setColorSource(DlColorSource::MakeConical(
        std::get<2>(array[i]), std::get<3>(array[i]), std::get<0>(array[i]),
        std::get<1>(array[i]), stops.size(), colors.data(), stops.data(),
        DlTileMode::kClamp));
    builder.DrawRect(SkRect::MakeXYWH(0, 0, size, size), paint);
    builder.Restore();
  }
  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanRenderGradientDecalWithBackground) {
  std::vector<DlColor> colors = {
      DlColor(Color::MakeRGBA8(0xF4, 0x43, 0x36, 0xFF).ToARGB()),
      DlColor(Color::MakeRGBA8(0xFF, 0xEB, 0x3B, 0xFF).ToARGB()),
      DlColor(Color::MakeRGBA8(0x4c, 0xAF, 0x50, 0xFF).ToARGB()),
      DlColor(Color::MakeRGBA8(0x21, 0x96, 0xF3, 0xFF).ToARGB())};
  std::vector<Scalar> stops = {0.0, 1.f / 3.f, 2.f / 3.f, 1.0};

  std::array<std::shared_ptr<DlColorSource>, 3> color_sources = {
      DlColorSource::MakeLinear({0, 0}, {100, 100}, stops.size(), colors.data(),
                                stops.data(), DlTileMode::kDecal),
      DlColorSource::MakeRadial({100, 100}, 100, stops.size(), colors.data(),
                                stops.data(), DlTileMode::kDecal),
      DlColorSource::MakeSweep({100, 100}, 45, 135, stops.size(), colors.data(),
                               stops.data(), DlTileMode::kDecal),
  };

  DisplayListBuilder builder;
  DlPaint paint;
  paint.setColor(DlColor::kWhite());
  builder.DrawRect(SkRect::MakeLTRB(0, 0, 605, 205), paint);
  for (int i = 0; i < 3; i++) {
    builder.Save();
    builder.Translate(i * 200.0f, 0);
    paint.setColorSource(color_sources[i]);
    builder.DrawRect(SkRect::MakeLTRB(0, 0, 200, 200), paint);
    builder.Restore();
  }
  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, GradientStrokesRenderCorrectly) {
  // Compare with https://fiddle.skia.org/c/027392122bec8ac2b5d5de00a4b9bbe2
  auto callback = [&]() -> sk_sp<DisplayList> {
    static float scale = 3;
    static bool add_circle_clip = true;
    const char* tile_mode_names[] = {"Clamp", "Repeat", "Mirror", "Decal"};
    const DlTileMode tile_modes[] = {DlTileMode::kClamp, DlTileMode::kRepeat,
                                     DlTileMode::kMirror, DlTileMode::kDecal};
    static int selected_tile_mode = 0;
    static float alpha = 1;

    if (AiksTest::ImGuiBegin("Controls", nullptr,
                             ImGuiWindowFlags_AlwaysAutoResize)) {
      ImGui::SliderFloat("Scale", &scale, 0, 6);
      ImGui::Checkbox("Circle clip", &add_circle_clip);
      ImGui::SliderFloat("Alpha", &alpha, 0, 1);
      ImGui::Combo("Tile mode", &selected_tile_mode, tile_mode_names,
                   sizeof(tile_mode_names) / sizeof(char*));
      ImGui::End();
    }

    DisplayListBuilder builder;
    builder.Scale(GetContentScale().x, GetContentScale().y);
    DlPaint paint;
    paint.setColor(DlColor::kWhite());
    builder.DrawPaint(paint);

    paint.setDrawStyle(DlDrawStyle::kStroke);
    paint.setColor(DlColor::kWhite().withAlpha(alpha * 255));
    paint.setStrokeWidth(10);
    auto tile_mode = tile_modes[selected_tile_mode];

    std::vector<DlColor> colors = {
        DlColor(Color{0.9568, 0.2627, 0.2118, 1.0}.ToARGB()),
        DlColor(Color{0.1294, 0.5882, 0.9529, 1.0}.ToARGB())};
    std::vector<Scalar> stops = {0.0, 1.0};

    paint.setColorSource(DlColorSource::MakeLinear({0, 0}, {50, 50},
                                                   stops.size(), colors.data(),
                                                   stops.data(), tile_mode));

    SkPath path;
    path.moveTo(20, 20);
    path.quadTo({60, 20}, {60, 60});
    path.close();
    path.moveTo(60, 20);
    path.quadTo({60, 60}, {20, 60});

    builder.Scale(scale, scale);

    if (add_circle_clip) {
      static PlaygroundPoint circle_clip_point_a(Point(60, 300), 20,
                                                 Color::Red());
      static PlaygroundPoint circle_clip_point_b(Point(600, 300), 20,
                                                 Color::Red());
      auto [handle_a, handle_b] =
          DrawPlaygroundLine(circle_clip_point_a, circle_clip_point_b);

      SkMatrix screen_to_canvas;
      if (!builder.GetTransform().invert(&screen_to_canvas)) {
        return nullptr;
      }
      Matrix ip_matrix = ToMatrix(screen_to_canvas);
      Point point_a = ip_matrix * handle_a * GetContentScale();
      Point point_b = ip_matrix * handle_b * GetContentScale();

      Point middle = (point_a + point_b) / 2;
      auto radius = point_a.GetDistance(middle);
      SkPath circle;
      circle.addCircle(middle.x, middle.y, radius);
      builder.ClipPath(circle);
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

// Draws two gradients that should look identical (except that one is an RRECT).
TEST_P(AiksTest, FastGradientTestHorizontal) {
  DisplayListBuilder builder;
  DlPaint paint;
  builder.Translate(100.0f, 0);

  std::vector<DlColor> colors = {DlColor::kRed(), DlColor::kBlue(),
                                 DlColor::kGreen()};
  std::vector<Scalar> stops = {0.0, 0.1, 1.0};

  paint.setColorSource(DlColorSource::MakeLinear({0, 0}, {300, 0}, stops.size(),
                                                 colors.data(), stops.data(),
                                                 DlTileMode::kClamp));

  paint.setColor(DlColor::kWhite());
  builder.DrawRect(SkRect::MakeXYWH(0, 0, 300, 300), paint);
  builder.Translate(400, 0);
  builder.DrawRRect(SkRRect::MakeRectXY(SkRect::MakeXYWH(0, 0, 300, 300), 4, 4),
                    paint);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

// Draws two gradients that should look identical (except that one is an RRECT).
TEST_P(AiksTest, FastGradientTestVertical) {
  DisplayListBuilder builder;
  DlPaint paint;
  builder.Translate(100.0f, 0);

  std::vector<DlColor> colors = {DlColor::kRed(), DlColor::kBlue(),
                                 DlColor::kGreen()};
  std::vector<Scalar> stops = {0.0, 0.1, 1.0};

  paint.setColorSource(DlColorSource::MakeLinear({0, 0}, {0, 300}, stops.size(),
                                                 colors.data(), stops.data(),
                                                 DlTileMode::kClamp));

  paint.setColor(DlColor::kWhite());
  builder.DrawRect(SkRect::MakeXYWH(0, 0, 300, 300), paint);
  builder.Translate(400, 0);
  builder.DrawRRect(SkRRect::MakeRectXY(SkRect::MakeXYWH(0, 0, 300, 300), 4, 4),
                    paint);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

// Draws two gradients that should look identical (except that one is an RRECT).
TEST_P(AiksTest, FastGradientTestHorizontalReversed) {
  DisplayListBuilder builder;
  DlPaint paint;
  builder.Translate(100.0f, 0);

  std::vector<DlColor> colors = {DlColor::kRed(), DlColor::kBlue(),
                                 DlColor::kGreen()};
  std::vector<Scalar> stops = {0.0, 0.1, 1.0};

  paint.setColorSource(DlColorSource::MakeLinear({300, 0}, {0, 0}, stops.size(),
                                                 colors.data(), stops.data(),
                                                 DlTileMode::kClamp));

  paint.setColor(DlColor::kWhite());
  builder.DrawRect(SkRect::MakeXYWH(0, 0, 300, 300), paint);
  builder.Translate(400, 0);
  builder.DrawRRect(SkRRect::MakeRectXY(SkRect::MakeXYWH(0, 0, 300, 300), 4, 4),
                    paint);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

// Draws two gradients that should look identical (except that one is an RRECT).
TEST_P(AiksTest, FastGradientTestVerticalReversed) {
  DisplayListBuilder builder;
  DlPaint paint;
  builder.Translate(100.0f, 0);

  std::vector<DlColor> colors = {DlColor::kRed(), DlColor::kBlue(),
                                 DlColor::kGreen()};
  std::vector<Scalar> stops = {0.0, 0.1, 1.0};

  paint.setColorSource(DlColorSource::MakeLinear({0, 300}, {0, 0}, stops.size(),
                                                 colors.data(), stops.data(),
                                                 DlTileMode::kClamp));

  paint.setColor(DlColor::kWhite());
  builder.DrawRect(SkRect::MakeXYWH(0, 0, 300, 300), paint);
  builder.Translate(400, 0);
  builder.DrawRRect(SkRRect::MakeRectXY(SkRect::MakeXYWH(0, 0, 300, 300), 4, 4),
                    paint);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, VerifyNonOptimizedGradient) {
  DisplayListBuilder builder;
  DlPaint paint;
  builder.Translate(100.0f, 0);

  std::vector<DlColor> colors = {DlColor::kRed(), DlColor::kBlue(),
                                 DlColor::kGreen()};
  std::vector<Scalar> stops = {0.0, 0.1, 1.0};

  // Inset the start and end point to verify that we do not apply
  // the fast gradient condition.
  paint.setColorSource(
      DlColorSource::MakeLinear({0, 150}, {0, 100}, stops.size(), colors.data(),
                                stops.data(), DlTileMode::kRepeat));

  paint.setColor(DlColor::kWhite());
  builder.DrawRect(SkRect::MakeXYWH(0, 0, 300, 300), paint);
  builder.Translate(400, 0);
  builder.DrawRRect(SkRRect::MakeRectXY(SkRect::MakeXYWH(0, 0, 300, 300), 4, 4),
                    paint);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

}  // namespace testing
}  // namespace impeller
