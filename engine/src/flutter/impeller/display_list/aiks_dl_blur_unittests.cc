// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/display_list/display_list.h"
#include "flutter/display_list/dl_blend_mode.h"
#include "flutter/display_list/dl_builder.h"
#include "flutter/display_list/dl_color.h"
#include "flutter/display_list/dl_paint.h"
#include "flutter/display_list/dl_sampling_options.h"
#include "flutter/display_list/dl_tile_mode.h"
#include "flutter/display_list/effects/dl_color_filter.h"
#include "flutter/display_list/effects/dl_color_source.h"
#include "flutter/display_list/effects/dl_image_filter.h"
#include "flutter/display_list/effects/dl_mask_filter.h"
#include "flutter/impeller/display_list/aiks_unittests.h"

#include "gmock/gmock.h"
#include "impeller/display_list/dl_dispatcher.h"
#include "impeller/display_list/dl_image_impeller.h"
#include "impeller/playground/widgets.h"
#include "impeller/renderer/testing/mocks.h"
#include "include/core/SkRRect.h"
#include "include/core/SkRect.h"
#include "third_party/imgui/imgui.h"

////////////////////////////////////////////////////////////////////////////////
// This is for tests of Canvas that are interested the results of rendering
// blurs.
////////////////////////////////////////////////////////////////////////////////

namespace impeller {
namespace testing {

using namespace flutter;

// The shapes of these ovals should appear equal. They are demonstrating the
// difference between the fast pass and not.
TEST_P(AiksTest, SolidColorOvalsMaskBlurTinySigma) {
  DisplayListBuilder builder;
  builder.Scale(GetContentScale().x, GetContentScale().y);

  std::vector<float> sigmas = {0.0, 0.01, 1.0};
  std::vector<DlColor> colors = {DlColor::kGreen(), DlColor::kYellow(),
                                 DlColor::kRed()};
  for (uint32_t i = 0; i < sigmas.size(); ++i) {
    DlPaint paint;
    paint.setColor(colors[i]);
    paint.setMaskFilter(
        DlBlurMaskFilter::Make(DlBlurStyle::kNormal, sigmas[i]));

    builder.Save();
    builder.Translate(100 + (i * 100), 100);
    SkRRect rrect =
        SkRRect::MakeRectXY(SkRect::MakeXYWH(0, 0, 60.0f, 160.0f), 50, 100);
    builder.DrawRRect(rrect, paint);
    builder.Restore();
  }

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

sk_sp<flutter::DisplayList> DoGradientOvalStrokeMaskBlur(Vector2 content_Scale,
                                                         Scalar sigma,
                                                         DlBlurStyle style) {
  DisplayListBuilder builder;
  builder.Scale(content_Scale.x, content_Scale.y);

  DlPaint background_paint;
  background_paint.setColor(DlColor(1, 0.1, 0.1, 0.1, DlColorSpace::kSRGB));
  builder.DrawPaint(background_paint);

  std::vector<DlColor> colors = {DlColor::kRed(), DlColor::kBlue()};
  std::vector<Scalar> stops = {0.0, 1.0};

  DlPaint paint;
  paint.setMaskFilter(DlBlurMaskFilter::Make(style, sigma));
  auto gradient = DlColorSource::MakeLinear(
      {0, 0}, {200, 200}, 2, colors.data(), stops.data(), DlTileMode::kClamp);
  paint.setColorSource(gradient);
  paint.setColor(DlColor::kWhite());
  paint.setDrawStyle(DlDrawStyle::kStroke);
  paint.setStrokeWidth(20);

  builder.Save();
  builder.Translate(100, 100);

  {
    DlPaint line_paint;
    line_paint.setColor(DlColor::kWhite());
    builder.DrawLine(SkPoint{100, 0}, SkPoint{100, 60}, line_paint);
    builder.DrawLine(SkPoint{0, 30}, SkPoint{200, 30}, line_paint);
  }

  SkRRect rrect =
      SkRRect::MakeRectXY(SkRect::MakeXYWH(0, 0, 200.0f, 60.0f), 50, 100);
  builder.DrawRRect(rrect, paint);
  builder.Restore();

  return builder.Build();
}

// https://github.com/flutter/flutter/issues/155930
TEST_P(AiksTest, GradientOvalStrokeMaskBlur) {
  ASSERT_TRUE(OpenPlaygroundHere(DoGradientOvalStrokeMaskBlur(
      GetContentScale(), /*sigma=*/10, DlBlurStyle::kNormal)));
}

TEST_P(AiksTest, GradientOvalStrokeMaskBlurSigmaZero) {
  ASSERT_TRUE(OpenPlaygroundHere(DoGradientOvalStrokeMaskBlur(
      GetContentScale(), /*sigma=*/0, DlBlurStyle::kNormal)));
}

TEST_P(AiksTest, GradientOvalStrokeMaskBlurOuter) {
  ASSERT_TRUE(OpenPlaygroundHere(DoGradientOvalStrokeMaskBlur(
      GetContentScale(), /*sigma=*/10, DlBlurStyle::kOuter)));
}

TEST_P(AiksTest, GradientOvalStrokeMaskBlurInner) {
  ASSERT_TRUE(OpenPlaygroundHere(DoGradientOvalStrokeMaskBlur(
      GetContentScale(), /*sigma=*/10, DlBlurStyle::kInner)));
}

TEST_P(AiksTest, GradientOvalStrokeMaskBlurSolid) {
  ASSERT_TRUE(OpenPlaygroundHere(DoGradientOvalStrokeMaskBlur(
      GetContentScale(), /*sigma=*/10, DlBlurStyle::kSolid)));
}

TEST_P(AiksTest, SolidColorCircleMaskBlurTinySigma) {
  DisplayListBuilder builder;
  builder.Scale(GetContentScale().x, GetContentScale().y);

  std::vector<float> sigmas = {0.0, 0.01, 1.0};
  std::vector<DlColor> colors = {DlColor::kGreen(), DlColor::kYellow(),
                                 DlColor::kRed()};
  for (uint32_t i = 0; i < sigmas.size(); ++i) {
    DlPaint paint;
    paint.setColor(colors[i]);
    paint.setMaskFilter(
        DlBlurMaskFilter::Make(DlBlurStyle::kNormal, sigmas[i]));

    builder.Save();
    builder.Translate(100 + (i * 100), 100);
    SkRRect rrect =
        SkRRect::MakeRectXY(SkRect::MakeXYWH(0, 0, 100.0f, 100.0f), 100, 100);
    builder.DrawRRect(rrect, paint);
    builder.Restore();
  }

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanRenderMaskBlurHugeSigma) {
  DisplayListBuilder builder;

  DlPaint paint;
  paint.setColor(DlColor::kGreen());
  paint.setMaskFilter(DlBlurMaskFilter::Make(DlBlurStyle::kNormal, 99999));
  builder.DrawCircle(SkPoint{400, 400}, 300, paint);
  builder.Restore();

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanRenderForegroundBlendWithMaskBlur) {
  // This case triggers the ForegroundPorterDuffBlend path. The color filter
  // should apply to the color only, and respect the alpha mask.
  DisplayListBuilder builder;
  builder.ClipRect(SkRect::MakeXYWH(100, 150, 400, 400));

  DlPaint paint;
  paint.setColor(DlColor::kWhite());

  Sigma sigma = Radius(20);
  paint.setMaskFilter(
      DlBlurMaskFilter::Make(DlBlurStyle::kNormal, sigma.sigma));
  paint.setColorFilter(
      DlColorFilter::MakeBlend(DlColor::kGreen(), DlBlendMode::kSrc));
  builder.DrawCircle(SkPoint{400, 400}, 200, paint);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanRenderForegroundAdvancedBlendWithMaskBlur) {
  // This case triggers the ForegroundAdvancedBlend path. The color filter
  // should apply to the color only, and respect the alpha mask.
  DisplayListBuilder builder;
  builder.ClipRect(SkRect::MakeXYWH(100, 150, 400, 400));

  DlPaint paint;
  paint.setColor(
      DlColor::RGBA(128.0f / 255.0f, 128.0f / 255.0f, 128.0f / 255.0f, 1.0f));

  Sigma sigma = Radius(20);
  paint.setMaskFilter(
      DlBlurMaskFilter::Make(DlBlurStyle::kNormal, sigma.sigma));
  paint.setColorFilter(
      DlColorFilter::MakeBlend(DlColor::kGreen(), DlBlendMode::kColor));
  builder.DrawCircle(SkPoint{400, 400}, 200, paint);
  builder.Restore();

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanRenderBackdropBlurInteractive) {
  auto callback = [&]() -> sk_sp<DisplayList> {
    static PlaygroundPoint point_a(Point(50, 50), 30, Color::White());
    static PlaygroundPoint point_b(Point(300, 200), 30, Color::White());
    auto [a, b] = DrawPlaygroundLine(point_a, point_b);

    DisplayListBuilder builder;
    DlPaint paint;
    paint.setColor(DlColor::kCornflowerBlue());
    builder.DrawCircle(SkPoint{100, 100}, 50, paint);

    paint.setColor(DlColor::kGreenYellow());
    builder.DrawCircle(SkPoint{300, 200}, 100, paint);

    paint.setColor(DlColor::kDarkMagenta());
    builder.DrawCircle(SkPoint{140, 170}, 75, paint);

    paint.setColor(DlColor::kOrangeRed());
    builder.DrawCircle(SkPoint{180, 120}, 100, paint);

    SkRRect rrect =
        SkRRect::MakeRectXY(SkRect::MakeLTRB(a.x, a.y, b.x, b.y), 20, 20);
    builder.ClipRRect(rrect);

    DlPaint save_paint;
    save_paint.setBlendMode(DlBlendMode::kSrc);

    auto backdrop_filter = DlImageFilter::MakeBlur(20, 20, DlTileMode::kClamp);
    builder.SaveLayer(nullptr, &save_paint, backdrop_filter.get());
    builder.Restore();

    return builder.Build();
  };

  ASSERT_TRUE(OpenPlaygroundHere(callback));
}

TEST_P(AiksTest, CanRenderBackdropBlur) {
  DisplayListBuilder builder;

  DlPaint paint;
  paint.setColor(DlColor::kCornflowerBlue());
  builder.DrawCircle(SkPoint{100, 100}, 50, paint);

  paint.setColor(DlColor::kGreenYellow());
  builder.DrawCircle(SkPoint{300, 200}, 100, paint);

  paint.setColor(DlColor::kDarkMagenta());
  builder.DrawCircle(SkPoint{140, 170}, 75, paint);

  paint.setColor(DlColor::kOrangeRed());
  builder.DrawCircle(SkPoint{180, 120}, 100, paint);

  SkRRect rrect =
      SkRRect::MakeRectXY(SkRect::MakeLTRB(75, 50, 375, 275), 20, 20);
  builder.ClipRRect(rrect);

  DlPaint save_paint;
  save_paint.setBlendMode(DlBlendMode::kSrc);
  auto backdrop_filter = DlImageFilter::MakeBlur(30, 30, DlTileMode::kClamp);
  builder.SaveLayer(nullptr, &save_paint, backdrop_filter.get());
  builder.Restore();

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanRenderBackdropBlurWithSingleBackdropId) {
  auto image = DlImageImpeller::Make(CreateTextureForFixture("kalimba.jpg"));

  DisplayListBuilder builder;

  DlPaint paint;
  builder.DrawImage(image, SkPoint::Make(50.0, 50.0),
                    DlImageSampling::kNearestNeighbor, &paint);

  SkRRect rrect =
      SkRRect::MakeRectXY(SkRect::MakeXYWH(50, 250, 100, 100), 20, 20);
  builder.Save();
  builder.ClipRRect(rrect);

  DlPaint save_paint;
  save_paint.setBlendMode(DlBlendMode::kSrc);
  auto backdrop_filter = DlImageFilter::MakeBlur(30, 30, DlTileMode::kClamp);
  builder.SaveLayer(nullptr, &save_paint, backdrop_filter.get(),
                    /*backdrop_id=*/1);
  builder.Restore();
  builder.Restore();

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanRenderMultipleBackdropBlurWithSingleBackdropId) {
  auto image = DlImageImpeller::Make(CreateTextureForFixture("kalimba.jpg"));

  DisplayListBuilder builder;

  DlPaint paint;
  builder.DrawImage(image, SkPoint::Make(50.0, 50.0),
                    DlImageSampling::kNearestNeighbor, &paint);

  for (int i = 0; i < 6; i++) {
    SkRRect rrect = SkRRect::MakeRectXY(
        SkRect::MakeXYWH(50 + (i * 100), 250, 100, 100), 20, 20);
    builder.Save();
    builder.ClipRRect(rrect);

    DlPaint save_paint;
    save_paint.setBlendMode(DlBlendMode::kSrc);
    auto backdrop_filter = DlImageFilter::MakeBlur(30, 30, DlTileMode::kClamp);
    builder.SaveLayer(nullptr, &save_paint, backdrop_filter.get(),
                      /*backdrop_id=*/1);
    builder.Restore();
    builder.Restore();
  }

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest,
       CanRenderMultipleBackdropBlurWithSingleBackdropIdAndDistinctFilters) {
  auto image = DlImageImpeller::Make(CreateTextureForFixture("kalimba.jpg"));

  DisplayListBuilder builder;

  DlPaint paint;
  builder.DrawImage(image, SkPoint::Make(50.0, 50.0),
                    DlImageSampling::kNearestNeighbor, &paint);

  for (int i = 0; i < 6; i++) {
    SkRRect rrect = SkRRect::MakeRectXY(
        SkRect::MakeXYWH(50 + (i * 100), 250, 100, 100), 20, 20);
    builder.Save();
    builder.ClipRRect(rrect);

    DlPaint save_paint;
    save_paint.setBlendMode(DlBlendMode::kSrc);
    auto backdrop_filter =
        DlImageFilter::MakeBlur(30 + i, 30, DlTileMode::kClamp);
    builder.SaveLayer(nullptr, &save_paint, backdrop_filter.get(),
                      /*backdrop_id=*/1);
    builder.Restore();
    builder.Restore();
  }

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanRenderBackdropBlurHugeSigma) {
  DisplayListBuilder builder;

  DlPaint paint;
  paint.setColor(DlColor::kGreen());
  builder.DrawCircle(SkPoint{400, 400}, 300, paint);

  DlPaint save_paint;
  save_paint.setBlendMode(DlBlendMode::kSrc);

  auto backdrop_filter =
      DlImageFilter::MakeBlur(999999, 999999, DlTileMode::kClamp);
  builder.SaveLayer(nullptr, &save_paint, backdrop_filter.get());
  builder.Restore();

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanRenderClippedBlur) {
  DisplayListBuilder builder;
  builder.ClipRect(SkRect::MakeXYWH(100, 150, 400, 400));

  DlPaint paint;
  paint.setColor(DlColor::kGreen());
  paint.setImageFilter(DlImageFilter::MakeBlur(20, 20, DlTileMode::kDecal));
  builder.DrawCircle(SkPoint{400, 400}, 200, paint);
  builder.Restore();

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, ClippedBlurFilterRendersCorrectlyInteractive) {
  auto callback = [&]() -> sk_sp<DisplayList> {
    static PlaygroundPoint playground_point(Point(400, 400), 20,
                                            Color::Green());
    auto point = DrawPlaygroundPoint(playground_point);

    DisplayListBuilder builder;
    auto location = point - Point(400, 400);
    builder.Translate(location.x, location.y);

    DlPaint paint;
    Sigma sigma = Radius{120 * 3};
    paint.setMaskFilter(
        DlBlurMaskFilter::Make(DlBlurStyle::kNormal, sigma.sigma));
    paint.setColor(DlColor::kRed());

    SkPath path = SkPath::Rect(SkRect::MakeLTRB(0, 0, 800, 800));
    path.addCircle(0, 0, 0.5);
    builder.DrawPath(path, paint);
    return builder.Build();
  };
  ASSERT_TRUE(OpenPlaygroundHere(callback));
}

TEST_P(AiksTest, ClippedBlurFilterRendersCorrectly) {
  DisplayListBuilder builder;
  builder.Translate(0, -400);
  DlPaint paint;

  Sigma sigma = Radius{120 * 3};
  paint.setMaskFilter(
      DlBlurMaskFilter::Make(DlBlurStyle::kNormal, sigma.sigma));
  paint.setColor(DlColor::kRed());

  SkPath path = SkPath::Rect(SkRect::MakeLTRB(0, 0, 800, 800));
  path.addCircle(0, 0, 0.5);
  builder.DrawPath(path, paint);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, ClearBlendWithBlur) {
  DisplayListBuilder builder;
  DlPaint paint;
  paint.setColor(DlColor::kBlue());
  builder.DrawRect(SkRect::MakeXYWH(0, 0, 600.0, 600.0), paint);

  DlPaint clear;
  clear.setBlendMode(DlBlendMode::kClear);
  clear.setMaskFilter(DlBlurMaskFilter::Make(DlBlurStyle::kNormal, 20));

  builder.DrawCircle(SkPoint{300.0, 300.0}, 200.0, clear);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, BlurHasNoEdge) {
  Scalar sigma = 47.6;
  auto callback = [&]() -> sk_sp<DisplayList> {
    if (AiksTest::ImGuiBegin("Controls", nullptr,
                             ImGuiWindowFlags_AlwaysAutoResize)) {
      ImGui::SliderFloat("Sigma", &sigma, 0, 50);
      ImGui::End();
    }
    DisplayListBuilder builder;
    builder.Scale(GetContentScale().x, GetContentScale().y);
    builder.DrawPaint({});

    DlPaint paint;
    paint.setColor(DlColor::kGreen());
    paint.setMaskFilter(DlBlurMaskFilter::Make(DlBlurStyle::kNormal, sigma));

    builder.DrawRect(SkRect::MakeXYWH(300, 300, 200, 200), paint);
    return builder.Build();
  };

  ASSERT_TRUE(OpenPlaygroundHere(callback));
}

TEST_P(AiksTest, MaskBlurWithZeroSigmaIsSkipped) {
  DisplayListBuilder builder;

  DlPaint paint;
  paint.setColor(DlColor::kBlue());
  paint.setMaskFilter(DlBlurMaskFilter::Make(DlBlurStyle::kNormal, 0));

  builder.DrawCircle(SkPoint{300, 300}, 200, paint);
  builder.DrawRect(SkRect::MakeLTRB(100, 300, 500, 600), paint);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

struct MaskBlurTestConfig {
  DlBlurStyle style = DlBlurStyle::kNormal;
  Scalar sigma = 1.0f;
  Scalar alpha = 1.0f;
  std::shared_ptr<DlImageFilter> image_filter;
  bool invert_colors = false;
  DlBlendMode blend_mode = DlBlendMode::kSrcOver;
};

static sk_sp<DisplayList> MaskBlurVariantTest(
    const AiksTest& test_context,
    const MaskBlurTestConfig& config) {
  DisplayListBuilder builder;
  builder.Scale(test_context.GetContentScale().x,
                test_context.GetContentScale().y);
  builder.Scale(0.8f, 0.8f);
  builder.Translate(50.f, 50.f);

  DlPaint draw_paint;
  draw_paint.setColor(
      DlColor::RGBA(Color::AntiqueWhite().red, Color::AntiqueWhite().green,
                    Color::AntiqueWhite().blue, Color::AntiqueWhite().alpha));
  builder.DrawPaint(draw_paint);

  DlPaint paint;
  paint.setMaskFilter(DlBlurMaskFilter::Make(config.style, config.sigma));
  paint.setInvertColors(config.invert_colors);
  paint.setImageFilter(config.image_filter);
  paint.setBlendMode(config.blend_mode);

  const Scalar x = 50;
  const Scalar radius = 20.0f;
  const Scalar y_spacing = 100.0f;
  Scalar alpha = config.alpha * 255;

  Scalar y = 50;
  paint.setColor(DlColor::kCrimson().withAlpha(alpha));
  builder.DrawRect(SkRect::MakeXYWH(x + 25 - radius / 2, y + radius / 2,  //
                                    radius, 60.0f - radius),
                   paint);

  y += y_spacing;
  paint.setColor(DlColor::kBlue().withAlpha(alpha));
  builder.DrawCircle(SkPoint{x + 25, y + 25}, radius, paint);

  y += y_spacing;
  paint.setColor(DlColor::kGreen().withAlpha(alpha));
  builder.DrawOval(SkRect::MakeXYWH(x + 25 - radius / 2, y + radius / 2,  //
                                    radius, 60.0f - radius),
                   paint);

  y += y_spacing;
  paint.setColor(DlColor::kPurple().withAlpha(alpha));
  SkRRect rrect =
      SkRRect::MakeRectXY(SkRect::MakeXYWH(x, y, 60.0f, 60.0f), radius, radius);
  builder.DrawRRect(rrect, paint);

  y += y_spacing;
  paint.setColor(DlColor::kOrange().withAlpha(alpha));

  rrect =
      SkRRect::MakeRectXY(SkRect::MakeXYWH(x, y, 60.0f, 60.0f), radius, 5.0);
  builder.DrawRRect(rrect, paint);

  y += y_spacing;
  paint.setColor(DlColor::kMaroon().withAlpha(alpha));

  {
    SkPath path;
    path.moveTo(x + 0, y + 60);
    path.lineTo(x + 30, y + 0);
    path.lineTo(x + 60, y + 60);
    path.close();

    builder.DrawPath(path, paint);
  }

  y += y_spacing;
  paint.setColor(DlColor::kMaroon().withAlpha(alpha));
  {
    SkPath path;
    path.addArc(SkRect::MakeXYWH(x + 5, y, 50, 50), 90, 180);
    path.addArc(SkRect::MakeXYWH(x + 25, y, 50, 50), 90, 180);
    path.close();
    builder.DrawPath(path, paint);
  }

  return builder.Build();
}

static const std::map<std::string, MaskBlurTestConfig> kPaintVariations = {
    // 1. Normal style, translucent, zero sigma.
    {"NormalTranslucentZeroSigma",
     {.style = DlBlurStyle::kNormal, .sigma = 0.0f, .alpha = 0.5f}},
    // 2. Normal style, translucent.
    {"NormalTranslucent",
     {.style = DlBlurStyle::kNormal, .sigma = 8.0f, .alpha = 0.5f}},
    // 3. Solid style, translucent.
    {"SolidTranslucent",
     {.style = DlBlurStyle::kSolid, .sigma = 8.0f, .alpha = 0.5f}},
    // 4. Solid style, opaque.
    {"SolidOpaque", {.style = DlBlurStyle::kSolid, .sigma = 8.0f}},
    // 5. Solid style, translucent, color & image filtered.
    {"SolidTranslucentWithFilters",
     {.style = DlBlurStyle::kSolid,
      .sigma = 8.0f,
      .alpha = 0.5f,
      .image_filter = DlImageFilter::MakeBlur(3, 3, DlTileMode::kClamp),
      .invert_colors = true}},
    // 6. Solid style, translucent, exclusion blended.
    {"SolidTranslucentExclusionBlend",
     {.style = DlBlurStyle::kSolid,
      .sigma = 8.0f,
      .alpha = 0.5f,
      .blend_mode = DlBlendMode::kExclusion}},
    // 7. Inner style, translucent.
    {"InnerTranslucent",
     {.style = DlBlurStyle::kInner, .sigma = 8.0f, .alpha = 0.5f}},
    // 8. Inner style, translucent, blurred.
    {"InnerTranslucentWithBlurImageFilter",
     {.style = DlBlurStyle::kInner,
      .sigma = 8.0f,
      .alpha = 0.5f,
      .image_filter = DlImageFilter::MakeBlur(3, 3, DlTileMode::kClamp)}},
    // 9. Outer style, translucent.
    {"OuterTranslucent",
     {.style = DlBlurStyle::kOuter, .sigma = 8.0f, .alpha = 0.5f}},
    // 10. Outer style, opaque, image filtered.
    {"OuterOpaqueWithBlurImageFilter",
     {.style = DlBlurStyle::kOuter,
      .sigma = 8.0f,
      .image_filter = DlImageFilter::MakeBlur(3, 3, DlTileMode::kClamp)}},
};

#define MASK_BLUR_VARIANT_TEST(config)                              \
  TEST_P(AiksTest, MaskBlurVariantTest##config) {                   \
    ASSERT_TRUE(OpenPlaygroundHere(                                 \
        MaskBlurVariantTest(*this, kPaintVariations.at(#config)))); \
  }

MASK_BLUR_VARIANT_TEST(NormalTranslucentZeroSigma)
MASK_BLUR_VARIANT_TEST(NormalTranslucent)
MASK_BLUR_VARIANT_TEST(SolidTranslucent)
MASK_BLUR_VARIANT_TEST(SolidOpaque)
MASK_BLUR_VARIANT_TEST(SolidTranslucentWithFilters)
MASK_BLUR_VARIANT_TEST(SolidTranslucentExclusionBlend)
MASK_BLUR_VARIANT_TEST(InnerTranslucent)
MASK_BLUR_VARIANT_TEST(InnerTranslucentWithBlurImageFilter)
MASK_BLUR_VARIANT_TEST(OuterTranslucent)
MASK_BLUR_VARIANT_TEST(OuterOpaqueWithBlurImageFilter)

#undef MASK_BLUR_VARIANT_TEST

TEST_P(AiksTest, GaussianBlurStyleInner) {
  DisplayListBuilder builder;
  builder.Scale(GetContentScale().x, GetContentScale().y);

  DlPaint paint;
  paint.setColor(DlColor::RGBA(0.1, 0.1, 0.1, 1));
  builder.DrawPaint(paint);

  paint.setColor(DlColor::kGreen());
  paint.setMaskFilter(DlBlurMaskFilter::Make(DlBlurStyle::kInner, 30));

  SkPath path;
  path.moveTo(200, 200);
  path.lineTo(300, 400);
  path.lineTo(100, 400);
  path.close();

  builder.DrawPath(path, paint);

  // Draw another thing to make sure the clip area is reset.
  DlPaint red;
  red.setColor(DlColor::kRed());
  builder.DrawRect(SkRect::MakeXYWH(0, 0, 200, 200), red);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, GaussianBlurStyleOuter) {
  DisplayListBuilder builder;
  builder.Scale(GetContentScale().x, GetContentScale().y);

  DlPaint paint;
  paint.setColor(DlColor::RGBA(0.1, 0.1, 0.1, 1.0));
  builder.DrawPaint(paint);

  paint.setColor(DlColor::kGreen());
  paint.setMaskFilter(DlBlurMaskFilter::Make(DlBlurStyle::kOuter, 30));

  SkPath path;
  path.moveTo(200, 200);
  path.lineTo(300, 400);
  path.lineTo(100, 400);
  path.close();

  builder.DrawPath(path, paint);

  // Draw another thing to make sure the clip area is reset.
  DlPaint red;
  red.setColor(DlColor::kRed());
  builder.DrawRect(SkRect::MakeXYWH(0, 0, 200, 200), red);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, GaussianBlurStyleSolid) {
  DisplayListBuilder builder;
  builder.Scale(GetContentScale().x, GetContentScale().y);

  DlPaint paint;
  paint.setColor(DlColor::RGBA(0.1, 0.1, 0.1, 1.0));
  builder.DrawPaint(paint);

  paint.setColor(DlColor::kGreen());
  paint.setMaskFilter(DlBlurMaskFilter::Make(DlBlurStyle::kSolid, 30));

  SkPath path;
  path.moveTo(200, 200);
  path.lineTo(300, 400);
  path.lineTo(100, 400);
  path.close();

  builder.DrawPath(path, paint);

  // Draw another thing to make sure the clip area is reset.
  DlPaint red;
  red.setColor(DlColor::kRed());
  builder.DrawRect(SkRect::MakeXYWH(0, 0, 200, 200), red);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, MaskBlurTexture) {
  Scalar sigma = 30;
  auto callback = [&]() -> sk_sp<DisplayList> {
    if (AiksTest::ImGuiBegin("Controls", nullptr,
                             ImGuiWindowFlags_AlwaysAutoResize)) {
      ImGui::SliderFloat("Sigma", &sigma, 0, 500);
      ImGui::End();
    }

    DisplayListBuilder builder;
    builder.Scale(GetContentScale().x, GetContentScale().y);

    DlPaint paint;
    paint.setColor(DlColor::kGreen());
    paint.setMaskFilter(DlBlurMaskFilter::Make(DlBlurStyle::kNormal, sigma));

    builder.DrawImage(
        DlImageImpeller::Make(CreateTextureForFixture("boston.jpg")),
        SkPoint{200, 200}, DlImageSampling::kNearestNeighbor, &paint);

    DlPaint red;
    red.setColor(DlColor::kRed());
    builder.DrawRect(SkRect::MakeXYWH(0, 0, 200, 200), red);

    return builder.Build();
  };
  ASSERT_TRUE(OpenPlaygroundHere(callback));
}

TEST_P(AiksTest, MaskBlurDoesntStretchContents) {
  Scalar sigma = 70;
  auto callback = [&]() -> sk_sp<DisplayList> {
    if (AiksTest::ImGuiBegin("Controls", nullptr,
                             ImGuiWindowFlags_AlwaysAutoResize)) {
      ImGui::SliderFloat("Sigma", &sigma, 0, 500);
      ImGui::End();
    }

    DisplayListBuilder builder;
    builder.Scale(GetContentScale().x, GetContentScale().y);

    DlPaint paint;
    paint.setColor(DlColor::RGBA(0.1, 0.1, 0.1, 1.0));
    builder.DrawPaint(paint);

    std::shared_ptr<Texture> boston = CreateTextureForFixture("boston.jpg");

    builder.Transform(SkMatrix::Translate(100, 100) *
                      SkMatrix::Scale(0.5, 0.5));

    paint.setColorSource(DlColorSource::MakeImage(
        DlImageImpeller::Make(boston), DlTileMode::kRepeat, DlTileMode::kRepeat,
        DlImageSampling::kMipmapLinear));
    paint.setMaskFilter(DlBlurMaskFilter::Make(DlBlurStyle::kNormal, sigma));

    builder.DrawRect(SkRect::MakeXYWH(0, 0, boston->GetSize().width,
                                      boston->GetSize().height),
                     paint);

    return builder.Build();
  };
  ASSERT_TRUE(OpenPlaygroundHere(callback));
}

TEST_P(AiksTest, GaussianBlurAtPeripheryVertical) {
  DisplayListBuilder builder;

  DlPaint paint;
  builder.Scale(GetContentScale().x, GetContentScale().y);

  paint.setColor(DlColor::kLimeGreen());
  SkRRect rrect = SkRRect::MakeRectXY(
      SkRect::MakeLTRB(0, 0, GetWindowSize().width, 100), 10, 10);
  builder.DrawRRect(rrect, paint);

  paint.setColor(DlColor::kMagenta());
  rrect = SkRRect::MakeRectXY(
      SkRect::MakeLTRB(0, 110, GetWindowSize().width, 210), 10, 10);
  builder.DrawRRect(rrect, paint);
  builder.ClipRect(SkRect::MakeLTRB(100, 0, 200, GetWindowSize().height));

  DlPaint save_paint;
  save_paint.setBlendMode(DlBlendMode::kSrc);

  auto backdrop_filter = DlImageFilter::MakeBlur(20, 20, DlTileMode::kClamp);

  builder.SaveLayer(nullptr, &save_paint, backdrop_filter.get());
  builder.Restore();

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, GaussianBlurAtPeripheryHorizontal) {
  DisplayListBuilder builder;

  builder.Scale(GetContentScale().x, GetContentScale().y);
  std::shared_ptr<Texture> boston = CreateTextureForFixture("boston.jpg");
  builder.DrawImageRect(
      DlImageImpeller::Make(boston),
      SkRect::MakeXYWH(0, 0, boston->GetSize().width, boston->GetSize().height),
      SkRect::MakeLTRB(0, 0, GetWindowSize().width, 100),
      DlImageSampling::kNearestNeighbor);

  DlPaint paint;
  paint.setColor(DlColor::kMagenta());

  SkRRect rrect = SkRRect::MakeRectXY(
      SkRect::MakeLTRB(0, 110, GetWindowSize().width, 210), 10, 10);
  builder.DrawRRect(rrect, paint);
  builder.ClipRect(SkRect::MakeLTRB(0, 50, GetWindowSize().width, 150));

  DlPaint save_paint;
  save_paint.setBlendMode(DlBlendMode::kSrc);

  auto backdrop_filter = DlImageFilter::MakeBlur(20, 20, DlTileMode::kClamp);
  builder.SaveLayer(nullptr, &save_paint, backdrop_filter.get());

  builder.Restore();
  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, GaussianBlurAnimatedBackdrop) {
  // This test is for checking out how stable rendering is when content is
  // translated underneath a blur.  Animating under a blur can cause
  // *shimmering* to happen as a result of pixel alignment.
  // See also: https://github.com/flutter/flutter/issues/140193
  auto boston =
      CreateTextureForFixture("boston.jpg", /*enable_mipmapping=*/true);
  ASSERT_TRUE(boston);
  int64_t count = 0;
  Scalar sigma = 20.0;
  Scalar freq = 0.1;
  Scalar amp = 50.0;
  auto callback = [&]() -> sk_sp<DisplayList> {
    if (AiksTest::ImGuiBegin("Controls", nullptr,
                             ImGuiWindowFlags_AlwaysAutoResize)) {
      ImGui::SliderFloat("Sigma", &sigma, 0, 200);
      ImGui::SliderFloat("Frequency", &freq, 0.01, 2.0);
      ImGui::SliderFloat("Amplitude", &amp, 1, 100);
      ImGui::End();
    }

    DisplayListBuilder builder;
    builder.Scale(GetContentScale().x, GetContentScale().y);
    Scalar y = amp * sin(freq * 2.0 * M_PI * count / 60);
    builder.DrawImage(
        DlImageImpeller::Make(boston),
        SkPoint::Make(1024 / 2 - boston->GetSize().width / 2,
                      (768 / 2 - boston->GetSize().height / 2) + y),
        DlImageSampling::kMipmapLinear);
    static PlaygroundPoint point_a(Point(100, 100), 20, Color::Red());
    static PlaygroundPoint point_b(Point(900, 700), 20, Color::Red());
    auto [handle_a, handle_b] = DrawPlaygroundLine(point_a, point_b);

    builder.ClipRect(
        SkRect::MakeLTRB(handle_a.x, handle_a.y, handle_b.x, handle_b.y));
    builder.ClipRect(SkRect::MakeLTRB(100, 100, 900, 700));

    DlPaint paint;
    paint.setBlendMode(DlBlendMode::kSrc);

    auto backdrop_filter =
        DlImageFilter::MakeBlur(sigma, sigma, DlTileMode::kClamp);
    builder.SaveLayer(nullptr, &paint, backdrop_filter.get());
    count += 1;
    return builder.Build();
  };
  ASSERT_TRUE(OpenPlaygroundHere(callback));
}

TEST_P(AiksTest, GaussianBlurStyleInnerGradient) {
  DisplayListBuilder builder;

  builder.Scale(GetContentScale().x, GetContentScale().y);

  DlPaint paint;
  paint.setColor(DlColor::RGBA(0.1, 0.1, 0.1, 1.0));
  builder.DrawPaint(paint);

  std::vector<DlColor> colors = {DlColor::RGBA(0.9568, 0.2627, 0.2118, 1.0),
                                 DlColor::RGBA(0.7568, 0.2627, 0.2118, 1.0)};
  std::vector<Scalar> stops = {0.0, 1.0};

  paint = DlPaint{};
  paint.setColorSource(DlColorSource::MakeLinear(
      /*start_point=*/{0, 0},
      /*end_point=*/{200, 200},
      /*stop_count=*/colors.size(),
      /*colors=*/colors.data(),
      /*stops=*/stops.data(),
      /*tile_mode=*/DlTileMode::kMirror));
  paint.setMaskFilter(DlBlurMaskFilter::Make(DlBlurStyle::kInner, 30));

  SkPath path;
  path.moveTo(200, 200);
  path.lineTo(300, 400);
  path.lineTo(100, 400);
  path.close();
  builder.DrawPath(path, paint);

  // Draw another thing to make sure the clip area is reset.
  DlPaint red;
  red.setColor(DlColor::kRed());
  builder.DrawRect(SkRect::MakeXYWH(0, 0, 200, 200), red);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, GaussianBlurStyleSolidGradient) {
  DisplayListBuilder builder;
  builder.Scale(GetContentScale().x, GetContentScale().y);

  DlPaint paint;
  paint.setColor(DlColor::RGBA(0.1, 0.1, 0.1, 1.0));
  builder.DrawPaint(paint);

  std::vector<DlColor> colors = {DlColor::RGBA(0.9568, 0.2627, 0.2118, 1.0),
                                 DlColor::RGBA(0.7568, 0.2627, 0.2118, 1.0)};
  std::vector<Scalar> stops = {0.0, 1.0};

  paint = DlPaint{};
  paint.setColorSource(DlColorSource::MakeLinear(
      /*start_point=*/{0, 0},
      /*end_point=*/{200, 200},
      /*stop_count=*/colors.size(),
      /*colors=*/colors.data(),
      /*stops=*/stops.data(),
      /*tile_mode=*/DlTileMode::kMirror));
  paint.setMaskFilter(DlBlurMaskFilter::Make(DlBlurStyle::kSolid, 30));

  SkPath path;
  path.moveTo(200, 200);
  path.lineTo(300, 400);
  path.lineTo(100, 400);
  path.close();
  builder.DrawPath(path, paint);

  // Draw another thing to make sure the clip area is reset.
  DlPaint red;
  red.setColor(DlColor::kRed());
  builder.DrawRect(SkRect::MakeXYWH(0, 0, 200, 200), red);
  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, GaussianBlurStyleOuterGradient) {
  DisplayListBuilder builder;
  builder.Scale(GetContentScale().x, GetContentScale().y);

  DlPaint paint;
  paint.setColor(DlColor::RGBA(0.1, 0.1, 0.1, 1.0));
  builder.DrawPaint(paint);

  std::vector<DlColor> colors = {DlColor::RGBA(0.9568, 0.2627, 0.2118, 1.0),
                                 DlColor::RGBA(0.7568, 0.2627, 0.2118, 1.0)};
  std::vector<Scalar> stops = {0.0, 1.0};

  paint = DlPaint{};
  paint.setColorSource(DlColorSource::MakeLinear(
      /*start_point=*/{0, 0},
      /*end_point=*/{200, 200},
      /*stop_count=*/colors.size(),
      /*colors=*/colors.data(),
      /*stops=*/stops.data(),
      /*tile_mode=*/DlTileMode::kMirror));
  paint.setMaskFilter(DlBlurMaskFilter::Make(DlBlurStyle::kOuter, 30));

  SkPath path;
  path.moveTo(200, 200);
  path.lineTo(300, 400);
  path.lineTo(100, 400);
  path.close();
  builder.DrawPath(path, paint);

  // Draw another thing to make sure the clip area is reset.
  DlPaint red;
  red.setColor(DlColor::kRed());
  builder.DrawRect(SkRect::MakeXYWH(0, 0, 200, 200), red);
  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, GaussianBlurScaledAndClipped) {
  DisplayListBuilder builder;
  std::shared_ptr<Texture> boston = CreateTextureForFixture("boston.jpg");
  Rect bounds =
      Rect::MakeXYWH(0, 0, boston->GetSize().width, boston->GetSize().height);
  Vector2 image_center = Vector2(bounds.GetSize() / 2);

  DlPaint paint;
  paint.setImageFilter(DlImageFilter::MakeBlur(20, 20, DlTileMode::kDecal));

  Vector2 clip_size = {150, 75};
  Vector2 center = Vector2(1024, 768) / 2;
  builder.Scale(GetContentScale().x, GetContentScale().y);

  auto rect =
      Rect::MakeLTRB(center.x, center.y, center.x, center.y).Expand(clip_size);
  builder.ClipRect(SkRect::MakeLTRB(rect.GetLeft(), rect.GetTop(),
                                    rect.GetRight(), rect.GetBottom()));
  builder.Translate(center.x, center.y);
  builder.Scale(0.6, 0.6);

  SkRect sk_bounds = SkRect::MakeLTRB(bounds.GetLeft(), bounds.GetTop(),
                                      bounds.GetRight(), bounds.GetBottom());
  Rect dest = bounds.Shift(-image_center);
  SkRect sk_dst = SkRect::MakeLTRB(dest.GetLeft(), dest.GetTop(),
                                   dest.GetRight(), dest.GetBottom());
  builder.DrawImageRect(DlImageImpeller::Make(boston), /*src=*/sk_bounds,
                        /*dst=*/sk_dst, DlImageSampling::kNearestNeighbor,
                        &paint);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, GaussianBlurRotatedAndClippedInteractive) {
  std::shared_ptr<Texture> boston = CreateTextureForFixture("boston.jpg");

  auto callback = [&]() -> sk_sp<DisplayList> {
    const char* tile_mode_names[] = {"Clamp", "Repeat", "Mirror", "Decal"};
    const DlTileMode tile_modes[] = {DlTileMode::kClamp, DlTileMode::kRepeat,
                                     DlTileMode::kMirror, DlTileMode::kDecal};

    static float rotation = 0;
    static float scale = 0.6;
    static int selected_tile_mode = 3;

    if (AiksTest::ImGuiBegin("Controls", nullptr,
                             ImGuiWindowFlags_AlwaysAutoResize)) {
      ImGui::SliderFloat("Rotation (degrees)", &rotation, -180, 180);
      ImGui::SliderFloat("Scale", &scale, 0, 2.0);
      ImGui::Combo("Tile mode", &selected_tile_mode, tile_mode_names,
                   sizeof(tile_mode_names) / sizeof(char*));
      ImGui::End();
    }

    DisplayListBuilder builder;
    Rect bounds =
        Rect::MakeXYWH(0, 0, boston->GetSize().width, boston->GetSize().height);
    Vector2 image_center = Vector2(bounds.GetSize() / 2);
    DlPaint paint;
    paint.setImageFilter(
        DlImageFilter::MakeBlur(20, 20, tile_modes[selected_tile_mode]));

    static PlaygroundPoint point_a(Point(362, 309), 20, Color::Red());
    static PlaygroundPoint point_b(Point(662, 459), 20, Color::Red());
    auto [handle_a, handle_b] = DrawPlaygroundLine(point_a, point_b);
    Vector2 center = Vector2(1024, 768) / 2;

    builder.Scale(GetContentScale().x, GetContentScale().y);
    builder.ClipRect(
        SkRect::MakeLTRB(handle_a.x, handle_a.y, handle_b.x, handle_b.y));
    builder.Translate(center.x, center.y);
    builder.Scale(scale, scale);
    builder.Rotate(rotation);

    SkRect sk_bounds = SkRect::MakeLTRB(bounds.GetLeft(), bounds.GetTop(),
                                        bounds.GetRight(), bounds.GetBottom());
    Rect dest = bounds.Shift(-image_center);
    SkRect sk_dst = SkRect::MakeLTRB(dest.GetLeft(), dest.GetTop(),
                                     dest.GetRight(), dest.GetBottom());
    builder.DrawImageRect(DlImageImpeller::Make(boston), /*src=*/sk_bounds,
                          /*dst=*/sk_dst, DlImageSampling::kNearestNeighbor,
                          &paint);
    return builder.Build();
  };

  ASSERT_TRUE(OpenPlaygroundHere(callback));
}

TEST_P(AiksTest, GaussianBlurOneDimension) {
  DisplayListBuilder builder;

  builder.Scale(GetContentScale().x, GetContentScale().y);
  builder.Scale(0.5, 0.5);

  std::shared_ptr<Texture> boston = CreateTextureForFixture("boston.jpg");
  builder.DrawImage(DlImageImpeller::Make(boston), SkPoint{100, 100}, {});

  DlPaint paint;
  paint.setBlendMode(DlBlendMode::kSrc);

  auto backdrop_filter = DlImageFilter::MakeBlur(50, 0, DlTileMode::kClamp);
  builder.SaveLayer(nullptr, &paint, backdrop_filter.get());
  builder.Restore();
  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

// Smoketest to catch issues with the coverage hint.
// Draws a rotated blurred image within a rectangle clip. The center of the clip
// rectangle is the center of the rotated image. The entire area of the clip
// rectangle should be filled with opaque colors output by the blur.
TEST_P(AiksTest, GaussianBlurRotatedAndClipped) {
  DisplayListBuilder builder;

  std::shared_ptr<Texture> boston = CreateTextureForFixture("boston.jpg");
  Rect bounds =
      Rect::MakeXYWH(0, 0, boston->GetSize().width, boston->GetSize().height);

  DlPaint paint;
  paint.setImageFilter(DlImageFilter::MakeBlur(20, 20, DlTileMode::kDecal));

  Vector2 image_center = Vector2(bounds.GetSize() / 2);
  Vector2 clip_size = {150, 75};
  Vector2 center = Vector2(1024, 768) / 2;
  builder.Scale(GetContentScale().x, GetContentScale().y);

  auto clip_bounds =
      Rect::MakeLTRB(center.x, center.y, center.x, center.y).Expand(clip_size);
  builder.ClipRect(SkRect::MakeLTRB(clip_bounds.GetLeft(), clip_bounds.GetTop(),
                                    clip_bounds.GetRight(),
                                    clip_bounds.GetBottom()));
  builder.Translate(center.x, center.y);
  builder.Scale(0.6, 0.6);
  builder.Rotate(25);

  auto dst_rect = bounds.Shift(-image_center);
  builder.DrawImageRect(
      DlImageImpeller::Make(boston), /*src=*/
      SkRect::MakeLTRB(bounds.GetLeft(), bounds.GetTop(), bounds.GetRight(),
                       bounds.GetBottom()),
      /*dst=*/
      SkRect::MakeLTRB(dst_rect.GetLeft(), dst_rect.GetTop(),
                       dst_rect.GetRight(), dst_rect.GetBottom()),
      DlImageSampling::kMipmapLinear, &paint);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, GaussianBlurRotatedNonUniform) {
  auto callback = [&]() -> sk_sp<DisplayList> {
    const char* tile_mode_names[] = {"Clamp", "Repeat", "Mirror", "Decal"};
    const DlTileMode tile_modes[] = {DlTileMode::kClamp, DlTileMode::kRepeat,
                                     DlTileMode::kMirror, DlTileMode::kDecal};

    static float rotation = 45;
    static float scale = 0.6;
    static int selected_tile_mode = 3;

    if (AiksTest::ImGuiBegin("Controls", nullptr,
                             ImGuiWindowFlags_AlwaysAutoResize)) {
      ImGui::SliderFloat("Rotation (degrees)", &rotation, -180, 180);
      ImGui::SliderFloat("Scale", &scale, 0, 2.0);
      ImGui::Combo("Tile mode", &selected_tile_mode, tile_mode_names,
                   sizeof(tile_mode_names) / sizeof(char*));
      ImGui::End();
    }

    DisplayListBuilder builder;

    DlPaint paint;
    paint.setColor(DlColor::kGreen());
    paint.setImageFilter(
        DlImageFilter::MakeBlur(50, 0, tile_modes[selected_tile_mode]));

    Vector2 center = Vector2(1024, 768) / 2;
    builder.Scale(GetContentScale().x, GetContentScale().y);
    builder.Translate(center.x, center.y);
    builder.Scale(scale, scale);
    builder.Rotate(rotation);

    SkRRect rrect =
        SkRRect::MakeRectXY(SkRect::MakeXYWH(-100, -100, 200, 200), 10, 10);
    builder.DrawRRect(rrect, paint);
    return builder.Build();
  };

  ASSERT_TRUE(OpenPlaygroundHere(callback));
}

TEST_P(AiksTest, BlurredRectangleWithShader) {
  DisplayListBuilder builder;
  builder.Scale(GetContentScale().x, GetContentScale().y);

  auto paint_lines = [&builder](Scalar dx, Scalar dy, DlPaint paint) {
    auto draw_line = [&builder, &paint](SkPoint a, SkPoint b) {
      SkPath line = SkPath::Line(a, b);
      builder.DrawPath(line, paint);
    };
    paint.setStrokeWidth(5);
    paint.setDrawStyle(DlDrawStyle::kStroke);
    draw_line(SkPoint::Make(dx + 100, dy + 100),
              SkPoint::Make(dx + 200, dy + 200));
    draw_line(SkPoint::Make(dx + 100, dy + 200),
              SkPoint::Make(dx + 200, dy + 100));
    draw_line(SkPoint::Make(dx + 150, dy + 100),
              SkPoint::Make(dx + 200, dy + 150));
    draw_line(SkPoint::Make(dx + 100, dy + 150),
              SkPoint::Make(dx + 150, dy + 200));
  };

  AiksContext renderer(GetContext(), nullptr);
  DisplayListBuilder recorder_builder;
  for (int x = 0; x < 5; ++x) {
    for (int y = 0; y < 5; ++y) {
      SkRect rect = SkRect::MakeXYWH(x * 20, y * 20, 20, 20);
      DlPaint paint;
      paint.setColor(((x + y) & 1) == 0 ? DlColor::kYellow()
                                        : DlColor::kBlue());

      recorder_builder.DrawRect(rect, paint);
    }
  }
  auto texture =
      DisplayListToTexture(recorder_builder.Build(), {100, 100}, renderer);

  auto image_source = DlColorSource::MakeImage(
      DlImageImpeller::Make(texture), DlTileMode::kRepeat, DlTileMode::kRepeat);
  auto blur_filter = DlImageFilter::MakeBlur(5, 5, DlTileMode::kDecal);

  DlPaint paint;
  paint.setColor(DlColor::kDarkGreen());
  builder.DrawRect(SkRect::MakeLTRB(0, 0, 300, 600), paint);

  paint.setColorSource(image_source);
  builder.DrawRect(SkRect::MakeLTRB(100, 100, 200, 200), paint);

  paint.setColorSource(nullptr);
  paint.setColor(DlColor::kRed());
  builder.DrawRect(SkRect::MakeLTRB(300, 0, 600, 600), paint);

  paint.setColorSource(image_source);
  paint.setImageFilter(blur_filter);
  builder.DrawRect(SkRect::MakeLTRB(400, 100, 500, 200), paint);

  paint.setImageFilter(nullptr);
  paint_lines(0, 300, paint);

  paint.setImageFilter(blur_filter);
  paint_lines(300, 300, paint);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, GaussianBlurWithoutDecalSupport) {
  if (GetParam() != PlaygroundBackend::kMetal) {
    GTEST_SKIP()
        << "This backend doesn't yet support setting device capabilities.";
  }
  if (!WillRenderSomething()) {
    // Sometimes these tests are run without playgrounds enabled which is
    // pointless for this test since we are asserting that
    // `SupportsDecalSamplerAddressMode` is called.
    GTEST_SKIP() << "This test requires playgrounds.";
  }

  std::shared_ptr<const Capabilities> old_capabilities =
      GetContext()->GetCapabilities();
  auto mock_capabilities = std::make_shared<MockCapabilities>();
  EXPECT_CALL(*mock_capabilities, SupportsDecalSamplerAddressMode())
      .Times(::testing::AtLeast(1))
      .WillRepeatedly(::testing::Return(false));
  FLT_FORWARD(mock_capabilities, old_capabilities, GetDefaultColorFormat);
  FLT_FORWARD(mock_capabilities, old_capabilities, GetDefaultStencilFormat);
  FLT_FORWARD(mock_capabilities, old_capabilities,
              GetDefaultDepthStencilFormat);
  FLT_FORWARD(mock_capabilities, old_capabilities, SupportsOffscreenMSAA);
  FLT_FORWARD(mock_capabilities, old_capabilities,
              SupportsImplicitResolvingMSAA);
  FLT_FORWARD(mock_capabilities, old_capabilities, SupportsReadFromResolve);
  FLT_FORWARD(mock_capabilities, old_capabilities, SupportsFramebufferFetch);
  FLT_FORWARD(mock_capabilities, old_capabilities, SupportsSSBO);
  FLT_FORWARD(mock_capabilities, old_capabilities, SupportsCompute);
  FLT_FORWARD(mock_capabilities, old_capabilities,
              SupportsTextureToTextureBlits);
  FLT_FORWARD(mock_capabilities, old_capabilities, GetDefaultGlyphAtlasFormat);
  FLT_FORWARD(mock_capabilities, old_capabilities, SupportsTriangleFan);
  FLT_FORWARD(mock_capabilities, old_capabilities, SupportsPrimitiveRestart);
  ASSERT_TRUE(SetCapabilities(mock_capabilities).ok());

  auto texture = DlImageImpeller::Make(CreateTextureForFixture("boston.jpg"));

  DisplayListBuilder builder;
  builder.Scale(GetContentScale().x * 0.5, GetContentScale().y * 0.5);

  DlPaint paint;
  paint.setColor(DlColor::kBlack());
  builder.DrawPaint(paint);

  auto blur_filter = DlImageFilter::MakeBlur(20, 20, DlTileMode::kDecal);
  paint.setImageFilter(blur_filter);
  builder.DrawImage(texture, SkPoint::Make(200, 200), {}, &paint);
  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

// This addresses a bug where tiny blurs could result in mip maps that beyond
// the limits for the textures used for blurring.
// See also: b/323402168
TEST_P(AiksTest, GaussianBlurSolidColorTinyMipMap) {
  AiksContext renderer(GetContext(), nullptr);

  for (int32_t i = 1; i < 5; ++i) {
    DisplayListBuilder builder;
    Scalar fi = i;
    SkPath path;
    path.moveTo(100, 100);
    path.lineTo(100 + fi, 100 + fi);

    DlPaint paint;
    paint.setColor(DlColor::kChartreuse());
    auto blur_filter = DlImageFilter::MakeBlur(0.1, 0.1, DlTileMode::kClamp);
    paint.setImageFilter(blur_filter);

    builder.DrawPath(path, paint);

    auto image = DisplayListToTexture(builder.Build(), {1024, 768}, renderer);
    EXPECT_TRUE(image) << " length " << i;
  }
}

// This addresses a bug where tiny blurs could result in mip maps that beyond
// the limits for the textures used for blurring.
// See also: b/323402168
TEST_P(AiksTest, GaussianBlurBackdropTinyMipMap) {
  AiksContext renderer(GetContext(), nullptr);
  for (int32_t i = 1; i < 5; ++i) {
    DisplayListBuilder builder;

    ISize clip_size = ISize(i, i);
    builder.Save();
    builder.ClipRect(
        SkRect::MakeXYWH(400, 400, clip_size.width, clip_size.height));

    DlPaint paint;
    paint.setColor(DlColor::kGreen());
    auto blur_filter = DlImageFilter::MakeBlur(0.1, 0.1, DlTileMode::kDecal);
    paint.setImageFilter(blur_filter);

    builder.DrawCircle(SkPoint{400, 400}, 200, paint);
    builder.Restore();

    auto image = DisplayListToTexture(builder.Build(), {1024, 768}, renderer);
    EXPECT_TRUE(image) << " length " << i;
  }
}

TEST_P(AiksTest,
       CanRenderMultipleBackdropBlurWithSingleBackdropIdDifferentLayers) {
  auto image = DlImageImpeller::Make(CreateTextureForFixture("kalimba.jpg"));

  DisplayListBuilder builder;

  DlPaint paint;
  builder.DrawImage(image, SkPoint::Make(50.0, 50.0),
                    DlImageSampling::kNearestNeighbor, &paint);

  for (int i = 0; i < 6; i++) {
    if (i != 0) {
      DlPaint paint;
      paint.setColor(DlColor::kWhite().withAlphaF(0.95));
      builder.SaveLayer(nullptr, &paint);
    }
    SkRRect rrect = SkRRect::MakeRectXY(
        SkRect::MakeXYWH(50 + (i * 100), 250, 100, 100), 20, 20);
    builder.Save();
    builder.ClipRRect(rrect);

    DlPaint save_paint;
    save_paint.setBlendMode(DlBlendMode::kSrc);
    auto backdrop_filter = DlImageFilter::MakeBlur(30, 30, DlTileMode::kClamp);
    builder.SaveLayer(nullptr, &save_paint, backdrop_filter.get(),
                      /*backdrop_id=*/1);
    builder.Restore();
    builder.Restore();
    if (i != 0) {
      builder.Restore();
    }
  }

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

}  // namespace testing
}  // namespace impeller
