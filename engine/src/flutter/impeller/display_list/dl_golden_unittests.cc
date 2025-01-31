// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/display_list/dl_golden_unittests.h"

#include "display_list/dl_color.h"
#include "display_list/dl_paint.h"
#include "display_list/geometry/dl_geometry_types.h"
#include "flutter/display_list/dl_builder.h"
#include "flutter/impeller/display_list/testing/render_text_in_canvas.h"
#include "flutter/impeller/display_list/testing/rmse.h"
#include "flutter/impeller/geometry/path_builder.h"
#include "flutter/testing/testing.h"
#include "gtest/gtest.h"

namespace flutter {
namespace testing {

using impeller::Degrees;
using impeller::PlaygroundBackend;
using impeller::PlaygroundTest;
using impeller::Point;
using impeller::Radians;
using impeller::Scalar;

INSTANTIATE_PLAYGROUND_SUITE(DlGoldenTest);

TEST_P(DlGoldenTest, CanDrawPaint) {
  auto draw = [](DlCanvas* canvas,
                 const std::vector<std::unique_ptr<DlImage>>& images) {
    canvas->Scale(0.2, 0.2);
    DlPaint paint;
    paint.setColor(DlColor::kCyan());
    canvas->DrawPaint(paint);
  };

  DisplayListBuilder builder;
  draw(&builder, /*images=*/{});

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(DlGoldenTest, CanRenderImage) {
  auto draw = [](DlCanvas* canvas, const std::vector<sk_sp<DlImage>>& images) {
    FML_CHECK(images.size() >= 1);
    DlPaint paint;
    paint.setColor(DlColor::kRed());
    canvas->DrawImage(images[0], DlPoint(100.0, 100.0),
                      DlImageSampling::kLinear, &paint);
  };

  DisplayListBuilder builder;
  std::vector<sk_sp<DlImage>> images;
  images.emplace_back(CreateDlImageForFixture("kalimba.jpg"));
  draw(&builder, images);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

// Asserts that subpass rendering of MatrixImageFilters works.
// https://github.com/flutter/flutter/issues/147807
TEST_P(DlGoldenTest, Bug147807) {
  Point content_scale = GetContentScale();
  auto draw = [content_scale](DlCanvas* canvas,
                              const std::vector<sk_sp<DlImage>>& images) {
    canvas->Scale(content_scale.x, content_scale.y);
    DlPaint paint;
    paint.setColor(DlColor(0xfffef7ff));
    canvas->DrawRect(DlRect::MakeLTRB(0, 0, 375, 667), paint);
    paint.setColor(DlColor(0xffff9800));
    canvas->DrawRect(DlRect::MakeLTRB(0, 0, 187.5, 333.5), paint);
    paint.setColor(DlColor(0xff9c27b0));
    canvas->DrawRect(DlRect::MakeLTRB(187.5, 0, 375, 333.5), paint);
    paint.setColor(DlColor(0xff4caf50));
    canvas->DrawRect(DlRect::MakeLTRB(0, 333.5, 187.5, 667), paint);
    paint.setColor(DlColor(0xfff44336));
    canvas->DrawRect(DlRect::MakeLTRB(187.5, 333.5, 375, 667), paint);

    canvas->Save();
    {
      canvas->ClipRoundRect(
          DlRoundRect::MakeOval(DlRect::MakeLTRB(201.25, 10, 361.25, 170)),
          DlClipOp::kIntersect, true);
      DlRect save_layer_bounds = DlRect::MakeLTRB(201.25, 10, 361.25, 170);
      auto backdrop =
          DlImageFilter::MakeMatrix(DlMatrix::MakeRow(3, 0, 0.0, -280,  //
                                                      0, 3, 0.0, -920,  //
                                                      0, 0, 1.0, 0.0,   //
                                                      0, 0, 0.0, 1.0),
                                    DlImageSampling::kLinear);
      canvas->SaveLayer(save_layer_bounds, /*paint=*/nullptr, backdrop.get());
      {
        canvas->Translate(201.25, 10);
        auto paint = DlPaint()
                         .setAntiAlias(true)
                         .setColor(DlColor(0xff2196f3))
                         .setStrokeWidth(5)
                         .setDrawStyle(DlDrawStyle::kStroke);
        canvas->DrawCircle(DlPoint(80, 80), 80, paint);
      }
      canvas->Restore();
    }
    canvas->Restore();
  };

  DisplayListBuilder builder;
  std::vector<sk_sp<DlImage>> images;
  draw(&builder, images);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

namespace {
void DrawBlurGrid(DlCanvas* canvas) {
  DlPaint paint;
  paint.setColor(DlColor(0xfffef7ff));
  Scalar width = 150;
  Scalar height = 150;
  Scalar gap = 80;
  std::vector<Scalar> blur_radii = {10, 30, 50};
  for (size_t i = 0; i < blur_radii.size(); ++i) {
    Scalar blur_radius = blur_radii[i];
    auto blur_filter = std::make_shared<flutter::DlBlurMaskFilter>(
        flutter::DlBlurStyle::kNormal, blur_radius);
    paint.setMaskFilter(blur_filter);
    Scalar yval = gap + i * (gap + height);
    canvas->DrawRoundRect(
        DlRoundRect::MakeNinePatch(DlRect::MakeXYWH(gap, yval, width, height),
                                   10, 10, 10, 10),
        paint);
    canvas->DrawRoundRect(
        DlRoundRect::MakeNinePatch(
            DlRect::MakeXYWH(2.0 * gap + width, yval, width, height),  //
            9, 10, 10, 10),
        paint);
  }
}
}  // namespace

TEST_P(DlGoldenTest, GaussianVsRRectBlur) {
  Point content_scale = GetContentScale();
  auto draw = [content_scale](DlCanvas* canvas,
                              const std::vector<sk_sp<DlImage>>& images) {
    canvas->Scale(content_scale.x, content_scale.y);
    canvas->DrawPaint(DlPaint().setColor(DlColor(0xff112233)));
    DrawBlurGrid(canvas);
  };

  DisplayListBuilder builder;
  std::vector<sk_sp<DlImage>> images;
  draw(&builder, images);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(DlGoldenTest, GaussianVsRRectBlurScaled) {
  Point content_scale = GetContentScale();
  auto draw = [content_scale](DlCanvas* canvas,
                              const std::vector<sk_sp<DlImage>>& images) {
    canvas->Scale(content_scale.x, content_scale.y);
    canvas->DrawPaint(DlPaint().setColor(DlColor(0xff112233)));
    canvas->Scale(0.33, 0.33);
    DrawBlurGrid(canvas);
  };

  DisplayListBuilder builder;
  std::vector<sk_sp<DlImage>> images;
  draw(&builder, images);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(DlGoldenTest, GaussianVsRRectBlurScaledRotated) {
  Point content_scale = GetContentScale();
  auto draw = [content_scale](DlCanvas* canvas,
                              const std::vector<sk_sp<DlImage>>& images) {
    canvas->Scale(content_scale.x, content_scale.y);
    canvas->Translate(200, 200);
    canvas->DrawPaint(DlPaint().setColor(DlColor(0xff112233)));
    canvas->Scale(0.33, 0.33);
    canvas->Translate(300, 300);
    canvas->Rotate(45);
    canvas->Translate(-300, -300);
    DrawBlurGrid(canvas);
  };

  DisplayListBuilder builder;
  std::vector<sk_sp<DlImage>> images;
  draw(&builder, images);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(DlGoldenTest, FastVsGeneralGaussianMaskBlur) {
  DisplayListBuilder builder;
  builder.Scale(GetContentScale().x, GetContentScale().y);
  builder.DrawColor(DlColor::kWhite(), DlBlendMode::kSrc);

  auto blur_sigmas = std::array{5.0f, 10.0f, 20.0f};
  auto blur_colors = std::array{
      DlColor::kBlue(),
      DlColor::kGreen(),
      DlColor::kMaroon(),
  };

  auto make_rrect_path = [](const DlRect& rect, DlScalar rx,
                            DlScalar ry) -> DlPath {
    auto add_corner = [](DlPathBuilder& path_builder, DlPoint corner,
                         DlVector2 relative_from, DlVector2 relative_to,
                         bool first) {
      static const auto magic = impeller::PathBuilder::kArcApproximationMagic;

      if (first) {
        path_builder.MoveTo(corner + relative_from);
      } else {
        path_builder.LineTo(corner + relative_from);
      }
      // These fractions should be (1 - magic) to make a proper rrect
      // path, but historically these equations were as written here.
      // On the plus side, they ensure that we will not optimize this
      // path as "Hey, look, it's an RRect", but the DrawPath gaussians
      // will otherwise not be identical to the versions drawn with
      // DrawRoundRect
      path_builder.CubicCurveTo(corner + relative_from * magic,
                                corner + relative_to * magic,
                                corner + relative_to);
    };

    DlPathBuilder path_builder;
    add_corner(path_builder, rect.GetRightTop(),  //
               DlVector2(-rx, 0.0f), DlVector2(0.0f, ry), true);
    add_corner(path_builder, rect.GetRightBottom(),  //
               DlVector2(0.0f, -ry), DlVector2(-rx, 0.0f), false);
    add_corner(path_builder, rect.GetLeftBottom(),  //
               DlVector2(rx, 0.0f), DlVector2(0.0f, -ry), false);
    add_corner(path_builder, rect.GetLeftTop(),  //
               DlVector2(0.0f, ry), DlVector2(rx, 0.0f), false);
    return DlPath(path_builder);
  };

  for (size_t i = 0; i < blur_sigmas.size(); i++) {
    auto rect = DlRect::MakeXYWH(i * 320.0f + 50.0f, 50.0f, 100.0f, 100.0f);
    DlPaint paint = DlPaint()  //
                        .setColor(blur_colors[i])
                        .setMaskFilter(DlBlurMaskFilter::Make(
                            DlBlurStyle::kNormal, blur_sigmas[i]));

    builder.DrawRoundRect(DlRoundRect::MakeRectXY(rect, 10.0f, 10.0f), paint);
    rect = rect.Shift(150.0f, 0.0f);
    builder.DrawPath(make_rrect_path(rect, 10.0f, 10.0f), paint);
    rect = rect.Shift(-150.0f, 0.0f);

    rect = rect.Shift(0.0f, 200.0f);
    builder.DrawRoundRect(DlRoundRect::MakeRectXY(rect, 10.0f, 30.0f), paint);
    rect = rect.Shift(150.0f, 0.0f);
    builder.DrawPath(make_rrect_path(rect, 10.0f, 20.0f), paint);
    rect = rect.Shift(-150.0f, 0.0f);

    rect = rect.Shift(0.0f, 200.0f);
    builder.DrawRoundRect(DlRoundRect::MakeRectXY(rect, 30.0f, 10.0f), paint);
    rect = rect.Shift(150.0f, 0.0f);
    builder.DrawPath(make_rrect_path(rect, 20.0f, 10.0f), paint);
    rect = rect.Shift(-150.0f, 0.0f);
  }

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(DlGoldenTest, DashedLinesTest) {
  Point content_scale = GetContentScale();
  auto draw = [content_scale](DlCanvas* canvas,
                              const std::vector<sk_sp<DlImage>>& images) {
    canvas->Scale(content_scale.x, content_scale.y);
    canvas->DrawPaint(DlPaint().setColor(DlColor::kWhite()));

    auto draw_one = [canvas](DlStrokeCap cap, Scalar x, Scalar y,
                             Scalar dash_on, Scalar dash_off) {
      Point center = Point(x, y);
      Scalar inner = 20.0f;
      Scalar outer = 100.0f;
      DlPaint thick_paint = DlPaint()
                                .setColor(DlColor::kBlue())
                                .setStrokeCap(cap)
                                .setStrokeWidth(8.0f);
      DlPaint middle_paint = DlPaint()
                                 .setColor(DlColor::kGreen())
                                 .setStrokeCap(cap)
                                 .setStrokeWidth(5.0f);
      DlPaint thin_paint = DlPaint()
                               .setColor(DlColor::kMagenta())
                               .setStrokeCap(cap)
                               .setStrokeWidth(2.0f);
      for (int degrees = 0; degrees < 360; degrees += 30) {
        Point delta = Point(1.0f, 0.0f).Rotate(Degrees(degrees));
        canvas->DrawDashedLine(center + inner * delta, center + outer * delta,
                               dash_on, dash_off, thick_paint);
        canvas->DrawDashedLine(center + inner * delta, center + outer * delta,
                               dash_on, dash_off, middle_paint);
        canvas->DrawDashedLine(center + inner * delta, center + outer * delta,
                               dash_on, dash_off, thin_paint);
      }
    };

    draw_one(DlStrokeCap::kButt, 150.0f, 150.0f, 15.0f, 10.0f);
    draw_one(DlStrokeCap::kSquare, 400.0f, 150.0f, 15.0f, 10.0f);
    draw_one(DlStrokeCap::kRound, 150.0f, 400.0f, 15.0f, 10.0f);
    draw_one(DlStrokeCap::kRound, 400.0f, 400.0f, 0.0f, 11.0f);

    // Make sure the rendering op responds appropriately to clipping
    canvas->Save();
    DlPathBuilder path_builder;
    path_builder.MoveTo(DlPoint(275.0f, 225.0f));
    path_builder.LineTo(DlPoint(325.0f, 275.0f));
    path_builder.LineTo(DlPoint(275.0f, 325.0f));
    path_builder.LineTo(DlPoint(225.0f, 275.0f));
    canvas->ClipPath(DlPath(path_builder));
    canvas->DrawColor(DlColor::kYellow());
    draw_one(DlStrokeCap::kRound, 275.0f, 275.0f, 15.0f, 10.0f);
    canvas->Restore();
  };

  DisplayListBuilder builder;
  std::vector<sk_sp<DlImage>> images;
  draw(&builder, images);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(DlGoldenTest, SaveLayerAtFractionalValue) {
  // Draws a stroked rounded rect at a fractional pixel value. The coverage must
  // be adjusted so that we still have room to draw it, even though it lies on
  // the fractional bounds of the saveLayer.
  DisplayListBuilder builder;
  builder.DrawPaint(DlPaint().setColor(DlColor::kWhite()));
  auto save_paint = DlPaint().setAlpha(100);
  builder.SaveLayer(nullptr, &save_paint);

  builder.DrawRoundRect(DlRoundRect::MakeRectRadius(
                            DlRect::MakeLTRB(10.5, 10.5, 200.5, 200.5), 10),
                        DlPaint()
                            .setDrawStyle(DlDrawStyle::kStroke)
                            .setStrokeWidth(1.5)
                            .setColor(DlColor::kBlack()));
  builder.DrawCircle(DlPoint::MakeXY(100, 100), 50.5,
                     DlPaint().setColor(DlColor::kAqua()));
  builder.DrawCircle(DlPoint::MakeXY(110, 110), 50.5,
                     DlPaint().setColor(DlColor::kCyan()));

  builder.Restore();

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

namespace {
int32_t CalculateMaxY(const impeller::testing::Screenshot* img) {
  const uint32_t* ptr = reinterpret_cast<const uint32_t*>(img->GetBytes());
  int32_t max_y = 0;
  for (uint32_t i = 0; i < img->GetHeight(); ++i) {
    for (uint32_t j = 0; j < img->GetWidth(); ++j) {
      uint32_t pixel = *ptr++;
      if ((pixel & 0x00ffffff) != 0) {
        max_y = std::max(max_y, static_cast<int32_t>(i));
      }
    }
  }
  return max_y;
}
}  // namespace

TEST_P(DlGoldenTest, BaselineHE) {
  SetWindowSize(impeller::ISize(1024, 200));
  impeller::Scalar font_size = 300;
  auto callback = [&](const char* text,
                      impeller::Scalar scale) -> sk_sp<DisplayList> {
    DisplayListBuilder builder;
    DlPaint paint;
    paint.setColor(DlColor::ARGB(1, 0, 0, 0));
    builder.DrawPaint(paint);
    builder.Scale(scale, scale);
    RenderTextInCanvasSkia(&builder, text, "Roboto-Regular.ttf",
                           DlPoint::MakeXY(10, 300),
                           TextRenderOptions{
                               .font_size = font_size,
                           });
    return builder.Build();
  };

  std::unique_ptr<impeller::testing::Screenshot> right =
      MakeScreenshot(callback("h", 0.444));
  if (!right) {
    GTEST_SKIP() << "making screenshots not supported.";
  }
  std::unique_ptr<impeller::testing::Screenshot> left =
      MakeScreenshot(callback("e", 0.444));

  int32_t left_max_y = CalculateMaxY(left.get());
  int32_t right_max_y = CalculateMaxY(right.get());
  int32_t y_diff = std::abs(left_max_y - right_max_y);
  EXPECT_TRUE(y_diff <= 2) << "y diff: " << y_diff;
}
}  // namespace testing
}  // namespace flutter
