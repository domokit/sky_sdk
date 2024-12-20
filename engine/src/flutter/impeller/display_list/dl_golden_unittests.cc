// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/display_list/dl_golden_unittests.h"

#include "flutter/display_list/dl_builder.h"
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
    canvas->DrawImage(images[0], SkPoint::Make(100.0, 100.0),
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
    canvas->DrawRect(SkRect::MakeLTRB(0, 0, 375, 667), paint);
    paint.setColor(DlColor(0xffff9800));
    canvas->DrawRect(SkRect::MakeLTRB(0, 0, 187.5, 333.5), paint);
    paint.setColor(DlColor(0xff9c27b0));
    canvas->DrawRect(SkRect::MakeLTRB(187.5, 0, 375, 333.5), paint);
    paint.setColor(DlColor(0xff4caf50));
    canvas->DrawRect(SkRect::MakeLTRB(0, 333.5, 187.5, 667), paint);
    paint.setColor(DlColor(0xfff44336));
    canvas->DrawRect(SkRect::MakeLTRB(187.5, 333.5, 375, 667), paint);

    canvas->Save();
    {
      canvas->ClipRRect(
          SkRRect::MakeOval(SkRect::MakeLTRB(201.25, 10, 361.25, 170)),
          DlCanvas::ClipOp::kIntersect, true);
      SkRect save_layer_bounds = SkRect::MakeLTRB(201.25, 10, 361.25, 170);
      auto backdrop =
          DlImageFilter::MakeMatrix(DlMatrix::MakeRow(3, 0, 0.0, -280,  //
                                                      0, 3, 0.0, -920,  //
                                                      0, 0, 1.0, 0.0,   //
                                                      0, 0, 0.0, 1.0),
                                    DlImageSampling::kLinear);
      canvas->SaveLayer(&save_layer_bounds, /*paint=*/nullptr, backdrop.get());
      {
        canvas->Translate(201.25, 10);
        auto paint = DlPaint()
                         .setAntiAlias(true)
                         .setColor(DlColor(0xff2196f3))
                         .setStrokeWidth(5)
                         .setDrawStyle(DlDrawStyle::kStroke);
        canvas->DrawCircle(SkPoint::Make(80, 80), 80, paint);
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
    SkRRect rrect;
    Scalar yval = gap + i * (gap + height);
    rrect.setNinePatch(SkRect::MakeXYWH(gap, yval, width, height), 10, 10, 10,
                       10);
    canvas->DrawRRect(rrect, paint);
    rrect.setNinePatch(SkRect::MakeXYWH(2.0 * gap + width, yval, width, height),
                       9, 10, 10, 10);
    canvas->DrawRRect(rrect, paint);
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

  auto make_rrect_path = [](const SkRect& rect, DlScalar rx,
                            DlScalar ry) -> SkPath {
    auto add_corner = [](SkPath& path, SkPoint rCorner, SkPoint rEnd) {
      static const auto magic = impeller::PathBuilder::kArcApproximationMagic;
      path.rCubicTo(rCorner.fX * (1.0f - magic), rCorner.fY * (1.0f - magic),
                    rCorner.fX + rEnd.fX * magic, rCorner.fY + rEnd.fY * magic,
                    rCorner.fX + rEnd.fX, rCorner.fY + rEnd.fY);
    };

    SkPath path;
    path.moveTo(rect.fRight - rx, rect.fTop);
    add_corner(path, {rx, 0.0f}, {0.0f, ry});
    path.lineTo(rect.fRight, rect.fBottom - ry);
    add_corner(path, {0.0f, ry}, {-rx, 0.0f});
    path.lineTo(rect.fLeft + rx, rect.fBottom);
    add_corner(path, {-rx, 0.0f}, {0.0f, -ry});
    path.lineTo(rect.fLeft, rect.fTop + ry);
    add_corner(path, {0.0f, -ry}, {rx, 0.0f});
    path.close();
    return path;
  };

  for (size_t i = 0; i < blur_sigmas.size(); i++) {
    auto rect = SkRect::MakeXYWH(i * 320.0f + 50.0f, 50.0f, 100.0f, 100.0f);
    DlPaint paint = DlPaint()  //
                        .setColor(blur_colors[i])
                        .setMaskFilter(DlBlurMaskFilter::Make(
                            DlBlurStyle::kNormal, blur_sigmas[i]));

    builder.DrawRRect(SkRRect::MakeRectXY(rect, 10.0f, 10.0f), paint);
    rect = rect.makeOffset(150.0f, 0.0f);
    builder.DrawPath(make_rrect_path(rect, 10.0f, 10.0f), paint);
    rect = rect.makeOffset(-150.0f, 0.0f);

    rect = rect.makeOffset(0.0f, 200.0f);
    builder.DrawRRect(SkRRect::MakeRectXY(rect, 10.0f, 30.0f), paint);
    rect = rect.makeOffset(150.0f, 0.0f);
    builder.DrawPath(make_rrect_path(rect, 10.0f, 20.0f), paint);
    rect = rect.makeOffset(-150.0f, 0.0f);

    rect = rect.makeOffset(0.0f, 200.0f);
    builder.DrawRRect(SkRRect::MakeRectXY(rect, 30.0f, 10.0f), paint);
    rect = rect.makeOffset(150.0f, 0.0f);
    builder.DrawPath(make_rrect_path(rect, 20.0f, 10.0f), paint);
    rect = rect.makeOffset(-150.0f, 0.0f);
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
    SkPath clip_path = SkPath();
    clip_path.moveTo(275.0f, 225.0f);
    clip_path.lineTo(325.0f, 275.0f);
    clip_path.lineTo(275.0f, 325.0f);
    clip_path.lineTo(225.0f, 275.0f);
    canvas->ClipPath(clip_path);
    canvas->DrawColor(DlColor::kYellow());
    draw_one(DlStrokeCap::kRound, 275.0f, 275.0f, 15.0f, 10.0f);
    canvas->Restore();
  };

  DisplayListBuilder builder;
  std::vector<sk_sp<DlImage>> images;
  draw(&builder, images);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

}  // namespace testing
}  // namespace flutter
