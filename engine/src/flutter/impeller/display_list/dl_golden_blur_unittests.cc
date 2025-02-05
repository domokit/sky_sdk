// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/display_list/dl_golden_unittests.h"

#include "flutter/display_list/dl_builder.h"
#include "flutter/display_list/effects/dl_mask_filter.h"
#include "flutter/impeller/display_list/testing/render_text_in_canvas.h"
#include "flutter/impeller/display_list/testing/rmse.h"
#include "flutter/impeller/geometry/round_rect.h"
#include "flutter/impeller/golden_tests/screenshot.h"
#include "flutter/testing/testing.h"
#include "gtest/gtest.h"

namespace flutter {
namespace testing {

using impeller::Font;

TEST_P(DlGoldenTest, TextBlurMaskFilterRespectCTM) {
  impeller::Point content_scale = GetContentScale();
  auto draw = [&](DlCanvas* canvas,
                  const std::vector<std::unique_ptr<DlImage>>& images) {
    canvas->DrawColor(DlColor(0xff111111));
    canvas->Scale(content_scale.x, content_scale.y);
    canvas->Scale(2, 2);
    TextRenderOptions options;
    options.mask_filter =
        DlBlurMaskFilter::Make(DlBlurStyle::kNormal, /*sigma=*/10,
                               /*respect_ctm=*/true);
    ASSERT_TRUE(RenderTextInCanvasSkia(canvas, "hello world",
                                       "Roboto-Regular.ttf",  //
                                       DlPoint(101, 101), options));
    options.mask_filter = nullptr;
    options.color = DlColor::kRed();
    ASSERT_TRUE(RenderTextInCanvasSkia(canvas, "hello world",
                                       "Roboto-Regular.ttf",  //
                                       DlPoint(100, 100), options));
  };

  DisplayListBuilder builder;
  draw(&builder, /*images=*/{});

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(DlGoldenTest, TextBlurMaskFilterDisrespectCTM) {
  impeller::Point content_scale = GetContentScale();
  auto draw = [&](DlCanvas* canvas,
                  const std::vector<std::unique_ptr<DlImage>>& images) {
    canvas->DrawColor(DlColor(0xff111111));
    canvas->Scale(content_scale.x, content_scale.y);
    canvas->Scale(2, 2);
    TextRenderOptions options;
    options.mask_filter =
        DlBlurMaskFilter::Make(DlBlurStyle::kNormal, /*sigma=*/10,
                               /*respect_ctm=*/false);
    ASSERT_TRUE(RenderTextInCanvasSkia(canvas, "hello world",
                                       "Roboto-Regular.ttf",  //
                                       DlPoint(101, 101), options));
    options.mask_filter = nullptr;
    options.color = DlColor::kRed();
    ASSERT_TRUE(RenderTextInCanvasSkia(canvas, "hello world",
                                       "Roboto-Regular.ttf",  //
                                       DlPoint(100, 100), options));
  };

  DisplayListBuilder builder;
  draw(&builder, /*images=*/{});

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

// This is a test to make sure that we don't regress "shimmering" in the
// gaussian blur. Shimmering is abrupt changes in signal when making tiny
// changes to the blur parameters.
//
// See also:
//   - https://github.com/flutter/flutter/issues/152195
TEST_P(DlGoldenTest, ShimmerTest) {
  impeller::Point content_scale = GetContentScale();
  auto draw = [&](DlCanvas* canvas, const std::vector<sk_sp<DlImage>>& images,
                  float sigma) {
    canvas->DrawColor(DlColor(0xff111111));
    canvas->Scale(content_scale.x, content_scale.y);

    DlPaint paint;
    canvas->DrawImage(images[0], DlPoint(10.135, 10.36334),
                      DlImageSampling::kLinear, &paint);

    DlRect save_layer_bounds = DlRect::MakeLTRB(0, 0, 1024, 768);
    auto blur = DlImageFilter::MakeBlur(sigma, sigma, DlTileMode::kDecal);
    canvas->ClipRect(DlRect::MakeLTRB(11.125, 10.3737, 911.25, 755.3333));
    canvas->SaveLayer(save_layer_bounds, /*paint=*/nullptr, blur.get());
    canvas->Restore();
  };

  std::vector<sk_sp<DlImage>> images;
  images.emplace_back(CreateDlImageForFixture("boston.jpg"));

  auto make_screenshot = [&](float sigma) {
    DisplayListBuilder builder;
    draw(&builder, images, sigma);

    std::unique_ptr<impeller::testing::Screenshot> screenshot =
        MakeScreenshot(builder.Build());
    return screenshot;
  };

  float start_sigma = 10.0f;
  std::unique_ptr<impeller::testing::Screenshot> left =
      make_screenshot(start_sigma);
  if (!left) {
    GTEST_SKIP() << "making screenshots not supported.";
  }

  double average_rmse = 0.0;
  const int32_t sample_count = 200;
  for (int i = 1; i <= sample_count; ++i) {
    float sigma = start_sigma + (i / 2.f);
    std::unique_ptr<impeller::testing::Screenshot> right =
        make_screenshot(sigma);
    double rmse = RMSE(left.get(), right.get());
    average_rmse += rmse;

    // To debug this output the frames can be written out to disk then
    // transformed to a video with ffmpeg.
    //
    // ## save images command
    // std::stringstream ss;
    // ss << "_" << std::setw(3) << std::setfill('0') << (i - 1);
    // SaveScreenshot(std::move(left), ss.str());
    //
    // ## ffmpeg command
    // ```
    // ffmpeg -framerate 30 -pattern_type glob -i '*.png' \
    //   -c:v libx264 -pix_fmt yuv420p out.mp4
    // ```
    left = std::move(right);
  }

  average_rmse = average_rmse / sample_count;

  // This is a somewhat arbitrary threshold. It could be increased if we wanted.
  // In the problematic cases previously we should values like 28. Before
  // increasing this you should manually inspect the behavior in
  // `AiksTest.GaussianBlurAnimatedBackdrop`. Average RMSE is a able to catch
  // shimmer but it isn't perfect.
  EXPECT_TRUE(average_rmse < 1.0) << "average_rmse: " << average_rmse;
  // An average rmse of 0 would mean that the blur isn't blurring.
  EXPECT_TRUE(average_rmse >= 0.0) << "average_rmse: " << average_rmse;
}

TEST_P(DlGoldenTest, StrokedRRectFastBlur) {
  impeller::Point content_scale = GetContentScale();

  DlRect rect = DlRect::MakeXYWH(50, 50, 100, 100);
  DlRoundRect rrect = DlRoundRect::MakeRectRadius(rect, 10.0f);
  DlPaint fill = DlPaint().setColor(DlColor::kBlue());
  DlPaint stroke =
      DlPaint(fill).setDrawStyle(DlDrawStyle::kStroke).setStrokeWidth(10.0f);
  DlPaint blur = DlPaint(fill).setMaskFilter(
      DlBlurMaskFilter::Make(DlBlurStyle::kNormal, 5.0, true));
  DlPaint blur_stroke =
      DlPaint(blur).setDrawStyle(DlDrawStyle::kStroke).setStrokeWidth(10.0f);

  DisplayListBuilder builder;
  builder.DrawColor(DlColor(0xff111111), DlBlendMode::kSrc);
  builder.Scale(content_scale.x, content_scale.y);
  builder.DrawRoundRect(rrect, fill);
  builder.DrawRoundRect(rrect.Shift(150, 0), stroke);
  builder.DrawRoundRect(rrect.Shift(0, 150), blur);
  builder.DrawRoundRect(rrect.Shift(150, 150), blur_stroke);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

}  // namespace testing
}  // namespace flutter
