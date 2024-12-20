// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "display_list/dl_blend_mode.h"
#include "display_list/dl_color.h"
#include "display_list/dl_tile_mode.h"
#include "flutter/testing/testing.h"
#include "impeller/core/formats.h"
#include "impeller/display_list/skia_conversions.h"
#include "impeller/geometry/color.h"
#include "impeller/geometry/scalar.h"
#include "include/core/SkMatrix.h"
#include "include/core/SkRRect.h"

namespace impeller {
namespace testing {

TEST(SkiaConversionTest, ToMatrixTranslate) {
  SkMatrix sk_matrix = SkMatrix::Translate(100, 100);
  Matrix matrix = skia_conversions::ToMatrix(sk_matrix);

  EXPECT_EQ(matrix.m[12], 100);
  EXPECT_EQ(matrix.m[13], 100);
  EXPECT_TRUE(matrix.IsTranslationScaleOnly());

  matrix.m[12] = 0;
  matrix.m[13] = 0;

  EXPECT_TRUE(matrix.IsIdentity());
}

TEST(SkiaConversionTest, ToMatrixScale) {
  SkMatrix sk_matrix = SkMatrix::Scale(2, 2);
  Matrix matrix = skia_conversions::ToMatrix(sk_matrix);

  EXPECT_EQ(matrix.m[0], 2);
  EXPECT_EQ(matrix.m[5], 2);
  EXPECT_TRUE(matrix.IsTranslationScaleOnly());

  matrix.m[0] = 1;
  matrix.m[5] = 1;

  EXPECT_TRUE(matrix.IsIdentity());
}

TEST(SkiaConversionTest, ToMatrixRotate) {
  SkMatrix sk_matrix = SkMatrix::RotateDeg(90);
  Matrix matrix = skia_conversions::ToMatrix(sk_matrix);

  EXPECT_EQ(matrix.vec[0], Vector4(0, 1, 0, 0));
  EXPECT_EQ(matrix.vec[1], Vector4(-1, 0, 0, 0));
  EXPECT_EQ(matrix.vec[2], Vector4(0, 0, 1, 0));
  EXPECT_EQ(matrix.vec[3], Vector4(0, 0, 0, 1));
  EXPECT_FALSE(matrix.IsTranslationScaleOnly());
}

TEST(SkiaConversionTest, ToMatrixSkew) {
  SkMatrix sk_matrix = SkMatrix::Skew(2, 2);
  Matrix matrix = skia_conversions::ToMatrix(sk_matrix);

  EXPECT_EQ(matrix.vec[0], Vector4(1, 2, 0, 0));
  EXPECT_EQ(matrix.vec[1], Vector4(2, 1, 0, 0));
  EXPECT_EQ(matrix.vec[2], Vector4(0, 0, 1, 0));
  EXPECT_EQ(matrix.vec[3], Vector4(0, 0, 0, 1));
  EXPECT_FALSE(matrix.IsTranslationScaleOnly());
}

TEST(SkiaConversionTest, ToSamplerDescriptor) {
  EXPECT_EQ(skia_conversions::ToSamplerDescriptor(
                flutter::DlImageSampling::kNearestNeighbor)
                .min_filter,
            impeller::MinMagFilter::kNearest);
  EXPECT_EQ(skia_conversions::ToSamplerDescriptor(
                flutter::DlImageSampling::kNearestNeighbor)
                .mip_filter,
            impeller::MipFilter::kBase);

  EXPECT_EQ(
      skia_conversions::ToSamplerDescriptor(flutter::DlImageSampling::kLinear)
          .min_filter,
      impeller::MinMagFilter::kLinear);
  EXPECT_EQ(
      skia_conversions::ToSamplerDescriptor(flutter::DlImageSampling::kLinear)
          .mip_filter,
      impeller::MipFilter::kBase);

  EXPECT_EQ(skia_conversions::ToSamplerDescriptor(
                flutter::DlImageSampling::kMipmapLinear)
                .min_filter,
            impeller::MinMagFilter::kLinear);
  EXPECT_EQ(skia_conversions::ToSamplerDescriptor(
                flutter::DlImageSampling::kMipmapLinear)
                .mip_filter,
            impeller::MipFilter::kLinear);
}

TEST(SkiaConversionsTest, SkPointToPoint) {
  for (int x = -100; x < 100; x += 4) {
    for (int y = -100; y < 100; y += 4) {
      EXPECT_EQ(skia_conversions::ToPoint(SkPoint::Make(x * 0.25f, y * 0.25f)),
                Point(x * 0.25f, y * 0.25f));
    }
  }
}

TEST(SkiaConversionsTest, SkPointToSize) {
  for (int x = -100; x < 100; x += 4) {
    for (int y = -100; y < 100; y += 4) {
      EXPECT_EQ(skia_conversions::ToSize(SkPoint::Make(x * 0.25f, y * 0.25f)),
                Size(x * 0.25f, y * 0.25f));
    }
  }
}

TEST(SkiaConversionsTest, ToColor) {
  // Create a color with alpha, red, green, and blue values that are all
  // trivially divisible by 255 so that we can test the conversion results in
  // correct scalar values.
  //                                                AARRGGBB
  const flutter::DlColor color = flutter::DlColor(0x8040C020);
  auto converted_color = skia_conversions::ToColor(color);

  ASSERT_TRUE(ScalarNearlyEqual(converted_color.alpha, 0x80 * (1.0f / 255)));
  ASSERT_TRUE(ScalarNearlyEqual(converted_color.red, 0x40 * (1.0f / 255)));
  ASSERT_TRUE(ScalarNearlyEqual(converted_color.green, 0xC0 * (1.0f / 255)));
  ASSERT_TRUE(ScalarNearlyEqual(converted_color.blue, 0x20 * (1.0f / 255)));
}

TEST(SkiaConversionsTest, GradientStopConversion) {
  // Typical gradient.
  std::vector<flutter::DlColor> colors = {flutter::DlColor::kBlue(),
                                          flutter::DlColor::kRed(),
                                          flutter::DlColor::kGreen()};
  std::vector<float> stops = {0.0, 0.5, 1.0};
  const auto gradient =
      flutter::DlColorSource::MakeLinear(flutter::DlPoint(0, 0),       //
                                         flutter::DlPoint(1.0, 1.0),   //
                                         3,                            //
                                         colors.data(),                //
                                         stops.data(),                 //
                                         flutter::DlTileMode::kClamp,  //
                                         nullptr                       //
      );

  std::vector<Color> converted_colors;
  std::vector<Scalar> converted_stops;
  skia_conversions::ConvertStops(gradient->asLinearGradient(), converted_colors,
                                 converted_stops);

  ASSERT_TRUE(ScalarNearlyEqual(converted_stops[0], 0.0f));
  ASSERT_TRUE(ScalarNearlyEqual(converted_stops[1], 0.5f));
  ASSERT_TRUE(ScalarNearlyEqual(converted_stops[2], 1.0f));
}

TEST(SkiaConversionsTest, GradientMissing0) {
  std::vector<flutter::DlColor> colors = {flutter::DlColor::kBlue(),
                                          flutter::DlColor::kRed()};
  std::vector<float> stops = {0.5, 1.0};
  const auto gradient =
      flutter::DlColorSource::MakeLinear(flutter::DlPoint(0, 0),       //
                                         flutter::DlPoint(1.0, 1.0),   //
                                         2,                            //
                                         colors.data(),                //
                                         stops.data(),                 //
                                         flutter::DlTileMode::kClamp,  //
                                         nullptr                       //
      );

  std::vector<Color> converted_colors;
  std::vector<Scalar> converted_stops;
  skia_conversions::ConvertStops(gradient->asLinearGradient(), converted_colors,
                                 converted_stops);

  // First color is inserted as blue.
  ASSERT_TRUE(ScalarNearlyEqual(converted_colors[0].blue, 1.0f));
  ASSERT_TRUE(ScalarNearlyEqual(converted_stops[0], 0.0f));
  ASSERT_TRUE(ScalarNearlyEqual(converted_stops[1], 0.5f));
  ASSERT_TRUE(ScalarNearlyEqual(converted_stops[2], 1.0f));
}

TEST(SkiaConversionsTest, GradientMissingLastValue) {
  std::vector<flutter::DlColor> colors = {flutter::DlColor::kBlue(),
                                          flutter::DlColor::kRed()};
  std::vector<float> stops = {0.0, .5};
  const auto gradient =
      flutter::DlColorSource::MakeLinear(flutter::DlPoint(0, 0),       //
                                         flutter::DlPoint(1.0, 1.0),   //
                                         2,                            //
                                         colors.data(),                //
                                         stops.data(),                 //
                                         flutter::DlTileMode::kClamp,  //
                                         nullptr                       //
      );

  std::vector<Color> converted_colors;
  std::vector<Scalar> converted_stops;
  skia_conversions::ConvertStops(gradient->asLinearGradient(), converted_colors,
                                 converted_stops);

  // Last color is inserted as red.
  ASSERT_TRUE(ScalarNearlyEqual(converted_colors[2].red, 1.0f));
  ASSERT_TRUE(ScalarNearlyEqual(converted_stops[0], 0.0f));
  ASSERT_TRUE(ScalarNearlyEqual(converted_stops[1], 0.5f));
  ASSERT_TRUE(ScalarNearlyEqual(converted_stops[2], 1.0f));
}

TEST(SkiaConversionsTest, GradientStopGreaterThan1) {
  std::vector<flutter::DlColor> colors = {flutter::DlColor::kBlue(),
                                          flutter::DlColor::kGreen(),
                                          flutter::DlColor::kRed()};
  std::vector<float> stops = {0.0, 100, 1.0};
  const auto gradient =
      flutter::DlColorSource::MakeLinear(flutter::DlPoint(0, 0),       //
                                         flutter::DlPoint(1.0, 1.0),   //
                                         3,                            //
                                         colors.data(),                //
                                         stops.data(),                 //
                                         flutter::DlTileMode::kClamp,  //
                                         nullptr                       //
      );

  std::vector<Color> converted_colors;
  std::vector<Scalar> converted_stops;
  skia_conversions::ConvertStops(gradient->asLinearGradient(), converted_colors,
                                 converted_stops);

  // Value is clamped to 1.0
  ASSERT_TRUE(ScalarNearlyEqual(converted_stops[0], 0.0f));
  ASSERT_TRUE(ScalarNearlyEqual(converted_stops[1], 1.0f));
  ASSERT_TRUE(ScalarNearlyEqual(converted_stops[2], 1.0f));
}

TEST(SkiaConversionsTest, GradientConversionNonMonotonic) {
  std::vector<flutter::DlColor> colors = {
      flutter::DlColor::kBlue(), flutter::DlColor::kGreen(),
      flutter::DlColor::kGreen(), flutter::DlColor::kRed()};
  std::vector<float> stops = {0.0, 0.5, 0.4, 1.0};
  const auto gradient =
      flutter::DlColorSource::MakeLinear(flutter::DlPoint(0, 0),       //
                                         flutter::DlPoint(1.0, 1.0),   //
                                         4,                            //
                                         colors.data(),                //
                                         stops.data(),                 //
                                         flutter::DlTileMode::kClamp,  //
                                         nullptr                       //
      );

  std::vector<Color> converted_colors;
  std::vector<Scalar> converted_stops;
  skia_conversions::ConvertStops(gradient->asLinearGradient(), converted_colors,
                                 converted_stops);

  // Value is clamped to 0.5
  ASSERT_TRUE(ScalarNearlyEqual(converted_stops[0], 0.0f));
  ASSERT_TRUE(ScalarNearlyEqual(converted_stops[1], 0.5f));
  ASSERT_TRUE(ScalarNearlyEqual(converted_stops[2], 0.5f));
  ASSERT_TRUE(ScalarNearlyEqual(converted_stops[3], 1.0f));
}

TEST(SkiaConversionsTest, IsNearlySimpleRRect) {
  EXPECT_TRUE(skia_conversions::IsNearlySimpleRRect(
      SkRRect::MakeRectXY(SkRect::MakeLTRB(0, 0, 10, 10), 10, 10)));
  EXPECT_TRUE(skia_conversions::IsNearlySimpleRRect(
      SkRRect::MakeRectXY(SkRect::MakeLTRB(0, 0, 10, 10), 10, 9.999)));
  EXPECT_TRUE(skia_conversions::IsNearlySimpleRRect(
      SkRRect::MakeRectXY(SkRect::MakeLTRB(0, 0, 10, 10), 10, 9)));
  EXPECT_TRUE(skia_conversions::IsNearlySimpleRRect(
      SkRRect::MakeRectXY(SkRect::MakeLTRB(0, 0, 10, 10), 10, 5)));
  SkRect rect = SkRect::MakeLTRB(0, 0, 10, 10);
  SkRRect rrect;
  union {
    SkPoint radii[4] = {
        {10.0f, 9.0f},
        {10.0f, 9.0f},
        {10.0f, 9.0f},
        {10.0f, 9.0f},
    };
    SkScalar values[8];
  } test;
  rrect.setRectRadii(rect, test.radii);
  EXPECT_TRUE(skia_conversions::IsNearlySimpleRRect(rrect));
  for (int i = 0; i < 8; i++) {
    auto save = test.values[i];
    test.values[i] -= kEhCloseEnough * 0.5f;
    rrect.setRectRadii(rect, test.radii);
    EXPECT_TRUE(skia_conversions::IsNearlySimpleRRect(rrect))
        << "values[" << i << "] == " << test.values[i];
    test.values[i] -= kEhCloseEnough * 2.0f;
    rrect.setRectRadii(rect, test.radii);
    EXPECT_FALSE(skia_conversions::IsNearlySimpleRRect(rrect))
        << "values[" << i << "] == " << test.values[i];
    test.values[i] = save;
  }
}

TEST(SkiaConversionsTest, BlendMode) {
  for (auto i = 0; i < static_cast<int>(flutter::DlBlendMode::kLastMode); i++) {
    EXPECT_EQ(
        skia_conversions::ToBlendMode(static_cast<flutter::DlBlendMode>(i)),
        static_cast<BlendMode>(i));
  }
}

}  // namespace testing
}  // namespace impeller
