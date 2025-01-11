// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <memory>
#include <vector>

#include "flutter/display_list/dl_color.h"
#include "flutter/display_list/dl_sampling_options.h"
#include "flutter/display_list/effects/dl_color_sources.h"
#include "flutter/display_list/effects/dl_runtime_effect.h"
#include "flutter/display_list/image/dl_image.h"
#include "flutter/display_list/testing/dl_test_equality.h"
#include "flutter/display_list/testing/dl_test_snippets.h"

namespace flutter {
namespace testing {

static const sk_sp<DlImage> kTestOpaqueImage =
    MakeTestImage(10, 10, DlColor::kGreen());
static const sk_sp<DlImage> kTestAlphaImage =
    MakeTestImage(10, 10, DlColor::kTransparent());
// clang-format off
static const DlMatrix kTestMatrix1 =
    DlMatrix::MakeRow(2, 0, 0, 10,
                      0, 3, 0, 12,
                      0, 0, 1, 0,
                      0, 0, 0, 1);
static const DlMatrix kTestMatrix2 =
    DlMatrix::MakeRow(4, 0, 0, 15,
                      0, 7, 0, 17,
                      0, 0, 1, 0,
                      0, 0, 0, 1);
// clang-format on
static constexpr int kTestStopCount = 3;
static constexpr DlColor kTestColors[kTestStopCount] = {
    DlColor::kRed(),
    DlColor::kGreen(),
    DlColor::kBlue(),
};
static const DlColor kTestAlphaColors[kTestStopCount] = {
    DlColor::kBlue().withAlpha(0x7F),
    DlColor::kRed().withAlpha(0x2F),
    DlColor::kGreen().withAlpha(0xCF),
};
static constexpr float kTestStops[kTestStopCount] = {
    0.0f,
    0.7f,
    1.0f,
};
static constexpr float kTestStops2[kTestStopCount] = {
    0.0f,
    0.3f,
    1.0f,
};
static constexpr DlPoint kTestPoints1[2] = {
    DlPoint(5, 15),
    DlPoint(7, 18),
};
static constexpr DlPoint kTestPoints2[2] = {
    DlPoint(100, 115),
    DlPoint(107, 118),
};

TEST(DisplayListColorSource, ImageConstructor) {
  DlImageColorSource source(kTestOpaqueImage, DlTileMode::kClamp,
                            DlTileMode::kClamp, DlImageSampling::kLinear,
                            &kTestMatrix1);
}

TEST(DisplayListColorSource, ImageShared) {
  DlImageColorSource source(kTestOpaqueImage, DlTileMode::kClamp,
                            DlTileMode::kClamp, DlImageSampling::kLinear,
                            &kTestMatrix1);
  ASSERT_NE(source.shared().get(), &source);
  ASSERT_EQ(*source.shared(), source);
}

TEST(DisplayListColorSource, ImageAsImage) {
  DlImageColorSource source(kTestOpaqueImage, DlTileMode::kClamp,
                            DlTileMode::kClamp, DlImageSampling::kLinear,
                            &kTestMatrix1);
  ASSERT_NE(source.asImage(), nullptr);
  ASSERT_EQ(source.asImage(), &source);

  ASSERT_EQ(source.asLinearGradient(), nullptr);
  ASSERT_EQ(source.asRadialGradient(), nullptr);
  ASSERT_EQ(source.asConicalGradient(), nullptr);
  ASSERT_EQ(source.asSweepGradient(), nullptr);
  ASSERT_EQ(source.asRuntimeEffect(), nullptr);
}

TEST(DisplayListColorSource, ImageContents) {
  DlImageColorSource source(kTestOpaqueImage, DlTileMode::kRepeat,
                            DlTileMode::kMirror, DlImageSampling::kLinear,
                            &kTestMatrix1);
  ASSERT_EQ(source.image(), kTestOpaqueImage);
  ASSERT_EQ(source.horizontal_tile_mode(), DlTileMode::kRepeat);
  ASSERT_EQ(source.vertical_tile_mode(), DlTileMode::kMirror);
  ASSERT_EQ(source.sampling(), DlImageSampling::kLinear);
  ASSERT_EQ(source.matrix(), kTestMatrix1);
  ASSERT_EQ(source.is_opaque(), true);
}

TEST(DisplayListColorSource, AlphaImageContents) {
  DlImageColorSource source(kTestAlphaImage, DlTileMode::kRepeat,
                            DlTileMode::kMirror, DlImageSampling::kLinear,
                            &kTestMatrix1);
  ASSERT_EQ(source.image(), kTestAlphaImage);
  ASSERT_EQ(source.horizontal_tile_mode(), DlTileMode::kRepeat);
  ASSERT_EQ(source.vertical_tile_mode(), DlTileMode::kMirror);
  ASSERT_EQ(source.sampling(), DlImageSampling::kLinear);
  ASSERT_EQ(source.matrix(), kTestMatrix1);
  ASSERT_EQ(source.is_opaque(), false);
}

TEST(DisplayListColorSource, ImageEquals) {
  DlImageColorSource source1(kTestOpaqueImage, DlTileMode::kClamp,
                             DlTileMode::kMirror, DlImageSampling::kLinear,
                             &kTestMatrix1);
  DlImageColorSource source2(kTestOpaqueImage, DlTileMode::kClamp,
                             DlTileMode::kMirror, DlImageSampling::kLinear,
                             &kTestMatrix1);
  TestEquals(source1, source2);
}

TEST(DisplayListColorSource, ImageNotEquals) {
  DlImageColorSource source1(kTestOpaqueImage, DlTileMode::kClamp,
                             DlTileMode::kMirror, DlImageSampling::kLinear,
                             &kTestMatrix1);
  {
    DlImageColorSource source2(kTestAlphaImage, DlTileMode::kClamp,
                               DlTileMode::kMirror, DlImageSampling::kLinear,
                               &kTestMatrix1);
    TestNotEquals(source1, source2, "Image differs");
  }
  {
    DlImageColorSource source2(kTestOpaqueImage, DlTileMode::kRepeat,
                               DlTileMode::kMirror, DlImageSampling::kLinear,
                               &kTestMatrix1);
    TestNotEquals(source1, source2, "hTileMode differs");
  }
  {
    DlImageColorSource source2(kTestOpaqueImage, DlTileMode::kClamp,
                               DlTileMode::kRepeat, DlImageSampling::kLinear,
                               &kTestMatrix1);
    TestNotEquals(source1, source2, "vTileMode differs");
  }
  {
    DlImageColorSource source2(kTestOpaqueImage, DlTileMode::kClamp,
                               DlTileMode::kMirror, DlImageSampling::kCubic,
                               &kTestMatrix1);
    TestNotEquals(source1, source2, "Sampling differs");
  }
  {
    DlImageColorSource source2(kTestOpaqueImage, DlTileMode::kClamp,
                               DlTileMode::kMirror, DlImageSampling::kLinear,
                               &kTestMatrix2);
    TestNotEquals(source1, source2, "Matrix differs");
  }
}

TEST(DisplayListColorSource, LinearGradientConstructor) {
  std::shared_ptr<DlColorSource> source = DlColorSource::MakeLinear(
      kTestPoints1[0], kTestPoints1[1], kTestStopCount, kTestColors, kTestStops,
      DlTileMode::kClamp, &kTestMatrix1);
}

TEST(DisplayListColorSource, LinearGradientARGBConstructor) {
  std::array<DlScalar, kTestStopCount * 4> colors;
  for (int i = 0; i < kTestStopCount; ++i) {
    colors[i * 4 + 0] = kTestColors[i].getAlphaF();  //
    colors[i * 4 + 1] = kTestColors[i].getRedF();    //
    colors[i * 4 + 2] = kTestColors[i].getGreenF();  //
    colors[i * 4 + 3] = kTestColors[i].getBlueF();
  }
  std::shared_ptr<DlColorSource> source = DlColorSource::MakeLinear(
      kTestPoints1[0], kTestPoints1[1], kTestStopCount, colors.data(),
      kTestStops, DlTileMode::kClamp, &kTestMatrix1);
  ASSERT_TRUE(source);
  ASSERT_TRUE(source->asLinearGradient());
  EXPECT_EQ(source->asLinearGradient()->start_point(), kTestPoints1[0]);
  EXPECT_EQ(source->asLinearGradient()->end_point(), kTestPoints1[1]);
  EXPECT_EQ(source->asLinearGradient()->stop_count(), kTestStopCount);
  for (int i = 0; i < kTestStopCount; i++) {
    EXPECT_EQ(source->asLinearGradient()->colors()[i],
              kTestColors[i].withColorSpace(DlColorSpace::kExtendedSRGB));
    EXPECT_EQ(source->asLinearGradient()->stops()[i], kTestStops[i]);
  }
  EXPECT_EQ(source->asLinearGradient()->tile_mode(), DlTileMode::kClamp);
  EXPECT_EQ(source->asLinearGradient()->matrix(), kTestMatrix1);
  EXPECT_EQ(source->is_opaque(), true);
}

TEST(DisplayListColorSource, LinearGradientShared) {
  std::shared_ptr<DlColorSource> source = DlColorSource::MakeLinear(
      kTestPoints1[0], kTestPoints1[1], kTestStopCount, kTestColors, kTestStops,
      DlTileMode::kClamp, &kTestMatrix1);
  ASSERT_NE(source->shared().get(), source.get());
  ASSERT_EQ(*source->shared().get(), *source.get());
}

TEST(DisplayListColorSource, LinearGradientAsLinear) {
  std::shared_ptr<DlColorSource> source = DlColorSource::MakeLinear(
      kTestPoints1[0], kTestPoints1[1], kTestStopCount, kTestColors, kTestStops,
      DlTileMode::kClamp, &kTestMatrix1);
  ASSERT_NE(source->asLinearGradient(), nullptr);
  ASSERT_EQ(source->asLinearGradient(), source.get());

  ASSERT_EQ(source->asImage(), nullptr);
  ASSERT_EQ(source->asRadialGradient(), nullptr);
  ASSERT_EQ(source->asConicalGradient(), nullptr);
  ASSERT_EQ(source->asSweepGradient(), nullptr);
  ASSERT_EQ(source->asRuntimeEffect(), nullptr);
}

TEST(DisplayListColorSource, LinearGradientContents) {
  std::shared_ptr<DlColorSource> source = DlColorSource::MakeLinear(
      kTestPoints1[0], kTestPoints1[1], kTestStopCount, kTestColors, kTestStops,
      DlTileMode::kClamp, &kTestMatrix1);
  ASSERT_EQ(source->asLinearGradient()->start_point(), kTestPoints1[0]);
  ASSERT_EQ(source->asLinearGradient()->end_point(), kTestPoints1[1]);
  ASSERT_EQ(source->asLinearGradient()->stop_count(), kTestStopCount);
  for (int i = 0; i < kTestStopCount; i++) {
    ASSERT_EQ(source->asLinearGradient()->colors()[i], kTestColors[i]);
    ASSERT_EQ(source->asLinearGradient()->stops()[i], kTestStops[i]);
  }
  ASSERT_EQ(source->asLinearGradient()->tile_mode(), DlTileMode::kClamp);
  ASSERT_EQ(source->asLinearGradient()->matrix(), kTestMatrix1);
  ASSERT_EQ(source->is_opaque(), true);
}

TEST(DisplayListColorSource, AlphaLinearGradientContents) {
  std::shared_ptr<DlColorSource> source = DlColorSource::MakeLinear(
      kTestPoints1[0], kTestPoints1[1], kTestStopCount, kTestAlphaColors,
      kTestStops, DlTileMode::kClamp, &kTestMatrix1);
  ASSERT_EQ(source->asLinearGradient()->start_point(), kTestPoints1[0]);
  ASSERT_EQ(source->asLinearGradient()->end_point(), kTestPoints1[1]);
  ASSERT_EQ(source->asLinearGradient()->stop_count(), kTestStopCount);
  for (int i = 0; i < kTestStopCount; i++) {
    ASSERT_EQ(source->asLinearGradient()->colors()[i], kTestAlphaColors[i]);
    ASSERT_EQ(source->asLinearGradient()->stops()[i], kTestStops[i]);
  }
  ASSERT_EQ(source->asLinearGradient()->tile_mode(), DlTileMode::kClamp);
  ASSERT_EQ(source->asLinearGradient()->matrix(), kTestMatrix1);
  ASSERT_EQ(source->is_opaque(), false);
}

TEST(DisplayListColorSource, LinearGradientEquals) {
  std::shared_ptr<DlColorSource> source1 = DlColorSource::MakeLinear(
      kTestPoints1[0], kTestPoints1[1], kTestStopCount, kTestColors, kTestStops,
      DlTileMode::kClamp, &kTestMatrix1);
  std::shared_ptr<DlColorSource> source2 = DlColorSource::MakeLinear(
      kTestPoints1[0], kTestPoints1[1], kTestStopCount, kTestColors, kTestStops,
      DlTileMode::kClamp, &kTestMatrix1);
  TestEquals(*source1, *source2);
}

TEST(DisplayListColorSource, LinearGradientNotEquals) {
  std::shared_ptr<DlColorSource> source1 = DlColorSource::MakeLinear(
      kTestPoints1[0], kTestPoints1[1], kTestStopCount, kTestColors, kTestStops,
      DlTileMode::kClamp, &kTestMatrix1);
  {
    std::shared_ptr<DlColorSource> source2 = DlColorSource::MakeLinear(
        kTestPoints2[0], kTestPoints1[1], kTestStopCount, kTestColors,
        kTestStops, DlTileMode::kClamp, &kTestMatrix1);
    TestNotEquals(*source1, *source2, "Point 0 differs");
  }
  {
    std::shared_ptr<DlColorSource> source2 = DlColorSource::MakeLinear(
        kTestPoints1[0], kTestPoints2[1], kTestStopCount, kTestColors,
        kTestStops, DlTileMode::kClamp, &kTestMatrix1);
    TestNotEquals(*source1, *source2, "Point 1 differs");
  }
  {
    std::shared_ptr<DlColorSource> source2 = DlColorSource::MakeLinear(
        kTestPoints1[0], kTestPoints1[1], 2, kTestColors, kTestStops,  //
        DlTileMode::kClamp, &kTestMatrix1);
    TestNotEquals(*source1, *source2, "Stop count differs");
  }
  {
    std::shared_ptr<DlColorSource> source2 = DlColorSource::MakeLinear(
        kTestPoints1[0], kTestPoints1[1], kTestStopCount, kTestAlphaColors,
        kTestStops, DlTileMode::kClamp, &kTestMatrix1);
    TestNotEquals(*source1, *source2, "Colors differ");
  }
  {
    std::shared_ptr<DlColorSource> source2 = DlColorSource::MakeLinear(
        kTestPoints1[0], kTestPoints1[1], kTestStopCount, kTestColors,
        kTestStops2, DlTileMode::kClamp, &kTestMatrix1);
    TestNotEquals(*source1, *source2, "Stops differ");
  }
  {
    std::shared_ptr<DlColorSource> source2 = DlColorSource::MakeLinear(
        kTestPoints1[0], kTestPoints1[1], kTestStopCount, kTestColors,
        kTestStops, DlTileMode::kMirror, &kTestMatrix1);
    TestNotEquals(*source1, *source2, "Tile Mode differs");
  }
  {
    std::shared_ptr<DlColorSource> source2 = DlColorSource::MakeLinear(
        kTestPoints1[0], kTestPoints1[1], kTestStopCount, kTestColors,
        kTestStops, DlTileMode::kClamp, &kTestMatrix2);
    TestNotEquals(*source1, *source2, "Matrix differs");
  }
}

TEST(DisplayListColorSource, RadialGradientConstructor) {
  std::shared_ptr<DlColorSource> source = DlColorSource::MakeRadial(
      kTestPoints1[0], 10.0, kTestStopCount, kTestColors, kTestStops,
      DlTileMode::kClamp, &kTestMatrix1);
}

TEST(DisplayListColorSource, RadialGradientARGBConstructor) {
  std::array<DlScalar, kTestStopCount * 4> colors;
  for (int i = 0; i < kTestStopCount; ++i) {
    colors[i * 4 + 0] = kTestColors[i].getAlphaF();  //
    colors[i * 4 + 1] = kTestColors[i].getRedF();    //
    colors[i * 4 + 2] = kTestColors[i].getGreenF();  //
    colors[i * 4 + 3] = kTestColors[i].getBlueF();
  }
  std::shared_ptr<DlColorSource> source = DlColorSource::MakeRadial(
      kTestPoints1[0], 10.f, kTestStopCount, colors.data(), kTestStops,
      DlTileMode::kClamp, &kTestMatrix1);
  ASSERT_TRUE(source);
  ASSERT_TRUE(source->asRadialGradient());
  EXPECT_EQ(source->asRadialGradient()->center(), kTestPoints1[0]);
  EXPECT_EQ(source->asRadialGradient()->radius(), 10.f);
  EXPECT_EQ(source->asRadialGradient()->stop_count(), kTestStopCount);
  for (int i = 0; i < kTestStopCount; i++) {
    EXPECT_EQ(source->asRadialGradient()->colors()[i],
              kTestColors[i].withColorSpace(DlColorSpace::kExtendedSRGB));
    EXPECT_EQ(source->asRadialGradient()->stops()[i], kTestStops[i]);
  }
  EXPECT_EQ(source->asRadialGradient()->tile_mode(), DlTileMode::kClamp);
  EXPECT_EQ(source->asRadialGradient()->matrix(), kTestMatrix1);
  EXPECT_EQ(source->is_opaque(), true);
}

TEST(DisplayListColorSource, RadialGradientShared) {
  std::shared_ptr<DlColorSource> source = DlColorSource::MakeRadial(
      kTestPoints1[0], 10.0, kTestStopCount, kTestColors, kTestStops,
      DlTileMode::kClamp, &kTestMatrix1);
  ASSERT_NE(source->shared().get(), source.get());
  ASSERT_EQ(*source->shared().get(), *source.get());
}

TEST(DisplayListColorSource, RadialGradientAsRadial) {
  std::shared_ptr<DlColorSource> source = DlColorSource::MakeRadial(
      kTestPoints1[0], 10.0, kTestStopCount, kTestColors, kTestStops,
      DlTileMode::kClamp, &kTestMatrix1);
  ASSERT_NE(source->asRadialGradient(), nullptr);
  ASSERT_EQ(source->asRadialGradient(), source.get());

  ASSERT_EQ(source->asImage(), nullptr);
  ASSERT_EQ(source->asLinearGradient(), nullptr);
  ASSERT_EQ(source->asConicalGradient(), nullptr);
  ASSERT_EQ(source->asSweepGradient(), nullptr);
  ASSERT_EQ(source->asRuntimeEffect(), nullptr);
}

TEST(DisplayListColorSource, RadialGradientContents) {
  std::shared_ptr<DlColorSource> source = DlColorSource::MakeRadial(
      kTestPoints1[0], 10.0, kTestStopCount, kTestColors, kTestStops,
      DlTileMode::kClamp, &kTestMatrix1);
  ASSERT_EQ(source->asRadialGradient()->center(), kTestPoints1[0]);
  ASSERT_EQ(source->asRadialGradient()->radius(), 10.0);
  ASSERT_EQ(source->asRadialGradient()->stop_count(), kTestStopCount);
  for (int i = 0; i < kTestStopCount; i++) {
    ASSERT_EQ(source->asRadialGradient()->colors()[i], kTestColors[i]);
    ASSERT_EQ(source->asRadialGradient()->stops()[i], kTestStops[i]);
  }
  ASSERT_EQ(source->asRadialGradient()->tile_mode(), DlTileMode::kClamp);
  ASSERT_EQ(source->asRadialGradient()->matrix(), kTestMatrix1);
  ASSERT_EQ(source->is_opaque(), true);
}

TEST(DisplayListColorSource, AlphaRadialGradientContents) {
  std::shared_ptr<DlColorSource> source = DlColorSource::MakeRadial(
      kTestPoints1[0], 10.0, kTestStopCount, kTestAlphaColors, kTestStops,
      DlTileMode::kClamp, &kTestMatrix1);
  ASSERT_EQ(source->asRadialGradient()->center(), kTestPoints1[0]);
  ASSERT_EQ(source->asRadialGradient()->radius(), 10.0);
  ASSERT_EQ(source->asRadialGradient()->stop_count(), kTestStopCount);
  for (int i = 0; i < kTestStopCount; i++) {
    ASSERT_EQ(source->asRadialGradient()->colors()[i], kTestAlphaColors[i]);
    ASSERT_EQ(source->asRadialGradient()->stops()[i], kTestStops[i]);
  }
  ASSERT_EQ(source->asRadialGradient()->tile_mode(), DlTileMode::kClamp);
  ASSERT_EQ(source->asRadialGradient()->matrix(), kTestMatrix1);
  ASSERT_EQ(source->is_opaque(), false);
}

TEST(DisplayListColorSource, RadialGradientEquals) {
  std::shared_ptr<DlColorSource> source1 = DlColorSource::MakeRadial(
      kTestPoints1[0], 10.0, kTestStopCount, kTestColors, kTestStops,
      DlTileMode::kClamp, &kTestMatrix1);
  std::shared_ptr<DlColorSource> source2 = DlColorSource::MakeRadial(
      kTestPoints1[0], 10.0, kTestStopCount, kTestColors, kTestStops,
      DlTileMode::kClamp, &kTestMatrix1);
  TestEquals(*source1, *source2);
}

TEST(DisplayListColorSource, RadialGradientNotEquals) {
  std::shared_ptr<DlColorSource> source1 = DlColorSource::MakeRadial(
      kTestPoints1[0], 10.0, kTestStopCount, kTestColors, kTestStops,
      DlTileMode::kClamp, &kTestMatrix1);
  {
    std::shared_ptr<DlColorSource> source2 = DlColorSource::MakeRadial(
        kTestPoints2[0], 10.0, kTestStopCount, kTestColors, kTestStops,
        DlTileMode::kClamp, &kTestMatrix1);
    TestNotEquals(*source1, *source2, "Center differs");
  }
  {
    std::shared_ptr<DlColorSource> source2 = DlColorSource::MakeRadial(
        kTestPoints1[0], 20.0, kTestStopCount, kTestColors, kTestStops,
        DlTileMode::kClamp, &kTestMatrix1);
    TestNotEquals(*source1, *source2, "Radius differs");
  }
  {
    std::shared_ptr<DlColorSource> source2 = DlColorSource::MakeRadial(
        kTestPoints1[0], 10.0, 2, kTestColors, kTestStops,  //
        DlTileMode::kClamp, &kTestMatrix1);
    TestNotEquals(*source1, *source2, "Stop count differs");
  }
  {
    std::shared_ptr<DlColorSource> source2 = DlColorSource::MakeRadial(
        kTestPoints1[0], 10.0, kTestStopCount, kTestAlphaColors, kTestStops,
        DlTileMode::kClamp, &kTestMatrix1);
    TestNotEquals(*source1, *source2, "Colors differ");
  }
  {
    std::shared_ptr<DlColorSource> source2 = DlColorSource::MakeRadial(
        kTestPoints1[0], 10.0, kTestStopCount, kTestColors, kTestStops2,
        DlTileMode::kClamp, &kTestMatrix1);
    TestNotEquals(*source1, *source2, "Stops differ");
  }
  {
    std::shared_ptr<DlColorSource> source2 = DlColorSource::MakeRadial(
        kTestPoints1[0], 10.0, kTestStopCount, kTestColors, kTestStops,
        DlTileMode::kMirror, &kTestMatrix1);
    TestNotEquals(*source1, *source2, "Tile Mode differs");
  }
  {
    std::shared_ptr<DlColorSource> source2 = DlColorSource::MakeRadial(
        kTestPoints1[0], 10.0, kTestStopCount, kTestColors, kTestStops,
        DlTileMode::kClamp, &kTestMatrix2);
    TestNotEquals(*source1, *source2, "Matrix differs");
  }
}

TEST(DisplayListColorSource, ConicalGradientConstructor) {
  std::shared_ptr<DlColorSource> source = DlColorSource::MakeConical(
      kTestPoints1[0], 10.0, kTestPoints1[1], 20.0, kTestStopCount, kTestColors,
      kTestStops, DlTileMode::kClamp, &kTestMatrix1);
}

TEST(DisplayListColorSource, ConicalGradientARGBConstructor) {
  std::array<DlScalar, kTestStopCount * 4> colors;
  for (int i = 0; i < kTestStopCount; ++i) {
    colors[i * 4 + 0] = kTestColors[i].getAlphaF();  //
    colors[i * 4 + 1] = kTestColors[i].getRedF();    //
    colors[i * 4 + 2] = kTestColors[i].getGreenF();  //
    colors[i * 4 + 3] = kTestColors[i].getBlueF();
  }
  std::shared_ptr<DlColorSource> source = DlColorSource::MakeConical(
      kTestPoints1[0], 10.f, kTestPoints1[1], 20.f, kTestStopCount,
      colors.data(), kTestStops, DlTileMode::kClamp, &kTestMatrix1);
  ASSERT_TRUE(source);
  ASSERT_TRUE(source->asConicalGradient());
  EXPECT_EQ(source->asConicalGradient()->start_center(), kTestPoints1[0]);
  EXPECT_EQ(source->asConicalGradient()->start_radius(), 10.f);
  EXPECT_EQ(source->asConicalGradient()->end_center(), kTestPoints1[1]);
  EXPECT_EQ(source->asConicalGradient()->end_radius(), 20.f);
  EXPECT_EQ(source->asConicalGradient()->stop_count(), kTestStopCount);
  for (int i = 0; i < kTestStopCount; i++) {
    EXPECT_EQ(source->asConicalGradient()->colors()[i],
              kTestColors[i].withColorSpace(DlColorSpace::kExtendedSRGB));
    EXPECT_EQ(source->asConicalGradient()->stops()[i], kTestStops[i]);
  }
  EXPECT_EQ(source->asConicalGradient()->tile_mode(), DlTileMode::kClamp);
  EXPECT_EQ(source->asConicalGradient()->matrix(), kTestMatrix1);
  EXPECT_EQ(source->is_opaque(), true);
}

TEST(DisplayListColorSource, ConicalGradientShared) {
  std::shared_ptr<DlColorSource> source = DlColorSource::MakeConical(
      kTestPoints1[0], 10.0, kTestPoints1[1], 20.0, kTestStopCount, kTestColors,
      kTestStops, DlTileMode::kClamp, &kTestMatrix1);
  ASSERT_NE(source->shared().get(), source.get());
  ASSERT_EQ(*source->shared().get(), *source.get());
}

TEST(DisplayListColorSource, ConicalGradientAsConical) {
  std::shared_ptr<DlColorSource> source = DlColorSource::MakeConical(
      kTestPoints1[0], 10.0, kTestPoints1[1], 20.0, kTestStopCount, kTestColors,
      kTestStops, DlTileMode::kClamp, &kTestMatrix1);
  ASSERT_NE(source->asConicalGradient(), nullptr);
  ASSERT_EQ(source->asConicalGradient(), source.get());

  ASSERT_EQ(source->asImage(), nullptr);
  ASSERT_EQ(source->asLinearGradient(), nullptr);
  ASSERT_EQ(source->asRadialGradient(), nullptr);
  ASSERT_EQ(source->asSweepGradient(), nullptr);
  ASSERT_EQ(source->asRuntimeEffect(), nullptr);
}

TEST(DisplayListColorSource, ConicalGradientContents) {
  std::shared_ptr<DlColorSource> source = DlColorSource::MakeConical(
      kTestPoints1[0], 10.0, kTestPoints1[1], 20.0, kTestStopCount, kTestColors,
      kTestStops, DlTileMode::kClamp, &kTestMatrix1);
  ASSERT_EQ(source->asConicalGradient()->start_center(), kTestPoints1[0]);
  ASSERT_EQ(source->asConicalGradient()->start_radius(), 10.0);
  ASSERT_EQ(source->asConicalGradient()->end_center(), kTestPoints1[1]);
  ASSERT_EQ(source->asConicalGradient()->end_radius(), 20.0);
  ASSERT_EQ(source->asConicalGradient()->stop_count(), kTestStopCount);
  for (int i = 0; i < kTestStopCount; i++) {
    ASSERT_EQ(source->asConicalGradient()->colors()[i], kTestColors[i]);
    ASSERT_EQ(source->asConicalGradient()->stops()[i], kTestStops[i]);
  }
  ASSERT_EQ(source->asConicalGradient()->tile_mode(), DlTileMode::kClamp);
  ASSERT_EQ(source->asConicalGradient()->matrix(), kTestMatrix1);
  ASSERT_EQ(source->is_opaque(), true);
}

TEST(DisplayListColorSource, AlphaConicalGradientContents) {
  std::shared_ptr<DlColorSource> source = DlColorSource::MakeConical(
      kTestPoints1[0], 10.0, kTestPoints1[1], 20.0, kTestStopCount,
      kTestAlphaColors, kTestStops, DlTileMode::kClamp, &kTestMatrix1);
  ASSERT_EQ(source->asConicalGradient()->start_center(), kTestPoints1[0]);
  ASSERT_EQ(source->asConicalGradient()->start_radius(), 10.0);
  ASSERT_EQ(source->asConicalGradient()->end_center(), kTestPoints1[1]);
  ASSERT_EQ(source->asConicalGradient()->end_radius(), 20.0);
  ASSERT_EQ(source->asConicalGradient()->stop_count(), kTestStopCount);
  for (int i = 0; i < kTestStopCount; i++) {
    ASSERT_EQ(source->asConicalGradient()->colors()[i], kTestAlphaColors[i]);
    ASSERT_EQ(source->asConicalGradient()->stops()[i], kTestStops[i]);
  }
  ASSERT_EQ(source->asConicalGradient()->tile_mode(), DlTileMode::kClamp);
  ASSERT_EQ(source->asConicalGradient()->matrix(), kTestMatrix1);
  ASSERT_EQ(source->is_opaque(), false);
}

TEST(DisplayListColorSource, ConicalGradientEquals) {
  std::shared_ptr<DlColorSource> source1 = DlColorSource::MakeConical(
      kTestPoints1[0], 10.0, kTestPoints1[1], 20.0, kTestStopCount, kTestColors,
      kTestStops, DlTileMode::kClamp, &kTestMatrix1);
  std::shared_ptr<DlColorSource> source2 = DlColorSource::MakeConical(
      kTestPoints1[0], 10.0, kTestPoints1[1], 20.0, kTestStopCount, kTestColors,
      kTestStops, DlTileMode::kClamp, &kTestMatrix1);
  TestEquals(*source1, *source2);
}

TEST(DisplayListColorSource, ConicalGradientNotEquals) {
  std::shared_ptr<DlColorSource> source1 = DlColorSource::MakeConical(
      kTestPoints1[0], 10.0, kTestPoints1[1], 20.0, kTestStopCount, kTestColors,
      kTestStops, DlTileMode::kClamp, &kTestMatrix1);
  {
    std::shared_ptr<DlColorSource> source2 = DlColorSource::MakeConical(
        kTestPoints2[0], 10.0, kTestPoints1[1], 20.0, kTestStopCount,
        kTestColors, kTestStops, DlTileMode::kClamp, &kTestMatrix1);
    TestNotEquals(*source1, *source2, "Start Center differs");
  }
  {
    std::shared_ptr<DlColorSource> source2 = DlColorSource::MakeConical(
        kTestPoints1[0], 15.0, kTestPoints1[1], 20.0, kTestStopCount,
        kTestColors, kTestStops, DlTileMode::kClamp, &kTestMatrix1);
    TestNotEquals(*source1, *source2, "Start Radius differs");
  }
  {
    std::shared_ptr<DlColorSource> source2 = DlColorSource::MakeConical(
        kTestPoints1[0], 10.0, kTestPoints2[1], 20.0, kTestStopCount,
        kTestColors, kTestStops, DlTileMode::kClamp, &kTestMatrix1);
    TestNotEquals(*source1, *source2, "End Center differs");
  }
  {
    std::shared_ptr<DlColorSource> source2 = DlColorSource::MakeConical(
        kTestPoints1[0], 10.0, kTestPoints1[1], 25.0, kTestStopCount,
        kTestColors, kTestStops, DlTileMode::kClamp, &kTestMatrix1);
    TestNotEquals(*source1, *source2, "End Radius differs");
  }
  {
    std::shared_ptr<DlColorSource> source2 = DlColorSource::MakeConical(
        kTestPoints1[0], 10.0, kTestPoints1[1], 20.0, 2, kTestColors,
        kTestStops, DlTileMode::kClamp, &kTestMatrix1);
    TestNotEquals(*source1, *source2, "Stop count differs");
  }
  {
    std::shared_ptr<DlColorSource> source2 = DlColorSource::MakeConical(
        kTestPoints1[0], 10.0, kTestPoints1[1], 20.0, kTestStopCount,
        kTestAlphaColors, kTestStops, DlTileMode::kClamp, &kTestMatrix1);
    TestNotEquals(*source1, *source2, "Colors differ");
  }
  {
    std::shared_ptr<DlColorSource> source2 = DlColorSource::MakeConical(
        kTestPoints1[0], 10.0, kTestPoints1[1], 20.0, kTestStopCount,
        kTestColors, kTestStops2, DlTileMode::kClamp, &kTestMatrix1);
    TestNotEquals(*source1, *source2, "Stops differ");
  }
  {
    std::shared_ptr<DlColorSource> source2 = DlColorSource::MakeConical(
        kTestPoints1[0], 10.0, kTestPoints1[1], 20.0, kTestStopCount,
        kTestColors, kTestStops, DlTileMode::kMirror, &kTestMatrix1);
    TestNotEquals(*source1, *source2, "Tile Mode differs");
  }
  {
    std::shared_ptr<DlColorSource> source2 = DlColorSource::MakeConical(
        kTestPoints1[0], 10.0, kTestPoints1[1], 20.0, kTestStopCount,
        kTestColors, kTestStops, DlTileMode::kClamp, &kTestMatrix2);
    TestNotEquals(*source1, *source2, "Matrix differs");
  }
}

TEST(DisplayListColorSource, SweepGradientConstructor) {
  std::shared_ptr<DlColorSource> source = DlColorSource::MakeSweep(
      kTestPoints1[0], 10.0, 20.0, kTestStopCount, kTestColors, kTestStops,
      DlTileMode::kClamp, &kTestMatrix1);
}

TEST(DisplayListColorSource, SweepGradientARGBConstructor) {
  std::array<DlScalar, kTestStopCount * 4> colors;
  for (int i = 0; i < kTestStopCount; ++i) {
    colors[i * 4 + 0] = kTestColors[i].getAlphaF();  //
    colors[i * 4 + 1] = kTestColors[i].getRedF();    //
    colors[i * 4 + 2] = kTestColors[i].getGreenF();  //
    colors[i * 4 + 3] = kTestColors[i].getBlueF();
  }
  std::shared_ptr<DlColorSource> source = DlColorSource::MakeSweep(
      kTestPoints1[0], 10.f, 20.f, kTestStopCount, colors.data(), kTestStops,
      DlTileMode::kClamp, &kTestMatrix1);
  ASSERT_TRUE(source);
  ASSERT_TRUE(source->asSweepGradient());
  EXPECT_EQ(source->asSweepGradient()->center(), kTestPoints1[0]);
  EXPECT_EQ(source->asSweepGradient()->start(), 10.f);
  EXPECT_EQ(source->asSweepGradient()->end(), 20.f);
  EXPECT_EQ(source->asSweepGradient()->stop_count(), kTestStopCount);
  for (int i = 0; i < kTestStopCount; i++) {
    EXPECT_EQ(source->asSweepGradient()->colors()[i],
              kTestColors[i].withColorSpace(DlColorSpace::kExtendedSRGB));
    EXPECT_EQ(source->asSweepGradient()->stops()[i], kTestStops[i]);
  }
  EXPECT_EQ(source->asSweepGradient()->tile_mode(), DlTileMode::kClamp);
  EXPECT_EQ(source->asSweepGradient()->matrix(), kTestMatrix1);
  EXPECT_EQ(source->is_opaque(), true);
}

TEST(DisplayListColorSource, SweepGradientShared) {
  std::shared_ptr<DlColorSource> source = DlColorSource::MakeSweep(
      kTestPoints1[0], 10.0, 20.0, kTestStopCount, kTestColors, kTestStops,
      DlTileMode::kClamp, &kTestMatrix1);
  ASSERT_NE(source->shared().get(), source.get());
  ASSERT_EQ(*source->shared().get(), *source.get());
}

TEST(DisplayListColorSource, SweepGradientAsSweep) {
  std::shared_ptr<DlColorSource> source = DlColorSource::MakeSweep(
      kTestPoints1[0], 10.0, 20.0, kTestStopCount, kTestColors, kTestStops,
      DlTileMode::kClamp, &kTestMatrix1);
  ASSERT_NE(source->asSweepGradient(), nullptr);
  ASSERT_EQ(source->asSweepGradient(), source.get());

  ASSERT_EQ(source->asImage(), nullptr);
  ASSERT_EQ(source->asLinearGradient(), nullptr);
  ASSERT_EQ(source->asRadialGradient(), nullptr);
  ASSERT_EQ(source->asConicalGradient(), nullptr);
  ASSERT_EQ(source->asRuntimeEffect(), nullptr);
}

TEST(DisplayListColorSource, SweepGradientContents) {
  std::shared_ptr<DlColorSource> source = DlColorSource::MakeSweep(
      kTestPoints1[0], 10.0, 20.0, kTestStopCount, kTestColors, kTestStops,
      DlTileMode::kClamp, &kTestMatrix1);
  ASSERT_EQ(source->asSweepGradient()->center(), kTestPoints1[0]);
  ASSERT_EQ(source->asSweepGradient()->start(), 10.0);
  ASSERT_EQ(source->asSweepGradient()->end(), 20.0);
  ASSERT_EQ(source->asSweepGradient()->stop_count(), kTestStopCount);
  for (int i = 0; i < kTestStopCount; i++) {
    ASSERT_EQ(source->asSweepGradient()->colors()[i], kTestColors[i]);
    ASSERT_EQ(source->asSweepGradient()->stops()[i], kTestStops[i]);
  }
  ASSERT_EQ(source->asSweepGradient()->tile_mode(), DlTileMode::kClamp);
  ASSERT_EQ(source->asSweepGradient()->matrix(), kTestMatrix1);
  ASSERT_EQ(source->is_opaque(), true);
}

TEST(DisplayListColorSource, AlphaSweepGradientContents) {
  std::shared_ptr<DlColorSource> source = DlColorSource::MakeSweep(
      kTestPoints1[0], 10.0, 20.0, kTestStopCount, kTestAlphaColors, kTestStops,
      DlTileMode::kClamp, &kTestMatrix1);
  ASSERT_EQ(source->asSweepGradient()->center(), kTestPoints1[0]);
  ASSERT_EQ(source->asSweepGradient()->start(), 10.0);
  ASSERT_EQ(source->asSweepGradient()->end(), 20.0);
  ASSERT_EQ(source->asSweepGradient()->stop_count(), kTestStopCount);
  for (int i = 0; i < kTestStopCount; i++) {
    ASSERT_EQ(source->asSweepGradient()->colors()[i], kTestAlphaColors[i]);
    ASSERT_EQ(source->asSweepGradient()->stops()[i], kTestStops[i]);
  }
  ASSERT_EQ(source->asSweepGradient()->tile_mode(), DlTileMode::kClamp);
  ASSERT_EQ(source->asSweepGradient()->matrix(), kTestMatrix1);
  ASSERT_EQ(source->is_opaque(), false);
}

TEST(DisplayListColorSource, SweepGradientEquals) {
  std::shared_ptr<DlColorSource> source1 = DlColorSource::MakeSweep(
      kTestPoints1[0], 10.0, 20.0, kTestStopCount, kTestColors, kTestStops,
      DlTileMode::kClamp, &kTestMatrix1);
  std::shared_ptr<DlColorSource> source2 = DlColorSource::MakeSweep(
      kTestPoints1[0], 10.0, 20.0, kTestStopCount, kTestColors, kTestStops,
      DlTileMode::kClamp, &kTestMatrix1);
  TestEquals(*source1, *source2);
}

TEST(DisplayListColorSource, SweepGradientNotEquals) {
  std::shared_ptr<DlColorSource> source1 = DlColorSource::MakeSweep(
      kTestPoints1[0], 10.0, 20.0, kTestStopCount, kTestColors, kTestStops,
      DlTileMode::kClamp, &kTestMatrix1);
  {
    std::shared_ptr<DlColorSource> source2 = DlColorSource::MakeSweep(
        kTestPoints2[0], 10.0, 20.0, kTestStopCount, kTestColors, kTestStops,
        DlTileMode::kClamp, &kTestMatrix1);
    TestNotEquals(*source1, *source2, "Center differs");
  }
  {
    std::shared_ptr<DlColorSource> source2 = DlColorSource::MakeSweep(
        kTestPoints1[0], 15.0, 20.0, kTestStopCount, kTestColors, kTestStops,
        DlTileMode::kClamp, &kTestMatrix1);
    TestNotEquals(*source1, *source2, "Start Angle differs");
  }
  {
    std::shared_ptr<DlColorSource> source2 = DlColorSource::MakeSweep(
        kTestPoints1[0], 10.0, 25.0, kTestStopCount, kTestColors, kTestStops,
        DlTileMode::kClamp, &kTestMatrix1);
    TestNotEquals(*source1, *source2, "End Angle differs");
  }
  {
    std::shared_ptr<DlColorSource> source2 = DlColorSource::MakeSweep(
        kTestPoints1[0], 10.0, 20.0, 2, kTestColors, kTestStops,  //
        DlTileMode::kClamp, &kTestMatrix1);
    TestNotEquals(*source1, *source2, "Stop count differs");
  }
  {
    std::shared_ptr<DlColorSource> source2 = DlColorSource::MakeSweep(
        kTestPoints1[0], 10.0, 20.0, kTestStopCount, kTestAlphaColors,
        kTestStops, DlTileMode::kClamp, &kTestMatrix1);
    TestNotEquals(*source1, *source2, "Colors differ");
  }
  {
    std::shared_ptr<DlColorSource> source2 = DlColorSource::MakeSweep(
        kTestPoints1[0], 10.0, 20.0, kTestStopCount, kTestColors, kTestStops2,
        DlTileMode::kClamp, &kTestMatrix1);
    TestNotEquals(*source1, *source2, "Stops differ");
  }
  {
    std::shared_ptr<DlColorSource> source2 = DlColorSource::MakeSweep(
        kTestPoints1[0], 10.0, 20.0, kTestStopCount, kTestColors, kTestStops,
        DlTileMode::kMirror, &kTestMatrix1);
    TestNotEquals(*source1, *source2, "Tile Mode differs");
  }
  {
    std::shared_ptr<DlColorSource> source2 = DlColorSource::MakeSweep(
        kTestPoints1[0], 10.0, 20.0, kTestStopCount, kTestColors, kTestStops,
        DlTileMode::kClamp, &kTestMatrix2);
    TestNotEquals(*source1, *source2, "Matrix differs");
  }
}

TEST(DisplayListColorSource, RuntimeEffect) {
  std::shared_ptr<DlColorSource> source1 = DlColorSource::MakeRuntimeEffect(
      kTestRuntimeEffect1, {}, std::make_shared<std::vector<uint8_t>>());
  std::shared_ptr<DlColorSource> source2 = DlColorSource::MakeRuntimeEffect(
      kTestRuntimeEffect2, {}, std::make_shared<std::vector<uint8_t>>());
  std::shared_ptr<DlColorSource> source3 = DlColorSource::MakeRuntimeEffect(
      nullptr, {}, std::make_shared<std::vector<uint8_t>>());

  ASSERT_EQ(source1->type(), DlColorSourceType::kRuntimeEffect);
  ASSERT_EQ(source1->asRuntimeEffect(), source1.get());
  ASSERT_NE(source2->asRuntimeEffect(), source1.get());

  ASSERT_EQ(source1->asImage(), nullptr);
  ASSERT_EQ(source1->asLinearGradient(), nullptr);
  ASSERT_EQ(source1->asRadialGradient(), nullptr);
  ASSERT_EQ(source1->asConicalGradient(), nullptr);
  ASSERT_EQ(source1->asSweepGradient(), nullptr);

  TestEquals(source1, source1);
  TestEquals(source3, source3);
  TestNotEquals(source1, source2, "RuntimeEffect differs");
  TestNotEquals(source2, source3, "RuntimeEffect differs");
}

}  // namespace testing
}  // namespace flutter
