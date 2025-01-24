// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/display_list/dl_color.h"
#include "flutter/testing/display_list_testing.h"
#include "flutter/testing/testing.h"

namespace flutter {
namespace testing {

static void arraysEqual(const uint32_t* ints,
                        const DlColor* colors,
                        int count) {
  for (int i = 0; i < count; i++) {
    EXPECT_EQ(ints[i], colors[i].argb()) << " index:" << i;
  }
}

TEST(DisplayListColor, DefaultValue) {
  DlColor color;
  EXPECT_EQ(color.getAlphaF(), 1.f);
  EXPECT_EQ(color.getRedF(), 0.f);
  EXPECT_EQ(color.getGreenF(), 0.f);
  EXPECT_EQ(color.getBlueF(), 0.f);
  EXPECT_EQ(color.getColorSpace(), DlColorSpace::kSRGB);
}

TEST(DisplayListColor, ArrayInterchangeableWithUint32) {
  uint32_t ints[5] = {
      0xFF000000,  //
      0xFFFF0000,  //
      0xFF00FF00,  //
      0xFF0000FF,  //
      0xF1F2F3F4,
  };
  DlColor colors[5] = {
      DlColor::kBlack(),  //
      DlColor::kRed(),    //
      DlColor::kGreen(),  //
      DlColor::kBlue(),   //
      DlColor(0xF1F2F3F4),
  };
  arraysEqual(ints, colors, 5);
}

TEST(DisplayListColor, DlColorFloatConstructor) {
  EXPECT_EQ(DlColor::ARGB(1.0f, 1.0f, 1.0f, 1.0f), DlColor(0xFFFFFFFF));
  EXPECT_EQ(DlColor::ARGB(0.0f, 0.0f, 0.0f, 0.0f), DlColor(0x00000000));
  EXPECT_TRUE(
      DlColor::ARGB(0.5f, 0.5f, 0.5f, 0.5f).isClose(DlColor(0x80808080)));
  EXPECT_TRUE(
      DlColor::ARGB(1.0f, 0.0f, 0.5f, 1.0f).isClose(DlColor(0xFF0080FF)));

  EXPECT_EQ(DlColor::RGBA(1.0f, 1.0f, 1.0f, 1.0f), DlColor(0xFFFFFFFF));
  EXPECT_EQ(DlColor::RGBA(0.0f, 0.0f, 0.0f, 0.0f), DlColor(0x00000000));
  EXPECT_TRUE(
      DlColor::RGBA(0.5f, 0.5f, 0.5f, 0.5f).isClose(DlColor(0x80808080)));
  EXPECT_TRUE(
      DlColor::RGBA(1.0f, 0.0f, 0.5f, 1.0f).isClose(DlColor(0xFFFF0080)));
}

TEST(DisplayListColor, DlColorComponentGetters) {
  {
    DlColor test(0xFFFFFFFF);

    EXPECT_EQ(test.getAlpha(), 0xFF);
    EXPECT_EQ(test.getRed(), 0xFF);
    EXPECT_EQ(test.getGreen(), 0xFF);
    EXPECT_EQ(test.getBlue(), 0xFF);

    EXPECT_EQ(test.getAlphaF(), 1.0f);
    EXPECT_EQ(test.getRedF(), 1.0f);
    EXPECT_EQ(test.getGreenF(), 1.0f);
    EXPECT_EQ(test.getBlueF(), 1.0f);
  }

  {
    DlColor test = DlColor::ARGB(1.0f, 1.0f, 1.0f, 1.0f);

    EXPECT_EQ(test.getAlpha(), 0xFF);
    EXPECT_EQ(test.getRed(), 0xFF);
    EXPECT_EQ(test.getGreen(), 0xFF);
    EXPECT_EQ(test.getBlue(), 0xFF);

    EXPECT_EQ(test.getAlphaF(), 1.0f);
    EXPECT_EQ(test.getRedF(), 1.0f);
    EXPECT_EQ(test.getGreenF(), 1.0f);
    EXPECT_EQ(test.getBlueF(), 1.0f);
  }

  {
    DlColor test(0x00000000);

    EXPECT_EQ(test.getAlpha(), 0x00);
    EXPECT_EQ(test.getRed(), 0x00);
    EXPECT_EQ(test.getGreen(), 0x00);
    EXPECT_EQ(test.getBlue(), 0x00);

    EXPECT_EQ(test.getAlphaF(), 0.0f);
    EXPECT_EQ(test.getRedF(), 0.0f);
    EXPECT_EQ(test.getGreenF(), 0.0f);
    EXPECT_EQ(test.getBlueF(), 0.0f);
  }

  {
    DlColor test = DlColor::ARGB(0.0f, 0.0f, 0.0f, 0.0f);

    EXPECT_EQ(test.getAlpha(), 0x00);
    EXPECT_EQ(test.getRed(), 0x00);
    EXPECT_EQ(test.getGreen(), 0x00);
    EXPECT_EQ(test.getBlue(), 0x00);

    EXPECT_EQ(test.getAlphaF(), 0.0f);
    EXPECT_EQ(test.getRedF(), 0.0f);
    EXPECT_EQ(test.getGreenF(), 0.0f);
    EXPECT_EQ(test.getBlueF(), 0.0f);
  }

  {
    DlColor test(0x7F7F7F7F);

    EXPECT_EQ(test.getAlpha(), 0x7F);
    EXPECT_EQ(test.getRed(), 0x7F);
    EXPECT_EQ(test.getGreen(), 0x7F);
    EXPECT_EQ(test.getBlue(), 0x7F);

    const DlScalar half = 127.0f * (1.0f / 255.0f);

    EXPECT_NEAR(test.getAlphaF(), half, 0.00001);
    EXPECT_NEAR(test.getRedF(), half, 0.00001);
    EXPECT_NEAR(test.getGreenF(), half, 0.00001);
    EXPECT_NEAR(test.getBlueF(), half, 0.00001);
  }

  {
    DlColor test = DlColor::ARGB(0.5f, 0.5f, 0.5f, 0.5f);

    EXPECT_EQ(test.getAlpha(), 0x80);
    EXPECT_EQ(test.getRed(), 0x80);
    EXPECT_EQ(test.getGreen(), 0x80);
    EXPECT_EQ(test.getBlue(), 0x80);

    EXPECT_EQ(test.getAlphaF(), 0.5);
    EXPECT_EQ(test.getRedF(), 0.5);
    EXPECT_EQ(test.getGreenF(), 0.5);
    EXPECT_EQ(test.getBlueF(), 0.5);
  }

  {
    DlColor test(0x1F2F3F4F);

    EXPECT_EQ(test.getAlpha(), 0x1F);
    EXPECT_EQ(test.getRed(), 0x2F);
    EXPECT_EQ(test.getGreen(), 0x3F);
    EXPECT_EQ(test.getBlue(), 0x4F);

    EXPECT_NEAR(test.getAlphaF(), 0x1f * (1.0f / 255.0f), 0.00001);
    EXPECT_NEAR(test.getRedF(), 0x2f * (1.0f / 255.0f), 0.00001);
    EXPECT_NEAR(test.getGreenF(), 0x3f * (1.0f / 255.0f), 0.00001);
    EXPECT_NEAR(test.getBlueF(), 0x4f * (1.0f / 255.0f), 0.00001);
  }

  {
    DlColor test = DlColor::ARGB(0.1f, 0.2f, 0.3f, 0.4f);

    EXPECT_EQ(test.getAlpha(), round(0.1f * 255));
    EXPECT_EQ(test.getRed(), round(0.2f * 255));
    EXPECT_EQ(test.getGreen(), round(0.3f * 255));
    EXPECT_EQ(test.getBlue(), round(0.4f * 255));

    EXPECT_EQ(test.getAlphaF(), 0.1f);
    EXPECT_EQ(test.getRedF(), 0.2f);
    EXPECT_EQ(test.getGreenF(), 0.3f);
    EXPECT_EQ(test.getBlueF(), 0.4f);
  }

  {
    DlColor test = DlColor::RGBA(0.2f, 0.3f, 0.4f, 0.1f);

    EXPECT_EQ(test.getAlpha(), round(0.1f * 255));
    EXPECT_EQ(test.getRed(), round(0.2f * 255));
    EXPECT_EQ(test.getGreen(), round(0.3f * 255));
    EXPECT_EQ(test.getBlue(), round(0.4f * 255));

    EXPECT_EQ(test.getAlphaF(), 0.1f);
    EXPECT_EQ(test.getRedF(), 0.2f);
    EXPECT_EQ(test.getGreenF(), 0.3f);
    EXPECT_EQ(test.getBlueF(), 0.4f);
  }
}

TEST(DisplayListColor, DlColorOpaqueTransparent) {
  auto test_argb = [](int a, int r, int g, int b) {
    ASSERT_TRUE(a >= 0 && a <= 255);
    ASSERT_TRUE(r >= 0 && r <= 255);
    ASSERT_TRUE(g >= 0 && g <= 255);
    ASSERT_TRUE(b >= 0 && b <= 255);

    int argb = ((a << 24) | (r << 16) | (g << 8) | b);
    bool is_opaque = (a == 255);
    bool is_transprent = (a == 0);

    EXPECT_EQ(DlColor(argb).isOpaque(), is_opaque);
    EXPECT_EQ(DlColor(argb).isTransparent(), is_transprent);

    DlScalar aF = a * (1.0f / 255.0f);
    DlScalar rF = r * (1.0f / 255.0f);
    DlScalar gF = g * (1.0f / 255.0f);
    DlScalar bF = b * (1.0f / 255.0f);

    EXPECT_EQ(DlColor::ARGB(aF, rF, gF, bF).isOpaque(), is_opaque);
    EXPECT_EQ(DlColor::ARGB(aF, rF, gF, bF).isTransparent(), is_transprent);

    EXPECT_EQ(DlColor::RGBA(rF, gF, bF, aF).isOpaque(), is_opaque);
    EXPECT_EQ(DlColor::RGBA(rF, gF, bF, aF).isTransparent(), is_transprent);
  };

  for (int r = 0; r <= 255; r += 15) {
    for (int g = 0; g <= 255; g += 15) {
      for (int b = 0; b <= 255; b += 15) {
        test_argb(0, r, g, b);
        for (int a = 15; a < 255; a += 15) {
          test_argb(a, r, g, b);
        }
        test_argb(255, r, g, b);
      }
    }
  }
}

TEST(DisplayListColor, EqualityWithColorspace) {
  EXPECT_TRUE(DlColor(0.9, 0.8, 0.7, 0.6, DlColorSpace::kSRGB) ==
              DlColor(0.9, 0.8, 0.7, 0.6, DlColorSpace::kSRGB));
  EXPECT_FALSE(DlColor(0.9, 0.8, 0.7, 0.6, DlColorSpace::kSRGB) ==
               DlColor(0.9, 0.8, 0.7, 0.6, DlColorSpace::kExtendedSRGB));
  EXPECT_FALSE(DlColor(0.9, 0.8, 0.7, 0.6, DlColorSpace::kSRGB) !=
               DlColor(0.9, 0.8, 0.7, 0.6, DlColorSpace::kSRGB));
  EXPECT_TRUE(DlColor(0.9, 0.8, 0.7, 0.6, DlColorSpace::kSRGB) !=
              DlColor(0.9, 0.8, 0.7, 0.6, DlColorSpace::kExtendedSRGB));
}

TEST(DisplayListColor, EqualityWithExtendedSRGB) {
  EXPECT_TRUE(DlColor(1.0, 1.1, -0.2, 0.1, DlColorSpace::kExtendedSRGB) ==
              DlColor(1.0, 1.1, -0.2, 0.1, DlColorSpace::kExtendedSRGB));
  EXPECT_FALSE(DlColor(1.0, 1.1, -0.2, 0.1, DlColorSpace::kExtendedSRGB) ==
               DlColor(1.0, 1.0, 0.0, 0.0, DlColorSpace::kExtendedSRGB));
}

TEST(DisplayListColor, ColorSpaceSRGBtoSRGB) {
  DlColor srgb(0.9, 0.8, 0.7, 0.6, DlColorSpace::kSRGB);
  EXPECT_EQ(srgb, srgb.withColorSpace(DlColorSpace::kSRGB));
}

TEST(DisplayListColor, ColorSpaceSRGBtoExtendedSRGB) {
  DlColor srgb(0.9, 0.8, 0.7, 0.6, DlColorSpace::kSRGB);
  EXPECT_EQ(DlColor(0.9, 0.8, 0.7, 0.6, DlColorSpace::kExtendedSRGB),
            srgb.withColorSpace(DlColorSpace::kExtendedSRGB));
}

TEST(DisplayListColor, ColorSpaceExtendedSRGBtoExtendedSRGB) {
  DlColor xsrgb(0.9, 0.8, 0.7, 0.6, DlColorSpace::kExtendedSRGB);
  EXPECT_EQ(DlColor(0.9, 0.8, 0.7, 0.6, DlColorSpace::kExtendedSRGB),
            xsrgb.withColorSpace(DlColorSpace::kExtendedSRGB));
}

TEST(DisplayListColor, ColorSpaceP3ToP3) {
  DlColor p3(0.9, 0.8, 0.7, 0.6, DlColorSpace::kDisplayP3);
  EXPECT_EQ(DlColor(0.9, 0.8, 0.7, 0.6, DlColorSpace::kDisplayP3),
            p3.withColorSpace(DlColorSpace::kDisplayP3));
}

TEST(DisplayListColor, ColorSpaceP3ToExtendedSRGB) {
  DlColor red(0.9, 1.0, 0.0, 0.0, DlColorSpace::kDisplayP3);
  EXPECT_TRUE(
      DlColor(0.9, 1.0931, -0.2268, -0.1501, DlColorSpace::kExtendedSRGB)
          .isClose(red.withColorSpace(DlColorSpace::kExtendedSRGB)))
      << red.withColorSpace(DlColorSpace::kExtendedSRGB);

  DlColor green(0.9, 0.0, 1.0, 0.0, DlColorSpace::kDisplayP3);
  EXPECT_TRUE(
      DlColor(0.9, -0.5116, 1.0183, -0.3106, DlColorSpace::kExtendedSRGB)
          .isClose(green.withColorSpace(DlColorSpace::kExtendedSRGB)))
      << green.withColorSpace(DlColorSpace::kExtendedSRGB);

  DlColor blue(0.9, 0.0, 0.0, 1.0, DlColorSpace::kDisplayP3);
  EXPECT_TRUE(DlColor(0.9, -0.0004, 0.0003, 1.0420, DlColorSpace::kExtendedSRGB)
                  .isClose(blue.withColorSpace(DlColorSpace::kExtendedSRGB)))
      << blue.withColorSpace(DlColorSpace::kExtendedSRGB);
}

TEST(DisplayListColor, isClose) {
  EXPECT_TRUE(DlColor(0xffaabbcc).isClose(DlColor(0xffaabbcc)));
}

TEST(DisplayListColor, isNotClose) {
  EXPECT_FALSE(DlColor(0xffaabbcc).isClose(DlColor(0xffaabbcd)));
}

}  // namespace testing
}  // namespace flutter
