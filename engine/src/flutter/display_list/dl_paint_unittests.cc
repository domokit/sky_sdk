// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/display_list/dl_paint.h"

#include "flutter/display_list/testing/dl_test_equality.h"
#include "flutter/display_list/utils/dl_comparable.h"
#include "gtest/gtest.h"

namespace flutter {
namespace testing {

TEST(DisplayListPaint, ConstructorDefaults) {
  DlPaint paint;
  EXPECT_FALSE(paint.isAntiAlias());
  EXPECT_FALSE(paint.isInvertColors());
  EXPECT_EQ(paint.getColor(), DlPaint::kDefaultColor);
  EXPECT_EQ(paint.getAlpha(), 0xFF);
  EXPECT_EQ(paint.getBlendMode(), DlBlendMode::kDefaultMode);
  EXPECT_EQ(paint.getDrawStyle(), DlDrawStyle::kDefaultStyle);
  EXPECT_EQ(paint.getStrokeCap(), DlStrokeCap::kDefaultCap);
  EXPECT_EQ(paint.getStrokeJoin(), DlStrokeJoin::kDefaultJoin);
  EXPECT_EQ(paint.getStrokeWidth(), DlPaint::kDefaultWidth);
  EXPECT_EQ(paint.getStrokeMiter(), DlPaint::kDefaultMiter);
  EXPECT_EQ(paint.getColorSource(), nullptr);
  EXPECT_EQ(paint.getColorFilter(), nullptr);
  EXPECT_EQ(paint.getImageFilter(), nullptr);
  EXPECT_EQ(paint.getMaskFilter(), nullptr);
  EXPECT_TRUE(paint.isDefault());
  EXPECT_EQ(paint, DlPaint::kDefault);

  EXPECT_EQ(DlBlendMode::kDefaultMode, DlBlendMode::kSrcOver);
  EXPECT_EQ(DlDrawStyle::kDefaultStyle, DlDrawStyle::kFill);
  EXPECT_EQ(DlStrokeCap::kDefaultCap, DlStrokeCap::kButt);
  EXPECT_EQ(DlStrokeJoin::kDefaultJoin, DlStrokeJoin::kMiter);

  EXPECT_EQ(DlPaint::kDefaultColor, DlColor::kBlack());
  EXPECT_EQ(DlPaint::kDefaultWidth, 0.0);
  EXPECT_EQ(DlPaint::kDefaultMiter, 4.0);

  EXPECT_EQ(paint, DlPaint());
  EXPECT_EQ(paint, DlPaint(DlColor::kBlack()));
  EXPECT_EQ(paint, DlPaint(DlColor(0xFF000000)));

  EXPECT_NE(paint, DlPaint().setAntiAlias(true));
  EXPECT_NE(paint, DlPaint().setInvertColors(true));
  EXPECT_NE(paint, DlPaint().setColor(DlColor::kGreen()));
  EXPECT_NE(paint, DlPaint(DlColor::kGreen()));
  EXPECT_NE(paint, DlPaint(DlColor(0xFF00FF00)));
  EXPECT_NE(paint, DlPaint().setAlpha(0x7f));
  EXPECT_NE(paint, DlPaint().setBlendMode(DlBlendMode::kDstIn));
  EXPECT_NE(paint, DlPaint().setDrawStyle(DlDrawStyle::kStrokeAndFill));
  EXPECT_NE(paint, DlPaint().setStrokeCap(DlStrokeCap::kRound));
  EXPECT_NE(paint, DlPaint().setStrokeJoin(DlStrokeJoin::kRound));
  EXPECT_NE(paint, DlPaint().setStrokeWidth(6));
  EXPECT_NE(paint, DlPaint().setStrokeMiter(7));

  DlColor colors[2] = {
      DlColor::kGreen(),
      DlColor::kBlue(),
  };
  float stops[2] = {
      0.0f,
      1.0f,
  };
  auto color_source = DlColorSource::MakeLinear({0, 0}, {10, 10}, 2, colors,
                                                stops, DlTileMode::kClamp);
  EXPECT_NE(paint, DlPaint().setColorSource(color_source));

  auto color_filter =
      DlColorFilter::MakeBlend(DlColor::kYellow(), DlBlendMode::kDstATop);
  EXPECT_NE(paint, DlPaint().setColorFilter(color_filter));

  auto image_filter = DlImageFilter::MakeBlur(1.3, 4.7, DlTileMode::kClamp);
  EXPECT_NE(paint, DlPaint().setImageFilter(image_filter));

  DlBlurMaskFilter mask_filter(DlBlurStyle::kInner, 3.14);
  EXPECT_NE(paint, DlPaint().setMaskFilter(mask_filter.shared()));
}

TEST(DisplayListPaint, NullPointerSetGet) {
  DlColorSource* null_color_source = nullptr;
  DlColorFilter* null_color_filter = nullptr;
  DlImageFilter* null_image_filter = nullptr;
  DlMaskFilter* null_mask_filter = nullptr;
  DlPaint paint;
  EXPECT_EQ(paint.setColorSource(null_color_source).getColorSource(), nullptr);
  EXPECT_EQ(paint.setColorFilter(null_color_filter).getColorFilter(), nullptr);
  EXPECT_EQ(paint.setImageFilter(null_image_filter).getImageFilter(), nullptr);
  EXPECT_EQ(paint.setMaskFilter(null_mask_filter).getMaskFilter(), nullptr);
}

TEST(DisplayListPaint, NullSharedPointerSetGet) {
  std::shared_ptr<DlColorSource> null_color_source;
  std::shared_ptr<DlColorFilter> null_color_filter;
  std::shared_ptr<DlImageFilter> null_image_filter;
  std::shared_ptr<DlMaskFilter> null_mask_filter;
  DlPaint paint;
  EXPECT_EQ(paint.setColorSource(null_color_source).getColorSource(), nullptr);
  EXPECT_EQ(paint.setColorFilter(null_color_filter).getColorFilter(), nullptr);
  EXPECT_EQ(paint.setImageFilter(null_image_filter).getImageFilter(), nullptr);
  EXPECT_EQ(paint.setMaskFilter(null_mask_filter).getMaskFilter(), nullptr);
}

TEST(DisplayListPaint, ChainingConstructor) {
  DlColor colors[2] = {
      DlColor::kGreen(),
      DlColor::kBlue(),
  };
  float stops[2] = {
      0.0f,
      1.0f,
  };
  DlPaint paint =
      DlPaint()                                                         //
          .setAntiAlias(true)                                           //
          .setInvertColors(true)                                        //
          .setColor(DlColor::kGreen())                                  //
          .setAlpha(0x7F)                                               //
          .setBlendMode(DlBlendMode::kLuminosity)                       //
          .setDrawStyle(DlDrawStyle::kStrokeAndFill)                    //
          .setStrokeCap(DlStrokeCap::kSquare)                           //
          .setStrokeJoin(DlStrokeJoin::kBevel)                          //
          .setStrokeWidth(42)                                           //
          .setStrokeMiter(1.5)                                          //
          .setColorSource(DlColorSource::MakeLinear(                    //
              {0, 0}, {10, 10}, 2, colors, stops, DlTileMode::kClamp))  //
          .setColorFilter(
              DlColorFilter::MakeBlend(DlColor::kYellow(), DlBlendMode::kDstIn))
          .setImageFilter(DlImageFilter::MakeBlur(1.3, 4.7, DlTileMode::kClamp))
          .setMaskFilter(DlBlurMaskFilter(DlBlurStyle::kInner, 3.14).shared());
  EXPECT_TRUE(paint.isAntiAlias());
  EXPECT_TRUE(paint.isInvertColors());
  EXPECT_EQ(paint.getColor(), DlColor::kGreen().withAlpha(0x7F));
  EXPECT_EQ(paint.getAlpha(), 0x7F);
  EXPECT_EQ(paint.getBlendMode(), DlBlendMode::kLuminosity);
  EXPECT_EQ(paint.getDrawStyle(), DlDrawStyle::kStrokeAndFill);
  EXPECT_EQ(paint.getStrokeCap(), DlStrokeCap::kSquare);
  EXPECT_EQ(paint.getStrokeJoin(), DlStrokeJoin::kBevel);
  EXPECT_EQ(paint.getStrokeWidth(), 42);
  EXPECT_EQ(paint.getStrokeMiter(), 1.5);
  EXPECT_TRUE(Equals(paint.getColorSource(),
                     DlColorSource::MakeLinear({0, 0}, {10, 10}, 2, colors,
                                               stops, DlTileMode::kClamp)));
  EXPECT_TRUE(Equals(
      paint.getColorFilter(),
      DlColorFilter::MakeBlend(DlColor::kYellow(), DlBlendMode::kDstIn)));
  EXPECT_TRUE(Equals(paint.getImageFilter(),
                     DlImageFilter::MakeBlur(1.3, 4.7, DlTileMode::kClamp)));
  EXPECT_EQ(*paint.getMaskFilter(),
            DlBlurMaskFilter(DlBlurStyle::kInner, 3.14));

  EXPECT_NE(paint, DlPaint());
}

TEST(DisplayListPaint, PaintDetectsRuntimeEffects) {
  const auto runtime_effect = DlRuntimeEffect::MakeSkia(
      SkRuntimeEffect::MakeForShader(
          SkString("vec4 main(vec2 p) { return vec4(0); }"))
          .effect);
  auto color_source = DlColorSource::MakeRuntimeEffect(
      runtime_effect, {}, std::make_shared<std::vector<uint8_t>>());
  auto image_filter = DlImageFilter::MakeRuntimeEffect(
      runtime_effect, {}, std::make_shared<std::vector<uint8_t>>());
  DlPaint paint;

  EXPECT_FALSE(paint.usesRuntimeEffect());
  paint.setColorSource(color_source);
  EXPECT_TRUE(paint.usesRuntimeEffect());
  paint.setColorSource(nullptr);
  EXPECT_FALSE(paint.usesRuntimeEffect());

  EXPECT_FALSE(paint.usesRuntimeEffect());
  paint.setImageFilter(image_filter);
  EXPECT_TRUE(paint.usesRuntimeEffect());
  paint.setImageFilter(nullptr);
  EXPECT_FALSE(paint.usesRuntimeEffect());

  EXPECT_FALSE(paint.usesRuntimeEffect());
  paint.setColorSource(color_source);
  EXPECT_TRUE(paint.usesRuntimeEffect());
  paint.setImageFilter(image_filter);
  EXPECT_TRUE(paint.usesRuntimeEffect());
  paint.setImageFilter(nullptr);
  EXPECT_TRUE(paint.usesRuntimeEffect());
  paint.setColorSource(nullptr);
  EXPECT_FALSE(paint.usesRuntimeEffect());

  EXPECT_FALSE(paint.usesRuntimeEffect());
  paint.setColorSource(color_source);
  EXPECT_TRUE(paint.usesRuntimeEffect());
  paint.setImageFilter(image_filter);
  EXPECT_TRUE(paint.usesRuntimeEffect());
  paint.setColorSource(nullptr);
  EXPECT_TRUE(paint.usesRuntimeEffect());
  paint.setImageFilter(nullptr);
  EXPECT_FALSE(paint.usesRuntimeEffect());
}

}  // namespace testing
}  // namespace flutter
