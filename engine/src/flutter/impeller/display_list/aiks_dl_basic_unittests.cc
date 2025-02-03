// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "display_list/display_list.h"
#include "display_list/dl_sampling_options.h"
#include "display_list/dl_tile_mode.h"
#include "display_list/effects/dl_color_filter.h"
#include "display_list/effects/dl_color_source.h"
#include "display_list/effects/dl_image_filter.h"
#include "display_list/effects/dl_mask_filter.h"
#include "flutter/impeller/display_list/aiks_unittests.h"

#include "flutter/display_list/dl_blend_mode.h"
#include "flutter/display_list/dl_builder.h"
#include "flutter/display_list/dl_color.h"
#include "flutter/display_list/dl_paint.h"
#include "flutter/impeller/display_list/dl_image_impeller.h"
#include "flutter/impeller/geometry/scalar.h"
#include "flutter/testing/display_list_testing.h"
#include "flutter/testing/testing.h"
#include "impeller/playground/widgets.h"

namespace impeller {
namespace testing {

using namespace flutter;

TEST_P(AiksTest, CanRenderColoredRect) {
  DisplayListBuilder builder;
  DlPaint paint;
  paint.setColor(DlColor::kBlue());
  builder.DrawPath(DlPath::MakeRectXYWH(100.0f, 100.0f, 100.0f, 100.0f), paint);
  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanRenderImage) {
  DisplayListBuilder builder;
  DlPaint paint;
  paint.setColor(DlColor::kRed());
  auto image = DlImageImpeller::Make(CreateTextureForFixture("kalimba.jpg"));
  builder.DrawImage(image, DlPoint(100.0, 100.0),
                    DlImageSampling::kNearestNeighbor, &paint);
  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanRenderInvertedImageWithColorFilter) {
  DisplayListBuilder builder;
  DlPaint paint;
  paint.setColor(DlColor::kRed());
  paint.setColorFilter(
      DlColorFilter::MakeBlend(DlColor::kYellow(), DlBlendMode::kSrcOver));
  paint.setInvertColors(true);
  auto image = DlImageImpeller::Make(CreateTextureForFixture("kalimba.jpg"));

  builder.DrawImage(image, DlPoint(100.0, 100.0),
                    DlImageSampling::kNearestNeighbor, &paint);
  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanRenderColorFilterWithInvertColors) {
  DisplayListBuilder builder;
  DlPaint paint;
  paint.setColor(DlColor::kRed());
  paint.setColorFilter(
      DlColorFilter::MakeBlend(DlColor::kYellow(), DlBlendMode::kSrcOver));
  paint.setInvertColors(true);

  builder.DrawRect(DlRect::MakeLTRB(0, 0, 100, 100), paint);
  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanRenderColorFilterWithInvertColorsDrawPaint) {
  DisplayListBuilder builder;
  DlPaint paint;
  paint.setColor(DlColor::kRed());
  paint.setColorFilter(
      DlColorFilter::MakeBlend(DlColor::kYellow(), DlBlendMode::kSrcOver));
  paint.setInvertColors(true);

  builder.DrawPaint(paint);
  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

namespace {
bool GenerateMipmap(const std::shared_ptr<Context>& context,
                    std::shared_ptr<Texture> texture,
                    std::string_view label) {
  auto buffer = context->CreateCommandBuffer();
  if (!buffer) {
    return false;
  }
  auto pass = buffer->CreateBlitPass();
  if (!pass) {
    return false;
  }
  pass->GenerateMipmap(std::move(texture), label);

  pass->EncodeCommands();
  return context->GetCommandQueue()->Submit({buffer}).ok();
}

void CanRenderTiledTexture(AiksTest* aiks_test,
                           DlTileMode tile_mode,
                           Matrix local_matrix = {}) {
  auto context = aiks_test->GetContext();
  ASSERT_TRUE(context);
  auto texture = aiks_test->CreateTextureForFixture("table_mountain_nx.png",
                                                    /*enable_mipmapping=*/true);
  GenerateMipmap(context, texture, "table_mountain_nx");
  auto image = DlImageImpeller::Make(texture);
  auto color_source = DlColorSource::MakeImage(
      image, tile_mode, tile_mode, DlImageSampling::kNearestNeighbor,
      &local_matrix);

  DisplayListBuilder builder;
  DlPaint paint;
  paint.setColor(DlColor::kWhite());
  paint.setColorSource(color_source);

  builder.Scale(aiks_test->GetContentScale().x, aiks_test->GetContentScale().y);
  builder.Translate(100.0f, 100.0f);
  builder.DrawRect(DlRect::MakeXYWH(0, 0, 600, 600), paint);

  // Should not change the image.
  constexpr auto stroke_width = 64;
  paint.setDrawStyle(DlDrawStyle::kStroke);
  paint.setStrokeWidth(stroke_width);
  if (tile_mode == DlTileMode::kDecal) {
    builder.DrawRect(DlRect::MakeXYWH(stroke_width, stroke_width, 600, 600),
                     paint);
  } else {
    builder.DrawRect(DlRect::MakeXYWH(0, 0, 600, 600), paint);
  }

  {
    // Should not change the image.
    DlPathBuilder path_builder;
    path_builder.AddCircle(DlPoint(150, 150), 150);
    path_builder.AddRoundRect(
        RoundRect::MakeRectXY(DlRect::MakeLTRB(300, 300, 600, 600), 10, 10));
    DlPath path(path_builder);

    // Make sure path cannot be simplified...
    EXPECT_FALSE(path.IsRect(nullptr));
    EXPECT_FALSE(path.IsOval(nullptr));
    EXPECT_FALSE(path.IsRoundRect(nullptr));

    // Make sure path will not trigger the optimal convex code
    EXPECT_FALSE(path.IsConvex());

    paint.setDrawStyle(DlDrawStyle::kFill);
    builder.DrawPath(path, paint);
  }

  {
    // Should not change the image. Tests the Convex short-cut code.
    DlPath circle = DlPath::MakeCircle(DlPoint(150, 450), 150);

    // Unfortunately, the circle path can be simplified...
    EXPECT_TRUE(circle.IsOval(nullptr));
    // At least it's convex, though...
    EXPECT_TRUE(circle.IsConvex());

    // Let's make a copy that doesn't remember that it's just a circle...
    DlPathBuilder path_builder;
    // This moveTo confuses addPath into appending rather than replacing,
    // which prevents it from noticing that it's just a circle...
    path_builder.MoveTo({10, 10});
    path_builder.AddPath(circle.GetPath());
    path_builder.SetConvexity(Convexity::kConvex);
    DlPath path(path_builder);

    // Make sure path cannot be simplified...
    EXPECT_FALSE(path.IsRect(nullptr));
    EXPECT_FALSE(path.IsOval(nullptr));
    EXPECT_FALSE(path.IsRoundRect(nullptr));

    // But check that we will trigger the optimal convex code
    EXPECT_TRUE(path.GetPath().IsConvex());

    paint.setDrawStyle(DlDrawStyle::kFill);
    builder.DrawPath(path, paint);
  }

  ASSERT_TRUE(aiks_test->OpenPlaygroundHere(builder.Build()));
}
}  // namespace

TEST_P(AiksTest, CanRenderTiledTextureClamp) {
  CanRenderTiledTexture(this, DlTileMode::kClamp);
}

TEST_P(AiksTest, CanRenderTiledTextureRepeat) {
  CanRenderTiledTexture(this, DlTileMode::kRepeat);
}

TEST_P(AiksTest, CanRenderTiledTextureMirror) {
  CanRenderTiledTexture(this, DlTileMode::kMirror);
}

TEST_P(AiksTest, CanRenderTiledTextureDecal) {
  CanRenderTiledTexture(this, DlTileMode::kDecal);
}

TEST_P(AiksTest, CanRenderTiledTextureClampWithTranslate) {
  CanRenderTiledTexture(this, DlTileMode::kClamp,
                        Matrix::MakeTranslation({172.f, 172.f, 0.f}));
}

TEST_P(AiksTest, CanRenderImageRect) {
  DisplayListBuilder builder;
  auto image = DlImageImpeller::Make(CreateTextureForFixture("kalimba.jpg"));

  DlISize image_half_size = DlISize(image->dimensions().fWidth * 0.5f,
                                    image->dimensions().fHeight * 0.5f);

  // Render the bottom right quarter of the source image in a stretched rect.
  auto source_rect = DlRect::MakeSize(image_half_size);
  source_rect =
      source_rect.Shift(image_half_size.width, image_half_size.height);

  builder.DrawImageRect(image, source_rect,
                        DlRect::MakeXYWH(100, 100, 600, 600),
                        DlImageSampling::kNearestNeighbor);
  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, DrawImageRectSrcOutsideBounds) {
  DisplayListBuilder builder;
  auto image = DlImageImpeller::Make(CreateTextureForFixture("kalimba.jpg"));

  // Use a source rect that is partially outside the bounds of the image.
  auto source_rect = DlRect::MakeXYWH(
      image->dimensions().fWidth * 0.25f, image->dimensions().fHeight * 0.4f,
      image->dimensions().fWidth, image->dimensions().fHeight);

  auto dest_rect = DlRect::MakeXYWH(100, 100, 600, 600);

  DlPaint paint;
  paint.setColor(DlColor::kMidGrey());
  builder.DrawRect(dest_rect, paint);

  builder.DrawImageRect(image, source_rect, dest_rect,
                        DlImageSampling::kNearestNeighbor);
  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanRenderSimpleClips) {
  DisplayListBuilder builder;
  builder.Scale(GetContentScale().x, GetContentScale().y);
  DlPaint paint;

  paint.setColor(DlColor::kWhite());
  builder.DrawPaint(paint);

  auto draw = [&builder](const DlPaint& paint, Scalar x, Scalar y) {
    builder.Save();
    builder.Translate(x, y);
    {
      builder.Save();
      builder.ClipRect(DlRect::MakeLTRB(50, 50, 150, 150));
      builder.DrawPaint(paint);
      builder.Restore();
    }
    {
      builder.Save();
      builder.ClipOval(DlRect::MakeLTRB(200, 50, 300, 150));
      builder.DrawPaint(paint);
      builder.Restore();
    }
    {
      builder.Save();
      builder.ClipRoundRect(
          DlRoundRect::MakeRectXY(DlRect::MakeLTRB(50, 200, 150, 300), 20, 20));
      builder.DrawPaint(paint);
      builder.Restore();
    }
    {
      builder.Save();
      builder.ClipRoundRect(DlRoundRect::MakeRectXY(
          DlRect::MakeLTRB(200, 230, 300, 270), 20, 20));
      builder.DrawPaint(paint);
      builder.Restore();
    }
    {
      builder.Save();
      builder.ClipRoundRect(DlRoundRect::MakeRectXY(
          DlRect::MakeLTRB(230, 200, 270, 300), 20, 20));
      builder.DrawPaint(paint);
      builder.Restore();
    }
    builder.Restore();
  };

  paint.setColor(DlColor::kBlue());
  draw(paint, 0, 0);

  DlColor gradient_colors[7] = {
      DlColor::RGBA(0x1f / 255.0, 0.0, 0x5c / 255.0, 1.0),
      DlColor::RGBA(0x5b / 255.0, 0.0, 0x60 / 255.0, 1.0),
      DlColor::RGBA(0x87 / 255.0, 0x01 / 255.0, 0x60 / 255.0, 1.0),
      DlColor::RGBA(0xac / 255.0, 0x25 / 255.0, 0x53 / 255.0, 1.0),
      DlColor::RGBA(0xe1 / 255.0, 0x6b / 255.0, 0x5c / 255.0, 1.0),
      DlColor::RGBA(0xf3 / 255.0, 0x90 / 255.0, 0x60 / 255.0, 1.0),
      DlColor::RGBA(0xff / 255.0, 0xb5 / 255.0, 0x6b / 250.0, 1.0),
  };
  Scalar stops[7] = {
      0.0,
      (1.0 / 6.0) * 1,
      (1.0 / 6.0) * 2,
      (1.0 / 6.0) * 3,
      (1.0 / 6.0) * 4,
      (1.0 / 6.0) * 5,
      1.0,
  };
  auto texture = CreateTextureForFixture("airplane.jpg",
                                         /*enable_mipmapping=*/true);
  auto image = DlImageImpeller::Make(texture);

  paint.setColorSource(DlColorSource::MakeRadial(
      DlPoint(500, 600), 75, 7, gradient_colors, stops, DlTileMode::kMirror));
  draw(paint, 0, 300);

  paint.setColorSource(
      DlColorSource::MakeImage(image, DlTileMode::kRepeat, DlTileMode::kRepeat,
                               DlImageSampling::kNearestNeighbor));
  draw(paint, 300, 0);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanSaveLayerStandalone) {
  DisplayListBuilder builder;

  DlPaint red;
  red.setColor(DlColor::kRed());

  DlPaint alpha;
  alpha.setColor(DlColor::kRed().modulateOpacity(0.5));

  builder.SaveLayer(nullptr, &alpha);

  builder.DrawCircle(DlPoint(125, 125), 125, red);

  builder.Restore();

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanRenderDifferentShapesWithSameColorSource) {
  DisplayListBuilder builder;
  DlPaint paint;

  DlColor colors[2] = {
      DlColor::RGBA(0.9568, 0.2627, 0.2118, 1.0),
      DlColor::RGBA(0.1294, 0.5882, 0.9529, 1.0),
  };
  DlScalar stops[2] = {
      0.0,
      1.0,
  };

  paint.setColorSource(DlColorSource::MakeLinear(
      /*start_point=*/DlPoint(0, 0),     //
      /*end_point=*/DlPoint(100, 100),   //
      /*stop_count=*/2,                  //
      /*colors=*/colors,                 //
      /*stops=*/stops,                   //
      /*tile_mode=*/DlTileMode::kRepeat  //
      ));

  builder.Save();
  builder.Translate(100, 100);
  builder.DrawRect(DlRect::MakeXYWH(0, 0, 200, 200), paint);
  builder.Restore();

  builder.Save();
  builder.Translate(100, 400);
  builder.DrawCircle(DlPoint(100, 100), 100, paint);
  builder.Restore();
  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanRenderRoundedRectWithNonUniformRadii) {
  DisplayListBuilder builder;
  DlPaint paint;
  paint.setColor(DlColor::kRed());

  RoundingRadii radii = {
      .top_left = DlSize(50, 25),
      .top_right = DlSize(25, 50),
      .bottom_left = DlSize(25, 50),
      .bottom_right = DlSize(50, 25),
  };
  DlRoundRect rrect =
      DlRoundRect::MakeRectRadii(DlRect::MakeXYWH(100, 100, 500, 500), radii);

  builder.DrawRoundRect(rrect, paint);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanDrawPaint) {
  auto medium_turquoise =
      DlColor::RGBA(72.0f / 255.0f, 209.0f / 255.0f, 204.0f / 255.0f, 1.0f);

  DisplayListBuilder builder;
  builder.Scale(0.2, 0.2);
  builder.DrawPaint(DlPaint().setColor(medium_turquoise));
  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanDrawPaintMultipleTimes) {
  auto medium_turquoise =
      DlColor::RGBA(72.0f / 255.0f, 209.0f / 255.0f, 204.0f / 255.0f, 1.0f);
  auto orange_red =
      DlColor::RGBA(255.0f / 255.0f, 69.0f / 255.0f, 0.0f / 255.0f, 1.0f);

  DisplayListBuilder builder;
  builder.Scale(0.2, 0.2);
  builder.DrawPaint(DlPaint().setColor(medium_turquoise));
  builder.DrawPaint(DlPaint().setColor(orange_red.modulateOpacity(0.5f)));
  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, FilledCirclesRenderCorrectly) {
  DisplayListBuilder builder;
  builder.Scale(GetContentScale().x, GetContentScale().y);
  DlPaint paint;
  const int color_count = 3;
  DlColor colors[color_count] = {
      DlColor::kBlue(),
      DlColor::kGreen(),
      DlColor::RGBA(220.0f / 255.0f, 20.0f / 255.0f, 60.0f / 255.0f, 1.0f),
  };

  paint.setColor(DlColor::kWhite());
  builder.DrawPaint(paint);

  int c_index = 0;
  int radius = 600;
  while (radius > 0) {
    paint.setColor(colors[(c_index++) % color_count]);
    builder.DrawCircle(DlPoint(10, 10), radius, paint);
    if (radius > 30) {
      radius -= 10;
    } else {
      radius -= 2;
    }
  }

  DlColor gradient_colors[7] = {
      DlColor::RGBA(0x1f / 255.0, 0.0, 0x5c / 255.0, 1.0),
      DlColor::RGBA(0x5b / 255.0, 0.0, 0x60 / 255.0, 1.0),
      DlColor::RGBA(0x87 / 255.0, 0x01 / 255.0, 0x60 / 255.0, 1.0),
      DlColor::RGBA(0xac / 255.0, 0x25 / 255.0, 0x53 / 255.0, 1.0),
      DlColor::RGBA(0xe1 / 255.0, 0x6b / 255.0, 0x5c / 255.0, 1.0),
      DlColor::RGBA(0xf3 / 255.0, 0x90 / 255.0, 0x60 / 255.0, 1.0),
      DlColor::RGBA(0xff / 255.0, 0xb5 / 255.0, 0x6b / 250.0, 1.0),
  };
  DlScalar stops[7] = {
      0.0,
      (1.0 / 6.0) * 1,
      (1.0 / 6.0) * 2,
      (1.0 / 6.0) * 3,
      (1.0 / 6.0) * 4,
      (1.0 / 6.0) * 5,
      1.0,
  };
  auto texture = CreateTextureForFixture("airplane.jpg",
                                         /*enable_mipmapping=*/true);
  auto image = DlImageImpeller::Make(texture);

  paint.setColorSource(DlColorSource::MakeRadial(
      DlPoint(500, 600), 75, 7, gradient_colors, stops, DlTileMode::kMirror));
  builder.DrawCircle(DlPoint(500, 600), 100, paint);

  DlMatrix local_matrix = DlMatrix::MakeTranslation({700, 200});
  paint.setColorSource(DlColorSource::MakeImage(
      image, DlTileMode::kRepeat, DlTileMode::kRepeat,
      DlImageSampling::kNearestNeighbor, &local_matrix));
  builder.DrawCircle(DlPoint(800, 300), 100, paint);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, StrokedCirclesRenderCorrectly) {
  DisplayListBuilder builder;
  builder.Scale(GetContentScale().x, GetContentScale().y);
  DlPaint paint;
  const int color_count = 3;
  DlColor colors[color_count] = {
      DlColor::kBlue(),
      DlColor::kGreen(),
      DlColor::RGBA(220.0f / 255.0f, 20.0f / 255.0f, 60.0f / 255.0f, 1.0f),
  };

  paint.setColor(DlColor::kWhite());
  builder.DrawPaint(paint);

  int c_index = 0;

  auto draw = [&paint, &colors, &c_index](DlCanvas& canvas, DlPoint center,
                                          Scalar r, Scalar dr, int n) {
    for (int i = 0; i < n; i++) {
      paint.setColor(colors[(c_index++) % color_count]);
      canvas.DrawCircle(center, r, paint);
      r += dr;
    }
  };

  paint.setDrawStyle(DlDrawStyle::kStroke);
  paint.setStrokeWidth(1);
  draw(builder, DlPoint(10, 10), 2, 2, 14);  // r = [2, 28], covers [1,29]
  paint.setStrokeWidth(5);
  draw(builder, DlPoint(10, 10), 35, 10, 56);  // r = [35, 585], covers [30,590]

  DlColor gradient_colors[7] = {
      DlColor::RGBA(0x1f / 255.0, 0.0, 0x5c / 255.0, 1.0),
      DlColor::RGBA(0x5b / 255.0, 0.0, 0x60 / 255.0, 1.0),
      DlColor::RGBA(0x87 / 255.0, 0x01 / 255.0, 0x60 / 255.0, 1.0),
      DlColor::RGBA(0xac / 255.0, 0x25 / 255.0, 0x53 / 255.0, 1.0),
      DlColor::RGBA(0xe1 / 255.0, 0x6b / 255.0, 0x5c / 255.0, 1.0),
      DlColor::RGBA(0xf3 / 255.0, 0x90 / 255.0, 0x60 / 255.0, 1.0),
      DlColor::RGBA(0xff / 255.0, 0xb5 / 255.0, 0x6b / 250.0, 1.0),
  };
  DlScalar stops[7] = {
      0.0,
      (1.0 / 6.0) * 1,
      (1.0 / 6.0) * 2,
      (1.0 / 6.0) * 3,
      (1.0 / 6.0) * 4,
      (1.0 / 6.0) * 5,
      1.0,
  };
  auto texture = CreateTextureForFixture("airplane.jpg",
                                         /*enable_mipmapping=*/true);
  auto image = DlImageImpeller::Make(texture);

  paint.setColorSource(DlColorSource::MakeRadial(
      DlPoint(500, 600), 75, 7, gradient_colors, stops, DlTileMode::kMirror));
  draw(builder, DlPoint(500, 600), 5, 10, 10);

  DlMatrix local_matrix = DlMatrix::MakeTranslation({700, 200});
  paint.setColorSource(DlColorSource::MakeImage(
      image, DlTileMode::kRepeat, DlTileMode::kRepeat,
      DlImageSampling::kNearestNeighbor, &local_matrix));
  draw(builder, DlPoint(800, 300), 5, 10, 10);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, FilledEllipsesRenderCorrectly) {
  DisplayListBuilder builder;
  builder.Scale(GetContentScale().x, GetContentScale().y);
  DlPaint paint;
  const int color_count = 3;
  DlColor colors[color_count] = {
      DlColor::kBlue(),
      DlColor::kGreen(),
      DlColor::RGBA(220.0f / 255.0f, 20.0f / 255.0f, 60.0f / 255.0f, 1.0f),
  };

  paint.setColor(DlColor::kWhite());
  builder.DrawPaint(paint);

  int c_index = 0;
  int long_radius = 600;
  int short_radius = 600;
  while (long_radius > 0 && short_radius > 0) {
    paint.setColor(colors[(c_index++) % color_count]);
    builder.DrawOval(DlRect::MakeXYWH(10 - long_radius, 10 - short_radius,
                                      long_radius * 2, short_radius * 2),
                     paint);
    builder.DrawOval(DlRect::MakeXYWH(1000 - short_radius, 750 - long_radius,
                                      short_radius * 2, long_radius * 2),
                     paint);
    if (short_radius > 30) {
      short_radius -= 10;
      long_radius -= 5;
    } else {
      short_radius -= 2;
      long_radius -= 1;
    }
  }

  DlColor gradient_colors[7] = {
      DlColor::RGBA(0x1f / 255.0, 0.0, 0x5c / 255.0, 1.0),
      DlColor::RGBA(0x5b / 255.0, 0.0, 0x60 / 255.0, 1.0),
      DlColor::RGBA(0x87 / 255.0, 0x01 / 255.0, 0x60 / 255.0, 1.0),
      DlColor::RGBA(0xac / 255.0, 0x25 / 255.0, 0x53 / 255.0, 1.0),
      DlColor::RGBA(0xe1 / 255.0, 0x6b / 255.0, 0x5c / 255.0, 1.0),
      DlColor::RGBA(0xf3 / 255.0, 0x90 / 255.0, 0x60 / 255.0, 1.0),
      DlColor::RGBA(0xff / 255.0, 0xb5 / 255.0, 0x6b / 250.0, 1.0),
  };
  DlScalar stops[7] = {
      0.0,
      (1.0 / 6.0) * 1,
      (1.0 / 6.0) * 2,
      (1.0 / 6.0) * 3,
      (1.0 / 6.0) * 4,
      (1.0 / 6.0) * 5,
      1.0,
  };
  auto texture = CreateTextureForFixture("airplane.jpg",
                                         /*enable_mipmapping=*/true);
  auto image = DlImageImpeller::Make(texture);

  paint.setColor(DlColor::kWhite().modulateOpacity(0.5));

  paint.setColorSource(DlColorSource::MakeRadial(
      DlPoint(300, 650), 75, 7, gradient_colors, stops, DlTileMode::kMirror));
  builder.DrawOval(DlRect::MakeXYWH(200, 625, 200, 50), paint);
  builder.DrawOval(DlRect::MakeXYWH(275, 550, 50, 200), paint);

  DlMatrix local_matrix = DlMatrix::MakeTranslation({610, 15});
  paint.setColorSource(DlColorSource::MakeImage(
      image, DlTileMode::kRepeat, DlTileMode::kRepeat,
      DlImageSampling::kNearestNeighbor, &local_matrix));
  builder.DrawOval(DlRect::MakeXYWH(610, 90, 200, 50), paint);
  builder.DrawOval(DlRect::MakeXYWH(685, 15, 50, 200), paint);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, FilledRoundRectsRenderCorrectly) {
  DisplayListBuilder builder;
  builder.Scale(GetContentScale().x, GetContentScale().y);
  DlPaint paint;
  const int color_count = 3;
  DlColor colors[color_count] = {
      DlColor::kBlue(),
      DlColor::kGreen(),
      DlColor::RGBA(220.0f / 255.0f, 20.0f / 255.0f, 60.0f / 255.0f, 1.0f),
  };

  paint.setColor(DlColor::kWhite());
  builder.DrawPaint(paint);

  int c_index = 0;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      paint.setColor(colors[(c_index++) % color_count]);
      builder.DrawRoundRect(
          DlRoundRect::MakeRectXY(
              DlRect::MakeXYWH(i * 100 + 10, j * 100 + 20, 80, 80),  //
              i * 5 + 10, j * 5 + 10),
          paint);
    }
  }
  paint.setColor(colors[(c_index++) % color_count]);
  builder.DrawRoundRect(
      DlRoundRect::MakeRectXY(DlRect::MakeXYWH(10, 420, 380, 80), 40, 40),
      paint);
  paint.setColor(colors[(c_index++) % color_count]);
  builder.DrawRoundRect(
      DlRoundRect::MakeRectXY(DlRect::MakeXYWH(410, 20, 80, 380), 40, 40),
      paint);

  DlColor gradient_colors[7] = {
      DlColor::RGBA(0x1f / 255.0, 0.0, 0x5c / 255.0, 1.0),
      DlColor::RGBA(0x5b / 255.0, 0.0, 0x60 / 255.0, 1.0),
      DlColor::RGBA(0x87 / 255.0, 0x01 / 255.0, 0x60 / 255.0, 1.0),
      DlColor::RGBA(0xac / 255.0, 0x25 / 255.0, 0x53 / 255.0, 1.0),
      DlColor::RGBA(0xe1 / 255.0, 0x6b / 255.0, 0x5c / 255.0, 1.0),
      DlColor::RGBA(0xf3 / 255.0, 0x90 / 255.0, 0x60 / 255.0, 1.0),
      DlColor::RGBA(0xff / 255.0, 0xb5 / 255.0, 0x6b / 250.0, 1.0),
  };
  DlScalar stops[7] = {
      0.0,
      (1.0 / 6.0) * 1,
      (1.0 / 6.0) * 2,
      (1.0 / 6.0) * 3,
      (1.0 / 6.0) * 4,
      (1.0 / 6.0) * 5,
      1.0,
  };
  auto texture = CreateTextureForFixture("airplane.jpg",
                                         /*enable_mipmapping=*/true);
  auto image = DlImageImpeller::Make(texture);

  paint.setColor(DlColor::kWhite().modulateOpacity(0.1));
  paint.setColorSource(DlColorSource::MakeRadial(
      DlPoint(550, 550), 75, 7, gradient_colors, stops, DlTileMode::kMirror));
  for (int i = 1; i <= 10; i++) {
    int j = 11 - i;
    builder.DrawRoundRect(
        DlRoundRect::MakeRectXY(DlRect::MakeLTRB(550 - i * 20, 550 - j * 20,  //
                                                 550 + i * 20, 550 + j * 20),
                                i * 10, j * 10),
        paint);
  }

  paint.setColor(DlColor::kWhite().modulateOpacity(0.5));
  paint.setColorSource(DlColorSource::MakeRadial(
      DlPoint(200, 650), 75, 7, gradient_colors, stops, DlTileMode::kMirror));
  paint.setColor(DlColor::kWhite().modulateOpacity(0.5));
  builder.DrawRoundRect(
      DlRoundRect::MakeRectXY(DlRect::MakeLTRB(100, 610, 300, 690), 40, 40),
      paint);
  builder.DrawRoundRect(
      DlRoundRect::MakeRectXY(DlRect::MakeLTRB(160, 550, 240, 750), 40, 40),
      paint);

  paint.setColor(DlColor::kWhite().modulateOpacity(0.1));
  DlMatrix local_matrix = DlMatrix::MakeTranslation({520, 20});
  paint.setColorSource(DlColorSource::MakeImage(
      image, DlTileMode::kRepeat, DlTileMode::kRepeat,
      DlImageSampling::kNearestNeighbor, &local_matrix));
  for (int i = 1; i <= 10; i++) {
    int j = 11 - i;
    builder.DrawRoundRect(
        DlRoundRect::MakeRectXY(DlRect::MakeLTRB(720 - i * 20, 220 - j * 20,  //
                                                 720 + i * 20, 220 + j * 20),
                                i * 10, j * 10),
        paint);
  }

  paint.setColor(DlColor::kWhite().modulateOpacity(0.5));
  local_matrix = DlMatrix::MakeTranslation({800, 300});
  paint.setColorSource(DlColorSource::MakeImage(
      image, DlTileMode::kRepeat, DlTileMode::kRepeat,
      DlImageSampling::kNearestNeighbor, &local_matrix));
  builder.DrawRoundRect(
      DlRoundRect::MakeRectXY(DlRect::MakeLTRB(800, 410, 1000, 490), 40, 40),
      paint);
  builder.DrawRoundRect(
      DlRoundRect::MakeRectXY(DlRect::MakeLTRB(860, 350, 940, 550), 40, 40),
      paint);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, SolidColorCirclesOvalsRRectsMaskBlurCorrectly) {
  DisplayListBuilder builder;
  builder.Scale(GetContentScale().x, GetContentScale().y);
  DlPaint paint;
  paint.setMaskFilter(DlBlurMaskFilter::Make(DlBlurStyle::kNormal, 1.0f));

  builder.DrawPaint(DlPaint().setColor(DlColor::kWhite()));

  paint.setColor(
      DlColor::RGBA(220.0f / 255.0f, 20.0f / 255.0f, 60.0f / 255.0f, 1.0f));
  Scalar y = 100.0f;
  for (int i = 0; i < 5; i++) {
    Scalar x = (i + 1) * 100;
    Scalar radius = x / 10.0f;
    builder.DrawRect(DlRect::MakeXYWH(x + 25 - radius / 2, y + radius / 2,  //
                                      radius, 60.0f - radius),
                     paint);
  }

  paint.setColor(DlColor::kBlue());
  y += 100.0f;
  for (int i = 0; i < 5; i++) {
    Scalar x = (i + 1) * 100;
    Scalar radius = x / 10.0f;
    builder.DrawCircle(DlPoint(x + 25, y + 25), radius, paint);
  }

  paint.setColor(DlColor::kGreen());
  y += 100.0f;
  for (int i = 0; i < 5; i++) {
    Scalar x = (i + 1) * 100;
    Scalar radius = x / 10.0f;
    builder.DrawOval(DlRect::MakeXYWH(x + 25 - radius / 2, y + radius / 2,  //
                                      radius, 60.0f - radius),
                     paint);
  }

  paint.setColor(
      DlColor::RGBA(128.0f / 255.0f, 0.0f / 255.0f, 128.0f / 255.0f, 1.0f));
  y += 100.0f;
  for (int i = 0; i < 5; i++) {
    Scalar x = (i + 1) * 100;
    Scalar radius = x / 20.0f;
    builder.DrawRoundRect(
        DlRoundRect::MakeRectXY(DlRect::MakeXYWH(x, y, 60.0f, 60.0f),  //
                                radius, radius),
        paint);
  }

  paint.setColor(
      DlColor::RGBA(255.0f / 255.0f, 165.0f / 255.0f, 0.0f / 255.0f, 1.0f));
  y += 100.0f;
  for (int i = 0; i < 5; i++) {
    Scalar x = (i + 1) * 100;
    Scalar radius = x / 20.0f;
    builder.DrawRoundRect(
        DlRoundRect::MakeRectXY(DlRect::MakeXYWH(x, y, 60.0f, 60.0f),  //
                                radius, 5.0f),
        paint);
  }

  auto dl = builder.Build();
  ASSERT_TRUE(OpenPlaygroundHere(dl));
}

TEST_P(AiksTest, CanRenderClippedBackdropFilter) {
  DisplayListBuilder builder;

  builder.Scale(GetContentScale().x, GetContentScale().y);

  // Draw something interesting in the background.
  std::vector<DlColor> colors = {DlColor::RGBA(0.9568, 0.2627, 0.2118, 1.0),
                                 DlColor::RGBA(0.1294, 0.5882, 0.9529, 1.0)};
  std::vector<Scalar> stops = {
      0.0,
      1.0,
  };
  DlPaint paint;
  paint.setColorSource(DlColorSource::MakeLinear(
      /*start_point=*/DlPoint(0, 0),     //
      /*end_point=*/DlPoint(100, 100),   //
      /*stop_count=*/2,                  //
      /*colors=*/colors.data(),          //
      /*stops=*/stops.data(),            //
      /*tile_mode=*/DlTileMode::kRepeat  //
      ));

  builder.DrawPaint(paint);

  DlRect clip_rect = DlRect::MakeLTRB(50, 50, 400, 300);
  DlRoundRect clip_rrect = DlRoundRect::MakeRectXY(clip_rect, 100, 100);

  // Draw a clipped SaveLayer, where the clip coverage and SaveLayer size are
  // the same.
  builder.ClipRoundRect(clip_rrect, DlClipOp::kIntersect);

  DlPaint save_paint;
  auto backdrop_filter = DlImageFilter::MakeColorFilter(
      DlColorFilter::MakeBlend(DlColor::kRed(), DlBlendMode::kExclusion));
  builder.SaveLayer(clip_rect, &save_paint, backdrop_filter.get());

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanDrawPerspectiveTransformWithClips) {
  // Avoiding `GetSecondsElapsed()` to reduce risk of golden flakiness.
  int time = 0;
  auto callback = [&]() -> sk_sp<DisplayList> {
    DisplayListBuilder builder;

    builder.Save();
    {
      builder.Translate(300, 300);

      // 1. Draw/restore a clip before drawing the image, which will get drawn
      //    to the depth buffer behind the image.
      builder.Save();
      {
        DlPaint paint;
        paint.setColor(DlColor::kGreen());
        builder.DrawPaint(paint);
        builder.ClipRect(DlRect::MakeLTRB(-180, -180, 180, 180),
                         DlClipOp::kDifference);

        paint.setColor(DlColor::kBlack());
        builder.DrawPaint(paint);
      }
      builder.Restore();  // Restore rectangle difference clip.

      builder.Save();
      {
        // 2. Draw an oval clip that applies to the image, which will get drawn
        //    in front of the image on the depth buffer.
        builder.ClipOval(DlRect::MakeLTRB(-200, -200, 200, 200));

        Matrix result =
            Matrix(1.0, 0.0, 0.0, 0.0,    //
                   0.0, 1.0, 0.0, 0.0,    //
                   0.0, 0.0, 1.0, 0.003,  //
                   0.0, 0.0, 0.0, 1.0) *
            Matrix::MakeRotationY({Radians{-1.0f + (time++ / 60.0f)}});

        // 3. Draw the rotating image with a perspective transform.
        builder.Transform(result);

        auto image =
            DlImageImpeller::Make(CreateTextureForFixture("airplane.jpg"));
        auto position =
            -DlPoint(image->dimensions().fWidth, image->dimensions().fHeight) *
            0.5;
        builder.DrawImage(image, position, {});
      }
      builder.Restore();  // Restore oval intersect clip.

      // 4. Draw a semi-translucent blue circle atop all previous draws.
      DlPaint paint;
      paint.setColor(DlColor::kBlue().modulateOpacity(0.4));
      builder.DrawCircle(DlPoint(), 230, paint);
    }
    builder.Restore();  // Restore translation.

    return builder.Build();
  };
  ASSERT_TRUE(OpenPlaygroundHere(callback));
}

TEST_P(AiksTest, ImageColorSourceEffectTransform) {
  // Compare with https://fiddle.skia.org/c/6cdc5aefb291fda3833b806ca347a885

  DisplayListBuilder builder;
  auto texture = DlImageImpeller::Make(CreateTextureForFixture("monkey.png"));

  DlPaint paint;
  paint.setColor(DlColor::kWhite());
  builder.DrawPaint(paint);

  // Translation
  {
    DlMatrix matrix = DlMatrix::MakeTranslation({50, 50});
    DlPaint paint;
    paint.setColorSource(DlColorSource::MakeImage(
        texture, DlTileMode::kRepeat, DlTileMode::kRepeat,
        DlImageSampling::kNearestNeighbor, &matrix));

    builder.DrawRect(DlRect::MakeLTRB(0, 0, 100, 100), paint);
  }

  // Rotation/skew
  {
    builder.Save();
    builder.Rotate(45);
    DlPaint paint;

    Matrix matrix(1, -1, 0, 0,  //
                  1, 1, 0, 0,   //
                  0, 0, 1, 0,   //
                  0, 0, 0, 1);
    paint.setColorSource(DlColorSource::MakeImage(
        texture, DlTileMode::kRepeat, DlTileMode::kRepeat,
        DlImageSampling::kNearestNeighbor, &matrix));
    builder.DrawRect(DlRect::MakeLTRB(100, 0, 200, 100), paint);
    builder.Restore();
  }

  // Scale
  {
    builder.Translate(100, 0);
    builder.Scale(100, 100);
    DlPaint paint;

    DlMatrix matrix = DlMatrix::MakeScale({0.005, 0.005, 1});
    paint.setColorSource(DlColorSource::MakeImage(
        texture, DlTileMode::kRepeat, DlTileMode::kRepeat,
        DlImageSampling::kNearestNeighbor, &matrix));

    builder.DrawRect(DlRect::MakeLTRB(0, 0, 1, 1), paint);
  }

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, SubpassWithClearColorOptimization) {
  DisplayListBuilder builder;

  // Use a non-srcOver blend mode to ensure that we don't detect this as an
  // opacity peephole optimization.
  DlPaint paint;
  paint.setColor(DlColor::kBlue().modulateOpacity(0.5));
  paint.setBlendMode(DlBlendMode::kSrc);

  DlRect bounds = DlRect::MakeLTRB(0, 0, 200, 200);
  builder.SaveLayer(bounds, &paint);

  paint.setColor(DlColor::kTransparent());
  paint.setBlendMode(DlBlendMode::kSrc);
  builder.DrawPaint(paint);
  builder.Restore();

  paint.setColor(DlColor::kBlue());
  paint.setBlendMode(DlBlendMode::kDstOver);
  builder.SaveLayer(nullptr, &paint);
  builder.Restore();

  // This playground should appear blank on CI since we are only drawing
  // transparent black. If the clear color optimization is broken, the texture
  // will be filled with NaNs and may produce a magenta texture on macOS or iOS.
  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

// Render a white circle at the top left corner of the screen.
TEST_P(AiksTest, MatrixImageFilterDoesntCullWhenTranslatedFromOffscreen) {
  DisplayListBuilder builder;
  builder.Scale(GetContentScale().x, GetContentScale().y);
  builder.Translate(100, 100);
  // Draw a circle in a SaveLayer at -300, but move it back on-screen with a
  // +300 translation applied by a SaveLayer image filter.
  DlPaint paint;
  DlMatrix translate = DlMatrix::MakeTranslation({300, 0});
  paint.setImageFilter(
      DlImageFilter::MakeMatrix(translate, DlImageSampling::kLinear));
  builder.SaveLayer(nullptr, &paint);

  DlPaint circle_paint;
  circle_paint.setColor(DlColor::kGreen());
  builder.DrawCircle(DlPoint(-300, 0), 100, circle_paint);
  builder.Restore();

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

// Render a white circle at the top left corner of the screen.
TEST_P(AiksTest,
       MatrixImageFilterDoesntCullWhenScaledAndTranslatedFromOffscreen) {
  DisplayListBuilder builder;
  builder.Scale(GetContentScale().x, GetContentScale().y);
  builder.Translate(100, 100);
  // Draw a circle in a SaveLayer at -300, but move it back on-screen with a
  // +300 translation applied by a SaveLayer image filter.

  DlPaint paint;
  paint.setImageFilter(DlImageFilter::MakeMatrix(
      DlMatrix::MakeTranslation({300, 0}) * DlMatrix::MakeScale({2, 2, 1}),
      DlImageSampling::kNearestNeighbor));
  builder.SaveLayer(nullptr, &paint);

  DlPaint circle_paint;
  circle_paint.setColor(DlColor::kGreen());
  builder.DrawCircle(DlPoint(-150, 0), 50, circle_paint);
  builder.Restore();

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

// This should be solid red, if you see a little red box this is broken.
TEST_P(AiksTest, ClearColorOptimizationWhenSubpassIsBiggerThanParentPass) {
  SetWindowSize({400, 400});
  DisplayListBuilder builder;

  builder.Scale(GetContentScale().x, GetContentScale().y);

  DlPaint paint;
  paint.setColor(DlColor::kRed());
  builder.DrawRect(DlRect::MakeLTRB(200, 200, 300, 300), paint);

  paint.setImageFilter(DlImageFilter::MakeMatrix(DlMatrix::MakeScale({2, 2, 1}),
                                                 DlImageSampling::kLinear));
  builder.SaveLayer(nullptr, &paint);
  // Draw a rectangle that would fully cover the parent pass size, but not
  // the subpass that it is rendered in.
  paint.setColor(DlColor::kGreen());
  builder.DrawRect(DlRect::MakeLTRB(0, 0, 400, 400), paint);
  // Draw a bigger rectangle to force the subpass to be bigger.

  paint.setColor(DlColor::kRed());
  builder.DrawRect(DlRect::MakeLTRB(0, 0, 800, 800), paint);
  builder.Restore();

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, EmptySaveLayerIgnoresPaint) {
  DisplayListBuilder builder;
  builder.Scale(GetContentScale().x, GetContentScale().y);

  DlPaint paint;
  paint.setColor(DlColor::kRed());
  builder.DrawPaint(paint);
  builder.ClipRect(DlRect::MakeXYWH(100, 100, 200, 200));
  paint.setColor(DlColor::kBlue());
  builder.SaveLayer(nullptr, &paint);
  builder.Restore();

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, EmptySaveLayerRendersWithClear) {
  DisplayListBuilder builder;
  builder.Scale(GetContentScale().x, GetContentScale().y);
  auto image = DlImageImpeller::Make(CreateTextureForFixture("airplane.jpg"));
  builder.DrawImage(image, DlPoint(10, 10), {});
  builder.ClipRect(DlRect::MakeXYWH(100, 100, 200, 200));

  DlPaint paint;
  paint.setBlendMode(DlBlendMode::kClear);
  builder.SaveLayer(nullptr, &paint);
  builder.Restore();

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest,
       CanPerformSaveLayerWithBoundsAndLargerIntermediateIsNotAllocated) {
  DisplayListBuilder builder;

  DlPaint red;
  red.setColor(DlColor::kRed());

  DlPaint green;
  green.setColor(DlColor::kGreen());

  DlPaint blue;
  blue.setColor(DlColor::kBlue());

  DlPaint save;
  save.setColor(DlColor::kBlack().modulateOpacity(0.5));

  DlRect huge_bounds = DlRect::MakeXYWH(0, 0, 100000, 100000);
  builder.SaveLayer(huge_bounds, &save);

  builder.DrawRect(DlRect::MakeXYWH(0, 0, 100, 100), red);
  builder.DrawRect(DlRect::MakeXYWH(10, 10, 100, 100), green);
  builder.DrawRect(DlRect::MakeXYWH(20, 20, 100, 100), blue);

  builder.Restore();

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

// This makes sure the WideGamut named tests use 16bit float pixel format.
TEST_P(AiksTest, FormatWideGamut) {
  EXPECT_EQ(GetContext()->GetCapabilities()->GetDefaultColorFormat(),
            PixelFormat::kB10G10R10A10XR);
}

TEST_P(AiksTest, FormatSRGB) {
  PixelFormat pixel_format =
      GetContext()->GetCapabilities()->GetDefaultColorFormat();
  EXPECT_TRUE(pixel_format == PixelFormat::kR8G8B8A8UNormInt ||
              pixel_format == PixelFormat::kB8G8R8A8UNormInt)
      << "pixel format: " << PixelFormatToString(pixel_format);
}

TEST_P(AiksTest, CoordinateConversionsAreCorrect) {
  DisplayListBuilder builder;

  // Render a texture directly.
  {
    auto image = DlImageImpeller::Make(CreateTextureForFixture("kalimba.jpg"));

    builder.Save();
    builder.Translate(100, 200);
    builder.Scale(0.5, 0.5);
    builder.DrawImage(image, DlPoint(100.0, 100.0),
                      DlImageSampling::kNearestNeighbor);
    builder.Restore();
  }

  // Render an offscreen rendered texture.
  {
    DlPaint alpha;
    alpha.setColor(DlColor::kRed().modulateOpacity(0.5));

    builder.SaveLayer(nullptr, &alpha);

    DlPaint paint;
    paint.setColor(DlColor::kRed());
    builder.DrawRect(DlRect::MakeXYWH(000, 000, 100, 100), paint);
    paint.setColor(DlColor::kGreen());
    builder.DrawRect(DlRect::MakeXYWH(020, 020, 100, 100), paint);
    paint.setColor(DlColor::kBlue());
    builder.DrawRect(DlRect::MakeXYWH(040, 040, 100, 100), paint);

    builder.Restore();
  }

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanPerformFullScreenMSAA) {
  DisplayListBuilder builder;

  DlPaint paint;
  paint.setColor(DlColor::kRed());
  builder.DrawCircle(DlPoint(250, 250), 125, paint);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanPerformSkew) {
  DisplayListBuilder builder;

  DlPaint red;
  red.setColor(DlColor::kRed());
  builder.Skew(2, 5);
  builder.DrawRect(DlRect::MakeXYWH(0, 0, 100, 100), red);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanPerformSaveLayerWithBounds) {
  DisplayListBuilder builder;

  DlPaint save;
  save.setColor(DlColor::kBlack());

  DlRect save_bounds = DlRect::MakeXYWH(0, 0, 50, 50);
  builder.SaveLayer(save_bounds, &save);

  DlPaint paint;
  paint.setColor(DlColor::kRed());
  builder.DrawRect(DlRect::MakeXYWH(0, 0, 100, 100), paint);
  paint.setColor(DlColor::kGreen());
  builder.DrawRect(DlRect::MakeXYWH(10, 10, 100, 100), paint);
  paint.setColor(DlColor::kBlue());
  builder.DrawRect(DlRect::MakeXYWH(20, 20, 100, 100), paint);

  builder.Restore();

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, FilledRoundRectPathsRenderCorrectly) {
  DisplayListBuilder builder;
  builder.Scale(GetContentScale().x, GetContentScale().y);

  DlPaint paint;
  const int color_count = 3;
  DlColor colors[color_count] = {
      DlColor::kBlue(),
      DlColor::kGreen(),
      DlColor::ARGB(1.0, 220.0f / 255.0f, 20.0f / 255.0f, 60.0f / 255.0f),
  };

  paint.setColor(DlColor::kWhite());
  builder.DrawPaint(paint);

  auto draw_rrect_as_path = [&builder](const DlRect& rect, Scalar x, Scalar y,
                                       const DlPaint& paint) {
    builder.DrawPath(DlPath::MakeRoundRectXY(rect, x, y), paint);
  };

  int c_index = 0;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      paint.setColor(colors[(c_index++) % color_count]);
      draw_rrect_as_path(DlRect::MakeXYWH(i * 100 + 10, j * 100 + 20, 80, 80),
                         i * 5 + 10, j * 5 + 10, paint);
    }
  }
  paint.setColor(colors[(c_index++) % color_count]);
  draw_rrect_as_path(DlRect::MakeXYWH(10, 420, 380, 80), 40, 40, paint);
  paint.setColor(colors[(c_index++) % color_count]);
  draw_rrect_as_path(DlRect::MakeXYWH(410, 20, 80, 380), 40, 40, paint);

  std::vector<DlColor> gradient_colors = {
      DlColor::RGBA(0x1f / 255.0, 0.0, 0x5c / 255.0, 1.0),
      DlColor::RGBA(0x5b / 255.0, 0.0, 0x60 / 255.0, 1.0),
      DlColor::RGBA(0x87 / 255.0, 0x01 / 255.0, 0x60 / 255.0, 1.0),
      DlColor::RGBA(0xac / 255.0, 0x25 / 255.0, 0x53 / 255.0, 1.0),
      DlColor::RGBA(0xe1 / 255.0, 0x6b / 255.0, 0x5c / 255.0, 1.0),
      DlColor::RGBA(0xf3 / 255.0, 0x90 / 255.0, 0x60 / 255.0, 1.0),
      DlColor::RGBA(0xff / 255.0, 0xb5 / 255.0, 0x6b / 250.0, 1.0)};
  std::vector<Scalar> stops = {
      0.0,
      (1.0 / 6.0) * 1,
      (1.0 / 6.0) * 2,
      (1.0 / 6.0) * 3,
      (1.0 / 6.0) * 4,
      (1.0 / 6.0) * 5,
      1.0,
  };
  auto texture = DlImageImpeller::Make(
      CreateTextureForFixture("airplane.jpg",
                              /*enable_mipmapping=*/true));

  paint.setColor(DlColor::kWhite().modulateOpacity(0.1));
  paint.setColorSource(DlColorSource::MakeRadial(
      /*center=*/DlPoint(550, 550),
      /*radius=*/75,
      /*stop_count=*/gradient_colors.size(),
      /*colors=*/gradient_colors.data(),
      /*stops=*/stops.data(),
      /*tile_mode=*/DlTileMode::kMirror));
  for (int i = 1; i <= 10; i++) {
    int j = 11 - i;
    draw_rrect_as_path(DlRect::MakeLTRB(550 - i * 20, 550 - j * 20,  //
                                        550 + i * 20, 550 + j * 20),
                       i * 10, j * 10, paint);
  }
  paint.setColor(DlColor::kWhite().modulateOpacity(0.5));
  paint.setColorSource(DlColorSource::MakeRadial(
      /*center=*/DlPoint(200, 650),
      /*radius=*/75,
      /*stop_count=*/gradient_colors.size(),
      /*colors=*/gradient_colors.data(),
      /*stops=*/stops.data(),
      /*tile_mode=*/DlTileMode::kMirror));
  draw_rrect_as_path(DlRect::MakeLTRB(100, 610, 300, 690), 40, 40, paint);
  draw_rrect_as_path(DlRect::MakeLTRB(160, 550, 240, 750), 40, 40, paint);

  auto matrix = DlMatrix::MakeTranslation({520, 20});
  paint.setColor(DlColor::kWhite().modulateOpacity(0.1));
  paint.setColorSource(DlColorSource::MakeImage(
      texture, DlTileMode::kRepeat, DlTileMode::kRepeat,
      DlImageSampling::kMipmapLinear, &matrix));
  for (int i = 1; i <= 10; i++) {
    int j = 11 - i;
    draw_rrect_as_path(DlRect::MakeLTRB(720 - i * 20, 220 - j * 20,  //
                                        720 + i * 20, 220 + j * 20),
                       i * 10, j * 10, paint);
  }
  matrix = DlMatrix::MakeTranslation({800, 300});
  paint.setColor(DlColor::kWhite().modulateOpacity(0.5));
  paint.setColorSource(DlColorSource::MakeImage(
      texture, DlTileMode::kRepeat, DlTileMode::kRepeat,
      DlImageSampling::kMipmapLinear, &matrix));

  draw_rrect_as_path(DlRect::MakeLTRB(800, 410, 1000, 490), 40, 40, paint);
  draw_rrect_as_path(DlRect::MakeLTRB(860, 350, 940, 550), 40, 40, paint);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CoverageOriginShouldBeAccountedForInSubpasses) {
  auto callback = [&]() -> sk_sp<DisplayList> {
    DisplayListBuilder builder;
    builder.Scale(GetContentScale().x, GetContentScale().y);

    DlPaint alpha;
    alpha.setColor(DlColor::kRed().modulateOpacity(0.5));

    auto current = Point{25, 25};
    const auto offset = Point{25, 25};
    const auto size = Size(100, 100);

    static PlaygroundPoint point_a(Point(40, 40), 10, Color::White());
    static PlaygroundPoint point_b(Point(160, 160), 10, Color::White());
    auto [b0, b1] = DrawPlaygroundLine(point_a, point_b);
    DlRect bounds = DlRect::MakeLTRB(b0.x, b0.y, b1.x, b1.y);

    DlPaint stroke_paint;
    stroke_paint.setColor(DlColor::kYellow());
    stroke_paint.setStrokeWidth(5);
    stroke_paint.setDrawStyle(DlDrawStyle::kStroke);
    builder.DrawRect(bounds, stroke_paint);

    builder.SaveLayer(bounds, &alpha);

    DlPaint paint;
    paint.setColor(DlColor::kRed());
    builder.DrawRect(
        DlRect::MakeXYWH(current.x, current.y, size.width, size.height), paint);

    paint.setColor(DlColor::kGreen());
    current += offset;
    builder.DrawRect(
        DlRect::MakeXYWH(current.x, current.y, size.width, size.height), paint);

    paint.setColor(DlColor::kBlue());
    current += offset;
    builder.DrawRect(
        DlRect::MakeXYWH(current.x, current.y, size.width, size.height), paint);

    builder.Restore();

    return builder.Build();
  };

  ASSERT_TRUE(OpenPlaygroundHere(callback));
}

TEST_P(AiksTest, SaveLayerDrawsBehindSubsequentEntities) {
  // Compare with https://fiddle.skia.org/c/9e03de8567ffb49e7e83f53b64bcf636
  DisplayListBuilder builder;
  DlPaint paint;

  paint.setColor(DlColor::kBlack());
  DlRect rect = DlRect::MakeXYWH(25, 25, 25, 25);
  builder.DrawRect(rect, paint);

  builder.Translate(10, 10);

  DlPaint save_paint;
  builder.SaveLayer(nullptr, &save_paint);

  paint.setColor(DlColor::kGreen());
  builder.DrawRect(rect, paint);

  builder.Restore();

  builder.Translate(10, 10);
  paint.setColor(DlColor::kRed());
  builder.DrawRect(rect, paint);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, SiblingSaveLayerBoundsAreRespected) {
  DisplayListBuilder builder;
  DlPaint paint;
  DlRect rect = DlRect::MakeXYWH(0, 0, 1000, 1000);

  // Black, green, and red squares offset by [10, 10].
  {
    DlPaint save_paint;
    DlRect bounds = DlRect::MakeXYWH(25, 25, 25, 25);
    builder.SaveLayer(bounds, &save_paint);
    paint.setColor(DlColor::kBlack());
    builder.DrawRect(rect, paint);
    builder.Restore();
  }

  {
    DlPaint save_paint;
    DlRect bounds = DlRect::MakeXYWH(35, 35, 25, 25);
    builder.SaveLayer(bounds, &save_paint);
    paint.setColor(DlColor::kGreen());
    builder.DrawRect(rect, paint);
    builder.Restore();
  }

  {
    DlPaint save_paint;
    DlRect bounds = DlRect::MakeXYWH(45, 45, 25, 25);
    builder.SaveLayer(bounds, &save_paint);
    paint.setColor(DlColor::kRed());
    builder.DrawRect(rect, paint);
    builder.Restore();
  }

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanRenderClippedLayers) {
  DisplayListBuilder builder;

  DlPaint paint;
  paint.setColor(DlColor::kWhite());
  builder.DrawPaint(paint);

  // Draw a green circle on the screen.
  {
    // Increase the clip depth for the savelayer to contend with.
    DlPath path = DlPath::MakeCircle(DlPoint(100, 100), 50);
    builder.ClipPath(path);

    DlRect bounds = DlRect::MakeXYWH(50, 50, 100, 100);
    DlPaint save_paint;
    builder.SaveLayer(bounds, &save_paint);

    // Fill the layer with white.
    paint.setColor(DlColor::kWhite());
    builder.DrawRect(DlRect::MakeSize(DlSize(400, 400)), paint);
    // Fill the layer with green, but do so with a color blend that can't be
    // collapsed into the parent pass.
    paint.setColor(DlColor::kGreen());
    paint.setBlendMode(DlBlendMode::kHardLight);
    builder.DrawRect(DlRect::MakeSize(DlSize(400, 400)), paint);
  }

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, SaveLayerFiltersScaleWithTransform) {
  DisplayListBuilder builder;

  builder.Scale(GetContentScale().x, GetContentScale().y);
  builder.Translate(100, 100);

  auto texture = DlImageImpeller::Make(CreateTextureForFixture("boston.jpg"));
  auto draw_image_layer = [&builder, &texture](const DlPaint& paint) {
    builder.SaveLayer(nullptr, &paint);
    builder.DrawImage(texture, DlPoint(), DlImageSampling::kLinear);
    builder.Restore();
  };

  DlPaint effect_paint;
  effect_paint.setMaskFilter(DlBlurMaskFilter::Make(DlBlurStyle::kNormal, 6));
  draw_image_layer(effect_paint);

  builder.Translate(300, 300);
  builder.Scale(3, 3);
  draw_image_layer(effect_paint);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, FastEllipticalRRectMaskBlursRenderCorrectly) {
  DisplayListBuilder builder;

  builder.Scale(GetContentScale().x, GetContentScale().y);
  DlPaint paint;
  paint.setMaskFilter(DlBlurMaskFilter::Make(DlBlurStyle::kNormal, 1));

  DlPaint save_paint;
  save_paint.setColor(DlColor::kWhite());
  builder.DrawPaint(save_paint);

  paint.setColor(DlColor::kBlue());
  for (int i = 0; i < 5; i++) {
    Scalar y = i * 125;
    Scalar y_radius = i * 15;
    for (int j = 0; j < 5; j++) {
      Scalar x = j * 125;
      Scalar x_radius = j * 15;
      builder.DrawRoundRect(
          DlRoundRect::MakeRectXY(
              DlRect::MakeXYWH(x + 50, y + 50, 100.0f, 100.0f),  //
              x_radius, y_radius),
          paint);
    }
  }

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, PipelineBlendSingleParameter) {
  DisplayListBuilder builder;

  // Should render a green square in the middle of a blue circle.
  DlPaint paint;
  builder.SaveLayer(nullptr, &paint);
  {
    builder.Translate(100, 100);
    paint.setColor(DlColor::kBlue());
    builder.DrawCircle(DlPoint(200, 200), 200, paint);
    builder.ClipRect(DlRect::MakeXYWH(100, 100, 200, 200));

    paint.setColor(DlColor::kGreen());
    paint.setBlendMode(DlBlendMode::kSrcOver);
    paint.setImageFilter(DlImageFilter::MakeColorFilter(
        DlColorFilter::MakeBlend(DlColor::kWhite(), DlBlendMode::kDst)));
    builder.DrawCircle(DlPoint(200, 200), 200, paint);
    builder.Restore();
  }

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

// Creates an image matrix filter that scales large content such that it would
// exceed the max texture size. See
// https://github.com/flutter/flutter/issues/128912
TEST_P(AiksTest, MassiveScalingMatrixImageFilter) {
  if (GetBackend() == PlaygroundBackend::kVulkan) {
    GTEST_SKIP() << "Swiftshader is running out of memory on this example.";
  }
  DisplayListBuilder builder(DlRect::MakeSize(DlSize(1000, 1000)));

  auto filter = DlImageFilter::MakeMatrix(
      DlMatrix::MakeScale({0.001, 0.001, 1}), DlImageSampling::kLinear);

  DlPaint paint;
  paint.setImageFilter(filter);
  builder.SaveLayer(nullptr, &paint);
  {
    DlPaint paint;
    paint.setColor(DlColor::kRed());
    builder.DrawRect(DlRect::MakeLTRB(0, 0, 100000, 100000), paint);
  }
  builder.Restore();

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, NoDimplesInRRectPath) {
  Scalar width = 200.f;
  Scalar height = 60.f;
  Scalar corner = 1.f;
  auto callback = [&]() -> sk_sp<DisplayList> {
    if (AiksTest::ImGuiBegin("Controls", nullptr,
                             ImGuiWindowFlags_AlwaysAutoResize)) {
      ImGui::SliderFloat("width", &width, 0, 200);
      ImGui::SliderFloat("height", &height, 0, 200);
      ImGui::SliderFloat("corner", &corner, 0, 1);
      ImGui::End();
    }

    DisplayListBuilder builder;
    builder.Scale(GetContentScale().x, GetContentScale().y);

    DlPaint background_paint;
    background_paint.setColor(DlColor(1, 0.1, 0.1, 0.1, DlColorSpace::kSRGB));
    builder.DrawPaint(background_paint);

    std::vector<DlColor> colors = {DlColor::kRed(), DlColor::kBlue()};
    std::vector<Scalar> stops = {0.0, 1.0};

    DlPaint paint;
    auto gradient = DlColorSource::MakeLinear(DlPoint(0, 0), DlPoint(200, 200),
                                              2, colors.data(), stops.data(),
                                              DlTileMode::kClamp);
    paint.setColorSource(gradient);
    paint.setColor(DlColor::kWhite());
    paint.setDrawStyle(DlDrawStyle::kStroke);
    paint.setStrokeWidth(20);

    builder.Save();
    builder.Translate(100, 100);

    Scalar corner_x = ((1 - corner) * 50) + 50;
    Scalar corner_y = corner * 50 + 50;
    DlRoundRect rrect = DlRoundRect::MakeRectXY(
        DlRect::MakeXYWH(0, 0, width, height), corner_x, corner_y);
    builder.DrawRoundRect(rrect, paint);
    builder.Restore();
    return builder.Build();
  };
  ASSERT_TRUE(OpenPlaygroundHere(callback));
}

TEST_P(AiksTest, BackdropFilterOverUnclosedClip) {
  DisplayListBuilder builder;

  builder.DrawPaint(DlPaint().setColor(DlColor::kWhite()));
  builder.Save();
  {
    builder.ClipRect(DlRect::MakeLTRB(100, 100, 800, 800));

    builder.Save();
    {
      builder.ClipRect(DlRect::MakeLTRB(600, 600, 800, 800));
      builder.DrawPaint(DlPaint().setColor(DlColor::kRed()));
      builder.DrawPaint(DlPaint().setColor(DlColor::kBlue().withAlphaF(0.5)));
      builder.ClipRect(DlRect::MakeLTRB(700, 700, 750, 800));
      builder.DrawPaint(DlPaint().setColor(DlColor::kRed().withAlphaF(0.5)));
    }
    builder.Restore();

    auto image_filter = DlImageFilter::MakeBlur(10, 10, DlTileMode::kDecal);
    builder.SaveLayer(std::nullopt, nullptr, image_filter.get());
  }
  builder.Restore();
  builder.DrawCircle(SkPoint{100, 100}, 100,
                     DlPaint().setColor(DlColor::kAqua()));

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

}  // namespace testing
}  // namespace impeller
