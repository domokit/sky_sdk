
// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <vector>
#include "display_list/dl_sampling_options.h"
#include "display_list/dl_tile_mode.h"
#include "display_list/effects/dl_color_filter.h"
#include "display_list/effects/dl_color_source.h"
#include "display_list/effects/dl_image_filter.h"
#include "display_list/geometry/dl_geometry_types.h"
#include "display_list/geometry/dl_path.h"
#include "display_list/image/dl_image.h"
#include "flutter/impeller/display_list/aiks_unittests.h"

#include "flutter/display_list/dl_blend_mode.h"
#include "flutter/display_list/dl_builder.h"
#include "flutter/display_list/dl_color.h"
#include "flutter/display_list/dl_paint.h"
#include "flutter/testing/testing.h"
#include "imgui.h"
#include "impeller/display_list/dl_dispatcher.h"
#include "impeller/display_list/dl_image_impeller.h"
#include "impeller/geometry/scalar.h"
#include "include/core/SkCanvas.h"
#include "include/core/SkMatrix.h"
#include "include/core/SkPath.h"
#include "include/core/SkRSXform.h"
#include "include/core/SkRefCnt.h"

namespace impeller {
namespace testing {

using namespace flutter;

namespace {
SkRect GetCullRect(ISize window_size) {
  return SkRect::MakeSize(SkSize::Make(window_size.width, window_size.height));
}
}  // namespace

TEST_P(AiksTest, CollapsedDrawPaintInSubpass) {
  DisplayListBuilder builder;

  DlPaint paint;
  paint.setColor(DlColor::kYellow());
  paint.setBlendMode(DlBlendMode::kSrc);
  builder.DrawPaint(paint);

  DlPaint save_paint;
  save_paint.setBlendMode(DlBlendMode::kMultiply);
  builder.SaveLayer(nullptr, &save_paint);

  DlPaint draw_paint;
  draw_paint.setColor(DlColor::kCornflowerBlue().modulateOpacity(0.75f));
  builder.DrawPaint(draw_paint);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CollapsedDrawPaintInSubpassBackdropFilter) {
  // Bug: https://github.com/flutter/flutter/issues/131576
  DisplayListBuilder builder;

  DlPaint paint;
  paint.setColor(DlColor::kYellow());
  paint.setBlendMode(DlBlendMode::kSrc);
  builder.DrawPaint(paint);

  auto filter = DlImageFilter::MakeBlur(20.0, 20.0, DlTileMode::kDecal);
  builder.SaveLayer(nullptr, nullptr, filter.get());

  DlPaint draw_paint;
  draw_paint.setColor(DlColor::kCornflowerBlue());
  builder.DrawPaint(draw_paint);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, ColorMatrixFilterSubpassCollapseOptimization) {
  DisplayListBuilder builder(GetCullRect(GetWindowSize()));

  const float matrix[20] = {
      -1.0, 0,    0,    1.0, 0,  //
      0,    -1.0, 0,    1.0, 0,  //
      0,    0,    -1.0, 1.0, 0,  //
      1.0,  1.0,  1.0,  1.0, 0   //
  };
  auto filter = DlColorFilter::MakeMatrix(matrix);

  DlPaint paint;
  paint.setColorFilter(filter);
  builder.SaveLayer(nullptr, &paint);

  builder.Translate(500, 300);
  builder.Rotate(120);  // 120 deg

  DlPaint draw_paint;
  draw_paint.setColor(DlColor::kBlue());
  builder.DrawRect(SkRect::MakeXYWH(100, 100, 200, 200), draw_paint);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, LinearToSrgbFilterSubpassCollapseOptimization) {
  DisplayListBuilder builder(GetCullRect(GetWindowSize()));

  DlPaint paint;
  paint.setColorFilter(DlColorFilter::MakeLinearToSrgbGamma());
  builder.SaveLayer(nullptr, &paint);

  builder.Translate(500, 300);
  builder.Rotate(120);  // 120 deg.

  DlPaint draw_paint;
  draw_paint.setColor(DlColor::kBlue());
  builder.DrawRect(SkRect::MakeXYWH(100, 100, 200, 200), draw_paint);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, SrgbToLinearFilterSubpassCollapseOptimization) {
  DisplayListBuilder builder(GetCullRect(GetWindowSize()));

  DlPaint paint;
  paint.setColorFilter(DlColorFilter::MakeLinearToSrgbGamma());
  builder.SaveLayer(nullptr, &paint);

  builder.Translate(500, 300);
  builder.Rotate(120);  // 120 deg

  DlPaint draw_paint;
  draw_paint.setColor(DlColor::kBlue());
  builder.DrawRect(SkRect::MakeXYWH(100, 100, 200, 200), draw_paint);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, TranslucentSaveLayerDrawsCorrectly) {
  DisplayListBuilder builder(GetCullRect(GetWindowSize()));

  DlPaint paint;
  paint.setColor(DlColor::kBlue());
  builder.DrawRect(SkRect::MakeXYWH(100, 100, 300, 300), paint);

  DlPaint save_paint;
  save_paint.setColor(DlColor::kBlack().withAlpha(128));
  builder.SaveLayer(nullptr, &save_paint);
  builder.DrawRect(SkRect::MakeXYWH(100, 500, 300, 300), paint);
  builder.Restore();

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, TranslucentSaveLayerWithBlendColorFilterDrawsCorrectly) {
  DisplayListBuilder builder(GetCullRect(GetWindowSize()));

  DlPaint paint;
  paint.setColor(DlColor::kBlue());
  builder.DrawRect(SkRect::MakeXYWH(100, 100, 300, 300), paint);

  DlPaint save_paint;
  paint.setColor(DlColor::kBlack().withAlpha(128));
  paint.setColorFilter(
      DlColorFilter::MakeBlend(DlColor::kRed(), DlBlendMode::kDstOver));
  builder.Save();
  builder.ClipRect(SkRect::MakeXYWH(100, 500, 300, 300));
  builder.SaveLayer(nullptr, &paint);

  DlPaint draw_paint;
  draw_paint.setColor(DlColor::kBlue());
  builder.DrawRect(SkRect::MakeXYWH(100, 500, 300, 300), draw_paint);
  builder.Restore();
  builder.Restore();

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, TranslucentSaveLayerWithBlendImageFilterDrawsCorrectly) {
  DisplayListBuilder builder(GetCullRect(GetWindowSize()));

  DlPaint paint;
  paint.setColor(DlColor::kBlue());
  builder.DrawRect(SkRect::MakeXYWH(100, 100, 300, 300), paint);

  DlPaint save_paint;
  save_paint.setColor(DlColor::kBlack().withAlpha(128));
  save_paint.setImageFilter(DlImageFilter::MakeColorFilter(
      DlColorFilter::MakeBlend(DlColor::kRed(), DlBlendMode::kDstOver)));

  builder.SaveLayer(nullptr, &save_paint);

  DlPaint draw_paint;
  draw_paint.setColor(DlColor::kBlue());
  builder.DrawRect(SkRect::MakeXYWH(100, 500, 300, 300), draw_paint);
  builder.Restore();

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, TranslucentSaveLayerWithColorAndImageFilterDrawsCorrectly) {
  DisplayListBuilder builder(GetCullRect(GetWindowSize()));

  DlPaint paint;
  paint.setColor(DlColor::kBlue());
  builder.DrawRect(SkRect::MakeXYWH(100, 100, 300, 300), paint);

  DlPaint save_paint;
  save_paint.setColor(DlColor::kBlack().withAlpha(128));
  save_paint.setColorFilter(
      DlColorFilter::MakeBlend(DlColor::kRed(), DlBlendMode::kDstOver));
  builder.Save();
  builder.ClipRect(SkRect::MakeXYWH(100, 500, 300, 300));
  builder.SaveLayer(nullptr, &save_paint);

  DlPaint draw_paint;
  draw_paint.setColor(DlColor::kBlue());
  builder.DrawRect(SkRect::MakeXYWH(100, 500, 300, 300), draw_paint);
  builder.Restore();
  builder.Restore();

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, ImageFilteredUnboundedSaveLayerWithUnboundedContents) {
  DisplayListBuilder builder(GetCullRect(GetWindowSize()));
  builder.Scale(GetContentScale().x, GetContentScale().y);

  DlPaint save_paint;
  save_paint.setImageFilter(
      DlImageFilter::MakeBlur(10.0, 10.0, DlTileMode::kDecal));
  builder.SaveLayer(nullptr, &save_paint);

  {
    // DrawPaint to verify correct behavior when the contents are unbounded.
    DlPaint draw_paint;
    draw_paint.setColor(DlColor::kYellow());
    builder.DrawPaint(draw_paint);

    // Contrasting rectangle to see interior blurring
    DlPaint draw_rect;
    draw_rect.setColor(DlColor::kBlue());
    builder.DrawRect(SkRect::MakeLTRB(125, 125, 175, 175), draw_rect);
  }
  builder.Restore();

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, TranslucentSaveLayerImageDrawsCorrectly) {
  DisplayListBuilder builder(GetCullRect(GetWindowSize()));

  auto image = DlImageImpeller::Make(CreateTextureForFixture("airplane.jpg"));
  builder.DrawImage(image, SkPoint{100, 100}, DlImageSampling::kMipmapLinear);

  DlPaint paint;
  paint.setColor(DlColor::kBlack().withAlpha(128));
  builder.SaveLayer(nullptr, &paint);
  builder.DrawImage(image, SkPoint{100, 500}, DlImageSampling::kMipmapLinear);
  builder.Restore();

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, TranslucentSaveLayerWithColorMatrixColorFilterDrawsCorrectly) {
  DisplayListBuilder builder(GetCullRect(GetWindowSize()));

  auto image = DlImageImpeller::Make(CreateTextureForFixture("airplane.jpg"));
  builder.DrawImage(image, SkPoint{100, 100}, {});

  const float matrix[20] = {
      1, 0, 0, 0, 0,  //
      0, 1, 0, 0, 0,  //
      0, 0, 1, 0, 0,  //
      0, 0, 0, 2, 0   //
  };
  DlPaint paint;
  paint.setColor(DlColor::kBlack().withAlpha(128));
  paint.setColorFilter(DlColorFilter::MakeMatrix(matrix));
  builder.SaveLayer(nullptr, &paint);
  builder.DrawImage(image, SkPoint{100, 500}, {});
  builder.Restore();

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, TranslucentSaveLayerWithColorMatrixImageFilterDrawsCorrectly) {
  DisplayListBuilder builder(GetCullRect(GetWindowSize()));

  auto image = DlImageImpeller::Make(CreateTextureForFixture("airplane.jpg"));
  builder.DrawImage(image, SkPoint{100, 100}, {});

  const float matrix[20] = {
      1, 0, 0, 0, 0,  //
      0, 1, 0, 0, 0,  //
      0, 0, 1, 0, 0,  //
      0, 0, 0, 2, 0   //
  };
  DlPaint paint;
  paint.setColor(DlColor::kBlack().withAlpha(128));
  paint.setColorFilter(DlColorFilter::MakeMatrix(matrix));
  builder.SaveLayer(nullptr, &paint);
  builder.DrawImage(image, SkPoint{100, 500}, {});
  builder.Restore();

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest,
       TranslucentSaveLayerWithColorFilterAndImageFilterDrawsCorrectly) {
  DisplayListBuilder builder(GetCullRect(GetWindowSize()));

  auto image = DlImageImpeller::Make(CreateTextureForFixture("airplane.jpg"));
  builder.DrawImage(image, SkPoint{100, 100}, {});

  const float matrix[20] = {
      1, 0,   0, 0,   0,  //
      0, 1,   0, 0,   0,  //
      0, 0.2, 1, 0,   0,  //
      0, 0,   0, 0.5, 0   //
  };
  DlPaint paint;
  paint.setColor(DlColor::kBlack().withAlpha(128));
  paint.setImageFilter(
      DlImageFilter::MakeColorFilter(DlColorFilter::MakeMatrix(matrix)));
  paint.setColorFilter(
      DlColorFilter::MakeBlend(DlColor::kGreen(), DlBlendMode::kModulate));
  builder.SaveLayer(nullptr, &paint);
  builder.DrawImage(image, SkPoint{100, 500}, {});
  builder.Restore();

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, TranslucentSaveLayerWithAdvancedBlendModeDrawsCorrectly) {
  DisplayListBuilder builder(GetCullRect(GetWindowSize()));

  DlPaint paint;
  paint.setColor(DlColor::kRed());
  builder.DrawRect(SkRect::MakeXYWH(0, 0, 400, 400), paint);

  DlPaint save_paint;
  save_paint.setAlpha(128);
  save_paint.setBlendMode(DlBlendMode::kLighten);
  builder.SaveLayer(nullptr, &save_paint);

  DlPaint draw_paint;
  draw_paint.setColor(DlColor::kGreen());
  builder.DrawCircle(SkPoint{200, 200}, 100, draw_paint);
  builder.Restore();

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

/// This is a regression check for https://github.com/flutter/engine/pull/41129
/// The entire screen is green if successful. If failing, no frames will render,
/// or the entire screen will be transparent black.
TEST_P(AiksTest, CanRenderTinyOverlappingSubpasses) {
  DisplayListBuilder builder(GetCullRect(GetWindowSize()));

  DlPaint paint;
  paint.setColor(DlColor::kRed());
  builder.DrawPaint(paint);

  // Draw two overlapping subpixel circles.
  builder.SaveLayer({});

  DlPaint yellow_paint;
  yellow_paint.setColor(DlColor::kYellow());
  builder.DrawCircle(SkPoint{100, 100}, 0.1, yellow_paint);
  builder.Restore();
  builder.SaveLayer({});
  builder.DrawCircle(SkPoint{100, 100}, 0.1, yellow_paint);
  builder.Restore();

  DlPaint draw_paint;
  draw_paint.setColor(DlColor::kGreen());
  builder.DrawPaint(draw_paint);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanRenderDestructiveSaveLayer) {
  DisplayListBuilder builder(GetCullRect(GetWindowSize()));

  DlPaint paint;
  paint.setColor(DlColor::kRed());
  builder.DrawPaint(paint);
  // Draw an empty savelayer with a destructive blend mode, which will replace
  // the entire red screen with fully transparent black, except for the green
  // circle drawn within the layer.

  DlPaint save_paint;
  save_paint.setBlendMode(DlBlendMode::kSrc);
  builder.SaveLayer(nullptr, &save_paint);

  DlPaint draw_paint;
  draw_paint.setColor(DlColor::kGreen());
  builder.DrawCircle(SkPoint{300, 300}, 100, draw_paint);
  builder.Restore();

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanDrawPoints) {
  std::vector<SkPoint> points = {
      {0, 0},      //
      {100, 100},  //
      {100, 0},    //
      {0, 100},    //
      {0, 0},      //
      {48, 48},    //
      {52, 52},    //
  };
  DlPaint paint_round;
  paint_round.setColor(DlColor::kYellow().withAlpha(128));
  paint_round.setStrokeCap(DlStrokeCap::kRound);
  paint_round.setStrokeWidth(20);

  DlPaint paint_square;
  paint_square.setColor(DlColor::kYellow().withAlpha(128));
  paint_square.setStrokeCap(DlStrokeCap::kSquare);
  paint_square.setStrokeWidth(20);

  DlPaint background;
  background.setColor(DlColor::kBlack());

  DisplayListBuilder builder(GetCullRect(GetWindowSize()));
  builder.DrawPaint(background);
  builder.Translate(200, 200);

  builder.DrawPoints(DlCanvas::PointMode::kPoints, points.size(), points.data(),
                     paint_round);
  builder.Translate(150, 0);
  builder.DrawPoints(DlCanvas::PointMode::kPoints, points.size(), points.data(),
                     paint_square);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanDrawPointsWithTextureMap) {
  auto texture = DlImageImpeller::Make(
      CreateTextureForFixture("table_mountain_nx.png",
                              /*enable_mipmapping=*/true));

  std::vector<SkPoint> points = {
      {0, 0},      //
      {100, 100},  //
      {100, 0},    //
      {0, 100},    //
      {0, 0},      //
      {48, 48},    //
      {52, 52},    //
  };

  auto image_src =
      DlColorSource::MakeImage(texture, DlTileMode::kClamp, DlTileMode::kClamp);

  DlPaint paint_round;
  paint_round.setStrokeCap(DlStrokeCap::kRound);
  paint_round.setColorSource(image_src);
  paint_round.setStrokeWidth(200);

  DlPaint paint_square;
  paint_square.setStrokeCap(DlStrokeCap::kSquare);
  paint_square.setColorSource(image_src);
  paint_square.setStrokeWidth(200);

  DisplayListBuilder builder(GetCullRect(GetWindowSize()));
  builder.Translate(200, 200);

  builder.DrawPoints(DlCanvas::PointMode::kPoints, points.size(), points.data(),
                     paint_round);
  builder.Translate(150, 0);
  builder.DrawPoints(DlCanvas::PointMode::kPoints, points.size(), points.data(),
                     paint_square);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, MipmapGenerationWorksCorrectly) {
  TextureDescriptor texture_descriptor;
  texture_descriptor.size = ISize{1024, 1024};
  texture_descriptor.format = PixelFormat::kR8G8B8A8UNormInt;
  texture_descriptor.storage_mode = StorageMode::kHostVisible;
  texture_descriptor.mip_count = texture_descriptor.size.MipCount();

  std::vector<uint8_t> bytes(4194304);
  bool alternate = false;
  for (auto i = 0u; i < 4194304; i += 4) {
    if (alternate) {
      bytes[i] = 255;
      bytes[i + 1] = 0;
      bytes[i + 2] = 0;
      bytes[i + 3] = 255;
    } else {
      bytes[i] = 0;
      bytes[i + 1] = 255;
      bytes[i + 2] = 0;
      bytes[i + 3] = 255;
    }
    alternate = !alternate;
  }

  ASSERT_EQ(texture_descriptor.GetByteSizeOfBaseMipLevel(), bytes.size());
  auto mapping = std::make_shared<fml::NonOwnedMapping>(
      bytes.data(),                                   // data
      texture_descriptor.GetByteSizeOfBaseMipLevel()  // size
  );
  auto texture =
      GetContext()->GetResourceAllocator()->CreateTexture(texture_descriptor);

  auto device_buffer =
      GetContext()->GetResourceAllocator()->CreateBufferWithCopy(*mapping);
  auto command_buffer = GetContext()->CreateCommandBuffer();
  auto blit_pass = command_buffer->CreateBlitPass();

  blit_pass->AddCopy(DeviceBuffer::AsBufferView(std::move(device_buffer)),
                     texture);
  blit_pass->GenerateMipmap(texture);
  EXPECT_TRUE(blit_pass->EncodeCommands(GetContext()->GetResourceAllocator()));
  EXPECT_TRUE(GetContext()->GetCommandQueue()->Submit({command_buffer}).ok());

  auto image = DlImageImpeller::Make(texture);

  DisplayListBuilder builder;
  builder.DrawImageRect(
      image,
      SkRect::MakeSize(
          SkSize::Make(texture->GetSize().width, texture->GetSize().height)),
      SkRect::MakeLTRB(0, 0, 100, 100), DlImageSampling::kMipmapLinear);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

// https://github.com/flutter/flutter/issues/146648
TEST_P(AiksTest, StrokedPathWithMoveToThenCloseDrawnCorrectly) {
  SkPath path;
  path.moveTo(0, 400)
      .lineTo(0, 0)
      .lineTo(400, 0)
      // MoveTo implicitly adds a contour, ensure that close doesn't
      // add another nearly-empty contour.
      .moveTo(0, 400)
      .close();

  DisplayListBuilder builder;
  builder.Translate(50, 50);

  DlPaint paint;
  paint.setColor(DlColor::kBlue());
  paint.setStrokeCap(DlStrokeCap::kRound);
  paint.setStrokeWidth(10);
  paint.setDrawStyle(DlDrawStyle::kStroke);
  builder.DrawPath(path, paint);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, SetContentsWithRegion) {
  auto bridge = CreateTextureForFixture("bay_bridge.jpg");

  // Replace part of the texture with a red rectangle.
  std::vector<uint8_t> bytes(100 * 100 * 4);
  for (auto i = 0u; i < bytes.size(); i += 4) {
    bytes[i] = 255;
    bytes[i + 1] = 0;
    bytes[i + 2] = 0;
    bytes[i + 3] = 255;
  }
  auto mapping =
      std::make_shared<fml::NonOwnedMapping>(bytes.data(), bytes.size());
  auto device_buffer =
      GetContext()->GetResourceAllocator()->CreateBufferWithCopy(*mapping);
  auto cmd_buffer = GetContext()->CreateCommandBuffer();
  auto blit_pass = cmd_buffer->CreateBlitPass();
  blit_pass->AddCopy(DeviceBuffer::AsBufferView(device_buffer), bridge,
                     IRect::MakeLTRB(50, 50, 150, 150));

  auto did_submit =
      blit_pass->EncodeCommands(GetContext()->GetResourceAllocator()) &&
      GetContext()->GetCommandQueue()->Submit({std::move(cmd_buffer)}).ok();
  ASSERT_TRUE(did_submit);

  auto image = DlImageImpeller::Make(bridge);

  DisplayListBuilder builder;
  builder.DrawImage(image, SkPoint{0, 0}, {});

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

// Regression test for https://github.com/flutter/flutter/issues/134678.
TEST_P(AiksTest, ReleasesTextureOnTeardown) {
  auto context = MakeContext();
  std::weak_ptr<Texture> weak_texture;

  {
    auto texture = CreateTextureForFixture("table_mountain_nx.png");
    weak_texture = texture;

    DisplayListBuilder builder;
    builder.Scale(GetContentScale().x, GetContentScale().y);
    builder.Translate(100.0f, 100.0f);

    DlPaint paint;
    paint.setColorSource(DlColorSource::MakeImage(
        DlImageImpeller::Make(texture), DlTileMode::kClamp, DlTileMode::kClamp,
        DlImageSampling::kLinear, nullptr));

    builder.DrawRect(SkRect::MakeXYWH(0, 0, 600, 600), paint);

    ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
  }

  // See https://github.com/flutter/flutter/issues/134751.
  //
  // If the fence waiter was working this may not be released by the end of the
  // scope above. Adding a manual shutdown so that future changes to the fence
  // waiter will not flake this test.
  context->Shutdown();

  // The texture should be released by now.
  ASSERT_TRUE(weak_texture.expired()) << "When the texture is no longer in use "
                                         "by the backend, it should be "
                                         "released.";
}

TEST_P(AiksTest, MatrixImageFilterMagnify) {
  Scalar scale = 2.0;
  auto callback = [&]() -> sk_sp<DisplayList> {
    if (AiksTest::ImGuiBegin("Controls", nullptr,
                             ImGuiWindowFlags_AlwaysAutoResize)) {
      ImGui::SliderFloat("Scale", &scale, 1, 2);
      ImGui::End();
    }
    DisplayListBuilder builder;
    builder.Scale(GetContentScale().x, GetContentScale().y);
    auto image = DlImageImpeller::Make(CreateTextureForFixture("airplane.jpg"));

    builder.Translate(600, -200);

    DlMatrix matrix = DlMatrix::MakeScale({scale, scale, 1});
    DlPaint paint;
    paint.setImageFilter(
        DlImageFilter::MakeMatrix(matrix, DlImageSampling::kLinear));
    builder.SaveLayer(nullptr, &paint);

    DlPaint rect_paint;
    rect_paint.setAlpha(0.5 * 255);
    builder.DrawImage(image, SkPoint{0, 0}, DlImageSampling::kLinear,
                      &rect_paint);
    builder.Restore();

    return builder.Build();
  };

  ASSERT_TRUE(OpenPlaygroundHere(callback));
}

TEST_P(AiksTest, ImageFilteredSaveLayerWithUnboundedContents) {
  DisplayListBuilder builder;
  builder.Scale(GetContentScale().x, GetContentScale().y);

  auto test = [&builder](const std::shared_ptr<DlImageFilter>& filter) {
    auto DrawLine = [&builder](const SkPoint& p0, const SkPoint& p1,
                               const DlPaint& p) {
      DlPaint paint = p;
      paint.setDrawStyle(DlDrawStyle::kStroke);
      builder.DrawPath(SkPath::Line(p0, p1), paint);
    };
    // Registration marks for the edge of the SaveLayer
    DlPaint paint;
    paint.setColor(DlColor::kWhite());
    DrawLine(SkPoint::Make(75, 100), SkPoint::Make(225, 100), paint);
    DrawLine(SkPoint::Make(75, 200), SkPoint::Make(225, 200), paint);
    DrawLine(SkPoint::Make(100, 75), SkPoint::Make(100, 225), paint);
    DrawLine(SkPoint::Make(200, 75), SkPoint::Make(200, 225), paint);

    DlPaint save_paint;
    save_paint.setImageFilter(filter);
    SkRect bounds = SkRect::MakeLTRB(100, 100, 200, 200);
    builder.SaveLayer(&bounds, &save_paint);

    {
      // DrawPaint to verify correct behavior when the contents are unbounded.
      DlPaint paint;
      paint.setColor(DlColor::kYellow());
      builder.DrawPaint(paint);

      // Contrasting rectangle to see interior blurring
      paint.setColor(DlColor::kBlue());
      builder.DrawRect(SkRect::MakeLTRB(125, 125, 175, 175), paint);
    }
    builder.Restore();
  };

  test(DlImageFilter::MakeBlur(10.0, 10.0, DlTileMode::kDecal));

  builder.Translate(200.0, 0.0);

  test(DlImageFilter::MakeDilate(10.0, 10.0));

  builder.Translate(200.0, 0.0);

  test(DlImageFilter::MakeErode(10.0, 10.0));

  builder.Translate(-400.0, 200.0);

  DlMatrix matrix = DlMatrix::MakeRotationZ(DlDegrees(10));

  auto rotate_filter =
      DlImageFilter::MakeMatrix(matrix, DlImageSampling::kLinear);
  test(rotate_filter);

  builder.Translate(200.0, 0.0);

  const float m[20] = {
      0, 1, 0, 0, 0,  //
      0, 0, 1, 0, 0,  //
      1, 0, 0, 0, 0,  //
      0, 0, 0, 1, 0   //
  };
  auto rgb_swap_filter =
      DlImageFilter::MakeColorFilter(DlColorFilter::MakeMatrix(m));
  test(rgb_swap_filter);

  builder.Translate(200.0, 0.0);

  test(DlImageFilter::MakeCompose(rotate_filter, rgb_swap_filter));

  builder.Translate(-400.0, 200.0);

  test(rotate_filter->makeWithLocalMatrix(
      DlMatrix::MakeTranslation({25.0, 25.0})));

  builder.Translate(200.0, 0.0);

  test(rgb_swap_filter->makeWithLocalMatrix(
      DlMatrix::MakeTranslation({25.0, 25.0})));

  builder.Translate(200.0, 0.0);

  test(DlImageFilter::MakeCompose(rotate_filter, rgb_swap_filter)
           ->makeWithLocalMatrix(DlMatrix::MakeTranslation({25.0, 25.0})));

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, MatrixBackdropFilter) {
  DisplayListBuilder builder;

  DlPaint paint;
  paint.setColor(DlColor::kBlack());
  builder.DrawPaint(paint);
  builder.SaveLayer(nullptr, nullptr);
  {
    DlPaint paint;
    paint.setColor(DlColor::kGreen().withAlpha(0.5 * 255));
    paint.setBlendMode(DlBlendMode::kPlus);

    DlPaint rect_paint;
    rect_paint.setColor(DlColor::kRed());
    rect_paint.setStrokeWidth(4);
    rect_paint.setDrawStyle(DlDrawStyle::kStroke);
    builder.DrawRect(SkRect::MakeLTRB(0, 0, 300, 300), rect_paint);
    builder.DrawCircle(SkPoint::Make(200, 200), 100, paint);
    // Should render a second circle, centered on the bottom-right-most edge of
    // the circle.
    DlMatrix matrix = DlMatrix::MakeTranslation({(100 + 100 * k1OverSqrt2),
                                                 (100 + 100 * k1OverSqrt2)}) *
                      DlMatrix::MakeScale({0.5, 0.5, 1}) *
                      DlMatrix::MakeTranslation({-100, -100});
    auto backdrop_filter =
        DlImageFilter::MakeMatrix(matrix, DlImageSampling::kLinear);
    builder.SaveLayer(nullptr, nullptr, backdrop_filter.get());
    builder.Restore();
  }
  builder.Restore();

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, MatrixSaveLayerFilter) {
  DisplayListBuilder builder;

  DlPaint paint;
  paint.setColor(DlColor::kBlack());
  builder.DrawPaint(paint);
  builder.SaveLayer(nullptr, nullptr);
  {
    paint.setColor(DlColor::kGreen().withAlpha(255 * 0.5));
    paint.setBlendMode(DlBlendMode::kPlus);
    builder.DrawCircle(SkPoint{200, 200}, 100, paint);
    // Should render a second circle, centered on the bottom-right-most edge of
    // the circle.

    DlMatrix matrix = DlMatrix::MakeTranslation({(200 + 100 * k1OverSqrt2),
                                                 (200 + 100 * k1OverSqrt2)}) *
                      DlMatrix::MakeScale({0.5, 0.5, 1}) *
                      DlMatrix::MakeTranslation({-200, -200});
    DlPaint save_paint;
    save_paint.setImageFilter(
        DlImageFilter::MakeMatrix(matrix, DlImageSampling::kLinear));

    builder.SaveLayer(nullptr, &save_paint);

    DlPaint circle_paint;
    circle_paint.setColor(DlColor::kGreen().withAlpha(255 * 0.5));
    circle_paint.setBlendMode(DlBlendMode::kPlus);
    builder.DrawCircle(SkPoint{200, 200}, 100, circle_paint);
    builder.Restore();
  }
  builder.Restore();

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

// Regression test for flutter/flutter#152780
TEST_P(AiksTest, CanDrawScaledPointsSmallScaleLargeRadius) {
  std::vector<SkPoint> point = {
      {0, 0},  //
  };

  DlPaint paint;
  paint.setStrokeCap(DlStrokeCap::kRound);
  paint.setColor(DlColor::kRed());
  paint.setStrokeWidth(100 * 1000000);

  DisplayListBuilder builder(GetCullRect(GetWindowSize()));
  builder.Translate(200, 200);
  builder.Scale(0.000001, 0.000001);

  builder.DrawPoints(DlCanvas::PointMode::kPoints, point.size(), point.data(),
                     paint);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

// Regression test for flutter/flutter#152780
TEST_P(AiksTest, CanDrawScaledPointsLargeScaleSmallRadius) {
  std::vector<SkPoint> point = {
      {0, 0},  //
  };

  DlPaint paint;
  paint.setStrokeCap(DlStrokeCap::kRound);
  paint.setColor(DlColor::kRed());
  paint.setStrokeWidth(100 * 0.000001);

  DisplayListBuilder builder(GetCullRect(GetWindowSize()));
  builder.Translate(200, 200);
  builder.Scale(1000000, 1000000);

  builder.DrawPoints(DlCanvas::PointMode::kPoints, point.size(), point.data(),
                     paint);
  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, TransparentShadowProducesCorrectColor) {
  DisplayListBuilder builder;
  builder.Save();
  builder.Scale(1.618, 1.618);
  SkPath path = SkPath{}.addRect(SkRect::MakeXYWH(0, 0, 200, 100));

  builder.DrawShadow(path, flutter::DlColor::kTransparent(), 15, false, 1);
  builder.Restore();

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

// Regression test for https://github.com/flutter/flutter/issues/130613
TEST_P(AiksTest, DispatcherDoesNotCullPerspectiveTransformedChildDisplayLists) {
  flutter::DisplayListBuilder sub_builder(true);
  sub_builder.DrawRect(SkRect::MakeXYWH(0, 0, 50, 50),
                       flutter::DlPaint(flutter::DlColor::kRed()));
  auto display_list = sub_builder.Build();

  AiksContext context(GetContext(), nullptr);
  RenderTarget render_target =
      context.GetContentContext().GetRenderTargetCache()->CreateOffscreen(
          *context.GetContext(), {2400, 1800}, 1);

  DisplayListBuilder builder;

  builder.Scale(2.0, 2.0);
  builder.Translate(-93.0, 0.0);

  // clang-format off
  builder.TransformFullPerspective(
     0.8, -0.2, -0.1, -0.0,
     0.0,  1.0,  0.0,  0.0,
     1.4,  1.3,  1.0,  0.0,
    63.2, 65.3, 48.6,  1.1
  );
  // clang-format on
  builder.Translate(35.0, 75.0);
  builder.DrawDisplayList(display_list, 1.0f);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

// Results in a 100x100 green square. If any red is drawn, there is a bug.
TEST_P(AiksTest, BackdropRestoreUsesCorrectCoverageForFirstRestoredClip) {
  DisplayListBuilder builder;

  DlPaint paint;
  // Add a difference clip that cuts out the bottom right corner
  builder.ClipRect(SkRect::MakeLTRB(50, 50, 100, 100),
                   DlCanvas::ClipOp::kDifference);

  // Draw a red rectangle that's going to be completely covered by green later.
  paint.setColor(DlColor::kRed());
  builder.DrawRect(SkRect::MakeLTRB(0, 0, 100, 100), paint);

  // Add a clip restricting the backdrop filter to the top right corner.
  auto count = builder.GetSaveCount();
  builder.Save();
  {
    builder.ClipRect(SkRect::MakeLTRB(0, 0, 100, 100));
    {
      // Create a save layer with a backdrop blur filter.
      auto backdrop_filter =
          DlImageFilter::MakeBlur(10.0, 10.0, DlTileMode::kDecal);
      builder.SaveLayer(nullptr, nullptr, backdrop_filter.get());
    }
  }
  builder.RestoreToCount(count);

  // Finally, overwrite all the previous stuff with green.
  paint.setColor(DlColor::kGreen());
  builder.DrawRect(SkRect::MakeLTRB(0, 0, 100, 100), paint);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanPictureConvertToImage) {
  DisplayListBuilder recorder_canvas;
  DlPaint paint;
  paint.setColor(DlColor::RGBA(0.9568, 0.2627, 0.2118, 1.0));
  recorder_canvas.DrawRect(SkRect::MakeXYWH(100.0, 100.0, 600, 600), paint);
  paint.setColor(DlColor::RGBA(0.1294, 0.5882, 0.9529, 1.0));
  recorder_canvas.DrawRect(SkRect::MakeXYWH(200.0, 200.0, 600, 600), paint);

  DisplayListBuilder canvas;
  AiksContext renderer(GetContext(), nullptr);
  paint.setColor(DlColor::kTransparent());
  canvas.DrawPaint(paint);

  auto image =
      DisplayListToTexture(recorder_canvas.Build(), {1000, 1000}, renderer);
  if (image) {
    canvas.DrawImage(DlImageImpeller::Make(image), SkPoint{}, {});
    paint.setColor(DlColor::RGBA(0.1, 0.1, 0.1, 0.2));
    canvas.DrawRect(SkRect::MakeSize({1000, 1000}), paint);
  }

  ASSERT_TRUE(OpenPlaygroundHere(canvas.Build()));
}

// Regression test for https://github.com/flutter/flutter/issues/142358 .
// Without a change to force render pass construction the image is left in an
// undefined layout and triggers a validation error.
TEST_P(AiksTest, CanEmptyPictureConvertToImage) {
  DisplayListBuilder recorder_builder;

  DisplayListBuilder builder;
  AiksContext renderer(GetContext(), nullptr);

  DlPaint paint;
  paint.setColor(DlColor::kTransparent());
  builder.DrawPaint(paint);

  auto result_image =
      DisplayListToTexture(builder.Build(), ISize{1000, 1000}, renderer);
  if (result_image) {
    recorder_builder.DrawImage(DlImageImpeller::Make(result_image), SkPoint{},
                               {});

    paint.setColor(DlColor::RGBA(0.1, 0.1, 0.1, 0.2));
    recorder_builder.DrawRect(SkRect::MakeSize({1000, 1000}), paint);
  }

  ASSERT_TRUE(OpenPlaygroundHere(recorder_builder.Build()));
}

TEST_P(AiksTest, DepthValuesForLineMode) {
  // Ensures that the additional draws created by line/polygon mode all
  // have the same depth values.
  DisplayListBuilder builder;

  SkPath path = SkPath::Circle(100, 100, 100);

  builder.DrawPath(path, DlPaint()
                             .setColor(DlColor::kRed())
                             .setDrawStyle(DlDrawStyle::kStroke)
                             .setStrokeWidth(5));
  builder.Save();
  builder.ClipPath(path);

  std::vector<DlPoint> points = {
      DlPoint::MakeXY(0, -200), DlPoint::MakeXY(400, 200),
      DlPoint::MakeXY(0, -100), DlPoint::MakeXY(400, 300),
      DlPoint::MakeXY(0, 0),    DlPoint::MakeXY(400, 400),
      DlPoint::MakeXY(0, 100),  DlPoint::MakeXY(400, 500),
      DlPoint::MakeXY(0, 150),  DlPoint::MakeXY(400, 600)};

  builder.DrawPoints(DisplayListBuilder::PointMode::kLines, points.size(),
                     points.data(),
                     DlPaint().setColor(DlColor::kBlue()).setStrokeWidth(10));
  builder.Restore();

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, DepthValuesForPolygonMode) {
  // Ensures that the additional draws created by line/polygon mode all
  // have the same depth values.
  DisplayListBuilder builder;

  SkPath path = SkPath::Circle(100, 100, 100);

  builder.DrawPath(path, DlPaint()
                             .setColor(DlColor::kRed())
                             .setDrawStyle(DlDrawStyle::kStroke)
                             .setStrokeWidth(5));
  builder.Save();
  builder.ClipPath(path);

  std::vector<DlPoint> points = {
      DlPoint::MakeXY(0, -200), DlPoint::MakeXY(400, 200),
      DlPoint::MakeXY(0, -100), DlPoint::MakeXY(400, 300),
      DlPoint::MakeXY(0, 0),    DlPoint::MakeXY(400, 400),
      DlPoint::MakeXY(0, 100),  DlPoint::MakeXY(400, 500),
      DlPoint::MakeXY(0, 150),  DlPoint::MakeXY(400, 600)};

  builder.DrawPoints(DisplayListBuilder::PointMode::kPolygon, points.size(),
                     points.data(),
                     DlPaint().setColor(DlColor::kBlue()).setStrokeWidth(10));
  builder.Restore();

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

}  // namespace testing
}  // namespace impeller
