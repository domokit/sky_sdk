// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "display_list/display_list.h"
#include "display_list/dl_blend_mode.h"
#include "display_list/dl_tile_mode.h"
#include "display_list/effects/dl_color_source.h"
#include "display_list/effects/dl_mask_filter.h"
#include "flutter/display_list/dl_builder.h"
#include "flutter/display_list/dl_color.h"
#include "flutter/display_list/dl_paint.h"
#include "flutter/fml/build_config.h"
#include "flutter/impeller/display_list/aiks_unittests.h"
#include "flutter/testing/testing.h"
#include "impeller/geometry/matrix.h"
#include "impeller/typographer/backends/skia/text_frame_skia.h"
#include "include/core/SkMatrix.h"
#include "include/core/SkRect.h"

#include "txt/platform.h"

using namespace flutter;
/////////////////////////////////////////////////////

namespace impeller {
namespace testing {

struct TextRenderOptions {
  bool stroke = false;
  Scalar font_size = 50;
  Scalar stroke_width = 1;
  DlColor color = DlColor::kYellow();
  SkPoint position = SkPoint::Make(100, 200);
  std::shared_ptr<DlMaskFilter> filter;
};

bool RenderTextInCanvasSkia(const std::shared_ptr<Context>& context,
                            DisplayListBuilder& canvas,
                            const std::string& text,
                            const std::string_view& font_fixture,
                            const TextRenderOptions& options = {}) {
  // Draw the baseline.
  DlPaint paint;
  paint.setColor(DlColor::kAqua().withAlpha(255 * 0.25));
  canvas.DrawRect(SkRect::MakeXYWH(options.position.x() - 50,
                                   options.position.y(), 900, 10),
                  paint);

  // Mark the point at which the text is drawn.
  paint.setColor(DlColor::kRed().withAlpha(255 * 0.25));
  canvas.DrawCircle(options.position, 5.0, paint);

  // Construct the text blob.
  auto c_font_fixture = std::string(font_fixture);
  auto mapping = flutter::testing::OpenFixtureAsSkData(c_font_fixture.c_str());
  if (!mapping) {
    return false;
  }
  sk_sp<SkFontMgr> font_mgr = txt::GetDefaultFontManager();
  SkFont sk_font(font_mgr->makeFromData(mapping), options.font_size);
  auto blob = SkTextBlob::MakeFromString(text.c_str(), sk_font);
  if (!blob) {
    return false;
  }

  // Create the Impeller text frame and draw it at the designated baseline.
  auto frame = MakeTextFrameFromTextBlobSkia(blob);

  DlPaint text_paint;
  text_paint.setColor(options.color);
  text_paint.setMaskFilter(options.filter);
  text_paint.setStrokeWidth(options.stroke_width);
  text_paint.setDrawStyle(options.stroke ? DlDrawStyle::kStroke
                                         : DlDrawStyle::kFill);
  canvas.DrawTextFrame(frame, options.position.x(), options.position.y(),
                       text_paint);
  return true;
}

TEST_P(AiksTest, CanRenderTextFrame) {
  DisplayListBuilder builder;

  DlPaint paint;
  paint.setColor(DlColor::ARGB(1, 0.1, 0.1, 0.1));
  builder.DrawPaint(paint);
  ASSERT_TRUE(RenderTextInCanvasSkia(
      GetContext(), builder, "the quick brown fox jumped over the lazy dog!.?",
      "Roboto-Regular.ttf"));

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanRenderTextFrameWithInvertedTransform) {
  DisplayListBuilder builder;

  DlPaint paint;
  paint.setColor(DlColor::ARGB(1, 0.1, 0.1, 0.1));
  builder.DrawPaint(paint);
  builder.Translate(1000, 0);
  builder.Scale(-1, 1);

  ASSERT_TRUE(RenderTextInCanvasSkia(
      GetContext(), builder, "the quick brown fox jumped over the lazy dog!.?",
      "Roboto-Regular.ttf"));

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanRenderStrokedTextFrame) {
  DisplayListBuilder builder;

  DlPaint paint;
  paint.setColor(DlColor::ARGB(1, 0.1, 0.1, 0.1));
  builder.DrawPaint(paint);

  ASSERT_TRUE(RenderTextInCanvasSkia(
      GetContext(), builder, "the quick brown fox jumped over the lazy dog!.?",
      "Roboto-Regular.ttf",
      {
          .stroke = true,
      }));
  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanRenderTextStrokeWidth) {
  DisplayListBuilder builder;

  DlPaint paint;
  paint.setColor(DlColor::ARGB(1, 0.1, 0.1, 0.1));
  builder.DrawPaint(paint);

  ASSERT_TRUE(RenderTextInCanvasSkia(GetContext(), builder, "LMNOP VWXYZ",
                                     "Roboto-Medium.ttf",
                                     {
                                         .stroke = true,
                                         .stroke_width = 4,
                                     }));
  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanRenderTextFrameWithHalfScaling) {
  DisplayListBuilder builder;

  DlPaint paint;
  paint.setColor(DlColor::ARGB(1, 0.1, 0.1, 0.1));
  builder.DrawPaint(paint);
  builder.Scale(0.5, 0.5);

  ASSERT_TRUE(RenderTextInCanvasSkia(
      GetContext(), builder, "the quick brown fox jumped over the lazy dog!.?",
      "Roboto-Regular.ttf"));
  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanRenderTextFrameWithFractionScaling) {
  DisplayListBuilder builder;

  DlPaint paint;
  paint.setColor(DlColor::ARGB(1, 0.1, 0.1, 0.1));
  builder.DrawPaint(paint);
  builder.Scale(2.625, 2.625);

  ASSERT_TRUE(RenderTextInCanvasSkia(
      GetContext(), builder, "the quick brown fox jumped over the lazy dog!.?",
      "Roboto-Regular.ttf"));
  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, TextFrameSubpixelAlignment) {
  // "Random" numbers between 0 and 1. Hardcoded to avoid flakiness in goldens.
  std::array<Scalar, 20> phase_offsets = {
      7.82637e-06, 0.131538,  0.755605,   0.45865,   0.532767,
      0.218959,    0.0470446, 0.678865,   0.679296,  0.934693,
      0.383502,    0.519416,  0.830965,   0.0345721, 0.0534616,
      0.5297,      0.671149,  0.00769819, 0.383416,  0.0668422};
  auto callback = [&]() -> sk_sp<DisplayList> {
    static float font_size = 20;
    static float phase_variation = 0.2;
    static float speed = 0.5;
    static float magnitude = 100;
    if (AiksTest::ImGuiBegin("Controls", nullptr,
                             ImGuiWindowFlags_AlwaysAutoResize)) {
      ImGui::SliderFloat("Font size", &font_size, 5, 50);
      ImGui::SliderFloat("Phase variation", &phase_variation, 0, 1);
      ImGui::SliderFloat("Oscillation speed", &speed, 0, 2);
      ImGui::SliderFloat("Oscillation magnitude", &magnitude, 0, 300);
      ImGui::End();
    }

    DisplayListBuilder builder;
    builder.Scale(GetContentScale().x, GetContentScale().y);

    for (size_t i = 0; i < phase_offsets.size(); i++) {
      SkPoint position = SkPoint::Make(
          200 +
              magnitude * std::sin((-phase_offsets[i] * k2Pi * phase_variation +
                                    GetSecondsElapsed() * speed)),  //
          200 + i * font_size * 1.1                                 //
      );
      if (!RenderTextInCanvasSkia(
              GetContext(), builder,
              "the quick brown fox jumped over "
              "the lazy dog!.?",
              "Roboto-Regular.ttf",
              {.font_size = font_size, .position = position})) {
        return nullptr;
      }
    }
    return builder.Build();
  };

  ASSERT_TRUE(OpenPlaygroundHere(callback));
}

TEST_P(AiksTest, CanRenderItalicizedText) {
  DisplayListBuilder builder;

  DlPaint paint;
  paint.setColor(DlColor::ARGB(1, 0.1, 0.1, 0.1));
  builder.DrawPaint(paint);

  ASSERT_TRUE(RenderTextInCanvasSkia(
      GetContext(), builder, "the quick brown fox jumped over the lazy dog!.?",
      "HomemadeApple.ttf"));
  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

static constexpr std::string_view kFontFixture =
#if FML_OS_MACOSX
    "Apple Color Emoji.ttc";
#else
    "NotoColorEmoji.ttf";
#endif

TEST_P(AiksTest, CanRenderEmojiTextFrame) {
  DisplayListBuilder builder;

  DlPaint paint;
  paint.setColor(DlColor::ARGB(1, 0.1, 0.1, 0.1));
  builder.DrawPaint(paint);

  ASSERT_TRUE(RenderTextInCanvasSkia(
      GetContext(), builder, "😀 😃 😄 😁 😆 😅 😂 🤣 🥲 😊", kFontFixture));
  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanRenderEmojiTextFrameWithBlur) {
  DisplayListBuilder builder;

  builder.Scale(GetContentScale().x, GetContentScale().y);
  DlPaint paint;
  paint.setColor(DlColor::ARGB(1, 0.1, 0.1, 0.1));
  builder.DrawPaint(paint);

  ASSERT_TRUE(RenderTextInCanvasSkia(
      GetContext(), builder, "😀 😃 😄 😁 😆 😅 😂 🤣 🥲 😊", kFontFixture,
      TextRenderOptions{
          .color = DlColor::kBlue(),
          .filter = DlBlurMaskFilter::Make(DlBlurStyle::kNormal, 4)}));
  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanRenderEmojiTextFrameWithAlpha) {
  DisplayListBuilder builder;

  DlPaint paint;
  paint.setColor(DlColor::ARGB(1, 0.1, 0.1, 0.1));
  builder.DrawPaint(paint);

  ASSERT_TRUE(RenderTextInCanvasSkia(
      GetContext(), builder, "😀 😃 😄 😁 😆 😅 😂 🤣 🥲 😊", kFontFixture,
      {.color = DlColor::kBlack().modulateOpacity(0.5)}));
  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanRenderTextInSaveLayer) {
  DisplayListBuilder builder;

  DlPaint paint;
  paint.setColor(DlColor::ARGB(0.1, 0.1, 0.1, 0.1));
  builder.DrawPaint(paint);

  builder.Translate(100, 100);
  builder.Scale(0.5, 0.5);

  // Blend the layer with the parent pass using kClear to expose the coverage.
  paint.setBlendMode(DlBlendMode::kClear);
  builder.SaveLayer(nullptr, &paint);
  ASSERT_TRUE(RenderTextInCanvasSkia(
      GetContext(), builder, "the quick brown fox jumped over the lazy dog!.?",
      "Roboto-Regular.ttf"));
  builder.Restore();

  // Render the text again over the cleared coverage rect.
  ASSERT_TRUE(RenderTextInCanvasSkia(
      GetContext(), builder, "the quick brown fox jumped over the lazy dog!.?",
      "Roboto-Regular.ttf"));

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanRenderTextOutsideBoundaries) {
  DisplayListBuilder builder;
  builder.Translate(200, 150);

  // Construct the text blob.
  auto mapping = flutter::testing::OpenFixtureAsSkData("wtf.otf");
  ASSERT_NE(mapping, nullptr);

  Scalar font_size = 80;
  sk_sp<SkFontMgr> font_mgr = txt::GetDefaultFontManager();
  SkFont sk_font(font_mgr->makeFromData(mapping), font_size);

  DlPaint text_paint;
  text_paint.setColor(DlColor::kBlue().withAlpha(255 * 0.8));

  struct {
    SkPoint position;
    const char* text;
  } text[] = {{SkPoint::Make(0, 0), "0F0F0F0"},
              {SkPoint::Make(1, 2), "789"},
              {SkPoint::Make(1, 3), "456"},
              {SkPoint::Make(1, 4), "123"},
              {SkPoint::Make(0, 6), "0F0F0F0"}};
  for (auto& t : text) {
    builder.Save();
    builder.Translate(t.position.x() * font_size * 2,
                      t.position.y() * font_size * 1.1);
    {
      auto blob = SkTextBlob::MakeFromString(t.text, sk_font);
      ASSERT_NE(blob, nullptr);
      auto frame = MakeTextFrameFromTextBlobSkia(blob);
      builder.DrawTextFrame(frame, 0, 0, text_paint);
    }
    builder.Restore();
  }

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, TextRotated) {
  DisplayListBuilder builder;

  builder.Scale(GetContentScale().x, GetContentScale().y);
  DlPaint paint;
  paint.setColor(DlColor::ARGB(0.1, 0.1, 0.1, 1.0));
  builder.DrawPaint(paint);

  builder.Transform(SkM44::ColMajor(Matrix(0.25, -0.3, 0, -0.002,  //
                                           0, 0.5, 0, 0,           //
                                           0, 0, 0.3, 0,           //
                                           100, 100, 0, 1.3)
                                        .m));
  ASSERT_TRUE(RenderTextInCanvasSkia(
      GetContext(), builder, "the quick brown fox jumped over the lazy dog!.?",
      "Roboto-Regular.ttf"));

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, DrawScaledTextWithPerspectiveNoSaveLayer) {
  DisplayListBuilder builder;

  Matrix matrix = Matrix(1.0, 0.0, 0.0, 0.0,    //
                         0.0, 1.0, 0.0, 0.0,    //
                         0.0, 0.0, 1.0, 0.01,   //
                         0.0, 0.0, 0.0, 1.0) *  //
                  Matrix::MakeRotationY({Degrees{10}});

  builder.Transform(SkM44::ColMajor(matrix.m));

  ASSERT_TRUE(RenderTextInCanvasSkia(GetContext(), builder, "Hello world",
                                     "Roboto-Regular.ttf"));
  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, DrawScaledTextWithPerspectiveSaveLayer) {
  DisplayListBuilder builder;

  Matrix matrix = Matrix(1.0, 0.0, 0.0, 0.0,    //
                         0.0, 1.0, 0.0, 0.0,    //
                         0.0, 0.0, 1.0, 0.01,   //
                         0.0, 0.0, 0.0, 1.0) *  //
                  Matrix::MakeRotationY({Degrees{10}});

  DlPaint save_paint;
  SkRect window_bounds =
      SkRect::MakeXYWH(0, 0, GetWindowSize().width, GetWindowSize().height);
  // Note: bounds were not needed by the AIKS version, which may indicate a bug.
  builder.SaveLayer(&window_bounds, &save_paint);
  builder.Transform(SkM44::ColMajor(matrix.m));

  ASSERT_TRUE(RenderTextInCanvasSkia(GetContext(), builder, "Hello world",
                                     "Roboto-Regular.ttf"));

  builder.Restore();
  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanRenderTextWithLargePerspectiveTransform) {
  // Verifies that text scales are clamped to work around
  // https://github.com/flutter/flutter/issues/136112 .

  DisplayListBuilder builder;

  DlPaint save_paint;
  builder.SaveLayer(nullptr, &save_paint);
  builder.Transform(SkM44::ColMajor(Matrix(2000, 0, 0, 0,   //
                                           0, 2000, 0, 0,   //
                                           0, 0, -1, 9000,  //
                                           0, 0, -1, 7000   //
                                           )
                                        .m));

  ASSERT_TRUE(RenderTextInCanvasSkia(GetContext(), builder, "Hello world",
                                     "Roboto-Regular.ttf"));
  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanRenderTextWithPerspectiveTransformInSublist) {
  DisplayListBuilder text_builder;
  ASSERT_TRUE(RenderTextInCanvasSkia(GetContext(), text_builder, "Hello world",
                                     "Roboto-Regular.ttf"));
  auto text_display_list = text_builder.Build();

  DisplayListBuilder builder;

  Matrix matrix = Matrix::MakeRow(2.0, 0.0, 0.0, 0.0,  //
                                  0.0, 2.0, 0.0, 0.0,  //
                                  0.0, 0.0, 1.0, 0.0,  //
                                  0.0, 0.002, 0.0, 1.0);

  DlPaint save_paint;
  SkRect window_bounds =
      SkRect::MakeXYWH(0, 0, GetWindowSize().width, GetWindowSize().height);
  builder.SaveLayer(&window_bounds, &save_paint);
  builder.Transform(SkM44::ColMajor(matrix.m));
  builder.DrawDisplayList(text_display_list, 1.0f);
  builder.Restore();

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

// This currently renders solid blue, as the support for text color sources was
// moved into DLDispatching. Path data requires the SkTextBlobs which are not
// used in impeller::TextFrames.
TEST_P(AiksTest, TextForegroundShaderWithTransform) {
  auto mapping = flutter::testing::OpenFixtureAsSkData("Roboto-Regular.ttf");
  ASSERT_NE(mapping, nullptr);

  Scalar font_size = 100;
  sk_sp<SkFontMgr> font_mgr = txt::GetDefaultFontManager();
  SkFont sk_font(font_mgr->makeFromData(mapping), font_size);

  DlPaint text_paint;
  text_paint.setColor(DlColor::kBlue());

  std::vector<DlColor> colors = {DlColor::RGBA(0.9568, 0.2627, 0.2118, 1.0),
                                 DlColor::RGBA(0.1294, 0.5882, 0.9529, 1.0)};
  std::vector<Scalar> stops = {
      0.0,
      1.0,
  };
  text_paint.setColorSource(DlColorSource::MakeLinear(
      /*start_point=*/DlPoint(0, 0),     //
      /*end_point=*/DlPoint(100, 100),   //
      /*stop_count=*/2,                  //
      /*colors=*/colors.data(),          //
      /*stops=*/stops.data(),            //
      /*tile_mode=*/DlTileMode::kRepeat  //
      ));

  DisplayListBuilder builder;
  builder.Translate(100, 100);
  builder.Rotate(45);

  auto blob = SkTextBlob::MakeFromString("Hello", sk_font);
  ASSERT_NE(blob, nullptr);
  auto frame = MakeTextFrameFromTextBlobSkia(blob);
  builder.DrawTextFrame(frame, 0, 0, text_paint);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

// Regression test for https://github.com/flutter/flutter/issues/157885.
TEST_P(AiksTest, DifferenceClipsMustRenderIdenticallyAcrossBackends) {
  DisplayListBuilder builder;

  DlPaint paint;
  DlColor clear_color(1.0, 0.5, 0.5, 0.5, DlColorSpace::kSRGB);
  paint.setColor(clear_color);
  builder.DrawPaint(paint);

  DlMatrix identity = {
      1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
      0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
  };
  builder.Save();
  builder.Transform(identity);

  DlRect frame = DlRect::MakeLTRB(1.0, 1.0, 1278.0, 763.0);
  DlColor white(1.0, 1.0, 1.0, 1.0, DlColorSpace::kSRGB);
  paint.setColor(white);
  builder.DrawRect(frame, paint);

  builder.Save();
  builder.ClipRect(frame, DlCanvas::ClipOp::kIntersect);

  DlMatrix rect_xform = {
      0.8241262, 0.56640625, 0.0, 0.0, -0.56640625, 0.8241262, 0.0, 0.0,
      0.0,       0.0,        1.0, 0.0, 271.1137,    489.4733,  0.0, 1.0,
  };
  builder.Save();
  builder.Transform(rect_xform);

  DlRect rect = DlRect::MakeLTRB(0.0, 0.0, 100.0, 100.0);
  DlColor bluish(1.0, 0.184, 0.501, 0.929, DlColorSpace::kSRGB);
  paint.setColor(bluish);
  DlRoundRect rrect = DlRoundRect::MakeRectRadius(rect, 18.0);
  builder.DrawRoundRect(rrect, paint);

  builder.Save();
  builder.ClipRect(rect, DlCanvas::ClipOp::kIntersect);
  builder.Restore();

  builder.Restore();

  DlMatrix path_xform = {
      1.0, 0.0, 0.0, 0.0, 0.0,   1.0,   0.0, 0.0,
      0.0, 0.0, 1.0, 0.0, 675.0, 279.5, 0.0, 1.0,
  };
  builder.Save();
  builder.Transform(path_xform);

  SkPath path;
  path.moveTo(87.5, 349.5);
  path.lineTo(25.0, 29.5);
  path.lineTo(150.0, 118.0);
  path.lineTo(25.0, 118.0);
  path.lineTo(150.0, 29.5);
  path.close();

  DlColor fill_color(1.0, 1.0, 0.0, 0.0, DlColorSpace::kSRGB);
  DlColor stroke_color(1.0, 0.0, 0.0, 0.0, DlColorSpace::kSRGB);
  paint.setColor(fill_color);
  paint.setDrawStyle(DlDrawStyle::kFill);
  builder.DrawPath(DlPath(path), paint);

  paint.setColor(stroke_color);
  paint.setStrokeWidth(2.0);
  paint.setDrawStyle(DlDrawStyle::kStroke);
  builder.DrawPath(path, paint);

  builder.Restore();
  builder.Restore();
  builder.Restore();

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

}  // namespace testing
}  // namespace impeller
