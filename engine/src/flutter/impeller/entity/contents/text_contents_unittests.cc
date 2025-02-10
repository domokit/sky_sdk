// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/impeller/geometry/geometry_asserts.h"
#include "flutter/impeller/renderer/testing/mocks.h"
#include "flutter/testing/testing.h"
#include "impeller/entity/contents/text_contents.h"
#include "impeller/playground/playground_test.h"
#include "impeller/typographer/backends/skia/text_frame_skia.h"
#include "impeller/typographer/backends/skia/typographer_context_skia.h"
#include "third_party/googletest/googletest/include/gtest/gtest.h"
#include "txt/platform.h"

#pragma GCC diagnostic ignored "-Wunreachable-code"

namespace impeller {
namespace testing {

using TextContentsTest = PlaygroundTest;
INSTANTIATE_PLAYGROUND_SUITE(TextContentsTest);

using ::testing::Return;

namespace {
std::shared_ptr<TextFrame> MakeTextFrame(const std::string& text,
                                         const std::string_view& font_fixture,
                                         Scalar font_size) {
  auto c_font_fixture = std::string(font_fixture);
  auto mapping = flutter::testing::OpenFixtureAsSkData(c_font_fixture.c_str());
  if (!mapping) {
    return nullptr;
  }
  sk_sp<SkFontMgr> font_mgr = txt::GetDefaultFontManager();
  SkFont sk_font(font_mgr->makeFromData(mapping), font_size);
  auto blob = SkTextBlob::MakeFromString(text.c_str(), sk_font);
  if (!blob) {
    return nullptr;
  }

  return MakeTextFrameFromTextBlobSkia(blob);
}

std::shared_ptr<GlyphAtlas> CreateGlyphAtlas(
    Context& context,
    const TypographerContext* typographer_context,
    HostBuffer& host_buffer,
    GlyphAtlas::Type type,
    Scalar scale,
    const std::shared_ptr<GlyphAtlasContext>& atlas_context,
    const std::shared_ptr<TextFrame>& frame) {
  frame->SetPerFrameData(scale, /*offset=*/{0, 0},
                         /*properties=*/std::nullopt);
  return typographer_context->CreateGlyphAtlas(context, type, host_buffer,
                                               atlas_context, {frame});
}

Rect PerVertexDataPositionToRect(
    GlyphAtlasPipeline::VertexShader::PerVertexData data[6]) {
  Scalar right = FLT_MIN;
  Scalar left = FLT_MAX;
  Scalar top = FLT_MAX;
  Scalar bottom = FLT_MIN;
  for (int i = 0; i < 6; ++i) {
    right = std::max(right, data[i].position.x);
    left = std::min(left, data[i].position.x);
    top = std::min(top, data[i].position.y);
    bottom = std::max(bottom, data[i].position.y);
  }

  return Rect::MakeLTRB(left, top, right, bottom);
}

Rect PerVertexDataUVToRect(
    GlyphAtlasPipeline::VertexShader::PerVertexData data[6],
    ISize texture_size) {
  Scalar right = FLT_MIN;
  Scalar left = FLT_MAX;
  Scalar top = FLT_MAX;
  Scalar bottom = FLT_MIN;
  for (int i = 0; i < 6; ++i) {
    right = std::max(right, data[i].uv.x * texture_size.width);
    left = std::min(left, data[i].uv.x * texture_size.width);
    top = std::min(top, data[i].uv.y * texture_size.height);
    bottom = std::max(bottom, data[i].uv.y * texture_size.height);
  }

  return Rect::MakeLTRB(left, top, right, bottom);
}

double GetAspectRatio(Rect rect) {
  return static_cast<double>(rect.GetWidth()) / rect.GetHeight();
}
}  // namespace

TEST_P(TextContentsTest, SimpleComputeVertexData) {
#ifndef FML_OS_MACOSX
  GTEST_SKIP() << "Results aren't stable across linux and macos.";
#endif

  GlyphAtlasPipeline::VertexShader::PerVertexData data[6];

  std::shared_ptr<TextFrame> text_frame =
      MakeTextFrame("1", "ahem.ttf", /*font_size=*/50);

  std::shared_ptr<TypographerContext> context = TypographerContextSkia::Make();
  std::shared_ptr<GlyphAtlasContext> atlas_context =
      context->CreateGlyphAtlasContext(GlyphAtlas::Type::kAlphaBitmap);
  std::shared_ptr<HostBuffer> host_buffer = HostBuffer::Create(
      GetContext()->GetResourceAllocator(), GetContext()->GetIdleWaiter());
  ASSERT_TRUE(context && context->IsValid());
  std::shared_ptr<GlyphAtlas> atlas =
      CreateGlyphAtlas(*GetContext(), context.get(), *host_buffer,
                       GlyphAtlas::Type::kAlphaBitmap, /*scale=*/1.0f,
                       atlas_context, text_frame);

  ISize texture_size = atlas->GetTexture()->GetSize();
  TextContents::ComputeVertexData(data, text_frame, /*scale=*/1.0,
                                  /*entity_transform=*/Matrix(),
                                  /*offset=*/Vector2(0, 0),
                                  /*glyph_properties=*/std::nullopt, atlas);

  Rect position_rect = PerVertexDataPositionToRect(data);
  Rect uv_rect = PerVertexDataUVToRect(data, texture_size);
  // The -1 offset comes from Skia in `ComputeGlyphSize`. So since the font size
  // is 50, the math appears to be to get back a 50x50 rect and apply 1 pixel
  // of padding.
  EXPECT_RECT_NEAR(position_rect, Rect::MakeXYWH(-1, -41, 52, 52));
  EXPECT_RECT_NEAR(uv_rect, Rect::MakeXYWH(1.0, 1.0, 52, 52));
}

TEST_P(TextContentsTest, SimpleComputeVertexData2x) {
#ifndef FML_OS_MACOSX
  GTEST_SKIP() << "Results aren't stable across linux and macos.";
#endif

  GlyphAtlasPipeline::VertexShader::PerVertexData data[6];

  std::shared_ptr<TextFrame> text_frame =
      MakeTextFrame("1", "ahem.ttf", /*font_size=*/50);

  std::shared_ptr<TypographerContext> context = TypographerContextSkia::Make();
  std::shared_ptr<GlyphAtlasContext> atlas_context =
      context->CreateGlyphAtlasContext(GlyphAtlas::Type::kAlphaBitmap);
  std::shared_ptr<HostBuffer> host_buffer = HostBuffer::Create(
      GetContext()->GetResourceAllocator(), GetContext()->GetIdleWaiter());
  ASSERT_TRUE(context && context->IsValid());
  Scalar font_scale = 2.f;
  std::shared_ptr<GlyphAtlas> atlas = CreateGlyphAtlas(
      *GetContext(), context.get(), *host_buffer,
      GlyphAtlas::Type::kAlphaBitmap, font_scale, atlas_context, text_frame);

  ISize texture_size = atlas->GetTexture()->GetSize();
  TextContents::ComputeVertexData(
      data, text_frame, font_scale,
      /*entity_transform=*/Matrix::MakeScale({font_scale, font_scale, 1}),
      /*offset=*/Vector2(0, 0),
      /*glyph_properties=*/std::nullopt, atlas);

  Rect position_rect = PerVertexDataPositionToRect(data);
  Rect uv_rect = PerVertexDataUVToRect(data, texture_size);
  EXPECT_RECT_NEAR(position_rect, Rect::MakeXYWH(-1, -81, 102, 102));
  EXPECT_RECT_NEAR(uv_rect, Rect::MakeXYWH(1.0, 1.0, 102, 102));
}

TEST_P(TextContentsTest, MaintainsShape) {
  std::shared_ptr<TextFrame> text_frame =
      MakeTextFrame("th", "ahem.ttf", /*font_size=*/50);

  std::shared_ptr<TypographerContext> context = TypographerContextSkia::Make();
  std::shared_ptr<GlyphAtlasContext> atlas_context =
      context->CreateGlyphAtlasContext(GlyphAtlas::Type::kAlphaBitmap);
  std::shared_ptr<HostBuffer> host_buffer = HostBuffer::Create(
      GetContext()->GetResourceAllocator(), GetContext()->GetIdleWaiter());
  ASSERT_TRUE(context && context->IsValid());
  for (int i = 0; i <= 1000; ++i) {
    Scalar font_scale = 0.440 + (i / 1000.0);
    Rect position_rect[2];
    Rect uv_rect[2];

    {
      GlyphAtlasPipeline::VertexShader::PerVertexData data[12];
      std::shared_ptr<GlyphAtlas> atlas =
          CreateGlyphAtlas(*GetContext(), context.get(), *host_buffer,
                           GlyphAtlas::Type::kAlphaBitmap, font_scale,
                           atlas_context, text_frame);
      ISize texture_size = atlas->GetTexture()->GetSize();

      TextContents::ComputeVertexData(
          data, text_frame, font_scale,
          /*entity_transform=*/Matrix::MakeScale({font_scale, font_scale, 1}),
          /*offset=*/Vector2(0, 0),
          /*glyph_properties=*/std::nullopt, atlas);
      position_rect[0] = PerVertexDataPositionToRect(data);
      uv_rect[0] = PerVertexDataUVToRect(data, texture_size);
      position_rect[1] = PerVertexDataPositionToRect(data + 6);
      uv_rect[1] = PerVertexDataUVToRect(data + 6, texture_size);
    }
    EXPECT_NEAR(GetAspectRatio(position_rect[1]), GetAspectRatio(uv_rect[1]),
                0.001)
        << i;
  }
}

}  // namespace testing
}  // namespace impeller
