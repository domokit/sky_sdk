// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "display_list/dl_sampling_options.h"
#include "flutter/impeller/display_list/aiks_unittests.h"

#include "flutter/display_list/dl_blend_mode.h"
#include "flutter/display_list/dl_builder.h"
#include "flutter/display_list/dl_color.h"
#include "flutter/display_list/dl_paint.h"
#include "flutter/testing/testing.h"
#include "impeller/core/formats.h"
#include "impeller/display_list/dl_atlas_geometry.h"
#include "impeller/display_list/dl_image_impeller.h"
#include "impeller/entity/contents/content_context.h"
#include "impeller/geometry/color.h"
#include "impeller/geometry/scalar.h"
#include "include/core/SkRSXform.h"
#include "include/core/SkRefCnt.h"

namespace impeller {
namespace testing {

using namespace flutter;

namespace {
SkRSXform MakeTranslation(Scalar tx, Scalar ty) {
  return SkRSXform::Make(1, 0, tx, ty);
}

std::tuple<std::vector<SkRect>, std::vector<SkRSXform>, sk_sp<DlImageImpeller>>
CreateTestData(const AiksTest* test) {
  // Draws the image as four squares stiched together.
  auto atlas =
      DlImageImpeller::Make(test->CreateTextureForFixture("bay_bridge.jpg"));
  auto size = atlas->impeller_texture()->GetSize();
  // Divide image into four quadrants.
  Scalar half_width = size.width / 2;
  Scalar half_height = size.height / 2;
  std::vector<SkRect> texture_coordinates = {
      SkRect::MakeLTRB(0, 0, half_width, half_height),
      SkRect::MakeLTRB(half_width, 0, size.width, half_height),
      SkRect::MakeLTRB(0, half_height, half_width, size.height),
      SkRect::MakeLTRB(half_width, half_height, size.width, size.height)};
  // Position quadrants adjacent to eachother.
  std::vector<SkRSXform> transforms = {
      MakeTranslation(0, 0), MakeTranslation(half_width, 0),
      MakeTranslation(0, half_height),
      MakeTranslation(half_width, half_height)};
  return std::make_tuple(texture_coordinates, transforms, atlas);
}

std::tuple<std::vector<DlRect>, std::vector<SkRSXform>, sk_sp<DlImageImpeller>>
CreateDlTestData(const AiksTest* test) {
  // Draws the image as four squares stiched together.
  auto atlas =
      DlImageImpeller::Make(test->CreateTextureForFixture("bay_bridge.jpg"));
  auto size = atlas->impeller_texture()->GetSize();
  // Divide image into four quadrants.
  Scalar half_width = size.width / 2;
  Scalar half_height = size.height / 2;
  std::vector<DlRect> texture_coordinates = {
      DlRect::MakeLTRB(0, 0, half_width, half_height),
      DlRect::MakeLTRB(half_width, 0, size.width, half_height),
      DlRect::MakeLTRB(0, half_height, half_width, size.height),
      DlRect::MakeLTRB(half_width, half_height, size.width, size.height)};
  // Position quadrants adjacent to eachother.
  std::vector<SkRSXform> transforms = {
      MakeTranslation(0, 0), MakeTranslation(half_width, 0),
      MakeTranslation(0, half_height),
      MakeTranslation(half_width, half_height)};
  return std::make_tuple(texture_coordinates, transforms, atlas);
}

}  // namespace

TEST_P(AiksTest, DrawAtlasNoColor) {
  DisplayListBuilder builder;
  auto [texture_coordinates, transforms, atlas] = CreateTestData(this);

  builder.Scale(GetContentScale().x, GetContentScale().y);
  builder.DrawAtlas(atlas, transforms.data(), texture_coordinates.data(),
                    /*colors=*/nullptr, /*count=*/4, DlBlendMode::kSrcOver,
                    DlImageSampling::kNearestNeighbor, nullptr);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, DrawAtlasWithColorAdvanced) {
  DisplayListBuilder builder;
  auto [texture_coordinates, transforms, atlas] = CreateTestData(this);

  std::vector<DlColor> colors = {DlColor::kRed(), DlColor::kGreen(),
                                 DlColor::kBlue(), DlColor::kYellow()};

  builder.Scale(GetContentScale().x, GetContentScale().y);
  builder.DrawAtlas(atlas, transforms.data(), texture_coordinates.data(),
                    colors.data(), /*count=*/4, DlBlendMode::kModulate,
                    DlImageSampling::kNearestNeighbor, /*cullRect=*/nullptr);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, DrawAtlasWithColorSimple) {
  DisplayListBuilder builder;
  // Draws the image as four squares stiched together.
  auto [texture_coordinates, transforms, atlas] = CreateTestData(this);

  std::vector<DlColor> colors = {DlColor::kRed(), DlColor::kGreen(),
                                 DlColor::kBlue(), DlColor::kYellow()};

  builder.Scale(GetContentScale().x, GetContentScale().y);
  builder.DrawAtlas(atlas, transforms.data(), texture_coordinates.data(),
                    colors.data(), /*count=*/4, DlBlendMode::kSrcATop,
                    DlImageSampling::kNearestNeighbor, /*cullRect=*/nullptr);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, DrawAtlasWithOpacity) {
  DisplayListBuilder builder;
  // Draws the image as four squares stiched together slightly
  // opaque
  auto [texture_coordinates, transforms, atlas] = CreateTestData(this);

  DlPaint paint;
  paint.setAlpha(128);
  builder.Scale(GetContentScale().x, GetContentScale().y);
  builder.DrawAtlas(atlas, transforms.data(), texture_coordinates.data(),
                    /*colors=*/nullptr, 4, DlBlendMode::kSrcOver,
                    DlImageSampling::kNearestNeighbor, /*cullRect=*/nullptr,
                    &paint);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, DrawAtlasNoColorFullSize) {
  auto atlas = DlImageImpeller::Make(CreateTextureForFixture("bay_bridge.jpg"));
  auto size = atlas->impeller_texture()->GetSize();
  std::vector<SkRect> texture_coordinates = {
      SkRect::MakeLTRB(0, 0, size.width, size.height)};
  std::vector<SkRSXform> transforms = {MakeTranslation(0, 0)};

  DisplayListBuilder builder;
  builder.Scale(GetContentScale().x, GetContentScale().y);
  builder.DrawAtlas(atlas, transforms.data(), texture_coordinates.data(),
                    /*colors=*/nullptr, /*count=*/1, DlBlendMode::kSrcOver,
                    DlImageSampling::kNearestNeighbor, /*cullRect=*/nullptr);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

// Regression test for https://github.com/flutter/flutter/issues/127374.
TEST_P(AiksTest, DrawAtlasAdvancedAndTransform) {
  DisplayListBuilder builder;
  // Draws the image as four squares stiched together.
  auto [texture_coordinates, transforms, atlas] = CreateTestData(this);

  builder.Scale(0.25, 0.25);
  builder.DrawAtlas(atlas, transforms.data(), texture_coordinates.data(),
                    /*colors=*/nullptr, /*count=*/4, DlBlendMode::kModulate,
                    DlImageSampling::kNearestNeighbor, /*cullRect=*/nullptr);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

// Regression test for https://github.com/flutter/flutter/issues/127374.
TEST_P(AiksTest, DrawAtlasWithColorAdvancedAndTransform) {
  DisplayListBuilder builder;
  // Draws the image as four squares stiched together.
  auto [texture_coordinates, transforms, atlas] = CreateTestData(this);
  std::vector<DlColor> colors = {DlColor::kRed(), DlColor::kGreen(),
                                 DlColor::kBlue(), DlColor::kYellow()};

  builder.Scale(0.25, 0.25);
  builder.DrawAtlas(atlas, transforms.data(), texture_coordinates.data(),
                    colors.data(), /*count=*/4, DlBlendMode::kModulate,
                    DlImageSampling::kNearestNeighbor, /*cullRect=*/nullptr);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, DrawAtlasPlusWideGamut) {
  DisplayListBuilder builder;
  EXPECT_EQ(GetContext()->GetCapabilities()->GetDefaultColorFormat(),
            PixelFormat::kB10G10R10A10XR);

  // Draws the image as four squares stiched together.
  auto [texture_coordinates, transforms, atlas] = CreateTestData(this);
  std::vector<DlColor> colors = {DlColor::kRed(), DlColor::kGreen(),
                                 DlColor::kBlue(), DlColor::kYellow()};

  builder.DrawAtlas(atlas, transforms.data(), texture_coordinates.data(),
                    colors.data(), /*count=*/4, DlBlendMode::kPlus,
                    DlImageSampling::kNearestNeighbor, /*cullRect=*/nullptr);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, DlAtlasGeometryNoBlend) {
  auto [texture_coordinates, transforms, atlas] = CreateDlTestData(this);

  DlAtlasGeometry geom(atlas->impeller_texture(), transforms.data(),
                       texture_coordinates.data(), nullptr, transforms.size(),
                       BlendMode::kSourceOver, {}, std::nullopt);

  EXPECT_FALSE(geom.ShouldUseBlend());
  EXPECT_FALSE(geom.ShouldSkip());

  ContentContext context(GetContext(), nullptr);
  auto vertex_buffer =
      geom.CreateSimpleVertexBuffer(context.GetTransientsBuffer());

  EXPECT_EQ(vertex_buffer.index_type, IndexType::kNone);
  EXPECT_EQ(vertex_buffer.vertex_count, texture_coordinates.size() * 6);
}

TEST_P(AiksTest, DlAtlasGeometryBlend) {
  auto [texture_coordinates, transforms, atlas] = CreateDlTestData(this);

  std::vector<DlColor> colors;
  colors.reserve(texture_coordinates.size());
  for (auto i = 0u; i < texture_coordinates.size(); i++) {
    colors.push_back(DlColor::ARGB(0.5, 1, 1, 1));
  }
  DlAtlasGeometry geom(atlas->impeller_texture(), transforms.data(),
                       texture_coordinates.data(), colors.data(),
                       transforms.size(), BlendMode::kSourceOver, {},
                       std::nullopt);

  EXPECT_TRUE(geom.ShouldUseBlend());
  EXPECT_FALSE(geom.ShouldSkip());

  ContentContext context(GetContext(), nullptr);
  auto vertex_buffer =
      geom.CreateBlendVertexBuffer(context.GetTransientsBuffer());

  EXPECT_EQ(vertex_buffer.index_type, IndexType::kNone);
  EXPECT_EQ(vertex_buffer.vertex_count, texture_coordinates.size() * 6);
}

TEST_P(AiksTest, DlAtlasGeometryColorButNoBlend) {
  auto [texture_coordinates, transforms, atlas] = CreateDlTestData(this);

  std::vector<DlColor> colors;
  colors.reserve(texture_coordinates.size());
  for (auto i = 0u; i < texture_coordinates.size(); i++) {
    colors.push_back(DlColor::ARGB(0.5, 1, 1, 1));
  }
  DlAtlasGeometry geom(atlas->impeller_texture(), transforms.data(),
                       texture_coordinates.data(), colors.data(),
                       transforms.size(), BlendMode::kSource, {}, std::nullopt);

  // Src blend mode means that colors would be ignored, even if provided.
  EXPECT_FALSE(geom.ShouldUseBlend());
  EXPECT_FALSE(geom.ShouldSkip());
}

TEST_P(AiksTest, DlAtlasGeometrySkip) {
  auto [texture_coordinates, transforms, atlas] = CreateDlTestData(this);

  std::vector<DlColor> colors;
  colors.reserve(texture_coordinates.size());
  for (auto i = 0u; i < texture_coordinates.size(); i++) {
    colors.push_back(DlColor::ARGB(0.5, 1, 1, 1));
  }
  DlAtlasGeometry geom(atlas->impeller_texture(), transforms.data(),
                       texture_coordinates.data(), colors.data(),
                       transforms.size(), BlendMode::kClear, {}, std::nullopt);
  EXPECT_TRUE(geom.ShouldSkip());
}

}  // namespace testing
}  // namespace impeller
