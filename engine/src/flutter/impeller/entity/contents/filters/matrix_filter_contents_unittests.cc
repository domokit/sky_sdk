// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/testing/testing.h"
#include "gmock/gmock.h"
#include "impeller/entity/contents/filters/matrix_filter_contents.h"
#include "impeller/entity/entity_playground.h"
#include "impeller/geometry/geometry_asserts.h"

namespace impeller {
namespace testing {

class MatrixFilterContentsTest : public EntityPlayground {
 public:
  /// Create a texture that has been cleared to transparent black.
  std::shared_ptr<Texture> MakeTexture(ISize size) {
    std::shared_ptr<CommandBuffer> command_buffer =
        GetContentContext()->GetContext()->CreateCommandBuffer();
    if (!command_buffer) {
      return nullptr;
    }

    auto render_target = GetContentContext()->MakeSubpass(
        "Clear Subpass", size, command_buffer,
        [](const ContentContext&, RenderPass&) { return true; });

    if (!GetContentContext()
             ->GetContext()
             ->GetCommandQueue()
             ->Submit(/*buffers=*/{command_buffer})
             .ok()) {
      return nullptr;
    }

    if (render_target.ok()) {
      return render_target.value().GetRenderTargetTexture();
    }
    return nullptr;
  }
};

INSTANTIATE_PLAYGROUND_SUITE(MatrixFilterContentsTest);

TEST(MatrixFilterContentsTest, CoverageEmpty) {
  MatrixFilterContents contents;
  FilterInput::Vector inputs = {};
  Entity entity;
  std::optional<Rect> coverage =
      contents.GetFilterCoverage(inputs, entity, /*effect_transform=*/Matrix());
  ASSERT_FALSE(coverage.has_value());
}

TEST(MatrixFilterContentsTest, CoverageSimple) {
  MatrixFilterContents contents;
  FilterInput::Vector inputs = {
      FilterInput::Make(Rect::MakeLTRB(10, 10, 110, 110))};
  Entity entity;
  std::optional<Rect> coverage =
      contents.GetFilterCoverage(inputs, entity, /*effect_transform=*/Matrix());

  ASSERT_EQ(coverage, Rect::MakeLTRB(10, 10, 110, 110));
}

TEST(MatrixFilterContentsTest, Coverage2x) {
  MatrixFilterContents contents;
  contents.SetMatrix(Matrix::MakeScale({2.0, 2.0, 1.0}));
  FilterInput::Vector inputs = {
      FilterInput::Make(Rect::MakeXYWH(10, 10, 100, 100))};
  Entity entity;
  std::optional<Rect> coverage =
      contents.GetFilterCoverage(inputs, entity, /*effect_transform=*/Matrix());

  ASSERT_EQ(coverage, Rect::MakeXYWH(20, 20, 200, 200));
}

TEST(MatrixFilterContentsTest, Coverage2xEffect) {
  MatrixFilterContents contents;
  FilterInput::Vector inputs = {
      FilterInput::Make(Rect::MakeXYWH(10, 10, 100, 100))};
  Entity entity;
  std::optional<Rect> coverage = contents.GetFilterCoverage(
      inputs, entity, /*effect_transform=*/Matrix::MakeScale({2.0, 2.0, 1.0}));

  ASSERT_EQ(coverage, Rect::MakeXYWH(10, 10, 100, 100));
}

namespace {
void expectRenderCoverageEqual(const std::optional<Entity>& result,
                               const std::optional<Rect> contents_coverage,
                               const Rect& expected) {
  EXPECT_TRUE(result.has_value());
  if (result.has_value()) {
    EXPECT_EQ(result.value().GetBlendMode(), BlendMode::kSourceOver);
    std::optional<Rect> result_coverage = result.value().GetCoverage();
    EXPECT_TRUE(result_coverage.has_value());
    EXPECT_TRUE(contents_coverage.has_value());
    if (result_coverage.has_value() && contents_coverage.has_value()) {
      EXPECT_TRUE(RectNear(contents_coverage.value(), expected));
      EXPECT_TRUE(RectNear(result_coverage.value(), expected));
    }
  }
}
}  // namespace

TEST_P(MatrixFilterContentsTest, RenderCoverageMatchesGetCoverageIdentity) {
  std::shared_ptr<Texture> texture = MakeTexture(ISize(100, 100));
  MatrixFilterContents contents;
  contents.SetInputs({FilterInput::Make(texture)});

  Entity entity;
  entity.SetTransform(Matrix::MakeTranslation({100, 200, 0}));

  std::shared_ptr<ContentContext> renderer = GetContentContext();
  std::optional<Entity> result =
      contents.GetEntity(*renderer, entity, /*coverage_hint=*/{});
  expectRenderCoverageEqual(result, contents.GetCoverage(entity),
                            Rect::MakeXYWH(100, 200, 100, 100));
}

TEST_P(MatrixFilterContentsTest, RenderCoverageMatchesGetCoverageTranslate) {
  std::shared_ptr<Texture> texture = MakeTexture(ISize(100, 100));
  MatrixFilterContents contents;
  contents.SetInputs({FilterInput::Make(texture)});
  contents.SetMatrix(Matrix::MakeTranslation({50, 100, 0}));
  contents.SetEffectTransform(Matrix::MakeScale({2, 2, 1}));

  Entity entity;
  entity.SetTransform(Matrix::MakeTranslation({100, 200, 0}));

  std::shared_ptr<ContentContext> renderer = GetContentContext();
  std::optional<Entity> result =
      contents.GetEntity(*renderer, entity, /*coverage_hint=*/{});
  expectRenderCoverageEqual(result, contents.GetCoverage(entity),
                            Rect::MakeXYWH(150, 300, 100, 100));
}

TEST_P(MatrixFilterContentsTest,
       RenderCoverageMatchesGetCoverageClippedSubpassTranslate) {
  std::shared_ptr<Texture> texture = MakeTexture(ISize(100, 100));
  MatrixFilterContents contents;
  contents.SetInputs({FilterInput::Make(texture)});
  contents.SetMatrix(Matrix::MakeTranslation({50, 100, 0}));
  contents.SetEffectTransform(Matrix::MakeScale({2, 2, 1}));
  contents.SetRenderingMode(
      Entity::RenderingMode::kSubpassAppendSnapshotTransform);

  Entity entity;
  entity.SetTransform(Matrix::MakeTranslation({100, 200, 0}));

  std::shared_ptr<ContentContext> renderer = GetContentContext();
  std::optional<Entity> result =
      contents.GetEntity(*renderer, entity, /*coverage_hint=*/{});
  expectRenderCoverageEqual(result, contents.GetCoverage(entity),
                            Rect::MakeXYWH(200, 400, 100, 100));
}

TEST_P(MatrixFilterContentsTest, RenderCoverageMatchesGetCoverageScale) {
  std::shared_ptr<Texture> texture = MakeTexture(ISize(100, 100));
  MatrixFilterContents contents;
  contents.SetInputs({FilterInput::Make(texture)});
  contents.SetMatrix(Matrix::MakeScale({3, 3, 1}));
  contents.SetEffectTransform(Matrix::MakeScale({2, 2, 1}));

  Entity entity;
  entity.SetTransform(Matrix::MakeTranslation({100, 200, 0}));

  std::shared_ptr<ContentContext> renderer = GetContentContext();
  std::optional<Entity> result =
      contents.GetEntity(*renderer, entity, /*coverage_hint=*/{});
  expectRenderCoverageEqual(result, contents.GetCoverage(entity),
                            Rect::MakeXYWH(100, 200, 300, 300));
}

TEST_P(MatrixFilterContentsTest,
       RenderCoverageMatchesGetCoverageClippedSubpassScale) {
  std::shared_ptr<Texture> texture = MakeTexture(ISize(100, 100));
  MatrixFilterContents contents;
  contents.SetInputs({FilterInput::Make(texture)});
  contents.SetMatrix(Matrix::MakeScale({3, 3, 1}));
  contents.SetEffectTransform(Matrix::MakeScale({2, 2, 1}));
  contents.SetRenderingMode(
      Entity::RenderingMode::kSubpassAppendSnapshotTransform);

  Entity entity;
  entity.SetTransform(Matrix::MakeTranslation({100, 200, 0}));

  std::shared_ptr<ContentContext> renderer = GetContentContext();
  std::optional<Entity> result =
      contents.GetEntity(*renderer, entity, /*coverage_hint=*/{});
  expectRenderCoverageEqual(result, contents.GetCoverage(entity),
                            Rect::MakeXYWH(100, 200, 300, 300));
}

TEST_P(MatrixFilterContentsTest, RenderCoverageMatchesGetCoverageSubpassScale) {
  std::shared_ptr<Texture> texture = MakeTexture(ISize(100, 100));
  MatrixFilterContents contents;
  contents.SetInputs({FilterInput::Make(texture)});
  contents.SetMatrix(Matrix::MakeScale({3, 3, 1}));
  contents.SetEffectTransform(Matrix::MakeScale({2, 2, 1}));
  contents.SetRenderingMode(
      Entity::RenderingMode::kSubpassPrependSnapshotTransform);

  Entity entity;
  entity.SetTransform(Matrix::MakeTranslation({100, 200, 0}));

  std::shared_ptr<ContentContext> renderer = GetContentContext();
  std::optional<Entity> result =
      contents.GetEntity(*renderer, entity, /*coverage_hint=*/{});
  expectRenderCoverageEqual(result, contents.GetCoverage(entity),
                            Rect::MakeXYWH(300, 600, 300, 300));
}

}  // namespace testing
}  // namespace impeller
