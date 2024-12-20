// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/testing/testing.h"
#include "fml/status_or.h"
#include "gmock/gmock.h"
#include "impeller/entity/contents/content_context.h"
#include "impeller/entity/contents/filters/gaussian_blur_filter_contents.h"
#include "impeller/entity/contents/texture_contents.h"
#include "impeller/entity/entity_playground.h"
#include "impeller/geometry/color.h"
#include "impeller/geometry/geometry_asserts.h"
#include "impeller/renderer/testing/mocks.h"

#if FML_OS_MACOSX
#define IMPELLER_RAND arc4random
#else
#define IMPELLER_RAND rand
#endif

namespace impeller {
namespace testing {

namespace {

// Use newtonian method to give the closest answer to target where
// f(x) is less than the target. We do this because the value is `ceil`'d to
// grab fractional pixels.
fml::StatusOr<float> LowerBoundNewtonianMethod(
    const std::function<float(float)>& func,
    float target,
    float guess,
    float tolerance) {
  const double delta = 1e-6;
  double x = guess;
  double fx;
  static const int kMaxIterations = 1000;
  int count = 0;

  do {
    fx = func(x) - target;
    double derivative = (func(x + delta) - func(x)) / delta;
    x = x - fx / derivative;
    if (++count > kMaxIterations) {
      return fml::Status(fml::StatusCode::kDeadlineExceeded,
                         "Did not converge on answer.");
    }
  } while (std::abs(fx) > tolerance ||
           fx < 0.0);  // fx < 0.0 makes this lower bound.

  return x;
}

Scalar GetCoefficient(const Vector4& vec) {
  return vec.z;
}

Vector2 GetUVOffset(const Vector4& vec) {
  return vec.xy();
}

fml::StatusOr<Scalar> CalculateSigmaForBlurRadius(
    Scalar radius,
    const Matrix& effect_transform) {
  auto f = [effect_transform](Scalar x) -> Scalar {
    Vector2 scaled_sigma = (effect_transform.Basis() *
                            Vector2(GaussianBlurFilterContents::ScaleSigma(x),
                                    GaussianBlurFilterContents::ScaleSigma(x)))
                               .Abs();
    Vector2 blur_radius = Vector2(
        GaussianBlurFilterContents::CalculateBlurRadius(scaled_sigma.x),
        GaussianBlurFilterContents::CalculateBlurRadius(scaled_sigma.y));
    return std::max(blur_radius.x, blur_radius.y);
  };
  // The newtonian method is used here since inverting the function is
  // non-trivial because of conditional logic and would be fragile to changes.
  return LowerBoundNewtonianMethod(f, radius, 2.f, 0.001f);
}

}  // namespace

class GaussianBlurFilterContentsTest : public EntityPlayground {
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
INSTANTIATE_PLAYGROUND_SUITE(GaussianBlurFilterContentsTest);

TEST(GaussianBlurFilterContentsTest, Create) {
  GaussianBlurFilterContents contents(
      /*sigma_x=*/0.0, /*sigma_y=*/0.0, Entity::TileMode::kDecal,
      FilterContents::BlurStyle::kNormal, /*mask_geometry=*/nullptr);
  EXPECT_EQ(contents.GetSigmaX(), 0.0);
  EXPECT_EQ(contents.GetSigmaY(), 0.0);
}

TEST(GaussianBlurFilterContentsTest, CoverageEmpty) {
  GaussianBlurFilterContents contents(
      /*sigma_x=*/0.0, /*sigma_y=*/0.0, Entity::TileMode::kDecal,
      FilterContents::BlurStyle::kNormal, /*mask_geometry=*/nullptr);
  FilterInput::Vector inputs = {};
  Entity entity;
  std::optional<Rect> coverage =
      contents.GetFilterCoverage(inputs, entity, /*effect_transform=*/Matrix());
  ASSERT_FALSE(coverage.has_value());
}

TEST(GaussianBlurFilterContentsTest, CoverageSimple) {
  GaussianBlurFilterContents contents(
      /*sigma_x=*/0.0, /*sigma_y=*/0.0, Entity::TileMode::kDecal,
      FilterContents::BlurStyle::kNormal, /*mask_geometry=*/nullptr);
  FilterInput::Vector inputs = {
      FilterInput::Make(Rect::MakeLTRB(10, 10, 110, 110))};
  Entity entity;
  std::optional<Rect> coverage =
      contents.GetFilterCoverage(inputs, entity, /*effect_transform=*/Matrix());

  ASSERT_EQ(coverage, Rect::MakeLTRB(10, 10, 110, 110));
}

TEST(GaussianBlurFilterContentsTest, CoverageWithSigma) {
  fml::StatusOr<Scalar> sigma_radius_1 =
      CalculateSigmaForBlurRadius(1.0, Matrix());
  ASSERT_TRUE(sigma_radius_1.ok());
  GaussianBlurFilterContents contents(
      /*sigma_x=*/sigma_radius_1.value(),
      /*sigma_y=*/sigma_radius_1.value(), Entity::TileMode::kDecal,
      FilterContents::BlurStyle::kNormal, /*mask_geometry=*/nullptr);
  FilterInput::Vector inputs = {
      FilterInput::Make(Rect::MakeLTRB(100, 100, 200, 200))};
  Entity entity;
  std::optional<Rect> coverage =
      contents.GetFilterCoverage(inputs, entity, /*effect_transform=*/Matrix());

  EXPECT_TRUE(coverage.has_value());
  if (coverage.has_value()) {
    EXPECT_RECT_NEAR(coverage.value(), Rect::MakeLTRB(99, 99, 201, 201));
  }
}

TEST_P(GaussianBlurFilterContentsTest, CoverageWithTexture) {
  fml::StatusOr<Scalar> sigma_radius_1 =
      CalculateSigmaForBlurRadius(1.0, Matrix());
  ASSERT_TRUE(sigma_radius_1.ok());
  GaussianBlurFilterContents contents(
      /*sigma_X=*/sigma_radius_1.value(),
      /*sigma_y=*/sigma_radius_1.value(), Entity::TileMode::kDecal,
      FilterContents::BlurStyle::kNormal, /*mask_geometry=*/nullptr);
  std::shared_ptr<Texture> texture = MakeTexture(ISize(100, 100));
  FilterInput::Vector inputs = {FilterInput::Make(texture)};
  Entity entity;
  entity.SetTransform(Matrix::MakeTranslation({100, 100, 0}));
  std::optional<Rect> coverage =
      contents.GetFilterCoverage(inputs, entity, /*effect_transform=*/Matrix());

  EXPECT_TRUE(coverage.has_value());
  if (coverage.has_value()) {
    EXPECT_RECT_NEAR(coverage.value(), Rect::MakeLTRB(99, 99, 201, 201));
  }
}

TEST_P(GaussianBlurFilterContentsTest, CoverageWithEffectTransform) {
  Matrix effect_transform = Matrix::MakeScale({2.0, 2.0, 1.0});
  fml::StatusOr<Scalar> sigma_radius_1 =
      CalculateSigmaForBlurRadius(1.0, effect_transform);
  ASSERT_TRUE(sigma_radius_1.ok());
  GaussianBlurFilterContents contents(
      /*sigma_x=*/sigma_radius_1.value(),
      /*sigma_y=*/sigma_radius_1.value(), Entity::TileMode::kDecal,
      FilterContents::BlurStyle::kNormal, /*mask_geometry=*/nullptr);
  std::shared_ptr<Texture> texture = MakeTexture(ISize(100, 100));
  FilterInput::Vector inputs = {FilterInput::Make(texture)};
  Entity entity;
  entity.SetTransform(Matrix::MakeTranslation({100, 100, 0}));
  std::optional<Rect> coverage =
      contents.GetFilterCoverage(inputs, entity, effect_transform);
  EXPECT_TRUE(coverage.has_value());
  if (coverage.has_value()) {
    EXPECT_RECT_NEAR(coverage.value(),
                     Rect::MakeLTRB(100 - 1, 100 - 1, 200 + 1, 200 + 1));
  }
}

TEST(GaussianBlurFilterContentsTest, FilterSourceCoverage) {
  fml::StatusOr<Scalar> sigma_radius_1 =
      CalculateSigmaForBlurRadius(1.0, Matrix());
  ASSERT_TRUE(sigma_radius_1.ok());
  auto contents = std::make_unique<GaussianBlurFilterContents>(
      sigma_radius_1.value(), sigma_radius_1.value(), Entity::TileMode::kDecal,
      FilterContents::BlurStyle::kNormal, /*mask_geometry=*/nullptr);
  std::optional<Rect> coverage = contents->GetFilterSourceCoverage(
      /*effect_transform=*/Matrix::MakeScale({2.0, 2.0, 1.0}),
      /*output_limit=*/Rect::MakeLTRB(100, 100, 200, 200));
  EXPECT_TRUE(coverage.has_value());
  if (coverage.has_value()) {
    EXPECT_RECT_NEAR(coverage.value(),
                     Rect::MakeLTRB(100 - 2, 100 - 2, 200 + 2, 200 + 2));
  }
}

TEST(GaussianBlurFilterContentsTest, CalculateSigmaValues) {
  EXPECT_EQ(GaussianBlurFilterContents::CalculateScale(1.0f), 1);
  EXPECT_EQ(GaussianBlurFilterContents::CalculateScale(2.0f), 1);
  EXPECT_EQ(GaussianBlurFilterContents::CalculateScale(3.0f), 1);
  EXPECT_EQ(GaussianBlurFilterContents::CalculateScale(4.0f), 1);
  EXPECT_EQ(GaussianBlurFilterContents::CalculateScale(16.0f), 0.25);
  // Hang on to 1/8 as long as possible.
  EXPECT_EQ(GaussianBlurFilterContents::CalculateScale(95.0f), 0.125);
  EXPECT_EQ(GaussianBlurFilterContents::CalculateScale(96.0f), 0.0625);
  // Downsample clamped to 1/16th.
  EXPECT_EQ(GaussianBlurFilterContents::CalculateScale(1024.0f), 0.0625);
}

TEST_P(GaussianBlurFilterContentsTest, RenderCoverageMatchesGetCoverage) {
  std::shared_ptr<Texture> texture = MakeTexture(ISize(100, 100));
  fml::StatusOr<Scalar> sigma_radius_1 =
      CalculateSigmaForBlurRadius(1.0, Matrix());
  ASSERT_TRUE(sigma_radius_1.ok());
  auto contents = std::make_unique<GaussianBlurFilterContents>(
      sigma_radius_1.value(), sigma_radius_1.value(), Entity::TileMode::kDecal,
      FilterContents::BlurStyle::kNormal, /*mask_geometry=*/nullptr);
  contents->SetInputs({FilterInput::Make(texture)});
  std::shared_ptr<ContentContext> renderer = GetContentContext();

  Entity entity;
  std::optional<Entity> result =
      contents->GetEntity(*renderer, entity, /*coverage_hint=*/{});
  EXPECT_TRUE(result.has_value());
  if (result.has_value()) {
    EXPECT_EQ(result.value().GetBlendMode(), BlendMode::kSourceOver);
    std::optional<Rect> result_coverage = result.value().GetCoverage();
    std::optional<Rect> contents_coverage = contents->GetCoverage(entity);
    EXPECT_TRUE(result_coverage.has_value());
    EXPECT_TRUE(contents_coverage.has_value());
    if (result_coverage.has_value() && contents_coverage.has_value()) {
      EXPECT_TRUE(RectNear(contents_coverage.value(),
                           Rect::MakeLTRB(-1, -1, 101, 101)));
      EXPECT_TRUE(
          RectNear(result_coverage.value(), Rect::MakeLTRB(-1, -1, 101, 101)));
    }
  }
}

TEST_P(GaussianBlurFilterContentsTest,
       RenderCoverageMatchesGetCoverageTranslate) {
  std::shared_ptr<Texture> texture = MakeTexture(ISize(100, 100));
  fml::StatusOr<Scalar> sigma_radius_1 =
      CalculateSigmaForBlurRadius(1.0, Matrix());
  ASSERT_TRUE(sigma_radius_1.ok());
  auto contents = std::make_unique<GaussianBlurFilterContents>(
      sigma_radius_1.value(), sigma_radius_1.value(), Entity::TileMode::kDecal,
      FilterContents::BlurStyle::kNormal, /*mask_geometry=*/nullptr);
  contents->SetInputs({FilterInput::Make(texture)});
  std::shared_ptr<ContentContext> renderer = GetContentContext();

  Entity entity;
  entity.SetTransform(Matrix::MakeTranslation({100, 200, 0}));
  std::optional<Entity> result =
      contents->GetEntity(*renderer, entity, /*coverage_hint=*/{});

  EXPECT_TRUE(result.has_value());
  if (result.has_value()) {
    EXPECT_EQ(result.value().GetBlendMode(), BlendMode::kSourceOver);
    std::optional<Rect> result_coverage = result.value().GetCoverage();
    std::optional<Rect> contents_coverage = contents->GetCoverage(entity);
    EXPECT_TRUE(result_coverage.has_value());
    EXPECT_TRUE(contents_coverage.has_value());
    if (result_coverage.has_value() && contents_coverage.has_value()) {
      EXPECT_TRUE(RectNear(contents_coverage.value(),
                           Rect::MakeLTRB(99, 199, 201, 301)));
      EXPECT_TRUE(
          RectNear(result_coverage.value(), Rect::MakeLTRB(99, 199, 201, 301)));
    }
  }
}

TEST_P(GaussianBlurFilterContentsTest,
       RenderCoverageMatchesGetCoverageRotated) {
  std::shared_ptr<Texture> texture = MakeTexture(ISize(400, 300));
  fml::StatusOr<Scalar> sigma_radius_1 =
      CalculateSigmaForBlurRadius(1.0, Matrix());
  auto contents = std::make_unique<GaussianBlurFilterContents>(
      sigma_radius_1.value(), sigma_radius_1.value(), Entity::TileMode::kDecal,
      FilterContents::BlurStyle::kNormal, /*mask_geometry=*/nullptr);
  contents->SetInputs({FilterInput::Make(texture)});
  std::shared_ptr<ContentContext> renderer = GetContentContext();

  Entity entity;
  // Rotate around the top left corner, then push it over to (100, 100).
  entity.SetTransform(Matrix::MakeTranslation({400, 100, 0}) *
                      Matrix::MakeRotationZ(Degrees(90.0)));
  std::optional<Entity> result =
      contents->GetEntity(*renderer, entity, /*coverage_hint=*/{});
  EXPECT_TRUE(result.has_value());
  if (result.has_value()) {
    EXPECT_EQ(result.value().GetBlendMode(), BlendMode::kSourceOver);
    std::optional<Rect> result_coverage = result.value().GetCoverage();
    std::optional<Rect> contents_coverage = contents->GetCoverage(entity);
    EXPECT_TRUE(result_coverage.has_value());
    EXPECT_TRUE(contents_coverage.has_value());
    if (result_coverage.has_value() && contents_coverage.has_value()) {
      EXPECT_TRUE(RectNear(contents_coverage.value(),
                           Rect::MakeLTRB(99, 99, 401, 501)));
      EXPECT_TRUE(
          RectNear(result_coverage.value(), Rect::MakeLTRB(99, 99, 401, 501)));
    }
  }
}

TEST_P(GaussianBlurFilterContentsTest, CalculateUVsSimple) {
  std::shared_ptr<Texture> texture = MakeTexture(ISize(100, 100));
  auto filter_input = FilterInput::Make(texture);
  Entity entity;
  Quad uvs = GaussianBlurFilterContents::CalculateUVs(
      filter_input, entity, Rect::MakeSize(ISize(100, 100)), ISize(100, 100));
  std::optional<Rect> uvs_bounds = Rect::MakePointBounds(uvs);
  EXPECT_TRUE(uvs_bounds.has_value());
  if (uvs_bounds.has_value()) {
    EXPECT_TRUE(RectNear(uvs_bounds.value(), Rect::MakeXYWH(0, 0, 1, 1)));
  }
}

TEST_P(GaussianBlurFilterContentsTest, TextureContentsWithDestinationRect) {
  std::shared_ptr<Texture> texture = MakeTexture(ISize(100, 100));
  auto texture_contents = std::make_shared<TextureContents>();
  texture_contents->SetSourceRect(Rect::MakeSize(texture->GetSize()));
  texture_contents->SetTexture(texture);
  texture_contents->SetDestinationRect(Rect::MakeXYWH(
      50, 40, texture->GetSize().width, texture->GetSize().height));

  fml::StatusOr<Scalar> sigma_radius_1 =
      CalculateSigmaForBlurRadius(1.0, Matrix());
  auto contents = std::make_unique<GaussianBlurFilterContents>(
      sigma_radius_1.value(), sigma_radius_1.value(), Entity::TileMode::kDecal,
      FilterContents::BlurStyle::kNormal, /*mask_geometry=*/nullptr);
  contents->SetInputs({FilterInput::Make(texture_contents)});
  std::shared_ptr<ContentContext> renderer = GetContentContext();

  Entity entity;
  std::optional<Entity> result =
      contents->GetEntity(*renderer, entity, /*coverage_hint=*/{});
  EXPECT_TRUE(result.has_value());
  if (result.has_value()) {
    EXPECT_EQ(result.value().GetBlendMode(), BlendMode::kSourceOver);
    std::optional<Rect> result_coverage = result.value().GetCoverage();
    std::optional<Rect> contents_coverage = contents->GetCoverage(entity);
    EXPECT_TRUE(result_coverage.has_value());
    EXPECT_TRUE(contents_coverage.has_value());
    if (result_coverage.has_value() && contents_coverage.has_value()) {
      EXPECT_TRUE(RectNear(result_coverage.value(), contents_coverage.value()));
      EXPECT_TRUE(RectNear(result_coverage.value(),
                           Rect::MakeLTRB(49.f, 39.f, 151.f, 141.f)));
    }
  }
}

TEST_P(GaussianBlurFilterContentsTest,
       TextureContentsWithDestinationRectScaled) {
  std::shared_ptr<Texture> texture = MakeTexture(ISize(100, 100));
  auto texture_contents = std::make_shared<TextureContents>();
  texture_contents->SetSourceRect(Rect::MakeSize(texture->GetSize()));
  texture_contents->SetTexture(texture);
  texture_contents->SetDestinationRect(Rect::MakeXYWH(
      50, 40, texture->GetSize().width, texture->GetSize().height));

  fml::StatusOr<Scalar> sigma_radius_1 =
      CalculateSigmaForBlurRadius(1.0, Matrix());
  auto contents = std::make_unique<GaussianBlurFilterContents>(
      sigma_radius_1.value(), sigma_radius_1.value(), Entity::TileMode::kDecal,
      FilterContents::BlurStyle::kNormal,
      /*mask_geometry=*/nullptr);
  contents->SetInputs({FilterInput::Make(texture_contents)});
  std::shared_ptr<ContentContext> renderer = GetContentContext();

  Entity entity;
  entity.SetTransform(Matrix::MakeScale({2.0, 2.0, 1.0}));
  std::optional<Entity> result =
      contents->GetEntity(*renderer, entity, /*coverage_hint=*/{});
  EXPECT_TRUE(result.has_value());
  if (result.has_value()) {
    EXPECT_EQ(result.value().GetBlendMode(), BlendMode::kSourceOver);
    std::optional<Rect> result_coverage = result.value().GetCoverage();
    std::optional<Rect> contents_coverage = contents->GetCoverage(entity);
    EXPECT_TRUE(result_coverage.has_value());
    EXPECT_TRUE(contents_coverage.has_value());
    if (result_coverage.has_value() && contents_coverage.has_value()) {
      EXPECT_TRUE(RectNear(result_coverage.value(), contents_coverage.value()));
      // Scaling a blurred entity doesn't seem to scale the blur radius linearly
      // when comparing results with rrect_blur. That's why this is not
      // Rect::MakeXYWH(98.f, 78.f, 204.0f, 204.f).
      EXPECT_TRUE(RectNear(contents_coverage.value(),
                           Rect::MakeXYWH(94.f, 74.f, 212.0f, 212.f)));
    }
  }
}

TEST_P(GaussianBlurFilterContentsTest, TextureContentsWithEffectTransform) {
  Matrix effect_transform = Matrix::MakeScale({2.0, 2.0, 1.0});
  std::shared_ptr<Texture> texture = MakeTexture(ISize(100, 100));
  auto texture_contents = std::make_shared<TextureContents>();
  texture_contents->SetSourceRect(Rect::MakeSize(texture->GetSize()));
  texture_contents->SetTexture(texture);
  texture_contents->SetDestinationRect(Rect::MakeXYWH(
      50, 40, texture->GetSize().width, texture->GetSize().height));

  fml::StatusOr<Scalar> sigma_radius_1 =
      CalculateSigmaForBlurRadius(1.0, effect_transform);
  ASSERT_TRUE(sigma_radius_1.ok());
  auto contents = std::make_unique<GaussianBlurFilterContents>(
      sigma_radius_1.value(), sigma_radius_1.value(), Entity::TileMode::kDecal,
      FilterContents::BlurStyle::kNormal, /*mask_geometry=*/nullptr);
  contents->SetInputs({FilterInput::Make(texture_contents)});
  contents->SetEffectTransform(effect_transform);
  std::shared_ptr<ContentContext> renderer = GetContentContext();

  Entity entity;
  std::optional<Entity> result =
      contents->GetEntity(*renderer, entity, /*coverage_hint=*/{});
  EXPECT_TRUE(result.has_value());
  if (result.has_value()) {
    EXPECT_EQ(result.value().GetBlendMode(), BlendMode::kSourceOver);
    std::optional<Rect> result_coverage = result.value().GetCoverage();
    std::optional<Rect> contents_coverage = contents->GetCoverage(entity);
    EXPECT_TRUE(result_coverage.has_value());
    EXPECT_TRUE(contents_coverage.has_value());
    if (result_coverage.has_value() && contents_coverage.has_value()) {
      EXPECT_TRUE(RectNear(result_coverage.value(), contents_coverage.value()));
      EXPECT_TRUE(RectNear(contents_coverage.value(),
                           Rect::MakeXYWH(49.f, 39.f, 102.f, 102.f)));
    }
  }
}

TEST(GaussianBlurFilterContentsTest, CalculateSigmaForBlurRadius) {
  Scalar sigma = 1.0;
  Scalar radius = GaussianBlurFilterContents::CalculateBlurRadius(
      GaussianBlurFilterContents::ScaleSigma(sigma));
  fml::StatusOr<Scalar> derived_sigma =
      CalculateSigmaForBlurRadius(radius, Matrix());
  ASSERT_TRUE(derived_sigma.ok());
  EXPECT_NEAR(sigma, derived_sigma.value(), 0.01f);
}

TEST(GaussianBlurFilterContentsTest, Coefficients) {
  BlurParameters parameters = {.blur_uv_offset = Point(1, 0),
                               .blur_sigma = 1,
                               .blur_radius = 5,
                               .step_size = 1};
  KernelSamples samples = GenerateBlurInfo(parameters);
  EXPECT_EQ(samples.sample_count, 9);

  // Coefficients should add up to 1.
  Scalar tally = 0;
  for (int i = 0; i < samples.sample_count; ++i) {
    tally += samples.samples[i].coefficient;
  }
  EXPECT_FLOAT_EQ(tally, 1.0f);

  // Verify the shape of the curve.
  for (int i = 0; i < 4; ++i) {
    EXPECT_FLOAT_EQ(samples.samples[i].coefficient,
                    samples.samples[8 - i].coefficient);
    EXPECT_TRUE(samples.samples[i + 1].coefficient >
                samples.samples[i].coefficient);
  }
}

TEST(GaussianBlurFilterContentsTest, LerpHackKernelSamplesSimple) {
  KernelSamples kernel_samples = {
      .sample_count = 5,
      .samples =
          {
              {
                  .uv_offset = Vector2(-2, 0),
                  .coefficient = 0.1f,
              },
              {
                  .uv_offset = Vector2(-1, 0),
                  .coefficient = 0.2f,
              },
              {
                  .uv_offset = Vector2(0, 0),
                  .coefficient = 0.4f,
              },
              {
                  .uv_offset = Vector2(1, 0),
                  .coefficient = 0.2f,
              },
              {
                  .uv_offset = Vector2(2, 0),
                  .coefficient = 0.1f,
              },
          },
  };

  GaussianBlurPipeline::FragmentShader::KernelSamples blur_info =
      LerpHackKernelSamples(kernel_samples);
  EXPECT_EQ(blur_info.sample_count, 3);

  KernelSample* samples = kernel_samples.samples;

  //////////////////////////////////////////////////////////////////////////////
  // Check output kernel.

  EXPECT_POINT_NEAR(GetUVOffset(blur_info.sample_data[0]),
                    Point(-1.3333333, 0));
  EXPECT_FLOAT_EQ(GetCoefficient(blur_info.sample_data[0]), 0.3);

  EXPECT_POINT_NEAR(GetUVOffset(blur_info.sample_data[1]), Point(0, 0));
  EXPECT_FLOAT_EQ(GetCoefficient(blur_info.sample_data[1]), 0.4);

  EXPECT_POINT_NEAR(GetUVOffset(blur_info.sample_data[2]), Point(1.333333, 0));
  EXPECT_FLOAT_EQ(GetCoefficient(blur_info.sample_data[2]), 0.3);

  //////////////////////////////////////////////////////////////////////////////
  // Check output of fast kernel versus original kernel.

  Scalar data[5] = {0.25, 0.5, 0.5, 1.0, 0.2};
  Scalar original_output =
      samples[0].coefficient * data[0] + samples[1].coefficient * data[1] +
      samples[2].coefficient * data[2] + samples[3].coefficient * data[3] +
      samples[4].coefficient * data[4];

  auto lerp = [](const Point& point, Scalar left, Scalar right) {
    Scalar int_part;
    Scalar fract = fabsf(modf(point.x, &int_part));
    if (point.x < 0) {
      return left * fract + right * (1.0 - fract);
    } else {
      return left * (1.0 - fract) + right * fract;
    }
  };
  Scalar fast_output =
      /*1st*/ lerp(GetUVOffset(blur_info.sample_data[0]), data[0], data[1]) *
          GetCoefficient(blur_info.sample_data[0]) +
      /*2nd*/ data[2] * GetCoefficient(blur_info.sample_data[1]) +
      /*3rd*/ lerp(GetUVOffset(blur_info.sample_data[2]), data[3], data[4]) *
          GetCoefficient(blur_info.sample_data[2]);

  EXPECT_NEAR(original_output, fast_output, 0.01);
}

TEST(GaussianBlurFilterContentsTest, LerpHackKernelSamplesComplex) {
  Scalar sigma = 10.0f;
  int32_t blur_radius = static_cast<int32_t>(
      std::ceil(GaussianBlurFilterContents::CalculateBlurRadius(sigma)));
  BlurParameters parameters = {.blur_uv_offset = Point(1, 0),
                               .blur_sigma = sigma,
                               .blur_radius = blur_radius,
                               .step_size = 1};
  KernelSamples kernel_samples = GenerateBlurInfo(parameters);
  EXPECT_EQ(kernel_samples.sample_count, 33);
  GaussianBlurPipeline::FragmentShader::KernelSamples fast_kernel_samples =
      LerpHackKernelSamples(kernel_samples);
  EXPECT_EQ(fast_kernel_samples.sample_count, 17);
  float data[33];
  srand(0);
  for (int i = 0; i < 33; i++) {
    data[i] = 255.0 * static_cast<double>(IMPELLER_RAND()) / RAND_MAX;
  }

  auto sampler = [data](Point point) -> Scalar {
    FML_CHECK(point.y == 0.0f);
    FML_CHECK(point.x >= -16);
    FML_CHECK(point.x <= 16);
    Scalar fint_part;
    Scalar fract = fabsf(modf(point.x, &fint_part));
    if (fract == 0) {
      int32_t int_part = static_cast<int32_t>(fint_part) + 16;
      return data[int_part];
    } else {
      int32_t left = static_cast<int32_t>(floor(point.x)) + 16;
      int32_t right = static_cast<int32_t>(ceil(point.x)) + 16;
      if (point.x < 0) {
        return fract * data[left] + (1.0 - fract) * data[right];
      } else {
        return (1.0 - fract) * data[left] + fract * data[right];
      }
    }
  };

  Scalar output = 0.0;
  for (int i = 0; i < kernel_samples.sample_count; ++i) {
    auto sample = kernel_samples.samples[i];
    output += sample.coefficient * sampler(sample.uv_offset);
  }

  Scalar fast_output = 0.0;
  for (int i = 0; i < fast_kernel_samples.sample_count; i++) {
    fast_output += GetCoefficient(fast_kernel_samples.sample_data[i]) *
                   sampler(GetUVOffset(fast_kernel_samples.sample_data[i]));
  }

  EXPECT_NEAR(output, fast_output, 0.1);
}

TEST(GaussianBlurFilterContentsTest, ChopHugeBlurs) {
  Scalar sigma = 30.5f;
  int32_t blur_radius = static_cast<int32_t>(
      std::ceil(GaussianBlurFilterContents::CalculateBlurRadius(sigma)));
  BlurParameters parameters = {.blur_uv_offset = Point(1, 0),
                               .blur_sigma = sigma,
                               .blur_radius = blur_radius,
                               .step_size = 1};
  KernelSamples kernel_samples = GenerateBlurInfo(parameters);
  GaussianBlurPipeline::FragmentShader::KernelSamples frag_kernel_samples =
      LerpHackKernelSamples(kernel_samples);
  EXPECT_TRUE(frag_kernel_samples.sample_count <= kGaussianBlurMaxKernelSize);
}

}  // namespace testing
}  // namespace impeller
