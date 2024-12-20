// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <future>

#include "flutter/fml/make_copyable.h"
#include "flutter/testing/testing.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "impeller/base/allocation.h"
#include "impeller/base/validation.h"
#include "impeller/core/runtime_types.h"
#include "impeller/core/shader_types.h"
#include "impeller/entity/runtime_effect.vert.h"
#include "impeller/playground/playground.h"
#include "impeller/renderer/pipeline_descriptor.h"
#include "impeller/renderer/pipeline_library.h"
#include "impeller/renderer/shader_library.h"
#include "impeller/runtime_stage/runtime_stage.h"
#include "impeller/runtime_stage/runtime_stage_playground.h"

namespace impeller {
namespace testing {

using RuntimeStageTest = RuntimeStagePlayground;
INSTANTIATE_PLAYGROUND_SUITE(RuntimeStageTest);

TEST_P(RuntimeStageTest, CanReadValidBlob) {
  const std::shared_ptr<fml::Mapping> fixture =
      flutter::testing::OpenFixtureAsMapping("ink_sparkle.frag.iplr");
  ASSERT_TRUE(fixture);
  ASSERT_GT(fixture->GetSize(), 0u);
  auto stages = RuntimeStage::DecodeRuntimeStages(fixture);
  auto stage = stages[PlaygroundBackendToRuntimeStageBackend(GetBackend())];
  ASSERT_TRUE(stage->IsValid());
  ASSERT_EQ(stage->GetShaderStage(), RuntimeShaderStage::kFragment);
}

TEST_P(RuntimeStageTest, CanRejectInvalidBlob) {
  ScopedValidationDisable disable_validation;
  const std::shared_ptr<fml::Mapping> fixture =
      flutter::testing::OpenFixtureAsMapping("ink_sparkle.frag.iplr");
  ASSERT_TRUE(fixture);
  auto junk_allocation = std::make_shared<Allocation>();
  ASSERT_TRUE(junk_allocation->Truncate(Bytes{fixture->GetSize()}, false));
  // Not meant to be secure. Just reject obviously bad blobs using magic
  // numbers.
  ::memset(junk_allocation->GetBuffer(), 127,
           junk_allocation->GetLength().GetByteSize());
  auto stages = RuntimeStage::DecodeRuntimeStages(
      CreateMappingFromAllocation(junk_allocation));
  ASSERT_FALSE(stages[PlaygroundBackendToRuntimeStageBackend(GetBackend())]);
}

TEST_P(RuntimeStageTest, CanReadUniforms) {
  const std::shared_ptr<fml::Mapping> fixture =
      flutter::testing::OpenFixtureAsMapping("ink_sparkle.frag.iplr");
  ASSERT_TRUE(fixture);
  ASSERT_GT(fixture->GetSize(), 0u);
  auto stages = RuntimeStage::DecodeRuntimeStages(fixture);
  auto stage = stages[PlaygroundBackendToRuntimeStageBackend(GetBackend())];

  ASSERT_TRUE(stage->IsValid());
  switch (GetBackend()) {
    case PlaygroundBackend::kMetal:
      [[fallthrough]];
    case PlaygroundBackend::kOpenGLES: {
      ASSERT_EQ(stage->GetUniforms().size(), 17u);
      {
        auto uni = stage->GetUniform("u_color");
        ASSERT_NE(uni, nullptr);
        EXPECT_EQ(uni->dimensions.rows, 4u);
        EXPECT_EQ(uni->dimensions.cols, 1u);
        EXPECT_EQ(uni->location, 0u);
        EXPECT_EQ(uni->type, RuntimeUniformType::kFloat);
      }
      {
        auto uni = stage->GetUniform("u_alpha");
        ASSERT_NE(uni, nullptr);
        EXPECT_EQ(uni->dimensions.rows, 1u);
        EXPECT_EQ(uni->dimensions.cols, 1u);
        EXPECT_EQ(uni->location, 1u);
        EXPECT_EQ(uni->type, RuntimeUniformType::kFloat);
      }
      {
        auto uni = stage->GetUniform("u_sparkle_color");
        ASSERT_NE(uni, nullptr);
        EXPECT_EQ(uni->dimensions.rows, 4u);
        EXPECT_EQ(uni->dimensions.cols, 1u);
        EXPECT_EQ(uni->location, 2u);
        EXPECT_EQ(uni->type, RuntimeUniformType::kFloat);
      }
      {
        auto uni = stage->GetUniform("u_sparkle_alpha");
        ASSERT_NE(uni, nullptr);
        EXPECT_EQ(uni->dimensions.rows, 1u);
        EXPECT_EQ(uni->dimensions.cols, 1u);
        EXPECT_EQ(uni->location, 3u);
        EXPECT_EQ(uni->type, RuntimeUniformType::kFloat);
      }
      {
        auto uni = stage->GetUniform("u_blur");
        ASSERT_NE(uni, nullptr);
        EXPECT_EQ(uni->dimensions.rows, 1u);
        EXPECT_EQ(uni->dimensions.cols, 1u);
        EXPECT_EQ(uni->location, 4u);
        EXPECT_EQ(uni->type, RuntimeUniformType::kFloat);
      }
      {
        auto uni = stage->GetUniform("u_radius_scale");
        ASSERT_NE(uni, nullptr);
        EXPECT_EQ(uni->dimensions.rows, 1u);
        EXPECT_EQ(uni->dimensions.cols, 1u);
        EXPECT_EQ(uni->location, 6u);
        EXPECT_EQ(uni->type, RuntimeUniformType::kFloat);
      }
      {
        auto uni = stage->GetUniform("u_max_radius");
        ASSERT_NE(uni, nullptr);
        EXPECT_EQ(uni->dimensions.rows, 1u);
        EXPECT_EQ(uni->dimensions.cols, 1u);
        EXPECT_EQ(uni->location, 7u);
        EXPECT_EQ(uni->type, RuntimeUniformType::kFloat);
      }
      {
        auto uni = stage->GetUniform("u_resolution_scale");
        ASSERT_NE(uni, nullptr);
        EXPECT_EQ(uni->dimensions.rows, 2u);
        EXPECT_EQ(uni->dimensions.cols, 1u);
        EXPECT_EQ(uni->location, 8u);
        EXPECT_EQ(uni->type, RuntimeUniformType::kFloat);
      }
      {
        auto uni = stage->GetUniform("u_noise_scale");
        ASSERT_NE(uni, nullptr);
        EXPECT_EQ(uni->dimensions.rows, 2u);
        EXPECT_EQ(uni->dimensions.cols, 1u);
        EXPECT_EQ(uni->location, 9u);
        EXPECT_EQ(uni->type, RuntimeUniformType::kFloat);
      }
      {
        auto uni = stage->GetUniform("u_noise_phase");
        ASSERT_NE(uni, nullptr);
        EXPECT_EQ(uni->dimensions.rows, 1u);
        EXPECT_EQ(uni->dimensions.cols, 1u);
        EXPECT_EQ(uni->location, 10u);
        EXPECT_EQ(uni->type, RuntimeUniformType::kFloat);
      }

      {
        auto uni = stage->GetUniform("u_circle1");
        ASSERT_NE(uni, nullptr);
        EXPECT_EQ(uni->dimensions.rows, 2u);
        EXPECT_EQ(uni->dimensions.cols, 1u);
        EXPECT_EQ(uni->location, 11u);
        EXPECT_EQ(uni->type, RuntimeUniformType::kFloat);
      }
      {
        auto uni = stage->GetUniform("u_circle2");
        ASSERT_NE(uni, nullptr);
        EXPECT_EQ(uni->dimensions.rows, 2u);
        EXPECT_EQ(uni->dimensions.cols, 1u);
        EXPECT_EQ(uni->location, 12u);
        EXPECT_EQ(uni->type, RuntimeUniformType::kFloat);
      }
      {
        auto uni = stage->GetUniform("u_circle3");
        ASSERT_NE(uni, nullptr);
        EXPECT_EQ(uni->dimensions.rows, 2u);
        EXPECT_EQ(uni->dimensions.cols, 1u);
        EXPECT_EQ(uni->location, 13u);
        EXPECT_EQ(uni->type, RuntimeUniformType::kFloat);
      }
      {
        auto uni = stage->GetUniform("u_rotation1");
        ASSERT_NE(uni, nullptr);
        EXPECT_EQ(uni->dimensions.rows, 2u);
        EXPECT_EQ(uni->dimensions.cols, 1u);
        EXPECT_EQ(uni->location, 14u);
        EXPECT_EQ(uni->type, RuntimeUniformType::kFloat);
      }
      {
        auto uni = stage->GetUniform("u_rotation2");
        ASSERT_NE(uni, nullptr);
        EXPECT_EQ(uni->dimensions.rows, 2u);
        EXPECT_EQ(uni->dimensions.cols, 1u);
        EXPECT_EQ(uni->location, 15u);
        EXPECT_EQ(uni->type, RuntimeUniformType::kFloat);
      }
      {
        auto uni = stage->GetUniform("u_rotation3");
        ASSERT_NE(uni, nullptr);
        EXPECT_EQ(uni->dimensions.rows, 2u);
        EXPECT_EQ(uni->dimensions.cols, 1u);
        EXPECT_EQ(uni->location, 16u);
        EXPECT_EQ(uni->type, RuntimeUniformType::kFloat);
      }
      break;
    }
    case PlaygroundBackend::kVulkan: {
      EXPECT_EQ(stage->GetUniforms().size(), 1u);
      auto uni = stage->GetUniform(RuntimeStage::kVulkanUBOName);
      ASSERT_TRUE(uni);
      EXPECT_EQ(uni->type, RuntimeUniformType::kStruct);
      EXPECT_EQ(uni->struct_float_count, 32u);

      // There are 36 4 byte chunks in the UBO: 32 for the 32 floats, and 4 for
      // padding. Initialize a vector as if they'll all be floats, then manually
      // set the few padding bytes. If the shader changes, the padding locations
      // will change as well. For example, if `u_alpha` was moved to the end,
      // three bytes of padding could potentially be dropped - or if some of the
      // scalar floats were changed to vec2 or vec4s, or if any vec3s are
      // introduced.
      // This means 36 * 4 = 144 bytes total.

      EXPECT_EQ(uni->GetSize(), 144u);
      std::vector<uint8_t> layout(uni->GetSize() / sizeof(float), 1);
      layout[5] = 0;
      layout[6] = 0;
      layout[7] = 0;
      layout[23] = 0;

      EXPECT_THAT(uni->struct_layout, ::testing::ElementsAreArray(layout));
      break;
    }
  }
}

TEST_P(RuntimeStageTest, CanReadUniformsSamplerBeforeUBO) {
  if (GetBackend() != PlaygroundBackend::kVulkan) {
    GTEST_SKIP() << "Test only relevant for Vulkan";
  }
  const std::shared_ptr<fml::Mapping> fixture =
      flutter::testing::OpenFixtureAsMapping(
          "uniforms_and_sampler_1.frag.iplr");
  ASSERT_TRUE(fixture);
  ASSERT_GT(fixture->GetSize(), 0u);
  auto stages = RuntimeStage::DecodeRuntimeStages(fixture);
  auto stage = stages[PlaygroundBackendToRuntimeStageBackend(GetBackend())];

  EXPECT_EQ(stage->GetUniforms().size(), 2u);
  auto uni = stage->GetUniform(RuntimeStage::kVulkanUBOName);
  ASSERT_TRUE(uni);
  // Struct must be offset at 65.
  EXPECT_EQ(uni->type, RuntimeUniformType::kStruct);
  EXPECT_EQ(uni->binding, 65u);
  // Sampler should be offset at 64 but due to current bug
  // has offset of 0, the correct offset is computed at runtime.
  auto sampler_uniform = stage->GetUniform("u_texture");
  EXPECT_EQ(sampler_uniform->type, RuntimeUniformType::kSampledImage);
  EXPECT_EQ(sampler_uniform->binding, 64u);
}

TEST_P(RuntimeStageTest, CanReadUniformsSamplerAfterUBO) {
  if (GetBackend() != PlaygroundBackend::kVulkan) {
    GTEST_SKIP() << "Test only relevant for Vulkan";
  }
  const std::shared_ptr<fml::Mapping> fixture =
      flutter::testing::OpenFixtureAsMapping(
          "uniforms_and_sampler_2.frag.iplr");
  ASSERT_TRUE(fixture);
  ASSERT_GT(fixture->GetSize(), 0u);
  auto stages = RuntimeStage::DecodeRuntimeStages(fixture);
  auto stage = stages[PlaygroundBackendToRuntimeStageBackend(GetBackend())];

  EXPECT_EQ(stage->GetUniforms().size(), 2u);
  auto uni = stage->GetUniform(RuntimeStage::kVulkanUBOName);
  ASSERT_TRUE(uni);
  // Struct must be offset at 45.
  EXPECT_EQ(uni->type, RuntimeUniformType::kStruct);
  EXPECT_EQ(uni->binding, 64u);
  // Sampler should be offset at 64 but due to current bug
  // has offset of 0, the correct offset is computed at runtime.
  auto sampler_uniform = stage->GetUniform("u_texture");
  EXPECT_EQ(sampler_uniform->type, RuntimeUniformType::kSampledImage);
  EXPECT_EQ(sampler_uniform->binding, 65u);
}

TEST_P(RuntimeStageTest, CanRegisterStage) {
  const std::shared_ptr<fml::Mapping> fixture =
      flutter::testing::OpenFixtureAsMapping("ink_sparkle.frag.iplr");
  ASSERT_TRUE(fixture);
  ASSERT_GT(fixture->GetSize(), 0u);
  auto stages = RuntimeStage::DecodeRuntimeStages(fixture);
  auto stage = stages[PlaygroundBackendToRuntimeStageBackend(GetBackend())];
  ASSERT_TRUE(stage->IsValid());
  std::promise<bool> registration;
  auto future = registration.get_future();
  auto library = GetContext()->GetShaderLibrary();
  library->RegisterFunction(
      stage->GetEntrypoint(),                  //
      ToShaderStage(stage->GetShaderStage()),  //
      stage->GetCodeMapping(),                 //
      fml::MakeCopyable([reg = std::move(registration)](bool result) mutable {
        reg.set_value(result);
      }));
  ASSERT_TRUE(future.get());
  {
    auto function =
        library->GetFunction(stage->GetEntrypoint(), ShaderStage::kFragment);
    ASSERT_NE(function, nullptr);
  }

  // Check if unregistering works.

  library->UnregisterFunction(stage->GetEntrypoint(), ShaderStage::kFragment);
  {
    auto function =
        library->GetFunction(stage->GetEntrypoint(), ShaderStage::kFragment);
    ASSERT_EQ(function, nullptr);
  }
}

TEST_P(RuntimeStageTest, CanCreatePipelineFromRuntimeStage) {
  auto stages = OpenAssetAsRuntimeStage("ink_sparkle.frag.iplr");
  auto stage = stages[PlaygroundBackendToRuntimeStageBackend(GetBackend())];

  ASSERT_TRUE(stage);
  ASSERT_NE(stage, nullptr);
  ASSERT_TRUE(RegisterStage(*stage));
  auto library = GetContext()->GetShaderLibrary();
  using VS = RuntimeEffectVertexShader;
  PipelineDescriptor desc;
  desc.SetLabel("Runtime Stage InkSparkle");
  desc.AddStageEntrypoint(
      library->GetFunction(VS::kEntrypointName, ShaderStage::kVertex));
  desc.AddStageEntrypoint(
      library->GetFunction(stage->GetEntrypoint(), ShaderStage::kFragment));
  auto vertex_descriptor = std::make_shared<VertexDescriptor>();
  vertex_descriptor->SetStageInputs(VS::kAllShaderStageInputs,
                                    VS::kInterleavedBufferLayout);

  std::array<DescriptorSetLayout, 2> descriptor_set_layouts = {
      VS::kDescriptorSetLayouts[0],
      DescriptorSetLayout{
          .binding = 64u,
          .descriptor_type = DescriptorType::kUniformBuffer,
          .shader_stage = ShaderStage::kFragment,
      },
  };
  vertex_descriptor->RegisterDescriptorSetLayouts(descriptor_set_layouts);

  desc.SetVertexDescriptor(std::move(vertex_descriptor));
  ColorAttachmentDescriptor color0;
  color0.format = GetContext()->GetCapabilities()->GetDefaultColorFormat();
  StencilAttachmentDescriptor stencil0;
  stencil0.stencil_compare = CompareFunction::kEqual;
  desc.SetColorAttachmentDescriptor(0u, color0);
  desc.SetStencilAttachmentDescriptors(stencil0);
  const auto stencil_fmt =
      GetContext()->GetCapabilities()->GetDefaultStencilFormat();
  desc.SetStencilPixelFormat(stencil_fmt);
  auto pipeline = GetContext()->GetPipelineLibrary()->GetPipeline(desc).Get();
  ASSERT_NE(pipeline, nullptr);
}

TEST_P(RuntimeStageTest, ContainsExpectedShaderTypes) {
  auto stages = OpenAssetAsRuntimeStage("ink_sparkle.frag.iplr");
  // Right now, SkSL gets implicitly bundled regardless of what the build rule
  // for this test requested. After
  // https://github.com/flutter/flutter/issues/138919, this may require a build
  // rule change or a new test.
  EXPECT_TRUE(stages[RuntimeStageBackend::kSkSL]);

  EXPECT_TRUE(stages[RuntimeStageBackend::kOpenGLES]);
  EXPECT_TRUE(stages[RuntimeStageBackend::kMetal]);
  EXPECT_TRUE(stages[RuntimeStageBackend::kVulkan]);
}

}  // namespace testing
}  // namespace impeller
