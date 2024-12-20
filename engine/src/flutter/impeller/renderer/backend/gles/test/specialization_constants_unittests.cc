// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/testing/testing.h"  // IWYU pragma: keep
#include "fml/mapping.h"
#include "gtest/gtest.h"
#include "impeller/renderer/backend/gles/proc_table_gles.h"
#include "impeller/renderer/backend/gles/test/mock_gles.h"

namespace impeller {
namespace testing {

TEST(SpecConstant, CanCreateShaderWithSpecializationConstant) {
  auto mock_gles = MockGLES::Init();
  auto& proc_table = mock_gles->GetProcTable();
  auto shader_source =
      "#version 100\n"
      "#ifndef SPIRV_CROSS_CONSTANT_ID_0\n"
      "#define SPIRV_CROSS_CONSTANT_ID_0 1\n"
      "#endif\n"
      "void main() { return vec4(0.0); }";
  auto test_shader = std::make_shared<fml::DataMapping>(shader_source);

  auto result = proc_table.ComputeShaderWithDefines(*test_shader, {0});

  auto expected_shader_source =
      "#version 100\n"
      "#define SPIRV_CROSS_CONSTANT_ID_0 0.000000\n"
      "#ifndef SPIRV_CROSS_CONSTANT_ID_0\n"
      "#define SPIRV_CROSS_CONSTANT_ID_0 1\n"
      "#endif\n"
      "void main() { return vec4(0.0); }";

  if (!result.has_value()) {
    GTEST_FAIL() << "Expected shader source";
  }
  ASSERT_EQ(result.value(), expected_shader_source);
}

TEST(SpecConstant, CanCreateShaderWithSpecializationConstantMultipleValues) {
  auto mock_gles = MockGLES::Init();
  auto& proc_table = mock_gles->GetProcTable();
  auto shader_source =
      "#version 100\n"
      "#ifndef SPIRV_CROSS_CONSTANT_ID_0\n"
      "#define SPIRV_CROSS_CONSTANT_ID_0 1\n"
      "#endif\n"
      "void main() { return vec4(0.0); }";
  auto test_shader = std::make_shared<fml::DataMapping>(shader_source);

  auto result =
      proc_table.ComputeShaderWithDefines(*test_shader, {0, 1, 2, 3, 4, 5});

  auto expected_shader_source =
      "#version 100\n"
      "#define SPIRV_CROSS_CONSTANT_ID_0 0.000000\n"
      "#define SPIRV_CROSS_CONSTANT_ID_1 1.000000\n"
      "#define SPIRV_CROSS_CONSTANT_ID_2 2.000000\n"
      "#define SPIRV_CROSS_CONSTANT_ID_3 3.000000\n"
      "#define SPIRV_CROSS_CONSTANT_ID_4 4.000000\n"
      "#define SPIRV_CROSS_CONSTANT_ID_5 5.000000\n"
      "#ifndef SPIRV_CROSS_CONSTANT_ID_0\n"
      "#define SPIRV_CROSS_CONSTANT_ID_0 1\n"
      "#endif\n"
      "void main() { return vec4(0.0); }";

  if (!result.has_value()) {
    GTEST_FAIL() << "Expected shader source";
  }
  ASSERT_EQ(result.value(), expected_shader_source);
}

}  // namespace testing
}  // namespace impeller
