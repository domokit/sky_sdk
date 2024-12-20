// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_CORE_RUNTIME_TYPES_H_
#define FLUTTER_IMPELLER_CORE_RUNTIME_TYPES_H_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace impeller {

enum class RuntimeStageBackend {
  kSkSL,
  kMetal,
  kOpenGLES,
  kOpenGLES3,
  kVulkan,
};

enum RuntimeUniformType {
  kFloat,
  kSampledImage,
  kStruct,
};

enum class RuntimeShaderStage {
  kVertex,
  kFragment,
  kCompute,
};

struct RuntimeUniformDimensions {
  size_t rows = 0;
  size_t cols = 0;
};

struct RuntimeUniformDescription {
  std::string name;
  size_t location = 0u;
  /// Location, but for Vulkan.
  size_t binding = 0u;
  RuntimeUniformType type = RuntimeUniformType::kFloat;
  RuntimeUniformDimensions dimensions = {};
  size_t bit_width = 0u;
  std::optional<size_t> array_elements;
  std::vector<uint8_t> struct_layout = {};
  size_t struct_float_count = 0u;

  /// @brief  Computes the total number of bytes that this uniform requires.
  size_t GetSize() const;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_CORE_RUNTIME_TYPES_H_
