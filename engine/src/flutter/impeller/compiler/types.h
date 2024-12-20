// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_COMPILER_TYPES_H_
#define FLUTTER_IMPELLER_COMPILER_TYPES_H_

#include <codecvt>
#include <locale>
#include <map>
#include <optional>
#include <string>

#include "shaderc/shaderc.hpp"
#include "spirv_cross.hpp"
#include "spirv_msl.hpp"

namespace impeller {
namespace compiler {

enum class SourceType {
  kUnknown,
  kVertexShader,
  kFragmentShader,
  kComputeShader,
};

enum class TargetPlatform {
  kUnknown,
  kMetalDesktop,
  kMetalIOS,
  kOpenGLES,
  kOpenGLDesktop,
  kVulkan,
  kRuntimeStageMetal,
  kRuntimeStageGLES,
  kRuntimeStageGLES3,
  kRuntimeStageVulkan,
  kSkSL,
};

enum class SourceLanguage {
  kUnknown,
  kGLSL,
  kHLSL,
};

struct UniformDescription {
  std::string name;
  size_t location = 0u;
  size_t binding = 0u;
  spirv_cross::SPIRType::BaseType type = spirv_cross::SPIRType::BaseType::Float;
  size_t rows = 0u;
  size_t columns = 0u;
  size_t bit_width = 0u;
  std::optional<size_t> array_elements = std::nullopt;
  std::vector<uint8_t> struct_layout = {};
  size_t struct_float_count = 0u;
};

struct InputDescription {
  std::string name;
  size_t location;
  size_t set;
  size_t binding;
  spirv_cross::SPIRType::BaseType type =
      spirv_cross::SPIRType::BaseType::Unknown;
  size_t bit_width;
  size_t vec_size;
  size_t columns;
  size_t offset;
};

/// A shader config parsed as part of a ShaderBundleConfig.
struct ShaderConfig {
  std::string source_file_name;
  SourceType type;
  SourceLanguage language;
  std::string entry_point;
};

using ShaderBundleConfig = std::unordered_map<std::string, ShaderConfig>;

bool TargetPlatformIsMetal(TargetPlatform platform);

bool TargetPlatformIsOpenGL(TargetPlatform platform);

bool TargetPlatformIsVulkan(TargetPlatform platform);

SourceType SourceTypeFromFileName(const std::string& file_name);

SourceType SourceTypeFromString(std::string name);

std::string SourceTypeToString(SourceType type);

std::string TargetPlatformToString(TargetPlatform platform);

SourceLanguage ToSourceLanguage(const std::string& source_language);

std::string SourceLanguageToString(SourceLanguage source_language);

std::string TargetPlatformSLExtension(TargetPlatform platform);

std::string EntryPointFunctionNameFromSourceName(
    const std::string& file_name,
    SourceType type,
    SourceLanguage source_language,
    const std::string& entry_point_name);

bool TargetPlatformNeedsReflection(TargetPlatform platform);

bool TargetPlatformBundlesSkSL(TargetPlatform platform);

std::string ShaderCErrorToString(shaderc_compilation_status status);

shaderc_shader_kind ToShaderCShaderKind(SourceType type);

spv::ExecutionModel ToExecutionModel(SourceType type);

spirv_cross::CompilerMSL::Options::Platform TargetPlatformToMSLPlatform(
    TargetPlatform platform);

}  // namespace compiler
}  // namespace impeller

#endif  // FLUTTER_IMPELLER_COMPILER_TYPES_H_
