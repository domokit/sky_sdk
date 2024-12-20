// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/compiler/types.h"

#include <cctype>
#include <filesystem>
#include <sstream>

#include "flutter/fml/logging.h"
#include "impeller/compiler/utilities.h"

namespace impeller {
namespace compiler {

static bool StringEndWith(const std::string& string,
                          const std::string& suffix) {
  if (suffix.size() > string.size()) {
    return false;
  }

  if (suffix.empty() || suffix.empty()) {
    return false;
  }

  return string.rfind(suffix) == (string.size() - suffix.size());
}

SourceType SourceTypeFromFileName(const std::string& file_name) {
  if (StringEndWith(file_name, ".vert")) {
    return SourceType::kVertexShader;
  }

  if (StringEndWith(file_name, ".frag")) {
    return SourceType::kFragmentShader;
  }

  if (StringEndWith(file_name, ".comp")) {
    return SourceType::kComputeShader;
  }

  return SourceType::kUnknown;
}

SourceType SourceTypeFromString(std::string name) {
  name = ToLowerCase(name);

  if (name == "vertex") {
    return SourceType::kVertexShader;
  }

  if (name == "fragment") {
    return SourceType::kFragmentShader;
  }

  if (name == "compute") {
    return SourceType::kComputeShader;
  }

  return SourceType::kUnknown;
}

SourceLanguage ToSourceLanguage(const std::string& source_language) {
  if (source_language == "glsl") {
    return SourceLanguage::kGLSL;
  }
  if (source_language == "hlsl") {
    return SourceLanguage::kHLSL;
  }
  return SourceLanguage::kUnknown;
}

std::string TargetPlatformToString(TargetPlatform platform) {
  switch (platform) {
    case TargetPlatform::kUnknown:
      return "Unknown";
    case TargetPlatform::kMetalDesktop:
      return "MetalDesktop";
    case TargetPlatform::kMetalIOS:
      return "MetaliOS";
    case TargetPlatform::kOpenGLES:
      return "OpenGLES";
    case TargetPlatform::kOpenGLDesktop:
      return "OpenGLDesktop";
    case TargetPlatform::kVulkan:
      return "Vulkan";
    case TargetPlatform::kRuntimeStageMetal:
      return "RuntimeStageMetal";
    case TargetPlatform::kRuntimeStageGLES:
      return "RuntimeStageGLES";
    case TargetPlatform::kRuntimeStageGLES3:
      return "RuntimeStageGLES3";
    case TargetPlatform::kRuntimeStageVulkan:
      return "RuntimeStageVulkan";
    case TargetPlatform::kSkSL:
      return "SkSL";
  }
  FML_UNREACHABLE();
}

std::string SourceLanguageToString(SourceLanguage source_language) {
  switch (source_language) {
    case SourceLanguage::kUnknown:
      return "Unknown";
    case SourceLanguage::kGLSL:
      return "GLSL";
    case SourceLanguage::kHLSL:
      return "HLSL";
  }
}

std::string EntryPointFunctionNameFromSourceName(
    const std::string& file_name,
    SourceType type,
    SourceLanguage source_language,
    const std::string& entry_point_name) {
  if (source_language == SourceLanguage::kHLSL) {
    return entry_point_name;
  }

  std::stringstream stream;
  std::filesystem::path file_path(file_name);
  stream << ConvertToEntrypointName(Utf8FromPath(file_path.stem())) << "_";
  switch (type) {
    case SourceType::kUnknown:
      stream << "unknown";
      break;
    case SourceType::kVertexShader:
      stream << "vertex";
      break;
    case SourceType::kFragmentShader:
      stream << "fragment";
      break;
    case SourceType::kComputeShader:
      stream << "compute";
      break;
  }
  stream << "_main";
  return stream.str();
}

bool TargetPlatformNeedsReflection(TargetPlatform platform) {
  switch (platform) {
    case TargetPlatform::kMetalIOS:
    case TargetPlatform::kMetalDesktop:
    case TargetPlatform::kOpenGLES:
    case TargetPlatform::kOpenGLDesktop:
    case TargetPlatform::kRuntimeStageMetal:
    case TargetPlatform::kRuntimeStageGLES:
    case TargetPlatform::kRuntimeStageGLES3:
    case TargetPlatform::kRuntimeStageVulkan:
    case TargetPlatform::kVulkan:
      return true;
    case TargetPlatform::kUnknown:
    case TargetPlatform::kSkSL:
      return false;
  }
  FML_UNREACHABLE();
}

std::string ShaderCErrorToString(shaderc_compilation_status status) {
  using Status = shaderc_compilation_status;
  switch (status) {
    case Status::shaderc_compilation_status_success:
      return "Success";
    case Status::shaderc_compilation_status_invalid_stage:
      return "Invalid shader stage specified";
    case Status::shaderc_compilation_status_compilation_error:
      return "Compilation error";
    case Status::shaderc_compilation_status_internal_error:
      return "Internal error";
    case Status::shaderc_compilation_status_null_result_object:
      return "Internal error. Null result object";
    case Status::shaderc_compilation_status_invalid_assembly:
      return "Invalid assembly";
    case Status::shaderc_compilation_status_validation_error:
      return "Validation error";
    case Status::shaderc_compilation_status_transformation_error:
      return "Transform error";
    case Status::shaderc_compilation_status_configuration_error:
      return "Configuration error";
  }
  return "Unknown internal error";
}

shaderc_shader_kind ToShaderCShaderKind(SourceType type) {
  switch (type) {
    case SourceType::kVertexShader:
      return shaderc_shader_kind::shaderc_vertex_shader;
    case SourceType::kFragmentShader:
      return shaderc_shader_kind::shaderc_fragment_shader;
    case SourceType::kComputeShader:
      return shaderc_shader_kind::shaderc_compute_shader;
    case SourceType::kUnknown:
      break;
  }
  return shaderc_shader_kind::shaderc_glsl_infer_from_source;
}

spv::ExecutionModel ToExecutionModel(SourceType type) {
  switch (type) {
    case SourceType::kVertexShader:
      return spv::ExecutionModel::ExecutionModelVertex;
    case SourceType::kFragmentShader:
      return spv::ExecutionModel::ExecutionModelFragment;
    case SourceType::kComputeShader:
      return spv::ExecutionModel::ExecutionModelGLCompute;
    case SourceType::kUnknown:
      break;
  }
  return spv::ExecutionModel::ExecutionModelMax;
}

spirv_cross::CompilerMSL::Options::Platform TargetPlatformToMSLPlatform(
    TargetPlatform platform) {
  switch (platform) {
    case TargetPlatform::kMetalIOS:
    case TargetPlatform::kRuntimeStageMetal:
      return spirv_cross::CompilerMSL::Options::Platform::iOS;
    case TargetPlatform::kMetalDesktop:
      return spirv_cross::CompilerMSL::Options::Platform::macOS;
    case TargetPlatform::kSkSL:
    case TargetPlatform::kOpenGLES:
    case TargetPlatform::kOpenGLDesktop:
    case TargetPlatform::kRuntimeStageGLES:
    case TargetPlatform::kRuntimeStageGLES3:
    case TargetPlatform::kRuntimeStageVulkan:
    case TargetPlatform::kVulkan:
    case TargetPlatform::kUnknown:
      return spirv_cross::CompilerMSL::Options::Platform::macOS;
  }
  FML_UNREACHABLE();
}

std::string SourceTypeToString(SourceType type) {
  switch (type) {
    case SourceType::kUnknown:
      return "unknown";
    case SourceType::kVertexShader:
      return "vert";
    case SourceType::kFragmentShader:
      return "frag";
    case SourceType::kComputeShader:
      return "comp";
  }
  FML_UNREACHABLE();
}

std::string TargetPlatformSLExtension(TargetPlatform platform) {
  switch (platform) {
    case TargetPlatform::kUnknown:
      return "unknown";
    case TargetPlatform::kMetalDesktop:
    case TargetPlatform::kMetalIOS:
    case TargetPlatform::kRuntimeStageMetal:
      return "metal";
    case TargetPlatform::kSkSL:
    case TargetPlatform::kOpenGLES:
    case TargetPlatform::kOpenGLDesktop:
    case TargetPlatform::kRuntimeStageGLES:
    case TargetPlatform::kRuntimeStageGLES3:
      return "glsl";
    case TargetPlatform::kVulkan:
    case TargetPlatform::kRuntimeStageVulkan:
      return "vk.spirv";
  }
  FML_UNREACHABLE();
}

bool TargetPlatformIsOpenGL(TargetPlatform platform) {
  switch (platform) {
    case TargetPlatform::kOpenGLES:
    case TargetPlatform::kOpenGLDesktop:
    case TargetPlatform::kRuntimeStageGLES:
    case TargetPlatform::kRuntimeStageGLES3:
      return true;
    case TargetPlatform::kMetalDesktop:
    case TargetPlatform::kRuntimeStageMetal:
    case TargetPlatform::kRuntimeStageVulkan:
    case TargetPlatform::kMetalIOS:
    case TargetPlatform::kUnknown:
    case TargetPlatform::kSkSL:
    case TargetPlatform::kVulkan:
      return false;
  }
  FML_UNREACHABLE();
}

bool TargetPlatformIsMetal(TargetPlatform platform) {
  switch (platform) {
    case TargetPlatform::kMetalDesktop:
    case TargetPlatform::kMetalIOS:
    case TargetPlatform::kRuntimeStageMetal:
      return true;
    case TargetPlatform::kUnknown:
    case TargetPlatform::kSkSL:
    case TargetPlatform::kOpenGLES:
    case TargetPlatform::kOpenGLDesktop:
    case TargetPlatform::kRuntimeStageGLES:
    case TargetPlatform::kRuntimeStageGLES3:
    case TargetPlatform::kRuntimeStageVulkan:
    case TargetPlatform::kVulkan:
      return false;
  }
  FML_UNREACHABLE();
}

bool TargetPlatformIsVulkan(TargetPlatform platform) {
  switch (platform) {
    case TargetPlatform::kRuntimeStageVulkan:
    case TargetPlatform::kVulkan:
      return true;
    case TargetPlatform::kMetalDesktop:
    case TargetPlatform::kMetalIOS:
    case TargetPlatform::kRuntimeStageMetal:
    case TargetPlatform::kUnknown:
    case TargetPlatform::kSkSL:
    case TargetPlatform::kOpenGLES:
    case TargetPlatform::kOpenGLDesktop:
    case TargetPlatform::kRuntimeStageGLES:
    case TargetPlatform::kRuntimeStageGLES3:
      return false;
  }
  FML_UNREACHABLE();
}

bool TargetPlatformBundlesSkSL(TargetPlatform platform) {
  switch (platform) {
    case TargetPlatform::kSkSL:
    case TargetPlatform::kRuntimeStageMetal:
    case TargetPlatform::kRuntimeStageGLES:
    case TargetPlatform::kRuntimeStageGLES3:
    case TargetPlatform::kRuntimeStageVulkan:
      return true;
    case TargetPlatform::kMetalDesktop:
    case TargetPlatform::kMetalIOS:
    case TargetPlatform::kUnknown:
    case TargetPlatform::kOpenGLES:
    case TargetPlatform::kOpenGLDesktop:
    case TargetPlatform::kVulkan:
      return false;
  }
  FML_UNREACHABLE();
}

}  // namespace compiler
}  // namespace impeller
