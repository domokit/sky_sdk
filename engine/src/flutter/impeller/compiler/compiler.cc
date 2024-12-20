// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/compiler/compiler.h"

#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>

#include "flutter/fml/paths.h"
#include "impeller/base/allocation.h"
#include "impeller/compiler/compiler_backend.h"
#include "impeller/compiler/constants.h"
#include "impeller/compiler/includer.h"
#include "impeller/compiler/logger.h"
#include "impeller/compiler/spirv_compiler.h"
#include "impeller/compiler/types.h"
#include "impeller/compiler/uniform_sorter.h"
#include "impeller/compiler/utilities.h"

namespace impeller {
namespace compiler {

namespace {
constexpr const char* kEGLImageExternalExtension = "GL_OES_EGL_image_external";
constexpr const char* kEGLImageExternalExtension300 =
    "GL_OES_EGL_image_external_essl3";
}  // namespace

static uint32_t ParseMSLVersion(const std::string& msl_version) {
  std::stringstream sstream(msl_version);
  std::string version_part;
  uint32_t major = 1;
  uint32_t minor = 2;
  uint32_t patch = 0;
  if (std::getline(sstream, version_part, '.')) {
    major = std::stoi(version_part);
    if (std::getline(sstream, version_part, '.')) {
      minor = std::stoi(version_part);
      if (std::getline(sstream, version_part, '.')) {
        patch = std::stoi(version_part);
      }
    }
  }
  if (major < 1 || (major == 1 && minor < 2)) {
    std::cerr << "--metal-version version must be at least 1.2. Have "
              << msl_version << std::endl;
  }
  return spirv_cross::CompilerMSL::Options::make_msl_version(major, minor,
                                                             patch);
}

static CompilerBackend CreateMSLCompiler(
    const spirv_cross::ParsedIR& ir,
    const SourceOptions& source_options,
    std::optional<uint32_t> msl_version_override = {}) {
  auto sl_compiler = std::make_shared<spirv_cross::CompilerMSL>(ir);
  spirv_cross::CompilerMSL::Options sl_options;
  sl_options.platform =
      TargetPlatformToMSLPlatform(source_options.target_platform);
  sl_options.msl_version = msl_version_override.value_or(
      ParseMSLVersion(source_options.metal_version));
  sl_options.ios_use_simdgroup_functions =
      sl_options.is_ios() &&
      sl_options.msl_version >=
          spirv_cross::CompilerMSL::Options::make_msl_version(2, 4, 0);
  sl_options.use_framebuffer_fetch_subpasses = true;
  sl_compiler->set_msl_options(sl_options);

  // Sort the float and sampler uniforms according to their declared/decorated
  // order. For user authored fragment shaders, the API for setting uniform
  // values uses the index of the uniform in the declared order. By default, the
  // metal backend of spirv-cross will order uniforms according to usage. To fix
  // this, we use the sorted order and the add_msl_resource_binding API to force
  // the ordering to match the declared order. Note that while this code runs
  // for all compiled shaders, it will only affect vertex and fragment shaders
  // due to the specified stage.
  auto floats =
      SortUniforms(&ir, sl_compiler.get(), spirv_cross::SPIRType::Float);
  auto images =
      SortUniforms(&ir, sl_compiler.get(), spirv_cross::SPIRType::SampledImage);

  spv::ExecutionModel execution_model =
      spv::ExecutionModel::ExecutionModelFragment;
  if (source_options.type == SourceType::kVertexShader) {
    execution_model = spv::ExecutionModel::ExecutionModelVertex;
  }

  uint32_t buffer_offset = 0;
  uint32_t sampler_offset = 0;
  for (auto& float_id : floats) {
    sl_compiler->add_msl_resource_binding(
        {.stage = execution_model,
         .basetype = spirv_cross::SPIRType::BaseType::Float,
         .desc_set = sl_compiler->get_decoration(float_id,
                                                 spv::DecorationDescriptorSet),
         .binding =
             sl_compiler->get_decoration(float_id, spv::DecorationBinding),
         .count = 1u,
         .msl_buffer = buffer_offset});
    buffer_offset++;
  }
  for (auto& image_id : images) {
    sl_compiler->add_msl_resource_binding({
        .stage = execution_model,
        .basetype = spirv_cross::SPIRType::BaseType::SampledImage,
        .desc_set =
            sl_compiler->get_decoration(image_id, spv::DecorationDescriptorSet),
        .binding =
            sl_compiler->get_decoration(image_id, spv::DecorationBinding),
        .count = 1u,
        // A sampled image is both an image and a sampler, so both
        // offsets need to be set or depending on the partiular shader
        // the bindings may be incorrect.
        .msl_texture = sampler_offset,
        .msl_sampler = sampler_offset,
    });
    sampler_offset++;
  }

  return CompilerBackend(sl_compiler);
}

static CompilerBackend CreateVulkanCompiler(
    const spirv_cross::ParsedIR& ir,
    const SourceOptions& source_options) {
  auto gl_compiler = std::make_shared<spirv_cross::CompilerGLSL>(ir);
  spirv_cross::CompilerGLSL::Options sl_options;
  sl_options.force_zero_initialized_variables = true;
  sl_options.vertex.fixup_clipspace = true;
  sl_options.vulkan_semantics = true;
  gl_compiler->set_common_options(sl_options);
  return CompilerBackend(gl_compiler);
}

static CompilerBackend CreateGLSLCompiler(const spirv_cross::ParsedIR& ir,
                                          const SourceOptions& source_options) {
  auto gl_compiler = std::make_shared<spirv_cross::CompilerGLSL>(ir);

  // Walk the variables and insert the external image extension if any of them
  // begins with the external texture prefix. Unfortunately, we can't walk
  // `gl_compiler->get_shader_resources().separate_samplers` until the compiler
  // is further along.
  //
  // Unfortunately, we can't just let the shader author add this extension and
  // use `samplerExternalOES` directly because compiling to spirv requires the
  // source language profile to be at least 310 ES, but this extension is
  // incompatible with ES 310+.
  for (auto& id : ir.ids_for_constant_or_variable) {
    if (StringStartsWith(ir.get_name(id), kExternalTexturePrefix)) {
      if (source_options.gles_language_version >= 300) {
        gl_compiler->require_extension(kEGLImageExternalExtension300);
      } else {
        gl_compiler->require_extension(kEGLImageExternalExtension);
      }
      break;
    }
  }

  spirv_cross::CompilerGLSL::Options sl_options;
  sl_options.force_zero_initialized_variables = true;
  sl_options.vertex.fixup_clipspace = true;
  if (source_options.target_platform == TargetPlatform::kOpenGLES ||
      source_options.target_platform == TargetPlatform::kRuntimeStageGLES ||
      source_options.target_platform == TargetPlatform::kRuntimeStageGLES3) {
    sl_options.version = source_options.gles_language_version > 0
                             ? source_options.gles_language_version
                             : 100;
    sl_options.es = true;
    if (source_options.target_platform == TargetPlatform::kRuntimeStageGLES3) {
      sl_options.version = 300;
    }
    if (source_options.require_framebuffer_fetch &&
        source_options.type == SourceType::kFragmentShader) {
      gl_compiler->remap_ext_framebuffer_fetch(0, 0, true);
    }
    gl_compiler->set_variable_type_remap_callback(
        [&](const spirv_cross::SPIRType& type, const std::string& var_name,
            std::string& name_of_type) {
          if (StringStartsWith(var_name, kExternalTexturePrefix)) {
            name_of_type = "samplerExternalOES";
          }
        });
  } else {
    sl_options.version = source_options.gles_language_version > 0
                             ? source_options.gles_language_version
                             : 120;
    sl_options.es = false;
  }
  gl_compiler->set_common_options(sl_options);
  return CompilerBackend(gl_compiler);
}

static CompilerBackend CreateSkSLCompiler(const spirv_cross::ParsedIR& ir,
                                          const SourceOptions& source_options) {
  auto sksl_compiler = std::make_shared<CompilerSkSL>(ir);
  return CompilerBackend(sksl_compiler);
}

static bool EntryPointMustBeNamedMain(TargetPlatform platform) {
  switch (platform) {
    case TargetPlatform::kUnknown:
      FML_UNREACHABLE();
    case TargetPlatform::kMetalDesktop:
    case TargetPlatform::kMetalIOS:
    case TargetPlatform::kVulkan:
    case TargetPlatform::kRuntimeStageMetal:
    case TargetPlatform::kRuntimeStageVulkan:
      return false;
    case TargetPlatform::kSkSL:
    case TargetPlatform::kOpenGLES:
    case TargetPlatform::kOpenGLDesktop:
    case TargetPlatform::kRuntimeStageGLES:
    case TargetPlatform::kRuntimeStageGLES3:
      return true;
  }
  FML_UNREACHABLE();
}

static CompilerBackend CreateCompiler(const spirv_cross::ParsedIR& ir,
                                      const SourceOptions& source_options) {
  CompilerBackend compiler;
  switch (source_options.target_platform) {
    case TargetPlatform::kMetalDesktop:
    case TargetPlatform::kMetalIOS:
    case TargetPlatform::kRuntimeStageMetal:
      compiler = CreateMSLCompiler(ir, source_options);
      break;
    case TargetPlatform::kVulkan:
    case TargetPlatform::kRuntimeStageVulkan:
      compiler = CreateVulkanCompiler(ir, source_options);
      break;
    case TargetPlatform::kUnknown:
    case TargetPlatform::kOpenGLES:
    case TargetPlatform::kOpenGLDesktop:
    case TargetPlatform::kRuntimeStageGLES:
    case TargetPlatform::kRuntimeStageGLES3:
      compiler = CreateGLSLCompiler(ir, source_options);
      break;
    case TargetPlatform::kSkSL:
      compiler = CreateSkSLCompiler(ir, source_options);
  }
  if (!compiler) {
    return {};
  }
  auto* backend = compiler.GetCompiler();
  if (!EntryPointMustBeNamedMain(source_options.target_platform) &&
      source_options.source_language == SourceLanguage::kGLSL) {
    backend->rename_entry_point("main", source_options.entry_point_name,
                                ToExecutionModel(source_options.type));
  }
  return compiler;
}

Compiler::Compiler(const std::shared_ptr<const fml::Mapping>& source_mapping,
                   const SourceOptions& source_options,
                   Reflector::Options reflector_options)
    : options_(source_options) {
  if (!source_mapping || source_mapping->GetMapping() == nullptr) {
    COMPILER_ERROR(error_stream_)
        << "Could not read shader source or shader source was empty.";
    return;
  }

  if (source_options.target_platform == TargetPlatform::kUnknown) {
    COMPILER_ERROR(error_stream_) << "Target platform not specified.";
    return;
  }

  SPIRVCompilerOptions spirv_options;

  // Make sure reflection is as effective as possible. The generated shaders
  // will be processed later by backend specific compilers.
  spirv_options.generate_debug_info = true;

  switch (options_.source_language) {
    case SourceLanguage::kGLSL:
      // Expects GLSL 4.60 (Core Profile).
      // https://www.khronos.org/registry/OpenGL/specs/gl/GLSLangSpec.4.60.pdf
      spirv_options.source_langauge =
          shaderc_source_language::shaderc_source_language_glsl;
      spirv_options.source_profile = SPIRVCompilerSourceProfile{
          shaderc_profile::shaderc_profile_core,  //
          460,                                    //
      };
      break;
    case SourceLanguage::kHLSL:
      spirv_options.source_langauge =
          shaderc_source_language::shaderc_source_language_hlsl;
      break;
    case SourceLanguage::kUnknown:
      COMPILER_ERROR(error_stream_) << "Source language invalid.";
      return;
  }

  switch (source_options.target_platform) {
    case TargetPlatform::kMetalDesktop:
    case TargetPlatform::kMetalIOS: {
      SPIRVCompilerTargetEnv target;

      if (source_options.use_half_textures) {
        target.env = shaderc_target_env::shaderc_target_env_opengl;
        target.version = shaderc_env_version::shaderc_env_version_opengl_4_5;
        target.spirv_version = shaderc_spirv_version::shaderc_spirv_version_1_0;
      } else {
        target.env = shaderc_target_env::shaderc_target_env_vulkan;
        target.version = shaderc_env_version::shaderc_env_version_vulkan_1_1;
        target.spirv_version = shaderc_spirv_version::shaderc_spirv_version_1_3;
      }

      spirv_options.target = target;
    } break;
    case TargetPlatform::kOpenGLES:
    case TargetPlatform::kOpenGLDesktop:
    case TargetPlatform::kVulkan:
    case TargetPlatform::kRuntimeStageVulkan: {
      SPIRVCompilerTargetEnv target;

      target.env = shaderc_target_env::shaderc_target_env_vulkan;
      target.version = shaderc_env_version::shaderc_env_version_vulkan_1_1;
      target.spirv_version = shaderc_spirv_version::shaderc_spirv_version_1_3;

      if (source_options.target_platform ==
          TargetPlatform::kRuntimeStageVulkan) {
        spirv_options.macro_definitions.push_back("IMPELLER_GRAPHICS_BACKEND");
        spirv_options.relaxed_vulkan_rules = true;
      }
      spirv_options.target = target;
    } break;
    case TargetPlatform::kRuntimeStageMetal:
    case TargetPlatform::kRuntimeStageGLES:
    case TargetPlatform::kRuntimeStageGLES3: {
      SPIRVCompilerTargetEnv target;

      target.env = shaderc_target_env::shaderc_target_env_opengl;
      target.version = shaderc_env_version::shaderc_env_version_opengl_4_5;
      target.spirv_version = shaderc_spirv_version::shaderc_spirv_version_1_0;

      spirv_options.target = target;
      spirv_options.macro_definitions.push_back("IMPELLER_GRAPHICS_BACKEND");
    } break;
    case TargetPlatform::kSkSL: {
      SPIRVCompilerTargetEnv target;

      target.env = shaderc_target_env::shaderc_target_env_opengl;
      target.version = shaderc_env_version::shaderc_env_version_opengl_4_5;
      target.spirv_version = shaderc_spirv_version::shaderc_spirv_version_1_0;

      // When any optimization level above 'zero' is enabled, the phi merges at
      // loop continue blocks are rendered using syntax that is supported in
      // GLSL, but not in SkSL.
      // https://bugs.chromium.org/p/skia/issues/detail?id=13518.
      spirv_options.optimization_level =
          shaderc_optimization_level::shaderc_optimization_level_zero;
      spirv_options.target = target;
      spirv_options.macro_definitions.push_back("SKIA_GRAPHICS_BACKEND");
    } break;
    case TargetPlatform::kUnknown:
      COMPILER_ERROR(error_stream_) << "Target platform invalid.";
      return;
  }

  // Implicit definition that indicates that this compilation is for the device
  // (instead of the host).
  spirv_options.macro_definitions.push_back("IMPELLER_DEVICE");
  for (const auto& define : source_options.defines) {
    spirv_options.macro_definitions.push_back(define);
  }

  std::vector<std::string> included_file_names;
  spirv_options.includer = std::make_shared<Includer>(
      options_.working_directory, options_.include_dirs,
      [&included_file_names](auto included_name) {
        included_file_names.emplace_back(std::move(included_name));
      });

  // SPIRV Generation.
  SPIRVCompiler spv_compiler(source_options, source_mapping);

  spirv_assembly_ = spv_compiler.CompileToSPV(
      error_stream_, spirv_options.BuildShadercOptions());

  if (!spirv_assembly_) {
    return;
  } else {
    included_file_names_ = std::move(included_file_names);
  }

  // SL Generation.
  spirv_cross::Parser parser(
      reinterpret_cast<const uint32_t*>(spirv_assembly_->GetMapping()),
      spirv_assembly_->GetSize() / sizeof(uint32_t));
  // The parser and compiler must be run separately because the parser contains
  // meta information (like type member names) that are useful for reflection.
  parser.parse();

  const auto parsed_ir =
      std::make_shared<spirv_cross::ParsedIR>(parser.get_parsed_ir());

  auto sl_compiler = CreateCompiler(*parsed_ir, options_);

  if (!sl_compiler) {
    COMPILER_ERROR(error_stream_)
        << "Could not create compiler for target platform.";
    return;
  }

  // We need to invoke the compiler even if we don't use the SL mapping later
  // for Vulkan. The reflector needs information that is only valid after a
  // successful compilation call.
  auto sl_compilation_result =
      CreateMappingWithString(sl_compiler.GetCompiler()->compile());

  // If the target is Vulkan, our shading language is SPIRV which we already
  // have. We just need to strip it of debug information. If it isn't, we need
  // to invoke the appropriate compiler to compile the SPIRV to the target SL.
  if (source_options.target_platform == TargetPlatform::kVulkan ||
      source_options.target_platform == TargetPlatform::kRuntimeStageVulkan) {
    auto stripped_spirv_options = spirv_options;
    stripped_spirv_options.generate_debug_info = false;
    sl_mapping_ = spv_compiler.CompileToSPV(
        error_stream_, stripped_spirv_options.BuildShadercOptions());
  } else {
    sl_mapping_ = sl_compilation_result;
  }

  if (!sl_mapping_) {
    COMPILER_ERROR(error_stream_) << "Could not generate SL from SPIRV";
    return;
  }

  reflector_ = std::make_unique<Reflector>(std::move(reflector_options),  //
                                           parsed_ir,                     //
                                           GetSLShaderSource(),           //
                                           sl_compiler                    //
  );

  if (!reflector_->IsValid()) {
    COMPILER_ERROR(error_stream_)
        << "Could not complete reflection on generated shader.";
    return;
  }

  is_valid_ = true;
}

Compiler::~Compiler() = default;

std::shared_ptr<fml::Mapping> Compiler::GetSPIRVAssembly() const {
  return spirv_assembly_;
}

std::shared_ptr<fml::Mapping> Compiler::GetSLShaderSource() const {
  return sl_mapping_;
}

bool Compiler::IsValid() const {
  return is_valid_;
}

std::string Compiler::GetSourcePrefix() const {
  std::stringstream stream;
  stream << options_.file_name << ": ";
  return stream.str();
}

std::string Compiler::GetErrorMessages() const {
  return error_stream_.str();
}

const std::vector<std::string>& Compiler::GetIncludedFileNames() const {
  return included_file_names_;
}

static std::string JoinStrings(std::vector<std::string> items,
                               const std::string& separator) {
  std::stringstream stream;
  for (size_t i = 0, count = items.size(); i < count; i++) {
    const auto is_last = (i == count - 1);

    stream << items[i];
    if (!is_last) {
      stream << separator;
    }
  }
  return stream.str();
}

std::string Compiler::GetDependencyNames(const std::string& separator) const {
  std::vector<std::string> dependencies = included_file_names_;
  dependencies.push_back(options_.file_name);
  return JoinStrings(dependencies, separator);
}

std::unique_ptr<fml::Mapping> Compiler::CreateDepfileContents(
    std::initializer_list<std::string> targets_names) const {
  // https://github.com/ninja-build/ninja/blob/master/src/depfile_parser.cc#L28
  const auto targets = JoinStrings(targets_names, " ");
  const auto dependencies = GetDependencyNames(" ");

  std::stringstream stream;
  stream << targets << ": " << dependencies << "\n";

  auto contents = std::make_shared<std::string>(stream.str());
  return std::make_unique<fml::NonOwnedMapping>(
      reinterpret_cast<const uint8_t*>(contents->data()), contents->size(),
      [contents](auto, auto) {});
}

const Reflector* Compiler::GetReflector() const {
  return reflector_.get();
}

}  // namespace compiler
}  // namespace impeller
