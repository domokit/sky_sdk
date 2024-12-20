// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/compiler/spirv_compiler.h"

#include <array>

#include "impeller/compiler/logger.h"
#include "impeller/compiler/types.h"

namespace impeller {
namespace compiler {

SPIRVCompiler::SPIRVCompiler(const SourceOptions& options,
                             std::shared_ptr<const fml::Mapping> sources)
    : options_(options), sources_(std::move(sources)) {}

SPIRVCompiler::~SPIRVCompiler() = default;

std::shared_ptr<fml::Mapping> SPIRVCompiler::CompileToSPV(
    std::stringstream& stream,
    const shaderc::CompileOptions& spirv_options) const {
  if (!sources_ || sources_->GetMapping() == nullptr) {
    COMPILER_ERROR(stream) << "Invalid sources for SPIRV Compiler.";
    return nullptr;
  }

  shaderc::Compiler spv_compiler;
  if (!spv_compiler.IsValid()) {
    COMPILER_ERROR(stream) << "Could not initialize the "
                           << SourceLanguageToString(options_.source_language)
                           << " to SPIRV compiler.";
    return nullptr;
  }

  const auto shader_kind = ToShaderCShaderKind(options_.type);

  if (shader_kind == shaderc_shader_kind::shaderc_glsl_infer_from_source) {
    COMPILER_ERROR(stream) << "Could not figure out shader stage.";
    return nullptr;
  }

  auto result = std::make_shared<shaderc::SpvCompilationResult>(
      spv_compiler.CompileGlslToSpv(
          reinterpret_cast<const char*>(sources_->GetMapping()),  // source_text
          sources_->GetSize(),                // source_text_size
          shader_kind,                        // shader_kind
          options_.file_name.c_str(),         // input_file_name
          options_.entry_point_name.c_str(),  // entry_point_name
          spirv_options                       // options
          ));
  if (result->GetCompilationStatus() !=
      shaderc_compilation_status::shaderc_compilation_status_success) {
    COMPILER_ERROR(stream) << SourceLanguageToString(options_.source_language)
                           << " to SPIRV failed; "
                           << ShaderCErrorToString(
                                  result->GetCompilationStatus())
                           << ". " << result->GetNumErrors() << " error(s) and "
                           << result->GetNumWarnings() << " warning(s).";
    // It should normally be enough to check that there are errors or warnings,
    // but some cases result in no errors or warnings and still have an error
    // message. If there's a message we should print it.
    if (result->GetNumErrors() > 0 || result->GetNumWarnings() > 0 ||
        !result->GetErrorMessage().empty()) {
      COMPILER_ERROR_NO_PREFIX(stream) << result->GetErrorMessage();
    }
    return nullptr;
  }

  if (!result) {
    COMPILER_ERROR(stream) << "Could not fetch SPIRV from compile job.";
    return nullptr;
  }

  const auto data_length = (result->cend() - result->cbegin()) *
                           sizeof(decltype(result)::element_type::element_type);

  return std::make_unique<fml::NonOwnedMapping>(
      reinterpret_cast<const uint8_t*>(result->cbegin()),  //
      data_length,                                         //
      [result](auto, auto) {}                              //
  );
}

std::string SPIRVCompiler::GetSourcePrefix() const {
  std::stringstream stream;
  stream << options_.file_name << ": ";
  return stream.str();
}

static void SetDefaultLimitations(shaderc::CompileOptions& compiler_opts) {
  using Limit = std::pair<shaderc_limit, int>;
  static constexpr std::array<Limit, 83> limits = {
      Limit{shaderc_limit::shaderc_limit_max_lights, 8},
      Limit{shaderc_limit::shaderc_limit_max_clip_planes, 6},
      Limit{shaderc_limit::shaderc_limit_max_texture_units, 2},
      Limit{shaderc_limit::shaderc_limit_max_texture_coords, 8},
      Limit{shaderc_limit::shaderc_limit_max_vertex_attribs, 16},
      Limit{shaderc_limit::shaderc_limit_max_vertex_uniform_components, 4096},
      Limit{shaderc_limit::shaderc_limit_max_varying_floats, 60},
      Limit{shaderc_limit::shaderc_limit_max_vertex_texture_image_units, 16},
      Limit{shaderc_limit::shaderc_limit_max_combined_texture_image_units, 80},
      Limit{shaderc_limit::shaderc_limit_max_texture_image_units, 16},
      Limit{shaderc_limit::shaderc_limit_max_fragment_uniform_components, 1024},
      Limit{shaderc_limit::shaderc_limit_max_draw_buffers, 8},
      Limit{shaderc_limit::shaderc_limit_max_vertex_uniform_vectors, 256},
      Limit{shaderc_limit::shaderc_limit_max_varying_vectors, 15},
      Limit{shaderc_limit::shaderc_limit_max_fragment_uniform_vectors, 256},
      Limit{shaderc_limit::shaderc_limit_max_vertex_output_vectors, 16},
      Limit{shaderc_limit::shaderc_limit_max_fragment_input_vectors, 15},
      Limit{shaderc_limit::shaderc_limit_min_program_texel_offset, -8},
      Limit{shaderc_limit::shaderc_limit_max_program_texel_offset, 7},
      Limit{shaderc_limit::shaderc_limit_max_clip_distances, 8},
      Limit{shaderc_limit::shaderc_limit_max_compute_work_group_count_x, 65535},
      Limit{shaderc_limit::shaderc_limit_max_compute_work_group_count_y, 65535},
      Limit{shaderc_limit::shaderc_limit_max_compute_work_group_count_z, 65535},
      Limit{shaderc_limit::shaderc_limit_max_compute_work_group_size_x, 1024},
      Limit{shaderc_limit::shaderc_limit_max_compute_work_group_size_y, 1024},
      Limit{shaderc_limit::shaderc_limit_max_compute_work_group_size_z, 64},
      Limit{shaderc_limit::shaderc_limit_max_compute_uniform_components, 512},
      Limit{shaderc_limit::shaderc_limit_max_compute_texture_image_units, 16},
      Limit{shaderc_limit::shaderc_limit_max_compute_image_uniforms, 8},
      Limit{shaderc_limit::shaderc_limit_max_compute_atomic_counters, 8},
      Limit{shaderc_limit::shaderc_limit_max_compute_atomic_counter_buffers, 1},
      Limit{shaderc_limit::shaderc_limit_max_varying_components, 60},
      Limit{shaderc_limit::shaderc_limit_max_vertex_output_components, 64},
      Limit{shaderc_limit::shaderc_limit_max_geometry_input_components, 64},
      Limit{shaderc_limit::shaderc_limit_max_geometry_output_components, 128},
      Limit{shaderc_limit::shaderc_limit_max_fragment_input_components, 128},
      Limit{shaderc_limit::shaderc_limit_max_image_units, 8},
      Limit{shaderc_limit::
                shaderc_limit_max_combined_image_units_and_fragment_outputs,
            8},
      Limit{shaderc_limit::shaderc_limit_max_combined_shader_output_resources,
            8},
      Limit{shaderc_limit::shaderc_limit_max_image_samples, 0},
      Limit{shaderc_limit::shaderc_limit_max_vertex_image_uniforms, 0},
      Limit{shaderc_limit::shaderc_limit_max_tess_control_image_uniforms, 0},
      Limit{shaderc_limit::shaderc_limit_max_tess_evaluation_image_uniforms, 0},
      Limit{shaderc_limit::shaderc_limit_max_geometry_image_uniforms, 0},
      Limit{shaderc_limit::shaderc_limit_max_fragment_image_uniforms, 8},
      Limit{shaderc_limit::shaderc_limit_max_combined_image_uniforms, 8},
      Limit{shaderc_limit::shaderc_limit_max_geometry_texture_image_units, 16},
      Limit{shaderc_limit::shaderc_limit_max_geometry_output_vertices, 256},
      Limit{shaderc_limit::shaderc_limit_max_geometry_total_output_components,
            1024},
      Limit{shaderc_limit::shaderc_limit_max_geometry_uniform_components, 512},
      Limit{shaderc_limit::shaderc_limit_max_geometry_varying_components, 60},
      Limit{shaderc_limit::shaderc_limit_max_tess_control_input_components,
            128},
      Limit{shaderc_limit::shaderc_limit_max_tess_control_output_components,
            128},
      Limit{shaderc_limit::shaderc_limit_max_tess_control_texture_image_units,
            16},
      Limit{shaderc_limit::shaderc_limit_max_tess_control_uniform_components,
            1024},
      Limit{
          shaderc_limit::shaderc_limit_max_tess_control_total_output_components,
          4096},
      Limit{shaderc_limit::shaderc_limit_max_tess_evaluation_input_components,
            128},
      Limit{shaderc_limit::shaderc_limit_max_tess_evaluation_output_components,
            128},
      Limit{
          shaderc_limit::shaderc_limit_max_tess_evaluation_texture_image_units,
          16},
      Limit{shaderc_limit::shaderc_limit_max_tess_evaluation_uniform_components,
            1024},
      Limit{shaderc_limit::shaderc_limit_max_tess_patch_components, 120},
      Limit{shaderc_limit::shaderc_limit_max_patch_vertices, 32},
      Limit{shaderc_limit::shaderc_limit_max_tess_gen_level, 64},
      Limit{shaderc_limit::shaderc_limit_max_viewports, 16},
      Limit{shaderc_limit::shaderc_limit_max_vertex_atomic_counters, 0},
      Limit{shaderc_limit::shaderc_limit_max_tess_control_atomic_counters, 0},
      Limit{shaderc_limit::shaderc_limit_max_tess_evaluation_atomic_counters,
            0},
      Limit{shaderc_limit::shaderc_limit_max_geometry_atomic_counters, 0},
      Limit{shaderc_limit::shaderc_limit_max_fragment_atomic_counters, 8},
      Limit{shaderc_limit::shaderc_limit_max_combined_atomic_counters, 8},
      Limit{shaderc_limit::shaderc_limit_max_atomic_counter_bindings, 1},
      Limit{shaderc_limit::shaderc_limit_max_vertex_atomic_counter_buffers, 0},
      Limit{
          shaderc_limit::shaderc_limit_max_tess_control_atomic_counter_buffers,
          0},
      Limit{shaderc_limit::
                shaderc_limit_max_tess_evaluation_atomic_counter_buffers,
            0},
      Limit{shaderc_limit::shaderc_limit_max_geometry_atomic_counter_buffers,
            0},
      Limit{shaderc_limit::shaderc_limit_max_fragment_atomic_counter_buffers,
            0},
      Limit{shaderc_limit::shaderc_limit_max_combined_atomic_counter_buffers,
            1},
      Limit{shaderc_limit::shaderc_limit_max_atomic_counter_buffer_size, 32},
      Limit{shaderc_limit::shaderc_limit_max_transform_feedback_buffers, 4},
      Limit{shaderc_limit::
                shaderc_limit_max_transform_feedback_interleaved_components,
            64},
      Limit{shaderc_limit::shaderc_limit_max_cull_distances, 8},
      Limit{shaderc_limit::shaderc_limit_max_combined_clip_and_cull_distances,
            8},
      Limit{shaderc_limit::shaderc_limit_max_samples, 4},
  };
  for (auto& [limit, value] : limits) {
    compiler_opts.SetLimit(limit, value);
  }
}

static void SetBindingBaseOffset(shaderc::CompileOptions& options) {
  constexpr uint32_t kBindingBaseOffset = 64;
  static const shaderc_uniform_kind kUniformKinds[] = {
      shaderc_uniform_kind::shaderc_uniform_kind_sampler,
      shaderc_uniform_kind::shaderc_uniform_kind_texture,
      shaderc_uniform_kind::shaderc_uniform_kind_image,
      shaderc_uniform_kind::shaderc_uniform_kind_buffer,          // UBOs
      shaderc_uniform_kind::shaderc_uniform_kind_storage_buffer,  // SSBOs
  };

  for (size_t i = 0u; i < sizeof(kUniformKinds) / sizeof(shaderc_uniform_kind);
       i++) {
    options.SetBindingBaseForStage(
        shaderc_shader_kind::shaderc_fragment_shader,  //
        kUniformKinds[i],                              //
        kBindingBaseOffset                             //
    );
  }
}

//------------------------------------------------------------------------------
/// @brief      Wraps a shared includer so unique includers may be created to
///             satisfy the shaderc API. This is a simple proxy object and does
///             nothing.
///
class UniqueIncluder final : public shaderc::CompileOptions::IncluderInterface {
 public:
  static std::unique_ptr<UniqueIncluder> Make(
      std::shared_ptr<Includer> includer) {
    return std::unique_ptr<UniqueIncluder>(
        new UniqueIncluder(std::move(includer)));
  }

  // |shaderc::CompileOptions::IncluderInterface|
  ~UniqueIncluder() = default;

  // |shaderc::CompileOptions::IncluderInterface|
  shaderc_include_result* GetInclude(const char* requested_source,
                                     shaderc_include_type type,
                                     const char* requesting_source,
                                     size_t include_depth) override {
    return includer_->GetInclude(requested_source,   //
                                 type,               //
                                 requesting_source,  //
                                 include_depth       //
    );
  }

  // |shaderc::CompileOptions::IncluderInterface|
  void ReleaseInclude(shaderc_include_result* data) override {
    return includer_->ReleaseInclude(data);
  }

 private:
  std::shared_ptr<Includer> includer_;

  explicit UniqueIncluder(std::shared_ptr<Includer> includer)
      : includer_(std::move(includer)) {
    FML_CHECK(includer_);
  }

  UniqueIncluder(const UniqueIncluder&) = delete;

  UniqueIncluder& operator=(const UniqueIncluder&) = delete;
};

shaderc::CompileOptions SPIRVCompilerOptions::BuildShadercOptions() const {
  shaderc::CompileOptions options;

  SetDefaultLimitations(options);
  SetBindingBaseOffset(options);

  options.SetAutoBindUniforms(true);
  options.SetAutoMapLocations(true);

  options.SetOptimizationLevel(optimization_level);

  if (generate_debug_info) {
    options.SetGenerateDebugInfo();
  }

  if (source_langauge.has_value()) {
    options.SetSourceLanguage(source_langauge.value());
  }

  if (source_profile.has_value()) {
    options.SetForcedVersionProfile(source_profile->version,
                                    source_profile->profile);
  }

  if (target.has_value()) {
    options.SetTargetEnvironment(target->env, target->version);
    options.SetTargetSpirv(target->spirv_version);
  }

  for (const auto& macro : macro_definitions) {
    options.AddMacroDefinition(macro);
  }

  if (includer) {
    options.SetIncluder(UniqueIncluder::Make(includer));
  }

  options.SetVulkanRulesRelaxed(relaxed_vulkan_rules);

  return options;
}

}  // namespace compiler
}  // namespace impeller
