// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// FLUTTER_NOLINT: https://github.com/flutter/flutter/issues/105732

#include "impeller/compiler/reflector.h"

#include <atomic>
#include <optional>
#include <set>
#include <sstream>

#include "flutter/fml/logging.h"
#include "fml/backtrace.h"
#include "impeller/base/strings.h"
#include "impeller/base/validation.h"
#include "impeller/compiler/code_gen_template.h"
#include "impeller/compiler/shader_bundle_data.h"
#include "impeller/compiler/types.h"
#include "impeller/compiler/uniform_sorter.h"
#include "impeller/compiler/utilities.h"
#include "impeller/core/runtime_types.h"
#include "impeller/geometry/half.h"
#include "impeller/geometry/matrix.h"
#include "impeller/geometry/scalar.h"
#include "impeller/runtime_stage/runtime_stage.h"
#include "spirv_common.hpp"

namespace impeller {
namespace compiler {

static std::string ExecutionModelToString(spv::ExecutionModel model) {
  switch (model) {
    case spv::ExecutionModel::ExecutionModelVertex:
      return "vertex";
    case spv::ExecutionModel::ExecutionModelFragment:
      return "fragment";
    case spv::ExecutionModel::ExecutionModelGLCompute:
      return "compute";
    default:
      return "unsupported";
  }
}

static std::string StringToShaderStage(const std::string& str) {
  if (str == "vertex") {
    return "ShaderStage::kVertex";
  }

  if (str == "fragment") {
    return "ShaderStage::kFragment";
  }

  if (str == "compute") {
    return "ShaderStage::kCompute";
  }

  return "ShaderStage::kUnknown";
}

Reflector::Reflector(Options options,
                     const std::shared_ptr<const spirv_cross::ParsedIR>& ir,
                     const std::shared_ptr<fml::Mapping>& shader_data,
                     const CompilerBackend& compiler)
    : options_(std::move(options)),
      ir_(ir),
      shader_data_(shader_data),
      compiler_(compiler) {
  if (!ir_ || !compiler_) {
    return;
  }

  if (auto template_arguments = GenerateTemplateArguments();
      template_arguments.has_value()) {
    template_arguments_ =
        std::make_unique<nlohmann::json>(std::move(template_arguments.value()));
  } else {
    return;
  }

  reflection_header_ = GenerateReflectionHeader();
  if (!reflection_header_) {
    return;
  }

  reflection_cc_ = GenerateReflectionCC();
  if (!reflection_cc_) {
    return;
  }

  runtime_stage_shader_ = GenerateRuntimeStageData();

  shader_bundle_data_ = GenerateShaderBundleData();
  if (!shader_bundle_data_) {
    return;
  }

  is_valid_ = true;
}

Reflector::~Reflector() = default;

bool Reflector::IsValid() const {
  return is_valid_;
}

std::shared_ptr<fml::Mapping> Reflector::GetReflectionJSON() const {
  if (!is_valid_) {
    return nullptr;
  }

  auto json_string =
      std::make_shared<std::string>(template_arguments_->dump(2u));

  return std::make_shared<fml::NonOwnedMapping>(
      reinterpret_cast<const uint8_t*>(json_string->data()),
      json_string->size(), [json_string](auto, auto) {});
}

std::shared_ptr<fml::Mapping> Reflector::GetReflectionHeader() const {
  return reflection_header_;
}

std::shared_ptr<fml::Mapping> Reflector::GetReflectionCC() const {
  return reflection_cc_;
}

std::shared_ptr<RuntimeStageData::Shader> Reflector::GetRuntimeStageShaderData()
    const {
  return runtime_stage_shader_;
}

std::shared_ptr<ShaderBundleData> Reflector::GetShaderBundleData() const {
  return shader_bundle_data_;
}

std::optional<nlohmann::json> Reflector::GenerateTemplateArguments() const {
  nlohmann::json root;

  const auto& entrypoints = compiler_->get_entry_points_and_stages();
  if (entrypoints.size() != 1) {
    VALIDATION_LOG << "Incorrect number of entrypoints in the shader. Found "
                   << entrypoints.size() << " but expected 1.";
    return std::nullopt;
  }

  auto execution_model = entrypoints.front().execution_model;
  {
    root["entrypoint"] = options_.entry_point_name;
    root["shader_name"] = options_.shader_name;
    root["shader_stage"] = ExecutionModelToString(execution_model);
    root["header_file_name"] = options_.header_file_name;
  }

  const auto shader_resources = compiler_->get_shader_resources();

  // Subpass Inputs.
  {
    auto& subpass_inputs = root["subpass_inputs"] = nlohmann::json::array_t{};
    if (auto subpass_inputs_json =
            ReflectResources(shader_resources.subpass_inputs);
        subpass_inputs_json.has_value()) {
      for (auto subpass_input : subpass_inputs_json.value()) {
        subpass_input["descriptor_type"] = "DescriptorType::kInputAttachment";
        subpass_inputs.emplace_back(std::move(subpass_input));
      }
    } else {
      return std::nullopt;
    }
  }

  // Uniform and storage buffers.
  {
    auto& buffers = root["buffers"] = nlohmann::json::array_t{};
    if (auto uniform_buffers_json =
            ReflectResources(shader_resources.uniform_buffers);
        uniform_buffers_json.has_value()) {
      for (auto uniform_buffer : uniform_buffers_json.value()) {
        uniform_buffer["descriptor_type"] = "DescriptorType::kUniformBuffer";
        buffers.emplace_back(std::move(uniform_buffer));
      }
    } else {
      return std::nullopt;
    }
    if (auto storage_buffers_json =
            ReflectResources(shader_resources.storage_buffers);
        storage_buffers_json.has_value()) {
      for (auto uniform_buffer : storage_buffers_json.value()) {
        uniform_buffer["descriptor_type"] = "DescriptorType::kStorageBuffer";
        buffers.emplace_back(std::move(uniform_buffer));
      }
    } else {
      return std::nullopt;
    }
  }

  {
    auto& stage_inputs = root["stage_inputs"] = nlohmann::json::array_t{};
    if (auto stage_inputs_json = ReflectResources(
            shader_resources.stage_inputs,
            /*compute_offsets=*/execution_model == spv::ExecutionModelVertex);
        stage_inputs_json.has_value()) {
      stage_inputs = std::move(stage_inputs_json.value());
    } else {
      return std::nullopt;
    }
  }

  {
    auto combined_sampled_images =
        ReflectResources(shader_resources.sampled_images);
    auto images = ReflectResources(shader_resources.separate_images);
    auto samplers = ReflectResources(shader_resources.separate_samplers);
    if (!combined_sampled_images.has_value() || !images.has_value() ||
        !samplers.has_value()) {
      return std::nullopt;
    }
    auto& sampled_images = root["sampled_images"] = nlohmann::json::array_t{};
    for (auto value : combined_sampled_images.value()) {
      value["descriptor_type"] = "DescriptorType::kSampledImage";
      sampled_images.emplace_back(std::move(value));
    }
    for (auto value : images.value()) {
      value["descriptor_type"] = "DescriptorType::kImage";
      sampled_images.emplace_back(std::move(value));
    }
    for (auto value : samplers.value()) {
      value["descriptor_type"] = "DescriptorType::kSampledSampler";
      sampled_images.emplace_back(std::move(value));
    }
  }

  if (auto stage_outputs = ReflectResources(shader_resources.stage_outputs);
      stage_outputs.has_value()) {
    root["stage_outputs"] = std::move(stage_outputs.value());
  } else {
    return std::nullopt;
  }

  {
    auto& struct_definitions = root["struct_definitions"] =
        nlohmann::json::array_t{};
    if (entrypoints.front().execution_model ==
            spv::ExecutionModel::ExecutionModelVertex &&
        !shader_resources.stage_inputs.empty()) {
      if (auto struc =
              ReflectPerVertexStructDefinition(shader_resources.stage_inputs);
          struc.has_value()) {
        struct_definitions.emplace_back(EmitStructDefinition(struc.value()));
      } else {
        // If there are stage inputs, it is an error to not generate a per
        // vertex data struct for a vertex like shader stage.
        return std::nullopt;
      }
    }

    std::set<spirv_cross::ID> known_structs;
    ir_->for_each_typed_id<spirv_cross::SPIRType>(
        [&](uint32_t, const spirv_cross::SPIRType& type) {
          if (type.basetype != spirv_cross::SPIRType::BaseType::Struct) {
            return;
          }
          // Skip structs that do not have layout offset decorations.
          // These structs are used internally within the shader and are not
          // part of the shader's interface.
          for (size_t i = 0; i < type.member_types.size(); i++) {
            if (!compiler_->has_member_decoration(type.self, i,
                                                  spv::DecorationOffset)) {
              return;
            }
          }
          if (known_structs.find(type.self) != known_structs.end()) {
            // Iterating over types this way leads to duplicates which may cause
            // duplicate struct definitions.
            return;
          }
          known_structs.insert(type.self);
          if (auto struc = ReflectStructDefinition(type.self);
              struc.has_value()) {
            struct_definitions.emplace_back(
                EmitStructDefinition(struc.value()));
          }
        });
  }

  root["bind_prototypes"] =
      EmitBindPrototypes(shader_resources, execution_model);

  return root;
}

std::shared_ptr<fml::Mapping> Reflector::GenerateReflectionHeader() const {
  return InflateTemplate(kReflectionHeaderTemplate);
}

std::shared_ptr<fml::Mapping> Reflector::GenerateReflectionCC() const {
  return InflateTemplate(kReflectionCCTemplate);
}

static std::optional<RuntimeStageBackend> GetRuntimeStageBackend(
    TargetPlatform target_platform) {
  switch (target_platform) {
    case TargetPlatform::kUnknown:
    case TargetPlatform::kMetalDesktop:
    case TargetPlatform::kMetalIOS:
    case TargetPlatform::kOpenGLES:
    case TargetPlatform::kOpenGLDesktop:
    case TargetPlatform::kVulkan:
      return std::nullopt;
    case TargetPlatform::kRuntimeStageMetal:
      return RuntimeStageBackend::kMetal;
    case TargetPlatform::kRuntimeStageGLES:
      return RuntimeStageBackend::kOpenGLES;
    case TargetPlatform::kRuntimeStageGLES3:
      return RuntimeStageBackend::kOpenGLES3;
    case TargetPlatform::kRuntimeStageVulkan:
      return RuntimeStageBackend::kVulkan;
    case TargetPlatform::kSkSL:
      return RuntimeStageBackend::kSkSL;
  }
  FML_UNREACHABLE();
}

std::shared_ptr<RuntimeStageData::Shader> Reflector::GenerateRuntimeStageData()
    const {
  auto backend = GetRuntimeStageBackend(options_.target_platform);
  if (!backend.has_value()) {
    return nullptr;
  }

  const auto& entrypoints = compiler_->get_entry_points_and_stages();
  if (entrypoints.size() != 1u) {
    VALIDATION_LOG << "Single entrypoint not found.";
    return nullptr;
  }
  auto data = std::make_unique<RuntimeStageData::Shader>();
  data->entrypoint = options_.entry_point_name;
  data->stage = entrypoints.front().execution_model;
  data->shader = shader_data_;
  data->backend = backend.value();

  // Sort the IR so that the uniforms are in declaration order.
  std::vector<spirv_cross::ID> uniforms =
      SortUniforms(ir_.get(), compiler_.GetCompiler());
  for (auto& sorted_id : uniforms) {
    auto var = ir_->ids[sorted_id].get<spirv_cross::SPIRVariable>();
    const auto spir_type = compiler_->get_type(var.basetype);
    UniformDescription uniform_description;
    uniform_description.name = compiler_->get_name(var.self);
    uniform_description.location = compiler_->get_decoration(
        var.self, spv::Decoration::DecorationLocation);
    uniform_description.binding =
        compiler_->get_decoration(var.self, spv::Decoration::DecorationBinding);
    uniform_description.type = spir_type.basetype;
    uniform_description.rows = spir_type.vecsize;
    uniform_description.columns = spir_type.columns;
    uniform_description.bit_width = spir_type.width;
    uniform_description.array_elements = GetArrayElements(spir_type);
    FML_CHECK(data->backend != RuntimeStageBackend::kVulkan ||
              spir_type.basetype ==
                  spirv_cross::SPIRType::BaseType::SampledImage)
        << "Vulkan runtime effect had unexpected uniforms outside of the "
           "uniform buffer object.";
    data->uniforms.emplace_back(std::move(uniform_description));
  }

  const auto ubos = compiler_->get_shader_resources().uniform_buffers;
  if (data->backend == RuntimeStageBackend::kVulkan && !ubos.empty()) {
    if (ubos.size() != 1 && ubos[0].name != RuntimeStage::kVulkanUBOName) {
      VALIDATION_LOG << "Expected a single UBO resource named "
                        "'"
                     << RuntimeStage::kVulkanUBOName
                     << "' "
                        "for Vulkan runtime stage backend.";
      return nullptr;
    }

    const auto& ubo = ubos[0];

    size_t binding =
        compiler_->get_decoration(ubo.id, spv::Decoration::DecorationBinding);
    auto members = ReadStructMembers(ubo.type_id);
    std::vector<uint8_t> struct_layout;
    size_t float_count = 0;

    for (size_t i = 0; i < members.size(); i += 1) {
      const auto& member = members[i];
      std::vector<int> bytes;
      switch (member.underlying_type) {
        case StructMember::UnderlyingType::kPadding: {
          size_t padding_count =
              (member.size + sizeof(float) - 1) / sizeof(float);
          while (padding_count > 0) {
            struct_layout.push_back(0);
            padding_count--;
          }
          break;
        }
        case StructMember::UnderlyingType::kFloat: {
          if (member.array_elements > 1) {
            // For each array element member, insert 1 layout property per byte
            // and 0 layout property per byte of padding
            for (auto i = 0; i < member.array_elements; i++) {
              for (auto j = 0u; j < member.size / sizeof(float); j++) {
                struct_layout.push_back(1);
              }
              for (auto j = 0u; j < member.element_padding / sizeof(float);
                   j++) {
                struct_layout.push_back(0);
              }
            }
          } else {
            size_t member_float_count = member.byte_length / sizeof(float);
            float_count += member_float_count;
            while (member_float_count > 0) {
              struct_layout.push_back(1);
              member_float_count--;
            }
          }
          break;
        }
        case StructMember::UnderlyingType::kOther:
          VALIDATION_LOG << "Non-floating-type struct member " << member.name
                         << " is not supported.";
          return nullptr;
      }
    }
    data->uniforms.emplace_back(UniformDescription{
        .name = ubo.name,
        .location = binding,
        .binding = binding,
        .type = spirv_cross::SPIRType::Struct,
        .struct_layout = std::move(struct_layout),
        .struct_float_count = float_count,
    });
  }

  // We only need to worry about storing vertex attributes.
  if (entrypoints.front().execution_model == spv::ExecutionModelVertex) {
    const auto inputs = compiler_->get_shader_resources().stage_inputs;
    auto input_offsets = ComputeOffsets(inputs);
    for (const auto& input : inputs) {
      std::optional<size_t> offset = GetOffset(input.id, input_offsets);

      const auto type = compiler_->get_type(input.type_id);

      InputDescription input_description;
      input_description.name = input.name;
      input_description.location = compiler_->get_decoration(
          input.id, spv::Decoration::DecorationLocation);
      input_description.set = compiler_->get_decoration(
          input.id, spv::Decoration::DecorationDescriptorSet);
      input_description.binding = compiler_->get_decoration(
          input.id, spv::Decoration::DecorationBinding);
      input_description.type = type.basetype;
      input_description.bit_width = type.width;
      input_description.vec_size = type.vecsize;
      input_description.columns = type.columns;
      input_description.offset = offset.value_or(0u);
      data->inputs.emplace_back(std::move(input_description));
    }
  }

  return data;
}

std::shared_ptr<ShaderBundleData> Reflector::GenerateShaderBundleData() const {
  const auto& entrypoints = compiler_->get_entry_points_and_stages();
  if (entrypoints.size() != 1u) {
    VALIDATION_LOG << "Single entrypoint not found.";
    return nullptr;
  }
  auto data = std::make_shared<ShaderBundleData>(
      options_.entry_point_name,            //
      entrypoints.front().execution_model,  //
      options_.target_platform              //
  );
  data->SetShaderData(shader_data_);

  const auto uniforms = compiler_->get_shader_resources().uniform_buffers;
  for (const auto& uniform : uniforms) {
    ShaderBundleData::ShaderUniformStruct uniform_struct;
    uniform_struct.name = uniform.name;
    uniform_struct.ext_res_0 = compiler_.GetExtendedMSLResourceBinding(
        CompilerBackend::ExtendedResourceIndex::kPrimary, uniform.id);
    uniform_struct.set = compiler_->get_decoration(
        uniform.id, spv::Decoration::DecorationDescriptorSet);
    uniform_struct.binding = compiler_->get_decoration(
        uniform.id, spv::Decoration::DecorationBinding);

    const auto type = compiler_->get_type(uniform.type_id);
    if (type.basetype != spirv_cross::SPIRType::BaseType::Struct) {
      std::cerr << "Error: Uniform \"" << uniform.name
                << "\" is not a struct. All Flutter GPU shader uniforms must "
                   "be structs."
                << std::endl;
      return nullptr;
    }

    size_t size_in_bytes = 0;
    for (const auto& struct_member : ReadStructMembers(uniform.type_id)) {
      size_in_bytes += struct_member.byte_length;
      if (StringStartsWith(struct_member.name, "_PADDING_")) {
        continue;
      }
      ShaderBundleData::ShaderUniformStructField uniform_struct_field;
      uniform_struct_field.name = struct_member.name;
      uniform_struct_field.type = struct_member.base_type;
      uniform_struct_field.offset_in_bytes = struct_member.offset;
      uniform_struct_field.element_size_in_bytes = struct_member.size;
      uniform_struct_field.total_size_in_bytes = struct_member.byte_length;
      uniform_struct_field.array_elements = struct_member.array_elements;
      uniform_struct.fields.push_back(uniform_struct_field);
    }
    uniform_struct.size_in_bytes = size_in_bytes;

    data->AddUniformStruct(uniform_struct);
  }

  const auto sampled_images = compiler_->get_shader_resources().sampled_images;
  for (const auto& image : sampled_images) {
    ShaderBundleData::ShaderUniformTexture uniform_texture;
    uniform_texture.name = image.name;
    uniform_texture.ext_res_0 = compiler_.GetExtendedMSLResourceBinding(
        CompilerBackend::ExtendedResourceIndex::kPrimary, image.id);
    uniform_texture.set = compiler_->get_decoration(
        image.id, spv::Decoration::DecorationDescriptorSet);
    uniform_texture.binding =
        compiler_->get_decoration(image.id, spv::Decoration::DecorationBinding);
    data->AddUniformTexture(uniform_texture);
  }

  // We only need to worry about storing vertex attributes.
  if (entrypoints.front().execution_model == spv::ExecutionModelVertex) {
    const auto inputs = compiler_->get_shader_resources().stage_inputs;
    auto input_offsets = ComputeOffsets(inputs);
    for (const auto& input : inputs) {
      std::optional<size_t> offset = GetOffset(input.id, input_offsets);

      const auto type = compiler_->get_type(input.type_id);

      InputDescription input_description;
      input_description.name = input.name;
      input_description.location = compiler_->get_decoration(
          input.id, spv::Decoration::DecorationLocation);
      input_description.set = compiler_->get_decoration(
          input.id, spv::Decoration::DecorationDescriptorSet);
      input_description.binding = compiler_->get_decoration(
          input.id, spv::Decoration::DecorationBinding);
      input_description.type = type.basetype;
      input_description.bit_width = type.width;
      input_description.vec_size = type.vecsize;
      input_description.columns = type.columns;
      input_description.offset = offset.value_or(0u);
      data->AddInputDescription(std::move(input_description));
    }
  }

  return data;
}

std::optional<uint32_t> Reflector::GetArrayElements(
    const spirv_cross::SPIRType& type) const {
  if (type.array.empty()) {
    return std::nullopt;
  }
  FML_CHECK(type.array.size() == 1)
      << "Multi-dimensional arrays are not supported.";
  FML_CHECK(type.array_size_literal.front())
      << "Must use a literal for array sizes.";
  return type.array.front();
}

static std::string ToString(CompilerBackend::Type type) {
  switch (type) {
    case CompilerBackend::Type::kMSL:
      return "Metal Shading Language";
    case CompilerBackend::Type::kGLSL:
      return "OpenGL Shading Language";
    case CompilerBackend::Type::kGLSLVulkan:
      return "OpenGL Shading Language (Relaxed Vulkan Semantics)";
    case CompilerBackend::Type::kSkSL:
      return "SkSL Shading Language";
  }
  FML_UNREACHABLE();
}

std::shared_ptr<fml::Mapping> Reflector::InflateTemplate(
    std::string_view tmpl) const {
  inja::Environment env;
  env.set_trim_blocks(true);
  env.set_lstrip_blocks(true);

  env.add_callback("camel_case", 1u, [](inja::Arguments& args) {
    return ToCamelCase(args.at(0u)->get<std::string>());
  });

  env.add_callback("to_shader_stage", 1u, [](inja::Arguments& args) {
    return StringToShaderStage(args.at(0u)->get<std::string>());
  });

  env.add_callback("get_generator_name", 0u,
                   [type = compiler_.GetType()](inja::Arguments& args) {
                     return ToString(type);
                   });

  auto inflated_template =
      std::make_shared<std::string>(env.render(tmpl, *template_arguments_));

  return std::make_shared<fml::NonOwnedMapping>(
      reinterpret_cast<const uint8_t*>(inflated_template->data()),
      inflated_template->size(), [inflated_template](auto, auto) {});
}

std::vector<size_t> Reflector::ComputeOffsets(
    const spirv_cross::SmallVector<spirv_cross::Resource>& resources) const {
  std::vector<size_t> offsets(resources.size(), 0);
  if (resources.size() == 0) {
    return offsets;
  }
  for (const auto& resource : resources) {
    const auto type = compiler_->get_type(resource.type_id);
    auto location = compiler_->get_decoration(
        resource.id, spv::Decoration::DecorationLocation);
    // Malformed shader, will be caught later on.
    if (location >= resources.size() || location < 0) {
      location = 0;
    }
    offsets[location] = (type.width * type.vecsize) / 8;
  }
  for (size_t i = 1; i < resources.size(); i++) {
    offsets[i] += offsets[i - 1];
  }
  for (size_t i = resources.size() - 1; i > 0; i--) {
    offsets[i] = offsets[i - 1];
  }
  offsets[0] = 0;

  return offsets;
}

std::optional<size_t> Reflector::GetOffset(
    spirv_cross::ID id,
    const std::vector<size_t>& offsets) const {
  uint32_t location =
      compiler_->get_decoration(id, spv::Decoration::DecorationLocation);
  if (location >= offsets.size()) {
    return std::nullopt;
  }
  return offsets[location];
}

std::optional<nlohmann::json::object_t> Reflector::ReflectResource(
    const spirv_cross::Resource& resource,
    std::optional<size_t> offset) const {
  nlohmann::json::object_t result;

  result["name"] = resource.name;
  result["descriptor_set"] = compiler_->get_decoration(
      resource.id, spv::Decoration::DecorationDescriptorSet);
  result["binding"] = compiler_->get_decoration(
      resource.id, spv::Decoration::DecorationBinding);
  result["set"] = compiler_->get_decoration(
      resource.id, spv::Decoration::DecorationDescriptorSet);
  result["location"] = compiler_->get_decoration(
      resource.id, spv::Decoration::DecorationLocation);
  result["index"] =
      compiler_->get_decoration(resource.id, spv::Decoration::DecorationIndex);
  result["ext_res_0"] = compiler_.GetExtendedMSLResourceBinding(
      CompilerBackend::ExtendedResourceIndex::kPrimary, resource.id);
  result["ext_res_1"] = compiler_.GetExtendedMSLResourceBinding(
      CompilerBackend::ExtendedResourceIndex::kSecondary, resource.id);
  result["relaxed_precision"] =
      compiler_->get_decoration(
          resource.id, spv::Decoration::DecorationRelaxedPrecision) == 1;
  result["offset"] = offset.value_or(0u);
  auto type = ReflectType(resource.type_id);
  if (!type.has_value()) {
    return std::nullopt;
  }
  result["type"] = std::move(type.value());
  return result;
}

std::optional<nlohmann::json::object_t> Reflector::ReflectType(
    const spirv_cross::TypeID& type_id) const {
  nlohmann::json::object_t result;

  const auto type = compiler_->get_type(type_id);

  result["type_name"] = StructMember::BaseTypeToString(type.basetype);
  result["bit_width"] = type.width;
  result["vec_size"] = type.vecsize;
  result["columns"] = type.columns;
  auto& members = result["members"] = nlohmann::json::array_t{};
  if (type.basetype == spirv_cross::SPIRType::BaseType::Struct) {
    for (const auto& struct_member : ReadStructMembers(type_id)) {
      auto member = nlohmann::json::object_t{};
      member["name"] = struct_member.name;
      member["type"] = struct_member.type;
      member["base_type"] =
          StructMember::BaseTypeToString(struct_member.base_type);
      member["offset"] = struct_member.offset;
      member["size"] = struct_member.size;
      member["byte_length"] = struct_member.byte_length;
      if (struct_member.array_elements.has_value()) {
        member["array_elements"] = struct_member.array_elements.value();
      } else {
        member["array_elements"] = "std::nullopt";
      }
      members.emplace_back(std::move(member));
    }
  }

  return result;
}

std::optional<nlohmann::json::array_t> Reflector::ReflectResources(
    const spirv_cross::SmallVector<spirv_cross::Resource>& resources,
    bool compute_offsets) const {
  nlohmann::json::array_t result;
  result.reserve(resources.size());
  std::vector<size_t> offsets;
  if (compute_offsets) {
    offsets = ComputeOffsets(resources);
  }
  for (const auto& resource : resources) {
    std::optional<size_t> maybe_offset = std::nullopt;
    if (compute_offsets) {
      maybe_offset = GetOffset(resource.id, offsets);
    }
    if (auto reflected = ReflectResource(resource, maybe_offset);
        reflected.has_value()) {
      result.emplace_back(std::move(reflected.value()));
    } else {
      return std::nullopt;
    }
  }
  return result;
}

static std::string TypeNameWithPaddingOfSize(size_t size) {
  std::stringstream stream;
  stream << "Padding<" << size << ">";
  return stream.str();
}

struct KnownType {
  std::string name;
  size_t byte_size = 0;
};

static std::optional<KnownType> ReadKnownScalarType(
    spirv_cross::SPIRType::BaseType type) {
  switch (type) {
    case spirv_cross::SPIRType::BaseType::Boolean:
      return KnownType{
          .name = "bool",
          .byte_size = sizeof(bool),
      };
    case spirv_cross::SPIRType::BaseType::Float:
      return KnownType{
          .name = "Scalar",
          .byte_size = sizeof(Scalar),
      };
    case spirv_cross::SPIRType::BaseType::Half:
      return KnownType{
          .name = "Half",
          .byte_size = sizeof(Half),
      };
    case spirv_cross::SPIRType::BaseType::UInt:
      return KnownType{
          .name = "uint32_t",
          .byte_size = sizeof(uint32_t),
      };
    case spirv_cross::SPIRType::BaseType::Int:
      return KnownType{
          .name = "int32_t",
          .byte_size = sizeof(int32_t),
      };
    default:
      break;
  }
  return std::nullopt;
}

//------------------------------------------------------------------------------
/// @brief      Get the reflected struct size. In the vast majority of the
///             cases, this is the same as the declared struct size as given by
///             the compiler. But, additional padding may need to be introduced
///             after the end of the struct to keep in line with the alignment
///             requirement of the individual struct members. This method
///             figures out the actual size of the reflected struct that can be
///             referenced in native code.
///
/// @param[in]  members  The members
///
/// @return     The reflected structure size.
///
static size_t GetReflectedStructSize(const std::vector<StructMember>& members) {
  auto struct_size = 0u;
  for (const auto& member : members) {
    struct_size += member.byte_length;
  }
  return struct_size;
}

std::vector<StructMember> Reflector::ReadStructMembers(
    const spirv_cross::TypeID& type_id) const {
  const auto& struct_type = compiler_->get_type(type_id);
  FML_CHECK(struct_type.basetype == spirv_cross::SPIRType::BaseType::Struct);

  std::vector<StructMember> result;

  size_t current_byte_offset = 0;
  size_t max_member_alignment = 0;

  for (size_t i = 0; i < struct_type.member_types.size(); i++) {
    const auto& member = compiler_->get_type(struct_type.member_types[i]);
    const auto struct_member_offset =
        compiler_->type_struct_member_offset(struct_type, i);
    auto array_elements = GetArrayElements(member);

    if (struct_member_offset > current_byte_offset) {
      const auto alignment_pad = struct_member_offset - current_byte_offset;
      result.emplace_back(StructMember{
          TypeNameWithPaddingOfSize(alignment_pad),  // type
          spirv_cross::SPIRType::BaseType::Void,     // basetype
          SPrintF("_PADDING_%s_",
                  GetMemberNameAtIndex(struct_type, i).c_str()),  // name
          current_byte_offset,                                    // offset
          alignment_pad,                                          // size
          alignment_pad,                                          // byte_length
          std::nullopt,  // array_elements
          0,             // element_padding
      });
      current_byte_offset += alignment_pad;
    }

    max_member_alignment =
        std::max<size_t>(max_member_alignment,
                         (member.width / 8) * member.columns * member.vecsize);

    FML_CHECK(current_byte_offset == struct_member_offset);

    // A user defined struct.
    if (member.basetype == spirv_cross::SPIRType::BaseType::Struct) {
      const size_t size =
          GetReflectedStructSize(ReadStructMembers(member.self));
      uint32_t stride = GetArrayStride<0>(struct_type, member, i);
      if (stride == 0) {
        stride = size;
      }
      uint32_t element_padding = stride - size;
      result.emplace_back(StructMember{
          compiler_->get_name(member.self),      // type
          member.basetype,                       // basetype
          GetMemberNameAtIndex(struct_type, i),  // name
          struct_member_offset,                  // offset
          size,                                  // size
          stride * array_elements.value_or(1),   // byte_length
          array_elements,                        // array_elements
          element_padding,                       // element_padding
      });
      current_byte_offset += stride * array_elements.value_or(1);
      continue;
    }

    // Tightly packed 4x4 Matrix is special cased as we know how to work with
    // those.
    if (member.basetype == spirv_cross::SPIRType::BaseType::Float &&  //
        member.width == sizeof(Scalar) * 8 &&                         //
        member.columns == 4 &&                                        //
        member.vecsize == 4                                           //
    ) {
      uint32_t stride = GetArrayStride<sizeof(Matrix)>(struct_type, member, i);
      uint32_t element_padding = stride - sizeof(Matrix);
      result.emplace_back(StructMember{
          "Matrix",                              // type
          member.basetype,                       // basetype
          GetMemberNameAtIndex(struct_type, i),  // name
          struct_member_offset,                  // offset
          sizeof(Matrix),                        // size
          stride * array_elements.value_or(1),   // byte_length
          array_elements,                        // array_elements
          element_padding,                       // element_padding
      });
      current_byte_offset += stride * array_elements.value_or(1);
      continue;
    }

    // Tightly packed UintPoint32 (uvec2)
    if (member.basetype == spirv_cross::SPIRType::BaseType::UInt &&  //
        member.width == sizeof(uint32_t) * 8 &&                      //
        member.columns == 1 &&                                       //
        member.vecsize == 2                                          //
    ) {
      uint32_t stride =
          GetArrayStride<sizeof(UintPoint32)>(struct_type, member, i);
      uint32_t element_padding = stride - sizeof(UintPoint32);
      result.emplace_back(StructMember{
          "UintPoint32",                         // type
          member.basetype,                       // basetype
          GetMemberNameAtIndex(struct_type, i),  // name
          struct_member_offset,                  // offset
          sizeof(UintPoint32),                   // size
          stride * array_elements.value_or(1),   // byte_length
          array_elements,                        // array_elements
          element_padding,                       // element_padding
      });
      current_byte_offset += stride * array_elements.value_or(1);
      continue;
    }

    // Tightly packed UintPoint32 (ivec2)
    if (member.basetype == spirv_cross::SPIRType::BaseType::Int &&  //
        member.width == sizeof(int32_t) * 8 &&                      //
        member.columns == 1 &&                                      //
        member.vecsize == 2                                         //
    ) {
      uint32_t stride =
          GetArrayStride<sizeof(IPoint32)>(struct_type, member, i);
      uint32_t element_padding = stride - sizeof(IPoint32);
      result.emplace_back(StructMember{
          "IPoint32",                            // type
          member.basetype,                       // basetype
          GetMemberNameAtIndex(struct_type, i),  // name
          struct_member_offset,                  // offset
          sizeof(IPoint32),                      // size
          stride * array_elements.value_or(1),   // byte_length
          array_elements,                        // array_elements
          element_padding,                       // element_padding
      });
      current_byte_offset += stride * array_elements.value_or(1);
      continue;
    }

    // Tightly packed Point (vec2).
    if (member.basetype == spirv_cross::SPIRType::BaseType::Float &&  //
        member.width == sizeof(float) * 8 &&                          //
        member.columns == 1 &&                                        //
        member.vecsize == 2                                           //
    ) {
      uint32_t stride = GetArrayStride<sizeof(Point)>(struct_type, member, i);
      uint32_t element_padding = stride - sizeof(Point);
      result.emplace_back(StructMember{
          "Point",                               // type
          member.basetype,                       // basetype
          GetMemberNameAtIndex(struct_type, i),  // name
          struct_member_offset,                  // offset
          sizeof(Point),                         // size
          stride * array_elements.value_or(1),   // byte_length
          array_elements,                        // array_elements
          element_padding,                       // element_padding
      });
      current_byte_offset += stride * array_elements.value_or(1);
      continue;
    }

    // Tightly packed Vector3.
    if (member.basetype == spirv_cross::SPIRType::BaseType::Float &&  //
        member.width == sizeof(float) * 8 &&                          //
        member.columns == 1 &&                                        //
        member.vecsize == 3                                           //
    ) {
      uint32_t stride = GetArrayStride<sizeof(Vector3)>(struct_type, member, i);
      uint32_t element_padding = stride - sizeof(Vector3);
      result.emplace_back(StructMember{
          "Vector3",                             // type
          member.basetype,                       // basetype
          GetMemberNameAtIndex(struct_type, i),  // name
          struct_member_offset,                  // offset
          sizeof(Vector3),                       // size
          stride * array_elements.value_or(1),   // byte_length
          array_elements,                        // array_elements
          element_padding,                       // element_padding
      });
      current_byte_offset += stride * array_elements.value_or(1);
      continue;
    }

    // Tightly packed Vector4.
    if (member.basetype == spirv_cross::SPIRType::BaseType::Float &&  //
        member.width == sizeof(float) * 8 &&                          //
        member.columns == 1 &&                                        //
        member.vecsize == 4                                           //
    ) {
      uint32_t stride = GetArrayStride<sizeof(Vector4)>(struct_type, member, i);
      uint32_t element_padding = stride - sizeof(Vector4);
      result.emplace_back(StructMember{
          "Vector4",                             // type
          member.basetype,                       // basetype
          GetMemberNameAtIndex(struct_type, i),  // name
          struct_member_offset,                  // offset
          sizeof(Vector4),                       // size
          stride * array_elements.value_or(1),   // byte_length
          array_elements,                        // array_elements
          element_padding,                       // element_padding
      });
      current_byte_offset += stride * array_elements.value_or(1);
      continue;
    }

    // Tightly packed half Point (vec2).
    if (member.basetype == spirv_cross::SPIRType::BaseType::Half &&  //
        member.width == sizeof(Half) * 8 &&                          //
        member.columns == 1 &&                                       //
        member.vecsize == 2                                          //
    ) {
      uint32_t stride =
          GetArrayStride<sizeof(HalfVector2)>(struct_type, member, i);
      uint32_t element_padding = stride - sizeof(HalfVector2);
      result.emplace_back(StructMember{
          "HalfVector2",                         // type
          member.basetype,                       // basetype
          GetMemberNameAtIndex(struct_type, i),  // name
          struct_member_offset,                  // offset
          sizeof(HalfVector2),                   // size
          stride * array_elements.value_or(1),   // byte_length
          array_elements,                        // array_elements
          element_padding,                       // element_padding
      });
      current_byte_offset += stride * array_elements.value_or(1);
      continue;
    }

    // Tightly packed Half Float Vector3.
    if (member.basetype == spirv_cross::SPIRType::BaseType::Half &&  //
        member.width == sizeof(Half) * 8 &&                          //
        member.columns == 1 &&                                       //
        member.vecsize == 3                                          //
    ) {
      uint32_t stride =
          GetArrayStride<sizeof(HalfVector3)>(struct_type, member, i);
      uint32_t element_padding = stride - sizeof(HalfVector3);
      result.emplace_back(StructMember{
          "HalfVector3",                         // type
          member.basetype,                       // basetype
          GetMemberNameAtIndex(struct_type, i),  // name
          struct_member_offset,                  // offset
          sizeof(HalfVector3),                   // size
          stride * array_elements.value_or(1),   // byte_length
          array_elements,                        // array_elements
          element_padding,                       // element_padding
      });
      current_byte_offset += stride * array_elements.value_or(1);
      continue;
    }

    // Tightly packed Half Float Vector4.
    if (member.basetype == spirv_cross::SPIRType::BaseType::Half &&  //
        member.width == sizeof(Half) * 8 &&                          //
        member.columns == 1 &&                                       //
        member.vecsize == 4                                          //
    ) {
      uint32_t stride =
          GetArrayStride<sizeof(HalfVector4)>(struct_type, member, i);
      uint32_t element_padding = stride - sizeof(HalfVector4);
      result.emplace_back(StructMember{
          "HalfVector4",                         // type
          member.basetype,                       // basetype
          GetMemberNameAtIndex(struct_type, i),  // name
          struct_member_offset,                  // offset
          sizeof(HalfVector4),                   // size
          stride * array_elements.value_or(1),   // byte_length
          array_elements,                        // array_elements
          element_padding,                       // element_padding
      });
      current_byte_offset += stride * array_elements.value_or(1);
      continue;
    }

    // Other isolated scalars (like bool, int, float/Scalar, etc..).
    {
      auto maybe_known_type = ReadKnownScalarType(member.basetype);
      if (maybe_known_type.has_value() &&  //
          member.columns == 1 &&           //
          member.vecsize == 1              //
      ) {
        uint32_t stride = GetArrayStride<0>(struct_type, member, i);
        if (stride == 0) {
          stride = maybe_known_type.value().byte_size;
        }
        uint32_t element_padding = stride - maybe_known_type.value().byte_size;
        // Add the type directly.
        result.emplace_back(StructMember{
            maybe_known_type.value().name,         // type
            member.basetype,                       // basetype
            GetMemberNameAtIndex(struct_type, i),  // name
            struct_member_offset,                  // offset
            maybe_known_type.value().byte_size,    // size
            stride * array_elements.value_or(1),   // byte_length
            array_elements,                        // array_elements
            element_padding,                       // element_padding
        });
        current_byte_offset += stride * array_elements.value_or(1);
        continue;
      }
    }

    // Catch all for unknown types. Just add the necessary padding to the struct
    // and move on.
    {
      const size_t size = (member.width * member.columns * member.vecsize) / 8u;
      uint32_t stride = GetArrayStride<0>(struct_type, member, i);
      if (stride == 0) {
        stride = size;
      }
      auto element_padding = stride - size;
      result.emplace_back(StructMember{
          TypeNameWithPaddingOfSize(size),       // type
          member.basetype,                       // basetype
          GetMemberNameAtIndex(struct_type, i),  // name
          struct_member_offset,                  // offset
          size,                                  // size
          stride * array_elements.value_or(1),   // byte_length
          array_elements,                        // array_elements
          element_padding,                       // element_padding
      });
      current_byte_offset += stride * array_elements.value_or(1);
      continue;
    }
  }

  if (max_member_alignment > 0u) {
    const auto struct_length = current_byte_offset;
    {
      const auto excess = struct_length % max_member_alignment;
      if (excess != 0) {
        const auto padding = max_member_alignment - excess;
        result.emplace_back(StructMember{
            TypeNameWithPaddingOfSize(padding),     // type
            spirv_cross::SPIRType::BaseType::Void,  // basetype
            "_PADDING_",                            // name
            current_byte_offset,                    // offset
            padding,                                // size
            padding,                                // byte_length
            std::nullopt,                           // array_elements
            0,                                      // element_padding
        });
      }
    }
  }

  return result;
}

std::optional<Reflector::StructDefinition> Reflector::ReflectStructDefinition(
    const spirv_cross::TypeID& type_id) const {
  const auto& type = compiler_->get_type(type_id);
  if (type.basetype != spirv_cross::SPIRType::BaseType::Struct) {
    return std::nullopt;
  }

  const auto struct_name = compiler_->get_name(type_id);
  if (struct_name.find("_RESERVED_IDENTIFIER_") != std::string::npos) {
    return std::nullopt;
  }

  auto struct_members = ReadStructMembers(type_id);
  auto reflected_struct_size = GetReflectedStructSize(struct_members);

  StructDefinition struc;
  struc.name = struct_name;
  struc.byte_length = reflected_struct_size;
  struc.members = std::move(struct_members);
  return struc;
}

nlohmann::json::object_t Reflector::EmitStructDefinition(
    std::optional<Reflector::StructDefinition> struc) const {
  nlohmann::json::object_t result;
  result["name"] = struc->name;
  result["byte_length"] = struc->byte_length;
  auto& members = result["members"] = nlohmann::json::array_t{};
  for (const auto& struct_member : struc->members) {
    auto& member = members.emplace_back(nlohmann::json::object_t{});
    member["name"] = struct_member.name;
    member["type"] = struct_member.type;
    member["base_type"] =
        StructMember::BaseTypeToString(struct_member.base_type);
    member["offset"] = struct_member.offset;
    member["byte_length"] = struct_member.byte_length;
    if (struct_member.array_elements.has_value()) {
      member["array_elements"] = struct_member.array_elements.value();
    } else {
      member["array_elements"] = "std::nullopt";
    }
    member["element_padding"] = struct_member.element_padding;
  }
  return result;
}

struct VertexType {
  std::string type_name;
  spirv_cross::SPIRType::BaseType base_type;
  std::string variable_name;
  size_t byte_length = 0u;
};

static VertexType VertexTypeFromInputResource(
    const spirv_cross::Compiler& compiler,
    const spirv_cross::Resource* resource) {
  VertexType result;
  result.variable_name = resource->name;
  const auto& type = compiler.get_type(resource->type_id);
  result.base_type = type.basetype;
  const auto total_size = type.columns * type.vecsize * type.width / 8u;
  result.byte_length = total_size;

  if (type.basetype == spirv_cross::SPIRType::BaseType::Float &&
      type.columns == 1u && type.vecsize == 2u &&
      type.width == sizeof(float) * 8u) {
    result.type_name = "Point";
  } else if (type.basetype == spirv_cross::SPIRType::BaseType::Float &&
             type.columns == 1u && type.vecsize == 4u &&
             type.width == sizeof(float) * 8u) {
    result.type_name = "Vector4";
  } else if (type.basetype == spirv_cross::SPIRType::BaseType::Float &&
             type.columns == 1u && type.vecsize == 3u &&
             type.width == sizeof(float) * 8u) {
    result.type_name = "Vector3";
  } else if (type.basetype == spirv_cross::SPIRType::BaseType::Float &&
             type.columns == 1u && type.vecsize == 1u &&
             type.width == sizeof(float) * 8u) {
    result.type_name = "Scalar";
  } else if (type.basetype == spirv_cross::SPIRType::BaseType::Int &&
             type.columns == 1u && type.vecsize == 1u &&
             type.width == sizeof(int32_t) * 8u) {
    result.type_name = "int32_t";
  } else {
    // Catch all unknown padding.
    result.type_name = TypeNameWithPaddingOfSize(total_size);
  }

  return result;
}

std::optional<Reflector::StructDefinition>
Reflector::ReflectPerVertexStructDefinition(
    const spirv_cross::SmallVector<spirv_cross::Resource>& stage_inputs) const {
  // Avoid emitting a zero sized structure. The code gen templates assume a
  // non-zero size.
  if (stage_inputs.empty()) {
    return std::nullopt;
  }

  // Validate locations are contiguous and there are no duplicates.
  std::set<uint32_t> locations;
  for (const auto& input : stage_inputs) {
    auto location = compiler_->get_decoration(
        input.id, spv::Decoration::DecorationLocation);
    if (locations.count(location) != 0) {
      // Duplicate location. Bail.
      return std::nullopt;
    }
    locations.insert(location);
  }

  for (size_t i = 0; i < locations.size(); i++) {
    if (locations.count(i) != 1) {
      // Locations are not contiguous. This usually happens when a single stage
      // input takes multiple input slots. No reflection information can be
      // generated for such cases anyway. So bail! It is up to the shader author
      // to make sure one stage input maps to a single input slot.
      return std::nullopt;
    }
  }

  auto input_for_location =
      [&](uint32_t queried_location) -> const spirv_cross::Resource* {
    for (const auto& input : stage_inputs) {
      auto location = compiler_->get_decoration(
          input.id, spv::Decoration::DecorationLocation);
      if (location == queried_location) {
        return &input;
      }
    }
    // This really cannot happen with all the validation above.
    FML_UNREACHABLE();
    return nullptr;
  };

  StructDefinition struc;
  struc.name = "PerVertexData";
  struc.byte_length = 0u;
  for (size_t i = 0; i < locations.size(); i++) {
    auto resource = input_for_location(i);
    if (resource == nullptr) {
      return std::nullopt;
    }
    const auto vertex_type =
        VertexTypeFromInputResource(*compiler_.GetCompiler(), resource);

    auto member = StructMember{
        vertex_type.type_name,      // type
        vertex_type.base_type,      // base type
        vertex_type.variable_name,  // name
        struc.byte_length,          // offset
        vertex_type.byte_length,    // size
        vertex_type.byte_length,    // byte_length
        std::nullopt,               // array_elements
        0,                          // element_padding
    };
    struc.byte_length += vertex_type.byte_length;
    struc.members.emplace_back(std::move(member));
  }
  return struc;
}

std::optional<std::string> Reflector::GetMemberNameAtIndexIfExists(
    const spirv_cross::SPIRType& parent_type,
    size_t index) const {
  if (parent_type.type_alias != 0) {
    return GetMemberNameAtIndexIfExists(
        compiler_->get_type(parent_type.type_alias), index);
  }

  if (auto found = ir_->meta.find(parent_type.self); found != ir_->meta.end()) {
    const auto& members = found->second.members;
    if (index < members.size() && !members[index].alias.empty()) {
      return members[index].alias;
    }
  }
  return std::nullopt;
}

std::string Reflector::GetMemberNameAtIndex(
    const spirv_cross::SPIRType& parent_type,
    size_t index,
    std::string suffix) const {
  if (auto name = GetMemberNameAtIndexIfExists(parent_type, index);
      name.has_value()) {
    return name.value();
  }
  static std::atomic_size_t sUnnamedMembersID;
  std::stringstream stream;
  stream << "unnamed_" << sUnnamedMembersID++ << suffix;
  return stream.str();
}

std::vector<Reflector::BindPrototype> Reflector::ReflectBindPrototypes(
    const spirv_cross::ShaderResources& resources,
    spv::ExecutionModel execution_model) const {
  std::vector<BindPrototype> prototypes;
  for (const auto& uniform_buffer : resources.uniform_buffers) {
    auto& proto = prototypes.emplace_back(BindPrototype{});
    proto.return_type = "bool";
    proto.name = ToCamelCase(uniform_buffer.name);
    proto.descriptor_type = "DescriptorType::kUniformBuffer";
    {
      std::stringstream stream;
      stream << "Bind uniform buffer for resource named " << uniform_buffer.name
             << ".";
      proto.docstring = stream.str();
    }
    proto.args.push_back(BindPrototypeArgument{
        .type_name = "ResourceBinder&",
        .argument_name = "command",
    });
    proto.args.push_back(BindPrototypeArgument{
        .type_name = "BufferView",
        .argument_name = "view",
    });
  }
  for (const auto& storage_buffer : resources.storage_buffers) {
    auto& proto = prototypes.emplace_back(BindPrototype{});
    proto.return_type = "bool";
    proto.name = ToCamelCase(storage_buffer.name);
    proto.descriptor_type = "DescriptorType::kStorageBuffer";
    {
      std::stringstream stream;
      stream << "Bind storage buffer for resource named " << storage_buffer.name
             << ".";
      proto.docstring = stream.str();
    }
    proto.args.push_back(BindPrototypeArgument{
        .type_name = "ResourceBinder&",
        .argument_name = "command",
    });
    proto.args.push_back(BindPrototypeArgument{
        .type_name = "BufferView",
        .argument_name = "view",
    });
  }
  for (const auto& sampled_image : resources.sampled_images) {
    auto& proto = prototypes.emplace_back(BindPrototype{});
    proto.return_type = "bool";
    proto.name = ToCamelCase(sampled_image.name);
    proto.descriptor_type = "DescriptorType::kSampledImage";
    {
      std::stringstream stream;
      stream << "Bind combined image sampler for resource named "
             << sampled_image.name << ".";
      proto.docstring = stream.str();
    }
    proto.args.push_back(BindPrototypeArgument{
        .type_name = "ResourceBinder&",
        .argument_name = "command",
    });
    proto.args.push_back(BindPrototypeArgument{
        .type_name = "std::shared_ptr<const Texture>",
        .argument_name = "texture",
    });
    proto.args.push_back(BindPrototypeArgument{
        .type_name = "raw_ptr<const Sampler>",
        .argument_name = "sampler",
    });
  }
  for (const auto& separate_image : resources.separate_images) {
    auto& proto = prototypes.emplace_back(BindPrototype{});
    proto.return_type = "bool";
    proto.name = ToCamelCase(separate_image.name);
    proto.descriptor_type = "DescriptorType::kImage";
    {
      std::stringstream stream;
      stream << "Bind separate image for resource named " << separate_image.name
             << ".";
      proto.docstring = stream.str();
    }
    proto.args.push_back(BindPrototypeArgument{
        .type_name = "Command&",
        .argument_name = "command",
    });
    proto.args.push_back(BindPrototypeArgument{
        .type_name = "std::shared_ptr<const Texture>",
        .argument_name = "texture",
    });
  }
  for (const auto& separate_sampler : resources.separate_samplers) {
    auto& proto = prototypes.emplace_back(BindPrototype{});
    proto.return_type = "bool";
    proto.name = ToCamelCase(separate_sampler.name);
    proto.descriptor_type = "DescriptorType::kSampler";
    {
      std::stringstream stream;
      stream << "Bind separate sampler for resource named "
             << separate_sampler.name << ".";
      proto.docstring = stream.str();
    }
    proto.args.push_back(BindPrototypeArgument{
        .type_name = "Command&",
        .argument_name = "command",
    });
    proto.args.push_back(BindPrototypeArgument{
        .type_name = "std::shared_ptr<const Sampler>",
        .argument_name = "sampler",
    });
  }
  return prototypes;
}

nlohmann::json::array_t Reflector::EmitBindPrototypes(
    const spirv_cross::ShaderResources& resources,
    spv::ExecutionModel execution_model) const {
  const auto prototypes = ReflectBindPrototypes(resources, execution_model);
  nlohmann::json::array_t result;
  for (const auto& res : prototypes) {
    auto& item = result.emplace_back(nlohmann::json::object_t{});
    item["return_type"] = res.return_type;
    item["name"] = res.name;
    item["docstring"] = res.docstring;
    item["descriptor_type"] = res.descriptor_type;
    auto& args = item["args"] = nlohmann::json::array_t{};
    for (const auto& arg : res.args) {
      auto& json_arg = args.emplace_back(nlohmann::json::object_t{});
      json_arg["type_name"] = arg.type_name;
      json_arg["argument_name"] = arg.argument_name;
    }
  }
  return result;
}

}  // namespace compiler
}  // namespace impeller
