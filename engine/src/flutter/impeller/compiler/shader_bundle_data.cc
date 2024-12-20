// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/compiler/shader_bundle_data.h"

#include <optional>

#include "impeller/shader_bundle/shader_bundle_flatbuffers.h"

#include "impeller/base/validation.h"

namespace impeller {
namespace compiler {

ShaderBundleData::ShaderBundleData(std::string entrypoint,
                                   spv::ExecutionModel stage,
                                   TargetPlatform target_platform)
    : entrypoint_(std::move(entrypoint)),
      stage_(stage),
      target_platform_(target_platform) {}

ShaderBundleData::~ShaderBundleData() = default;

void ShaderBundleData::AddUniformStruct(ShaderUniformStruct uniform_struct) {
  uniform_structs_.emplace_back(std::move(uniform_struct));
}

void ShaderBundleData::AddUniformTexture(ShaderUniformTexture uniform_texture) {
  uniform_textures_.emplace_back(std::move(uniform_texture));
}

void ShaderBundleData::AddInputDescription(InputDescription input) {
  inputs_.emplace_back(std::move(input));
}

void ShaderBundleData::SetShaderData(std::shared_ptr<fml::Mapping> shader) {
  shader_ = std::move(shader);
}

static std::optional<fb::shaderbundle::ShaderStage> ToStage(
    spv::ExecutionModel stage) {
  switch (stage) {
    case spv::ExecutionModel::ExecutionModelVertex:
      return fb::shaderbundle::ShaderStage::kVertex;
    case spv::ExecutionModel::ExecutionModelFragment:
      return fb::shaderbundle::ShaderStage::kFragment;
    case spv::ExecutionModel::ExecutionModelGLCompute:
      return fb::shaderbundle::ShaderStage::kCompute;
    default:
      return std::nullopt;
  }
  FML_UNREACHABLE();
}

static std::optional<fb::shaderbundle::UniformDataType> ToUniformType(
    spirv_cross::SPIRType::BaseType type) {
  switch (type) {
    case spirv_cross::SPIRType::Boolean:
      return fb::shaderbundle::UniformDataType::kBoolean;
    case spirv_cross::SPIRType::SByte:
      return fb::shaderbundle::UniformDataType::kSignedByte;
    case spirv_cross::SPIRType::UByte:
      return fb::shaderbundle::UniformDataType::kUnsignedByte;
    case spirv_cross::SPIRType::Short:
      return fb::shaderbundle::UniformDataType::kSignedShort;
    case spirv_cross::SPIRType::UShort:
      return fb::shaderbundle::UniformDataType::kUnsignedShort;
    case spirv_cross::SPIRType::Int:
      return fb::shaderbundle::UniformDataType::kSignedInt;
    case spirv_cross::SPIRType::UInt:
      return fb::shaderbundle::UniformDataType::kUnsignedInt;
    case spirv_cross::SPIRType::Int64:
      return fb::shaderbundle::UniformDataType::kSignedInt64;
    case spirv_cross::SPIRType::UInt64:
      return fb::shaderbundle::UniformDataType::kUnsignedInt64;
    case spirv_cross::SPIRType::Half:
      return fb::shaderbundle::UniformDataType::kHalfFloat;
    case spirv_cross::SPIRType::Float:
      return fb::shaderbundle::UniformDataType::kFloat;
    case spirv_cross::SPIRType::Double:
      return fb::shaderbundle::UniformDataType::kDouble;
    case spirv_cross::SPIRType::SampledImage:
      return fb::shaderbundle::UniformDataType::kSampledImage;
    case spirv_cross::SPIRType::AccelerationStructure:
    case spirv_cross::SPIRType::AtomicCounter:
    case spirv_cross::SPIRType::Char:
    case spirv_cross::SPIRType::ControlPointArray:
    case spirv_cross::SPIRType::Image:
    case spirv_cross::SPIRType::Interpolant:
    case spirv_cross::SPIRType::RayQuery:
    case spirv_cross::SPIRType::Sampler:
    case spirv_cross::SPIRType::Struct:
    case spirv_cross::SPIRType::Unknown:
    case spirv_cross::SPIRType::Void:
      return std::nullopt;
  }
  FML_UNREACHABLE();
}
static std::optional<fb::shaderbundle::InputDataType> ToInputType(
    spirv_cross::SPIRType::BaseType type) {
  switch (type) {
    case spirv_cross::SPIRType::Boolean:
      return fb::shaderbundle::InputDataType::kBoolean;
    case spirv_cross::SPIRType::SByte:
      return fb::shaderbundle::InputDataType::kSignedByte;
    case spirv_cross::SPIRType::UByte:
      return fb::shaderbundle::InputDataType::kUnsignedByte;
    case spirv_cross::SPIRType::Short:
      return fb::shaderbundle::InputDataType::kSignedShort;
    case spirv_cross::SPIRType::UShort:
      return fb::shaderbundle::InputDataType::kUnsignedShort;
    case spirv_cross::SPIRType::Int:
      return fb::shaderbundle::InputDataType::kSignedInt;
    case spirv_cross::SPIRType::UInt:
      return fb::shaderbundle::InputDataType::kUnsignedInt;
    case spirv_cross::SPIRType::Int64:
      return fb::shaderbundle::InputDataType::kSignedInt64;
    case spirv_cross::SPIRType::UInt64:
      return fb::shaderbundle::InputDataType::kUnsignedInt64;
    case spirv_cross::SPIRType::Float:
      return fb::shaderbundle::InputDataType::kFloat;
    case spirv_cross::SPIRType::Double:
      return fb::shaderbundle::InputDataType::kDouble;
    case spirv_cross::SPIRType::Unknown:
    case spirv_cross::SPIRType::Void:
    case spirv_cross::SPIRType::Half:
    case spirv_cross::SPIRType::AtomicCounter:
    case spirv_cross::SPIRType::Struct:
    case spirv_cross::SPIRType::Image:
    case spirv_cross::SPIRType::SampledImage:
    case spirv_cross::SPIRType::Sampler:
    case spirv_cross::SPIRType::AccelerationStructure:
    case spirv_cross::SPIRType::RayQuery:
    case spirv_cross::SPIRType::ControlPointArray:
    case spirv_cross::SPIRType::Interpolant:
    case spirv_cross::SPIRType::Char:
      return std::nullopt;
  }
  FML_UNREACHABLE();
}

std::unique_ptr<fb::shaderbundle::BackendShaderT>
ShaderBundleData::CreateFlatbuffer() const {
  auto shader_bundle = std::make_unique<fb::shaderbundle::BackendShaderT>();

  // The high level object API is used here for writing to the buffer. This is
  // just a convenience.
  shader_bundle->entrypoint = entrypoint_;
  const auto stage = ToStage(stage_);
  if (!stage.has_value()) {
    VALIDATION_LOG << "Invalid shader bundle.";
    return nullptr;
  }
  shader_bundle->stage = stage.value();
  // This field is ignored, so just set it to anything.
  if (!shader_) {
    VALIDATION_LOG << "No shader specified for shader bundle.";
    return nullptr;
  }
  if (shader_->GetSize() > 0u) {
    shader_bundle->shader = {shader_->GetMapping(),
                             shader_->GetMapping() + shader_->GetSize()};
  }
  for (const auto& uniform : uniform_structs_) {
    auto desc = std::make_unique<fb::shaderbundle::ShaderUniformStructT>();

    desc->name = uniform.name;
    if (desc->name.empty()) {
      VALIDATION_LOG << "Uniform name cannot be empty.";
      return nullptr;
    }
    desc->ext_res_0 = uniform.ext_res_0;
    desc->set = uniform.set;
    desc->binding = uniform.binding;
    desc->size_in_bytes = uniform.size_in_bytes;

    for (const auto& field : uniform.fields) {
      auto field_desc =
          std::make_unique<fb::shaderbundle::ShaderUniformStructFieldT>();
      field_desc->name = field.name;
      auto type = ToUniformType(field.type);
      if (!type.has_value()) {
        VALIDATION_LOG << " Invalid shader type " << field.type << ".";
        return nullptr;
      }
      field_desc->type = type.value();
      field_desc->offset_in_bytes = field.offset_in_bytes;
      field_desc->element_size_in_bytes = field.element_size_in_bytes;
      field_desc->total_size_in_bytes = field.total_size_in_bytes;
      field_desc->array_elements = field.array_elements.value_or(0);
      desc->fields.push_back(std::move(field_desc));
    }

    shader_bundle->uniform_structs.emplace_back(std::move(desc));
  }

  for (const auto& texture : uniform_textures_) {
    auto desc = std::make_unique<fb::shaderbundle::ShaderUniformTextureT>();
    desc->name = texture.name;
    if (desc->name.empty()) {
      VALIDATION_LOG << "Uniform name cannot be empty.";
      return nullptr;
    }
    desc->ext_res_0 = texture.ext_res_0;
    desc->set = texture.set;
    desc->binding = texture.binding;
    shader_bundle->uniform_textures.emplace_back(std::move(desc));
  }

  for (const auto& input : inputs_) {
    auto desc = std::make_unique<fb::shaderbundle::ShaderInputT>();

    desc->name = input.name;

    if (desc->name.empty()) {
      VALIDATION_LOG << "Stage input name cannot be empty.";
      return nullptr;
    }
    desc->location = input.location;
    desc->set = input.set;
    desc->binding = input.binding;
    auto input_type = ToInputType(input.type);
    if (!input_type.has_value()) {
      VALIDATION_LOG << "Invalid uniform type for runtime stage.";
      return nullptr;
    }
    desc->type = input_type.value();
    desc->bit_width = input.bit_width;
    desc->vec_size = input.vec_size;
    desc->columns = input.columns;
    desc->offset = input.offset;

    shader_bundle->inputs.emplace_back(std::move(desc));
  }

  return shader_bundle;
}

}  // namespace compiler
}  // namespace impeller
