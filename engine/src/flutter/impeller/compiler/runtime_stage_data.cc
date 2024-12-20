// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/compiler/runtime_stage_data.h"

#include <array>
#include <cstdint>
#include <memory>
#include <optional>

#include "fml/backtrace.h"
#include "impeller/core/runtime_types.h"
#include "inja/inja.hpp"

#include "impeller/base/validation.h"
#include "impeller/runtime_stage/runtime_stage_flatbuffers.h"
#include "runtime_stage_types_flatbuffers.h"

namespace impeller {
namespace compiler {

RuntimeStageData::RuntimeStageData() = default;

RuntimeStageData::~RuntimeStageData() = default;

void RuntimeStageData::AddShader(const std::shared_ptr<Shader>& data) {
  FML_DCHECK(data);
  FML_DCHECK(data_.find(data->backend) == data_.end());
  data_[data->backend] = data;
}

static std::optional<fb::Stage> ToStage(spv::ExecutionModel stage) {
  switch (stage) {
    case spv::ExecutionModel::ExecutionModelVertex:
      return fb::Stage::kVertex;
    case spv::ExecutionModel::ExecutionModelFragment:
      return fb::Stage::kFragment;
    case spv::ExecutionModel::ExecutionModelGLCompute:
      return fb::Stage::kCompute;
    default:
      return std::nullopt;
  }
  FML_UNREACHABLE();
}

static std::optional<fb::Stage> ToJsonStage(spv::ExecutionModel stage) {
  switch (stage) {
    case spv::ExecutionModel::ExecutionModelVertex:
      return fb::Stage::kVertex;
    case spv::ExecutionModel::ExecutionModelFragment:
      return fb::Stage::kFragment;
    case spv::ExecutionModel::ExecutionModelGLCompute:
      return fb::Stage::kCompute;
    default:
      return std::nullopt;
  }
  FML_UNREACHABLE();
}

static std::optional<fb::UniformDataType> ToUniformType(
    spirv_cross::SPIRType::BaseType type) {
  switch (type) {
    case spirv_cross::SPIRType::Float:
      return fb::UniformDataType::kFloat;
    case spirv_cross::SPIRType::SampledImage:
      return fb::UniformDataType::kSampledImage;
    case spirv_cross::SPIRType::Struct:
      return fb::UniformDataType::kStruct;
    case spirv_cross::SPIRType::Boolean:
    case spirv_cross::SPIRType::SByte:
    case spirv_cross::SPIRType::UByte:
    case spirv_cross::SPIRType::Short:
    case spirv_cross::SPIRType::UShort:
    case spirv_cross::SPIRType::Int:
    case spirv_cross::SPIRType::UInt:
    case spirv_cross::SPIRType::Int64:
    case spirv_cross::SPIRType::UInt64:
    case spirv_cross::SPIRType::Half:
    case spirv_cross::SPIRType::Double:
    case spirv_cross::SPIRType::AccelerationStructure:
    case spirv_cross::SPIRType::AtomicCounter:
    case spirv_cross::SPIRType::Char:
    case spirv_cross::SPIRType::ControlPointArray:
    case spirv_cross::SPIRType::Image:
    case spirv_cross::SPIRType::Interpolant:
    case spirv_cross::SPIRType::RayQuery:
    case spirv_cross::SPIRType::Sampler:
    case spirv_cross::SPIRType::Unknown:
    case spirv_cross::SPIRType::Void:
      return std::nullopt;
  }
  FML_UNREACHABLE();
}
static std::optional<fb::InputDataType> ToInputType(
    spirv_cross::SPIRType::BaseType type) {
  switch (type) {
    case spirv_cross::SPIRType::Boolean:
      return fb::InputDataType::kBoolean;
    case spirv_cross::SPIRType::SByte:
      return fb::InputDataType::kSignedByte;
    case spirv_cross::SPIRType::UByte:
      return fb::InputDataType::kUnsignedByte;
    case spirv_cross::SPIRType::Short:
      return fb::InputDataType::kSignedShort;
    case spirv_cross::SPIRType::UShort:
      return fb::InputDataType::kUnsignedShort;
    case spirv_cross::SPIRType::Int:
      return fb::InputDataType::kSignedInt;
    case spirv_cross::SPIRType::UInt:
      return fb::InputDataType::kUnsignedInt;
    case spirv_cross::SPIRType::Int64:
      return fb::InputDataType::kSignedInt64;
    case spirv_cross::SPIRType::UInt64:
      return fb::InputDataType::kUnsignedInt64;
    case spirv_cross::SPIRType::Float:
      return fb::InputDataType::kFloat;
    case spirv_cross::SPIRType::Double:
      return fb::InputDataType::kDouble;
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

static std::optional<uint32_t> ToJsonType(
    spirv_cross::SPIRType::BaseType type) {
  switch (type) {
    case spirv_cross::SPIRType::Boolean:
      return 0;  // fb::UniformDataType::kBoolean;
    case spirv_cross::SPIRType::SByte:
      return 1;  // fb::UniformDataType::kSignedByte;
    case spirv_cross::SPIRType::UByte:
      return 2;  // fb::UniformDataType::kUnsignedByte;
    case spirv_cross::SPIRType::Short:
      return 3;  // fb::UniformDataType::kSignedShort;
    case spirv_cross::SPIRType::UShort:
      return 4;  // fb::UniformDataType::kUnsignedShort;
    case spirv_cross::SPIRType::Int:
      return 5;  // fb::UniformDataType::kSignedInt;
    case spirv_cross::SPIRType::UInt:
      return 6;  // fb::UniformDataType::kUnsignedInt;
    case spirv_cross::SPIRType::Int64:
      return 7;  // fb::UniformDataType::kSignedInt64;
    case spirv_cross::SPIRType::UInt64:
      return 8;  // fb::UniformDataType::kUnsignedInt64;
    case spirv_cross::SPIRType::Half:
      return 9;  // b::UniformDataType::kHalfFloat;
    case spirv_cross::SPIRType::Float:
      return 10;  // fb::UniformDataType::kFloat;
    case spirv_cross::SPIRType::Double:
      return 11;  // fb::UniformDataType::kDouble;
    case spirv_cross::SPIRType::SampledImage:
      return 12;  // fb::UniformDataType::kSampledImage;
    case spirv_cross::SPIRType::Struct:
      return 13;
    case spirv_cross::SPIRType::AccelerationStructure:
    case spirv_cross::SPIRType::AtomicCounter:
    case spirv_cross::SPIRType::Char:
    case spirv_cross::SPIRType::ControlPointArray:
    case spirv_cross::SPIRType::Image:
    case spirv_cross::SPIRType::Interpolant:
    case spirv_cross::SPIRType::RayQuery:
    case spirv_cross::SPIRType::Sampler:
    case spirv_cross::SPIRType::Unknown:
    case spirv_cross::SPIRType::Void:
      return std::nullopt;
  }
  FML_UNREACHABLE();
}

static const char* kStageKey = "stage";
static const char* kTargetPlatformKey = "target_platform";
static const char* kEntrypointKey = "entrypoint";
static const char* kUniformsKey = "uniforms";
static const char* kShaderKey = "shader";
static const char* kUniformNameKey = "name";
static const char* kUniformLocationKey = "location";
static const char* kUniformTypeKey = "type";
static const char* kUniformRowsKey = "rows";
static const char* kUniformColumnsKey = "columns";
static const char* kUniformBitWidthKey = "bit_width";
static const char* kUniformArrayElementsKey = "array_elements";

static std::string RuntimeStageBackendToString(RuntimeStageBackend backend) {
  switch (backend) {
    case RuntimeStageBackend::kSkSL:
      return "sksl";
    case RuntimeStageBackend::kMetal:
      return "metal";
    case RuntimeStageBackend::kOpenGLES:
      return "opengles";
    case RuntimeStageBackend::kVulkan:
      return "vulkan";
    case RuntimeStageBackend::kOpenGLES3:
      return "opengles3";
  }
}

std::shared_ptr<fml::Mapping> RuntimeStageData::CreateJsonMapping() const {
  // Runtime Stage Data JSON format
  //   {
  //      "sksl": {
  //        "stage": 0,
  //        "entrypoint": "",
  //        "shader": "",
  //        "uniforms": [
  //          {
  //             "name": "..",
  //             "location": 0,
  //             "type": 0,
  //             "rows": 0,
  //             "columns": 0,
  //             "bit_width": 0,
  //             "array_elements": 0,
  //          }
  //        ]
  //      },
  //      "metal": ...
  //   },
  nlohmann::json root;

  for (const auto& kvp : data_) {
    nlohmann::json platform_object;

    const auto stage = ToJsonStage(kvp.second->stage);
    if (!stage.has_value()) {
      VALIDATION_LOG << "Invalid runtime stage.";
      return nullptr;
    }
    platform_object[kStageKey] = static_cast<uint32_t>(stage.value());
    platform_object[kEntrypointKey] = kvp.second->entrypoint;

    if (kvp.second->shader->GetSize() > 0u) {
      std::string shader(
          reinterpret_cast<const char*>(kvp.second->shader->GetMapping()),
          kvp.second->shader->GetSize());
      platform_object[kShaderKey] = shader.c_str();
    }

    auto& uniforms = platform_object[kUniformsKey] = nlohmann::json::array_t{};
    for (const auto& uniform : kvp.second->uniforms) {
      nlohmann::json uniform_object;
      uniform_object[kUniformNameKey] = uniform.name.c_str();
      uniform_object[kUniformLocationKey] = uniform.location;
      uniform_object[kUniformRowsKey] = uniform.rows;
      uniform_object[kUniformColumnsKey] = uniform.columns;

      auto uniform_type = ToJsonType(uniform.type);
      if (!uniform_type.has_value()) {
        VALIDATION_LOG << "Invalid uniform type for runtime stage.";
        return nullptr;
      }

      uniform_object[kUniformTypeKey] = uniform_type.value();
      uniform_object[kUniformBitWidthKey] = uniform.bit_width;
      uniform_object[kUniformArrayElementsKey] =
          uniform.array_elements.value_or(0);

      uniforms.push_back(uniform_object);
    }

    root[RuntimeStageBackendToString(kvp.first)] = platform_object;
  }

  auto json_string = std::make_shared<std::string>(root.dump(2u));

  return std::make_shared<fml::NonOwnedMapping>(
      reinterpret_cast<const uint8_t*>(json_string->data()),
      json_string->size(), [json_string](auto, auto) {});
}

std::unique_ptr<fb::RuntimeStageT> RuntimeStageData::CreateStageFlatbuffer(
    impeller::RuntimeStageBackend backend) const {
  auto kvp = data_.find(backend);
  if (kvp == data_.end()) {
    return nullptr;
  }

  auto runtime_stage = std::make_unique<fb::RuntimeStageT>();
  runtime_stage->entrypoint = kvp->second->entrypoint;
  const auto stage = ToStage(kvp->second->stage);
  if (!stage.has_value()) {
    VALIDATION_LOG << "Invalid runtime stage.";
    return nullptr;
  }
  runtime_stage->stage = stage.value();
  if (!kvp->second->shader) {
    VALIDATION_LOG << "No shader specified for runtime stage.";
    return nullptr;
  }
  if (kvp->second->shader->GetSize() > 0u) {
    runtime_stage->shader = {
        kvp->second->shader->GetMapping(),
        kvp->second->shader->GetMapping() + kvp->second->shader->GetSize()};
  }
  for (const auto& uniform : kvp->second->uniforms) {
    auto desc = std::make_unique<fb::UniformDescriptionT>();

    desc->name = uniform.name;
    if (desc->name.empty()) {
      VALIDATION_LOG << "Uniform name cannot be empty.";
      return nullptr;
    }
    desc->location = uniform.location;
    desc->rows = uniform.rows;
    desc->columns = uniform.columns;
    auto uniform_type = ToUniformType(uniform.type);
    if (!uniform_type.has_value()) {
      VALIDATION_LOG << "Invalid uniform type for runtime stage.";
      return nullptr;
    }
    desc->type = uniform_type.value();
    desc->bit_width = uniform.bit_width;
    if (uniform.array_elements.has_value()) {
      desc->array_elements = uniform.array_elements.value();
    }

    for (const auto& byte_type : uniform.struct_layout) {
      desc->struct_layout.push_back(static_cast<fb::StructByteType>(byte_type));
    }
    desc->struct_float_count = uniform.struct_float_count;

    runtime_stage->uniforms.emplace_back(std::move(desc));
  }

  for (const auto& input : kvp->second->inputs) {
    auto desc = std::make_unique<fb::StageInputT>();

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

    runtime_stage->inputs.emplace_back(std::move(desc));
  }

  return runtime_stage;
}

std::unique_ptr<fb::RuntimeStagesT>
RuntimeStageData::CreateMultiStageFlatbuffer() const {
  // The high level object API is used here for writing to the buffer. This is
  // just a convenience.
  auto runtime_stages = std::make_unique<fb::RuntimeStagesT>();

  for (const auto& kvp : data_) {
    auto runtime_stage = CreateStageFlatbuffer(kvp.first);
    switch (kvp.first) {
      case RuntimeStageBackend::kSkSL:
        runtime_stages->sksl = std::move(runtime_stage);
        break;
      case RuntimeStageBackend::kMetal:
        runtime_stages->metal = std::move(runtime_stage);
        break;
      case RuntimeStageBackend::kOpenGLES:
        runtime_stages->opengles = std::move(runtime_stage);
        break;
      case RuntimeStageBackend::kVulkan:
        runtime_stages->vulkan = std::move(runtime_stage);
        break;
      case RuntimeStageBackend::kOpenGLES3:
        runtime_stages->opengles3 = std::move(runtime_stage);
        break;
    }
  }
  return runtime_stages;
}

std::shared_ptr<fml::Mapping> RuntimeStageData::CreateMapping() const {
  auto runtime_stages = CreateMultiStageFlatbuffer();
  if (!runtime_stages) {
    return nullptr;
  }

  auto builder = std::make_shared<flatbuffers::FlatBufferBuilder>();
  builder->Finish(fb::RuntimeStages::Pack(*builder.get(), runtime_stages.get()),
                  fb::RuntimeStagesIdentifier());
  return std::make_shared<fml::NonOwnedMapping>(builder->GetBufferPointer(),
                                                builder->GetSize(),
                                                [builder](auto, auto) {});
}

}  // namespace compiler
}  // namespace impeller
