// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/lib/gpu/shader_library.h"

#include <optional>
#include <utility>

#include "flutter/assets/asset_manager.h"
#include "flutter/lib/gpu/shader.h"
#include "flutter/lib/ui/ui_dart_state.h"
#include "flutter/lib/ui/window/platform_configuration.h"
#include "fml/mapping.h"
#include "fml/memory/ref_ptr.h"
#include "impeller/base/validation.h"
#include "impeller/core/shader_types.h"
#include "impeller/shader_bundle/shader_bundle_flatbuffers.h"
#include "lib/gpu/context.h"

namespace flutter {
namespace gpu {

IMPLEMENT_WRAPPERTYPEINFO(flutter_gpu, ShaderLibrary);

fml::RefPtr<ShaderLibrary> ShaderLibrary::override_shader_library_;

fml::RefPtr<ShaderLibrary> ShaderLibrary::MakeFromAsset(
    impeller::Context::BackendType backend_type,
    const std::string& name,
    std::string& out_error) {
  if (override_shader_library_) {
    return override_shader_library_;
  }

  auto dart_state = UIDartState::Current();
  std::shared_ptr<AssetManager> asset_manager =
      dart_state->platform_configuration()->client()->GetAssetManager();

  std::unique_ptr<fml::Mapping> data = asset_manager->GetAsMapping(name);
  if (data == nullptr) {
    out_error = std::string("Asset '") + name + std::string("' not found.");
    return nullptr;
  }

  return MakeFromFlatbuffer(backend_type, std::move(data));
}

fml::RefPtr<ShaderLibrary> ShaderLibrary::MakeFromShaders(ShaderMap shaders) {
  return fml::MakeRefCounted<flutter::gpu::ShaderLibrary>(nullptr,
                                                          std::move(shaders));
}

static impeller::ShaderStage ToShaderStage(
    impeller::fb::shaderbundle::ShaderStage stage) {
  switch (stage) {
    case impeller::fb::shaderbundle::ShaderStage::kVertex:
      return impeller::ShaderStage::kVertex;
    case impeller::fb::shaderbundle::ShaderStage::kFragment:
      return impeller::ShaderStage::kFragment;
    case impeller::fb::shaderbundle::ShaderStage::kCompute:
      return impeller::ShaderStage::kCompute;
  }
  FML_UNREACHABLE();
}

static impeller::ShaderType FromInputType(
    impeller::fb::shaderbundle::InputDataType input_type) {
  switch (input_type) {
    case impeller::fb::shaderbundle::InputDataType::kBoolean:
      return impeller::ShaderType::kBoolean;
    case impeller::fb::shaderbundle::InputDataType::kSignedByte:
      return impeller::ShaderType::kSignedByte;
    case impeller::fb::shaderbundle::InputDataType::kUnsignedByte:
      return impeller::ShaderType::kUnsignedByte;
    case impeller::fb::shaderbundle::InputDataType::kSignedShort:
      return impeller::ShaderType::kSignedShort;
    case impeller::fb::shaderbundle::InputDataType::kUnsignedShort:
      return impeller::ShaderType::kUnsignedShort;
    case impeller::fb::shaderbundle::InputDataType::kSignedInt:
      return impeller::ShaderType::kSignedInt;
    case impeller::fb::shaderbundle::InputDataType::kUnsignedInt:
      return impeller::ShaderType::kUnsignedInt;
    case impeller::fb::shaderbundle::InputDataType::kSignedInt64:
      return impeller::ShaderType::kSignedInt64;
    case impeller::fb::shaderbundle::InputDataType::kUnsignedInt64:
      return impeller::ShaderType::kUnsignedInt64;
    case impeller::fb::shaderbundle::InputDataType::kFloat:
      return impeller::ShaderType::kFloat;
    case impeller::fb::shaderbundle::InputDataType::kDouble:
      return impeller::ShaderType::kDouble;
  }
}

static impeller::ShaderType FromUniformType(
    impeller::fb::shaderbundle::UniformDataType uniform_type) {
  switch (uniform_type) {
    case impeller::fb::shaderbundle::UniformDataType::kBoolean:
      return impeller::ShaderType::kBoolean;
    case impeller::fb::shaderbundle::UniformDataType::kSignedByte:
      return impeller::ShaderType::kSignedByte;
    case impeller::fb::shaderbundle::UniformDataType::kUnsignedByte:
      return impeller::ShaderType::kUnsignedByte;
    case impeller::fb::shaderbundle::UniformDataType::kSignedShort:
      return impeller::ShaderType::kSignedShort;
    case impeller::fb::shaderbundle::UniformDataType::kUnsignedShort:
      return impeller::ShaderType::kUnsignedShort;
    case impeller::fb::shaderbundle::UniformDataType::kSignedInt:
      return impeller::ShaderType::kSignedInt;
    case impeller::fb::shaderbundle::UniformDataType::kUnsignedInt:
      return impeller::ShaderType::kUnsignedInt;
    case impeller::fb::shaderbundle::UniformDataType::kSignedInt64:
      return impeller::ShaderType::kSignedInt64;
    case impeller::fb::shaderbundle::UniformDataType::kUnsignedInt64:
      return impeller::ShaderType::kUnsignedInt64;
    case impeller::fb::shaderbundle::UniformDataType::kFloat:
      return impeller::ShaderType::kFloat;
    case impeller::fb::shaderbundle::UniformDataType::kDouble:
      return impeller::ShaderType::kDouble;
    case impeller::fb::shaderbundle::UniformDataType::kHalfFloat:
      return impeller::ShaderType::kHalfFloat;
    case impeller::fb::shaderbundle::UniformDataType::kSampledImage:
      return impeller::ShaderType::kSampledImage;
  }
}

static size_t SizeOfInputType(
    impeller::fb::shaderbundle::InputDataType input_type) {
  switch (input_type) {
    case impeller::fb::shaderbundle::InputDataType::kBoolean:
      return 1;
    case impeller::fb::shaderbundle::InputDataType::kSignedByte:
      return 1;
    case impeller::fb::shaderbundle::InputDataType::kUnsignedByte:
      return 1;
    case impeller::fb::shaderbundle::InputDataType::kSignedShort:
      return 2;
    case impeller::fb::shaderbundle::InputDataType::kUnsignedShort:
      return 2;
    case impeller::fb::shaderbundle::InputDataType::kSignedInt:
      return 4;
    case impeller::fb::shaderbundle::InputDataType::kUnsignedInt:
      return 4;
    case impeller::fb::shaderbundle::InputDataType::kSignedInt64:
      return 8;
    case impeller::fb::shaderbundle::InputDataType::kUnsignedInt64:
      return 8;
    case impeller::fb::shaderbundle::InputDataType::kFloat:
      return 4;
    case impeller::fb::shaderbundle::InputDataType::kDouble:
      return 8;
  }
}

static const impeller::fb::shaderbundle::BackendShader* GetShaderBackend(
    impeller::Context::BackendType backend_type,
    const impeller::fb::shaderbundle::Shader* shader) {
  switch (backend_type) {
    case impeller::Context::BackendType::kMetal:
#ifdef FML_OS_IOS
      return shader->metal_ios();
#else
      return shader->metal_desktop();
#endif
    case impeller::Context::BackendType::kOpenGLES:
      return shader->opengl_es();
    case impeller::Context::BackendType::kVulkan:
      return shader->vulkan();
  }
}

fml::RefPtr<ShaderLibrary> ShaderLibrary::MakeFromFlatbuffer(
    impeller::Context::BackendType backend_type,
    std::shared_ptr<fml::Mapping> payload) {
  if (payload == nullptr || !payload->GetMapping()) {
    return nullptr;
  }
  if (!impeller::fb::shaderbundle::ShaderBundleBufferHasIdentifier(
          payload->GetMapping())) {
    return nullptr;
  }
  auto* bundle =
      impeller::fb::shaderbundle::GetShaderBundle(payload->GetMapping());
  if (!bundle) {
    return nullptr;
  }

  ShaderLibrary::ShaderMap shader_map;

  for (const auto* bundled_shader : *bundle->shaders()) {
    const impeller::fb::shaderbundle::BackendShader* backend_shader =
        GetShaderBackend(backend_type, bundled_shader);
    if (!backend_shader) {
      VALIDATION_LOG << "Failed to unpack shader \""
                     << bundled_shader->name()->c_str() << "\" from bundle.";
      continue;
    }

    auto code_mapping = std::make_shared<fml::NonOwnedMapping>(
        backend_shader->shader()->data(),   //
        backend_shader->shader()->size(),   //
        [payload = payload](auto, auto) {}  //
    );

    std::vector<impeller::DescriptorSetLayout> descriptor_set_layouts;

    std::unordered_map<std::string, Shader::UniformBinding> uniform_structs;
    if (backend_shader->uniform_structs() != nullptr) {
      for (const auto& uniform : *backend_shader->uniform_structs()) {
        std::vector<impeller::ShaderStructMemberMetadata> members;
        if (uniform->fields() != nullptr) {
          for (const auto& struct_member : *uniform->fields()) {
            members.push_back(impeller::ShaderStructMemberMetadata{
                .type = FromUniformType(struct_member->type()),
                .name = struct_member->name()->c_str(),
                .offset = static_cast<size_t>(struct_member->offset_in_bytes()),
                .size =
                    static_cast<size_t>(struct_member->element_size_in_bytes()),
                .byte_length =
                    static_cast<size_t>(struct_member->total_size_in_bytes()),
                .array_elements =
                    struct_member->array_elements() == 0
                        ? std::optional<size_t>(std::nullopt)
                        : static_cast<size_t>(struct_member->array_elements()),
            });
          }
        }

        uniform_structs[uniform->name()->str()] = Shader::UniformBinding{
            .slot =
                impeller::ShaderUniformSlot{
                    .name = uniform->name()->c_str(),
                    .ext_res_0 = static_cast<size_t>(uniform->ext_res_0()),
                    .set = static_cast<size_t>(uniform->set()),
                    .binding = static_cast<size_t>(uniform->binding()),
                },
            .metadata =
                impeller::ShaderMetadata{
                    .name = uniform->name()->c_str(),
                    .members = members,
                },
            .size_in_bytes = static_cast<size_t>(uniform->size_in_bytes()),
        };

        descriptor_set_layouts.push_back(impeller::DescriptorSetLayout{
            static_cast<uint32_t>(uniform->binding()),
            impeller::DescriptorType::kUniformBuffer,
            ToShaderStage(backend_shader->stage()),
        });
      }
    }

    std::unordered_map<std::string, Shader::TextureBinding> uniform_textures;
    if (backend_shader->uniform_textures() != nullptr) {
      for (const auto& uniform : *backend_shader->uniform_textures()) {
        Shader::TextureBinding texture_binding;
        texture_binding.slot = impeller::SampledImageSlot{
            .name = uniform->name()->c_str(),
            .texture_index = static_cast<size_t>(uniform->ext_res_0()),
            .set = static_cast<size_t>(uniform->set()),
            .binding = static_cast<size_t>(uniform->binding()),
        };
        texture_binding.metadata = impeller::ShaderMetadata{
            .name = uniform->name()->c_str(),
            .members = {},
        };

        uniform_textures[uniform->name()->str()] = texture_binding;

        descriptor_set_layouts.push_back(impeller::DescriptorSetLayout{
            static_cast<uint32_t>(uniform->binding()),
            impeller::DescriptorType::kSampledImage,
            ToShaderStage(backend_shader->stage()),
        });
      }
    }

    std::vector<impeller::ShaderStageIOSlot> inputs;
    std::vector<impeller::ShaderStageBufferLayout> layouts;
    if (backend_shader->stage() ==
        impeller::fb::shaderbundle::ShaderStage::kVertex) {
      auto inputs_fb = backend_shader->inputs();

      inputs.reserve(inputs_fb->size());
      size_t default_stride = 0;
      for (const auto& input : *inputs_fb) {
        impeller::ShaderStageIOSlot slot;
        slot.name = input->name()->c_str();
        slot.location = input->location();
        slot.set = input->set();
        slot.binding = input->binding();
        slot.type = FromInputType(input->type());
        slot.bit_width = input->bit_width();
        slot.vec_size = input->vec_size();
        slot.columns = input->columns();
        slot.offset = input->offset();
        inputs.emplace_back(slot);

        default_stride +=
            SizeOfInputType(input->type()) * slot.vec_size * slot.columns;
      }
      layouts = {impeller::ShaderStageBufferLayout{
          .stride = default_stride,
          .binding = 0u,
      }};
    }

    auto shader = flutter::gpu::Shader::Make(
        backend_shader->entrypoint()->str(),
        ToShaderStage(backend_shader->stage()), std::move(code_mapping),
        std::move(inputs), std::move(layouts), std::move(uniform_structs),
        std::move(uniform_textures), std::move(descriptor_set_layouts));
    shader_map[bundled_shader->name()->str()] = std::move(shader);
  }

  return fml::MakeRefCounted<flutter::gpu::ShaderLibrary>(
      std::move(payload), std::move(shader_map));
}

void ShaderLibrary::SetOverride(
    fml::RefPtr<ShaderLibrary> override_shader_library) {
  override_shader_library_ = std::move(override_shader_library);
}

fml::RefPtr<Shader> ShaderLibrary::GetShader(const std::string& shader_name,
                                             Dart_Handle shader_wrapper) const {
  auto it = shaders_.find(shader_name);
  if (it == shaders_.end()) {
    return nullptr;  // No matching shaders.
  }
  auto shader = it->second;

  if (shader->dart_wrapper() == nullptr) {
    shader->AssociateWithDartWrapper(shader_wrapper);
  }
  return shader;
}

ShaderLibrary::ShaderLibrary(std::shared_ptr<fml::Mapping> payload,
                             ShaderMap shaders)
    : payload_(std::move(payload)), shaders_(std::move(shaders)) {}

ShaderLibrary::~ShaderLibrary() = default;

}  // namespace gpu
}  // namespace flutter

//----------------------------------------------------------------------------
/// Exports
///

Dart_Handle InternalFlutterGpu_ShaderLibrary_InitializeWithAsset(
    Dart_Handle wrapper,
    Dart_Handle asset_name) {
  if (!Dart_IsString(asset_name)) {
    return tonic::ToDart("Asset name must be a string");
  }

  std::optional<std::string> out_error;
  auto impeller_context = flutter::gpu::Context::GetDefaultContext(out_error);
  if (out_error.has_value()) {
    return tonic::ToDart(out_error.value());
  }

  std::string error;
  auto res = flutter::gpu::ShaderLibrary::MakeFromAsset(
      impeller_context->GetBackendType(), tonic::StdStringFromDart(asset_name),
      error);
  if (!res) {
    return tonic::ToDart(error);
  }
  res->AssociateWithDartWrapper(wrapper);
  return Dart_Null();
}

Dart_Handle InternalFlutterGpu_ShaderLibrary_GetShader(
    flutter::gpu::ShaderLibrary* wrapper,
    Dart_Handle shader_name,
    Dart_Handle shader_wrapper) {
  FML_DCHECK(Dart_IsString(shader_name));
  auto shader =
      wrapper->GetShader(tonic::StdStringFromDart(shader_name), shader_wrapper);
  if (!shader) {
    return Dart_Null();
  }
  return tonic::ToDart(shader.get());
}
