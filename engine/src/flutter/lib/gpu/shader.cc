// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/lib/gpu/shader.h"

#include <utility>

#include "flutter/lib/gpu/formats.h"
#include "fml/make_copyable.h"
#include "impeller/core/runtime_types.h"
#include "impeller/renderer/shader_function.h"
#include "impeller/renderer/shader_library.h"
#include "tonic/converter/dart_converter.h"

namespace flutter {
namespace gpu {

const impeller::ShaderStructMemberMetadata*
Shader::UniformBinding::GetMemberMetadata(const std::string& name) const {
  auto result =
      std::find_if(metadata.members.begin(), metadata.members.end(),
                   [&name](const impeller::ShaderStructMemberMetadata& member) {
                     return member.name == name;
                   });
  if (result == metadata.members.end()) {
    return nullptr;
  }
  return &(*result);
}

IMPLEMENT_WRAPPERTYPEINFO(flutter_gpu, Shader);

Shader::Shader() = default;

Shader::~Shader() = default;

fml::RefPtr<Shader> Shader::Make(
    std::string entrypoint,
    impeller::ShaderStage stage,
    std::shared_ptr<fml::Mapping> code_mapping,
    std::vector<impeller::ShaderStageIOSlot> inputs,
    std::vector<impeller::ShaderStageBufferLayout> layouts,
    std::unordered_map<std::string, UniformBinding> uniform_structs,
    std::unordered_map<std::string, TextureBinding> uniform_textures,
    std::vector<impeller::DescriptorSetLayout> descriptor_set_layouts) {
  auto shader = fml::MakeRefCounted<Shader>();
  shader->entrypoint_ = std::move(entrypoint);
  shader->stage_ = stage;
  shader->code_mapping_ = std::move(code_mapping);
  shader->inputs_ = std::move(inputs);
  shader->layouts_ = std::move(layouts);
  shader->uniform_structs_ = std::move(uniform_structs);
  shader->uniform_textures_ = std::move(uniform_textures);
  shader->descriptor_set_layouts_ = std::move(descriptor_set_layouts);
  return shader;
}

std::shared_ptr<const impeller::ShaderFunction> Shader::GetFunctionFromLibrary(
    impeller::ShaderLibrary& library) {
  return library.GetFunction(entrypoint_, stage_);
}

bool Shader::IsRegistered(Context& context) {
  auto& lib = *context.GetContext()->GetShaderLibrary();
  return GetFunctionFromLibrary(lib) != nullptr;
}

bool Shader::RegisterSync(Context& context) {
  if (IsRegistered(context)) {
    return true;  // Already registered.
  }

  auto& lib = *context.GetContext()->GetShaderLibrary();

  std::promise<bool> promise;
  auto future = promise.get_future();
  lib.RegisterFunction(
      entrypoint_, stage_, code_mapping_,
      fml::MakeCopyable([promise = std::move(promise)](bool result) mutable {
        promise.set_value(result);
      }));
  if (!future.get()) {
    return false;  // Registration failed.
  }
  return true;
}

std::shared_ptr<impeller::VertexDescriptor> Shader::CreateVertexDescriptor()
    const {
  auto vertex_descriptor = std::make_shared<impeller::VertexDescriptor>();
  vertex_descriptor->SetStageInputs(inputs_, layouts_);
  return vertex_descriptor;
}

impeller::ShaderStage Shader::GetShaderStage() const {
  return stage_;
}

const std::vector<impeller::DescriptorSetLayout>&
Shader::GetDescriptorSetLayouts() const {
  return descriptor_set_layouts_;
}

const Shader::UniformBinding* Shader::GetUniformStruct(
    const std::string& name) const {
  auto uniform = uniform_structs_.find(name);
  if (uniform == uniform_structs_.end()) {
    return nullptr;
  }
  return &uniform->second;
}

const Shader::TextureBinding* Shader::GetUniformTexture(
    const std::string& name) const {
  auto uniform = uniform_textures_.find(name);
  if (uniform == uniform_textures_.end()) {
    return nullptr;
  }
  return &uniform->second;
}

}  // namespace gpu
}  // namespace flutter

//----------------------------------------------------------------------------
/// Exports
///

int InternalFlutterGpu_Shader_GetUniformStructSize(
    flutter::gpu::Shader* wrapper,
    Dart_Handle struct_name_handle) {
  auto name = tonic::StdStringFromDart(struct_name_handle);
  const auto* uniform = wrapper->GetUniformStruct(name);
  if (uniform == nullptr) {
    return -1;
  }

  return uniform->size_in_bytes;
}

int InternalFlutterGpu_Shader_GetUniformMemberOffset(
    flutter::gpu::Shader* wrapper,
    Dart_Handle struct_name_handle,
    Dart_Handle member_name_handle) {
  auto struct_name = tonic::StdStringFromDart(struct_name_handle);
  const auto* uniform = wrapper->GetUniformStruct(struct_name);
  if (uniform == nullptr) {
    return -1;
  }

  auto member_name = tonic::StdStringFromDart(member_name_handle);
  const auto* member = uniform->GetMemberMetadata(member_name);
  if (member == nullptr) {
    return -1;
  }

  return member->offset;
}
