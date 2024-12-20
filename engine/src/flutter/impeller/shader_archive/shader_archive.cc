// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/shader_archive/shader_archive.h"

#include <array>
#include <string>
#include <utility>

#include "impeller/base/validation.h"
#include "impeller/shader_archive/shader_archive_flatbuffers.h"

namespace impeller {

constexpr ArchiveShaderType ToShaderType(fb::Stage stage) {
  switch (stage) {
    case fb::Stage::kVertex:
      return ArchiveShaderType::kVertex;
    case fb::Stage::kFragment:
      return ArchiveShaderType::kFragment;
    case fb::Stage::kCompute:
      return ArchiveShaderType::kCompute;
  }
  FML_UNREACHABLE();
}

ShaderArchive::ShaderArchive(std::shared_ptr<fml::Mapping> payload)
    : payload_(std::move(payload)) {
  if (!payload_ || payload_->GetMapping() == nullptr) {
    VALIDATION_LOG << "Shader mapping was absent.";
    return;
  }

  if (!fb::ShaderArchiveBufferHasIdentifier(payload_->GetMapping())) {
    VALIDATION_LOG << "Invalid shader magic.";
    return;
  }

  auto shader_archive = fb::GetShaderArchive(payload_->GetMapping());
  if (!shader_archive) {
    return;
  }

  if (auto items = shader_archive->items()) {
    for (auto i = items->begin(), end = items->end(); i != end; i++) {
      ShaderKey key;
      key.name = i->name()->str();
      key.type = ToShaderType(i->stage());
      shaders_[key] = std::make_shared<fml::NonOwnedMapping>(
          i->mapping()->Data(), i->mapping()->size(),
          [payload = payload_](auto, auto) {
            // The pointers are into the base payload. Instead of copying the
            // data, just hold onto the payload.
          });
    }
  }

  is_valid_ = true;
}

ShaderArchive::ShaderArchive(ShaderArchive&&) = default;

ShaderArchive::~ShaderArchive() = default;

bool ShaderArchive::IsValid() const {
  return is_valid_;
}

size_t ShaderArchive::GetShaderCount() const {
  return shaders_.size();
}

std::shared_ptr<fml::Mapping> ShaderArchive::GetMapping(
    ArchiveShaderType type,
    std::string name) const {
  ShaderKey key;
  key.type = type;
  key.name = std::move(name);
  auto found = shaders_.find(key);
  return found == shaders_.end() ? nullptr : found->second;
}

size_t ShaderArchive::IterateAllShaders(
    const std::function<bool(ArchiveShaderType type,
                             const std::string& name,
                             const std::shared_ptr<fml::Mapping>& mapping)>&
        callback) const {
  if (!IsValid() || !callback) {
    return 0u;
  }
  size_t count = 0u;
  for (const auto& shader : shaders_) {
    count++;
    if (!callback(shader.first.type, shader.first.name, shader.second)) {
      break;
    }
  }
  return count;
}

}  // namespace impeller
