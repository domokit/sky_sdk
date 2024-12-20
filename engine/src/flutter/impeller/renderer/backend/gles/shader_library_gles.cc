// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/backend/gles/shader_library_gles.h"

#include <sstream>

#include "flutter/fml/closure.h"
#include "impeller/base/config.h"
#include "impeller/base/validation.h"
#include "impeller/renderer/backend/gles/shader_function_gles.h"
#include "impeller/shader_archive/shader_archive.h"

namespace impeller {

static ShaderStage ToShaderStage(ArchiveShaderType type) {
  switch (type) {
    case ArchiveShaderType::kVertex:
      return ShaderStage::kVertex;
    case ArchiveShaderType::kFragment:
      return ShaderStage::kFragment;
    case ArchiveShaderType::kCompute:
      return ShaderStage::kCompute;
  }
  FML_UNREACHABLE();
}

static std::string GLESShaderNameToShaderKeyName(const std::string& name,
                                                 ShaderStage stage) {
  std::stringstream stream;
  stream << name;
  switch (stage) {
    case ShaderStage::kUnknown:
      stream << "_unknown_";
      break;
    case ShaderStage::kVertex:
      stream << "_vertex_";
      break;
    case ShaderStage::kFragment:
      stream << "_fragment_";
      break;
    case ShaderStage::kCompute:
      stream << "_compute_";
      break;
  }
  stream << "main";
  return stream.str();
}

ShaderLibraryGLES::ShaderLibraryGLES(
    const std::vector<std::shared_ptr<fml::Mapping>>& shader_libraries) {
  ShaderFunctionMap functions;
  auto iterator = [&functions, library_id = library_id_](auto type,           //
                                                         const auto& name,    //
                                                         const auto& mapping  //
                                                         ) -> bool {
    const auto stage = ToShaderStage(type);
    const auto key_name = GLESShaderNameToShaderKeyName(name, stage);
    functions[ShaderKey{key_name, stage}] = std::shared_ptr<ShaderFunctionGLES>{
        new ShaderFunctionGLES(library_id,  //
                               stage,       //
                               key_name,    //
                               mapping      //
                               )};

    return true;
  };
  for (auto library : shader_libraries) {
    auto blob_library = ShaderArchive{std::move(library)};
    if (!blob_library.IsValid()) {
      VALIDATION_LOG << "Could not construct blob library for shaders.";
      return;
    }
    blob_library.IterateAllShaders(iterator);
  }

  functions_ = functions;
  is_valid_ = true;
}

// |ShaderLibrary|
ShaderLibraryGLES::~ShaderLibraryGLES() = default;

// |ShaderLibrary|
bool ShaderLibraryGLES::IsValid() const {
  return is_valid_;
}

// |ShaderLibrary|
std::shared_ptr<const ShaderFunction> ShaderLibraryGLES::GetFunction(
    std::string_view name,
    ShaderStage stage) {
  ReaderLock lock(functions_mutex_);
  const auto key = ShaderKey{name, stage};
  if (auto found = functions_.find(key); found != functions_.end()) {
    return found->second;
  }
  return nullptr;
}

// |ShaderLibrary|
void ShaderLibraryGLES::RegisterFunction(std::string name,
                                         ShaderStage stage,
                                         std::shared_ptr<fml::Mapping> code,
                                         RegistrationCallback callback) {
  if (!callback) {
    callback = [](auto) {};
  }
  fml::ScopedCleanupClosure auto_fail([callback]() { callback(false); });
  if (name.empty() || stage == ShaderStage::kUnknown || code == nullptr ||
      code->GetMapping() == nullptr) {
    VALIDATION_LOG << "Invalid runtime stage registration.";
    return;
  }
  const auto key = ShaderKey{name, stage};
  WriterLock lock(functions_mutex_);
  if (functions_.count(key) != 0) {
    VALIDATION_LOG << "Runtime stage named " << name
                   << " has already been registered.";
    return;
  }
  functions_[key] = std::shared_ptr<ShaderFunctionGLES>(new ShaderFunctionGLES(
      library_id_,                                 //
      stage,                                       //
      GLESShaderNameToShaderKeyName(name, stage),  //
      code                                         //
      ));
  auto_fail.Release();
  callback(true);
}

// |ShaderLibrary|
void ShaderLibraryGLES::UnregisterFunction(std::string name,
                                           ShaderStage stage) {
  ReaderLock lock(functions_mutex_);

  const auto key = ShaderKey{name, stage};

  auto found = functions_.find(key);
  if (found == functions_.end()) {
    VALIDATION_LOG << "Library function named " << name
                   << " was not found, so it couldn't be unregistered.";
    return;
  }

  functions_.erase(found);

  return;
}

}  // namespace impeller
