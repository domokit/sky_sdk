// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_COMPILER_SHADER_BUNDLE_DATA_H_
#define FLUTTER_IMPELLER_COMPILER_SHADER_BUNDLE_DATA_H_

#include <memory>
#include <vector>

#include "flutter/fml/mapping.h"
#include "impeller/compiler/types.h"
#include "impeller/shader_bundle/shader_bundle_flatbuffers.h"

namespace impeller {
namespace compiler {

class ShaderBundleData {
 public:
  struct ShaderUniformStructField {
    std::string name;
    spirv_cross::SPIRType::BaseType type =
        spirv_cross::SPIRType::BaseType::Float;
    size_t offset_in_bytes = 0u;
    size_t element_size_in_bytes = 0u;
    size_t total_size_in_bytes = 0u;
    std::optional<size_t> array_elements = std::nullopt;
  };

  struct ShaderUniformStruct {
    std::string name;
    size_t ext_res_0 = 0u;
    size_t set = 0u;
    size_t binding = 0u;
    size_t size_in_bytes = 0u;
    std::vector<ShaderUniformStructField> fields;
  };

  struct ShaderUniformTexture {
    std::string name;
    size_t ext_res_0 = 0u;
    size_t set = 0u;
    size_t binding = 0u;
  };

  ShaderBundleData(std::string entrypoint,
                   spv::ExecutionModel stage,
                   TargetPlatform target_platform);

  ~ShaderBundleData();

  void AddUniformStruct(ShaderUniformStruct uniform_struct);

  void AddUniformTexture(ShaderUniformTexture uniform_texture);

  void AddInputDescription(InputDescription input);

  void SetShaderData(std::shared_ptr<fml::Mapping> shader);

  void SetSkSLData(std::shared_ptr<fml::Mapping> sksl);

  std::unique_ptr<fb::shaderbundle::BackendShaderT> CreateFlatbuffer() const;

 private:
  const std::string entrypoint_;
  const spv::ExecutionModel stage_;
  const TargetPlatform target_platform_;
  std::vector<ShaderUniformStruct> uniform_structs_;
  std::vector<ShaderUniformTexture> uniform_textures_;
  std::vector<InputDescription> inputs_;
  std::shared_ptr<fml::Mapping> shader_;
  std::shared_ptr<fml::Mapping> sksl_;

  ShaderBundleData(const ShaderBundleData&) = delete;

  ShaderBundleData& operator=(const ShaderBundleData&) = delete;
};

}  // namespace compiler
}  // namespace impeller

#endif  // FLUTTER_IMPELLER_COMPILER_SHADER_BUNDLE_DATA_H_
