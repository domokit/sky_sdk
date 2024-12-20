// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RUNTIME_STAGE_RUNTIME_STAGE_H_
#define FLUTTER_IMPELLER_RUNTIME_STAGE_RUNTIME_STAGE_H_

#include <map>
#include <memory>
#include <string>

#include "flutter/fml/mapping.h"

#include "flutter/impeller/core/runtime_types.h"
#include "impeller/core/shader_types.h"
#include "runtime_stage_types_flatbuffers.h"

namespace impeller {

class RuntimeStage {
 public:
  static const char* kVulkanUBOName;

  using Map = std::map<RuntimeStageBackend, std::shared_ptr<RuntimeStage>>;
  static Map DecodeRuntimeStages(const std::shared_ptr<fml::Mapping>& payload);

  RuntimeStage(const fb::RuntimeStage* runtime_stage,
               const std::shared_ptr<fml::Mapping>& payload);
  ~RuntimeStage();
  RuntimeStage(RuntimeStage&&);
  RuntimeStage& operator=(RuntimeStage&&);

  bool IsValid() const;

  RuntimeShaderStage GetShaderStage() const;

  const std::vector<RuntimeUniformDescription>& GetUniforms() const;

  const std::vector<DescriptorSetLayout>& GetDescriptorSetLayouts() const;

  const std::string& GetEntrypoint() const;

  const RuntimeUniformDescription* GetUniform(const std::string& name) const;

  const std::shared_ptr<fml::Mapping>& GetCodeMapping() const;

  bool IsDirty() const;

  void SetClean();

 private:
  std::shared_ptr<fml::Mapping> payload_;
  RuntimeShaderStage stage_ = RuntimeShaderStage::kVertex;
  std::string entrypoint_;
  std::shared_ptr<fml::Mapping> code_mapping_;
  std::vector<RuntimeUniformDescription> uniforms_;
  std::vector<DescriptorSetLayout> descriptor_set_layouts_;
  bool is_valid_ = false;
  bool is_dirty_ = true;

  RuntimeStage(const RuntimeStage&) = delete;

  static std::unique_ptr<RuntimeStage> RuntimeStageIfPresent(
      const fb::RuntimeStage* runtime_stage,
      const std::shared_ptr<fml::Mapping>& payload);

  RuntimeStage& operator=(const RuntimeStage&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RUNTIME_STAGE_RUNTIME_STAGE_H_
