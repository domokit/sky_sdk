// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_COMPILER_RUNTIME_STAGE_DATA_H_
#define FLUTTER_IMPELLER_COMPILER_RUNTIME_STAGE_DATA_H_

#include <memory>
#include <vector>

#include "flutter/fml/mapping.h"
#include "impeller/compiler/types.h"
#include "impeller/core/runtime_types.h"
#include "runtime_stage_types_flatbuffers.h"
#include "spirv_parser.hpp"

namespace impeller {
namespace compiler {

class RuntimeStageData {
 public:
  struct Shader {
    Shader() = default;

    std::string entrypoint;
    spv::ExecutionModel stage;
    std::vector<UniformDescription> uniforms;
    std::vector<InputDescription> inputs;
    std::shared_ptr<fml::Mapping> shader;
    RuntimeStageBackend backend;

    Shader(const Shader&) = delete;
    Shader& operator=(const Shader&) = delete;
  };

  RuntimeStageData();

  ~RuntimeStageData();

  void AddShader(const std::shared_ptr<Shader>& data);

  std::unique_ptr<fb::RuntimeStageT> CreateStageFlatbuffer(
      impeller::RuntimeStageBackend backend) const;

  std::unique_ptr<fb::RuntimeStagesT> CreateMultiStageFlatbuffer() const;

  std::shared_ptr<fml::Mapping> CreateJsonMapping() const;

  std::shared_ptr<fml::Mapping> CreateMapping() const;

 private:
  std::map<RuntimeStageBackend, std::shared_ptr<Shader>> data_;

  RuntimeStageData(const RuntimeStageData&) = delete;

  RuntimeStageData& operator=(const RuntimeStageData&) = delete;
};

}  // namespace compiler
}  // namespace impeller

#endif  // FLUTTER_IMPELLER_COMPILER_RUNTIME_STAGE_DATA_H_
