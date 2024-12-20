// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_COMPILER_SPIRV_COMPILER_H_
#define FLUTTER_IMPELLER_COMPILER_SPIRV_COMPILER_H_

#include <cstdint>
#include <vector>

#include "flutter/fml/mapping.h"
#include "impeller/compiler/includer.h"
#include "impeller/compiler/source_options.h"
#include "shaderc/shaderc.hpp"

namespace impeller {
namespace compiler {

struct SPIRVCompilerSourceProfile {
  shaderc_profile profile = shaderc_profile_core;
  uint32_t version = 460;
};

struct SPIRVCompilerTargetEnv {
  shaderc_target_env env = shaderc_target_env::shaderc_target_env_vulkan;
  shaderc_env_version version =
      shaderc_env_version::shaderc_env_version_vulkan_1_1;
  shaderc_spirv_version spirv_version =
      shaderc_spirv_version::shaderc_spirv_version_1_3;
};

struct SPIRVCompilerOptions {
  bool generate_debug_info = true;
  //----------------------------------------------------------------------------
  // Source Options.
  //----------------------------------------------------------------------------
  std::optional<shaderc_source_language> source_langauge;
  std::optional<SPIRVCompilerSourceProfile> source_profile;

  shaderc_optimization_level optimization_level =
      shaderc_optimization_level::shaderc_optimization_level_performance;

  //----------------------------------------------------------------------------
  // Target Options.
  //----------------------------------------------------------------------------
  std::optional<SPIRVCompilerTargetEnv> target;

  std::vector<std::string> macro_definitions;

  std::shared_ptr<Includer> includer;

  bool relaxed_vulkan_rules = false;

  shaderc::CompileOptions BuildShadercOptions() const;
};

class SPIRVCompiler {
 public:
  SPIRVCompiler(const SourceOptions& options,
                std::shared_ptr<const fml::Mapping> sources);

  ~SPIRVCompiler();

  std::shared_ptr<fml::Mapping> CompileToSPV(
      std::stringstream& error_stream,
      const shaderc::CompileOptions& spirv_options) const;

 private:
  SourceOptions options_;
  const std::shared_ptr<const fml::Mapping> sources_;

  std::string GetSourcePrefix() const;

  SPIRVCompiler(const SPIRVCompiler&) = delete;

  SPIRVCompiler& operator=(const SPIRVCompiler&) = delete;
};

}  // namespace compiler
}  // namespace impeller

#endif  // FLUTTER_IMPELLER_COMPILER_SPIRV_COMPILER_H_
