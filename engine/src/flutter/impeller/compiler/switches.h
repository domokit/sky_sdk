// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_COMPILER_SWITCHES_H_
#define FLUTTER_IMPELLER_COMPILER_SWITCHES_H_

#include <cstdint>
#include <iostream>
#include <memory>

#include "flutter/fml/command_line.h"
#include "flutter/fml/unique_fd.h"
#include "impeller/compiler/include_dir.h"
#include "impeller/compiler/source_options.h"
#include "impeller/compiler/types.h"

namespace impeller {
namespace compiler {

class Switches {
 public:
  std::shared_ptr<fml::UniqueFD> working_directory = nullptr;
  std::vector<IncludeDir> include_directories = {};
  std::string source_file_name = "";
  SourceType input_type = SourceType::kUnknown;
  /// The raw shader file output by the compiler. For --iplr and
  /// --shader-bundle modes, this is used as the filename for the output
  /// flatbuffer output.
  std::string sl_file_name = "";
  bool iplr = false;
  std::string shader_bundle = "";
  std::string spirv_file_name = "";
  std::string reflection_json_name = "";
  std::string reflection_header_name = "";
  std::string reflection_cc_name = "";
  std::string depfile_path = "";
  std::vector<std::string> defines = {};
  bool json_format = false;
  SourceLanguage source_language = SourceLanguage::kUnknown;
  uint32_t gles_language_version = 0;
  std::string metal_version = "";
  std::string entry_point = "";
  bool use_half_textures = false;
  bool require_framebuffer_fetch = false;

  Switches();

  ~Switches();

  explicit Switches(const fml::CommandLine& command_line);

  bool AreValid(std::ostream& explain) const;

  /// A vector containing at least one valid platform.
  std::vector<TargetPlatform> PlatformsToCompile() const;
  TargetPlatform SelectDefaultTargetPlatform() const;

  // Creates source options from these switches for the specified
  // TargetPlatform. Uses SelectDefaultTargetPlatform if not specified.
  SourceOptions CreateSourceOptions(
      std::optional<TargetPlatform> target_platform = std::nullopt) const;

  static void PrintHelp(std::ostream& stream);

 private:
  // Use |SelectDefaultTargetPlatform|.
  TargetPlatform target_platform_ = TargetPlatform::kUnknown;
  // Use |PlatformsToCompile|.
  std::vector<TargetPlatform> runtime_stages_;
};

}  // namespace compiler
}  // namespace impeller

#endif  // FLUTTER_IMPELLER_COMPILER_SWITCHES_H_
