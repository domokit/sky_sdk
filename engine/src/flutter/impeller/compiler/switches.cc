// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/compiler/switches.h"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <map>

#include "flutter/fml/file.h"
#include "fml/command_line.h"
#include "impeller/compiler/types.h"
#include "impeller/compiler/utilities.h"

namespace impeller {
namespace compiler {

static const std::map<std::string, TargetPlatform> kKnownPlatforms = {
    {"metal-desktop", TargetPlatform::kMetalDesktop},
    {"metal-ios", TargetPlatform::kMetalIOS},
    {"vulkan", TargetPlatform::kVulkan},
    {"opengl-es", TargetPlatform::kOpenGLES},
    {"opengl-desktop", TargetPlatform::kOpenGLDesktop},
};

static const std::map<std::string, TargetPlatform> kKnownRuntimeStages = {
    {"sksl", TargetPlatform::kSkSL},
    {"runtime-stage-metal", TargetPlatform::kRuntimeStageMetal},
    {"runtime-stage-gles", TargetPlatform::kRuntimeStageGLES},
    {"runtime-stage-gles3", TargetPlatform::kRuntimeStageGLES3},
    {"runtime-stage-vulkan", TargetPlatform::kRuntimeStageVulkan},
};

static const std::map<std::string, SourceType> kKnownSourceTypes = {
    {"vert", SourceType::kVertexShader},
    {"frag", SourceType::kFragmentShader},
    {"comp", SourceType::kComputeShader},
};

void Switches::PrintHelp(std::ostream& stream) {
  // clang-format off
  const std::string optional_prefix =          "[optional]          ";
  const std::string optional_multiple_prefix = "[optional,multiple] ";
  // clang-format on

  stream << std::endl;
  stream << "ImpellerC is an offline shader processor and reflection engine."
         << std::endl;
  stream << "---------------------------------------------------------------"
         << std::endl;
  stream << "Expected invocation is:" << std::endl << std::endl;
  stream << "./impellerc <One platform or multiple runtime stages> "
            "--input=<source_file> --sl=<sl_output_file> <optional arguments>"
         << std::endl
         << std::endl;

  stream << "Valid platforms are:" << std::endl << std::endl;
  stream << "One of [";
  for (const auto& platform : kKnownPlatforms) {
    stream << " --" << platform.first;
  }
  stream << " ]" << std::endl << std::endl;

  stream << "Valid runtime stages are:" << std::endl << std::endl;
  stream << "At least one of [";
  for (const auto& platform : kKnownRuntimeStages) {
    stream << " --" << platform.first;
  }
  stream << " ]" << std::endl << std::endl;

  stream << "Optional arguments:" << std::endl << std::endl;
  stream << optional_prefix
         << "--spirv=<spirv_output_file> (ignored for --shader-bundle)"
         << std::endl;
  stream << optional_prefix << "--input-type={";
  for (const auto& source_type : kKnownSourceTypes) {
    stream << source_type.first << ", ";
  }
  stream << "}" << std::endl;
  stream << optional_prefix << "--source-language=glsl|hlsl (default: glsl)"
         << std::endl;
  stream << optional_prefix
         << "--entry-point=<entry_point_name> (default: main; "
            "ignored for glsl)"
         << std::endl;
  stream << optional_prefix
         << "--iplr (causes --sl file to be emitted in "
            "iplr format)"
         << std::endl;
  stream << optional_prefix
         << "--shader-bundle=<bundle_spec> (causes --sl "
            "file to be "
            "emitted in Flutter GPU's shader bundle format)"
         << std::endl;
  stream << optional_prefix << "--reflection-json=<reflection_json_file>"
         << std::endl;
  stream << optional_prefix << "--reflection-header=<reflection_header_file>"
         << std::endl;
  stream << optional_prefix << "--reflection-cc=<reflection_cc_file>"
         << std::endl;
  stream << optional_multiple_prefix << "--include=<include_directory>"
         << std::endl;
  stream << optional_multiple_prefix << "--define=<define>" << std::endl;
  stream << optional_prefix << "--depfile=<depfile_path>" << std::endl;
  stream << optional_prefix << "--gles-language-version=<number>" << std::endl;
  stream << optional_prefix << "--json" << std::endl;
  stream << optional_prefix
         << "--use-half-textures (force openGL semantics when "
            "targeting metal)"
         << std::endl;
  stream << optional_prefix << "--require-framebuffer-fetch" << std::endl;
}

Switches::Switches() = default;

Switches::~Switches() = default;

static TargetPlatform TargetPlatformFromCommandLine(
    const fml::CommandLine& command_line) {
  auto target = TargetPlatform::kUnknown;
  for (const auto& platform : kKnownPlatforms) {
    if (command_line.HasOption(platform.first)) {
      // If the platform has already been determined, the caller may have
      // specified multiple platforms. This is an error and only one must be
      // selected.
      if (target != TargetPlatform::kUnknown) {
        return TargetPlatform::kUnknown;
      }
      target = platform.second;
      // Keep going to detect duplicates.
    }
  }
  return target;
}

static std::vector<TargetPlatform> RuntimeStagesFromCommandLine(
    const fml::CommandLine& command_line) {
  std::vector<TargetPlatform> stages;
  for (const auto& platform : kKnownRuntimeStages) {
    if (command_line.HasOption(platform.first)) {
      stages.push_back(platform.second);
    }
  }
  return stages;
}

static SourceType SourceTypeFromCommandLine(
    const fml::CommandLine& command_line) {
  auto source_type_option =
      command_line.GetOptionValueWithDefault("input-type", "");
  auto source_type_search = kKnownSourceTypes.find(source_type_option);
  if (source_type_search == kKnownSourceTypes.end()) {
    return SourceType::kUnknown;
  }
  return source_type_search->second;
}

Switches::Switches(const fml::CommandLine& command_line)
    : working_directory(std::make_shared<fml::UniqueFD>(fml::OpenDirectory(
          Utf8FromPath(std::filesystem::current_path()).c_str(),
          false,  // create if necessary,
          fml::FilePermission::kRead))),
      source_file_name(command_line.GetOptionValueWithDefault("input", "")),
      input_type(SourceTypeFromCommandLine(command_line)),
      sl_file_name(command_line.GetOptionValueWithDefault("sl", "")),
      iplr(command_line.HasOption("iplr")),
      shader_bundle(
          command_line.GetOptionValueWithDefault("shader-bundle", "")),
      spirv_file_name(command_line.GetOptionValueWithDefault("spirv", "")),
      reflection_json_name(
          command_line.GetOptionValueWithDefault("reflection-json", "")),
      reflection_header_name(
          command_line.GetOptionValueWithDefault("reflection-header", "")),
      reflection_cc_name(
          command_line.GetOptionValueWithDefault("reflection-cc", "")),
      depfile_path(command_line.GetOptionValueWithDefault("depfile", "")),
      json_format(command_line.HasOption("json")),
      gles_language_version(
          stoi(command_line.GetOptionValueWithDefault("gles-language-version",
                                                      "0"))),
      metal_version(
          command_line.GetOptionValueWithDefault("metal-version", "1.2")),
      entry_point(
          command_line.GetOptionValueWithDefault("entry-point", "main")),
      use_half_textures(command_line.HasOption("use-half-textures")),
      require_framebuffer_fetch(
          command_line.HasOption("require-framebuffer-fetch")),
      target_platform_(TargetPlatformFromCommandLine(command_line)),
      runtime_stages_(RuntimeStagesFromCommandLine(command_line)) {
  auto language = ToLowerCase(
      command_line.GetOptionValueWithDefault("source-language", "glsl"));

  source_language = ToSourceLanguage(language);

  if (!working_directory || !working_directory->is_valid()) {
    return;
  }

  for (const auto& include_dir_path : command_line.GetOptionValues("include")) {
    if (!include_dir_path.data()) {
      continue;
    }

    // fml::OpenDirectoryReadOnly for Windows doesn't handle relative paths
    // beginning with `../` well, so we build an absolute path.

    // Get the current working directory as a utf8 encoded string.
    // Note that the `include_dir_path` is already utf8 encoded, and so we
    // mustn't attempt to double-convert it to utf8 lest multi-byte characters
    // will become mangled.
    std::filesystem::path include_dir_absolute;
    if (std::filesystem::path(include_dir_path).is_absolute()) {
      include_dir_absolute = std::filesystem::path(include_dir_path);
    } else {
      auto cwd = Utf8FromPath(std::filesystem::current_path());
      include_dir_absolute = std::filesystem::absolute(
          std::filesystem::path(cwd) / include_dir_path);
    }

    auto dir = std::make_shared<fml::UniqueFD>(fml::OpenDirectoryReadOnly(
        *working_directory, include_dir_absolute.string().c_str()));
    if (!dir || !dir->is_valid()) {
      continue;
    }

    IncludeDir dir_entry;
    dir_entry.name = include_dir_path;
    dir_entry.dir = std::move(dir);

    include_directories.emplace_back(std::move(dir_entry));
  }

  for (const auto& define : command_line.GetOptionValues("define")) {
    defines.emplace_back(define);
  }
}

bool Switches::AreValid(std::ostream& explain) const {
  // When producing a shader bundle, all flags related to single shader inputs
  // and outputs such as `--input` and `--spirv-file-name` are ignored. Instead,
  // input files are read from the shader bundle spec and a single flatbuffer
  // containing all compiled shaders and reflection state is output to `--sl`.
  const bool shader_bundle_mode = !shader_bundle.empty();

  bool valid = true;
  if (target_platform_ == TargetPlatform::kUnknown && runtime_stages_.empty() &&
      !shader_bundle_mode) {
    explain << "Either a target platform was not specified, or no runtime "
               "stages were specified."
            << std::endl;
    valid = false;
  }

  if (source_language == SourceLanguage::kUnknown && !shader_bundle_mode) {
    explain << "Invalid source language type." << std::endl;
    valid = false;
  }

  if (!working_directory || !working_directory->is_valid()) {
    explain << "Could not open the working directory: \""
            << Utf8FromPath(std::filesystem::current_path()).c_str() << "\""
            << std::endl;
    valid = false;
  }

  if (source_file_name.empty() && !shader_bundle_mode) {
    explain << "Input file name was empty." << std::endl;
    valid = false;
  }

  if (sl_file_name.empty()) {
    explain << "Target shading language file name was empty." << std::endl;
    valid = false;
  }

  if (spirv_file_name.empty() && !shader_bundle_mode) {
    explain << "Spirv file name was empty." << std::endl;
    valid = false;
  }

  if (iplr && shader_bundle_mode) {
    explain << "--iplr and --shader-bundle flag cannot be specified at the "
               "same time"
            << std::endl;
    valid = false;
  }

  return valid;
}

std::vector<TargetPlatform> Switches::PlatformsToCompile() const {
  if (target_platform_ == TargetPlatform::kUnknown) {
    return runtime_stages_;
  }
  return {target_platform_};
}

TargetPlatform Switches::SelectDefaultTargetPlatform() const {
  if (target_platform_ == TargetPlatform::kUnknown &&
      !runtime_stages_.empty()) {
    return runtime_stages_.front();
  }
  return target_platform_;
}

SourceOptions Switches::CreateSourceOptions(
    std::optional<TargetPlatform> target_platform) const {
  SourceOptions options;
  options.target_platform =
      target_platform.value_or(SelectDefaultTargetPlatform());
  options.source_language = source_language;
  if (input_type == SourceType::kUnknown) {
    options.type = SourceTypeFromFileName(source_file_name);
  } else {
    options.type = input_type;
  }
  options.working_directory = working_directory;
  options.file_name = source_file_name;
  options.include_dirs = include_directories;
  options.defines = defines;
  options.entry_point_name = EntryPointFunctionNameFromSourceName(
      source_file_name, options.type, options.source_language, entry_point);
  options.json_format = json_format;
  options.gles_language_version = gles_language_version;
  options.metal_version = metal_version;
  options.use_half_textures = use_half_textures;
  options.require_framebuffer_fetch = require_framebuffer_fetch;
  return options;
}

}  // namespace compiler
}  // namespace impeller
