// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/compiler/compiler_test.h"
#include "flutter/fml/paths.h"
#include "flutter/fml/process.h"

#include <algorithm>
#include <filesystem>

namespace impeller {
namespace compiler {
namespace testing {

static std::string GetIntermediatesPath() {
  auto test_name = flutter::testing::GetCurrentTestName();
  std::replace(test_name.begin(), test_name.end(), '/', '_');
  std::replace(test_name.begin(), test_name.end(), '.', '_');
  std::stringstream dir_name;
  dir_name << test_name << "_" << std::to_string(fml::GetCurrentProcId());
  return fml::paths::JoinPaths(
      {flutter::testing::GetFixturesPath(), dir_name.str()});
}

CompilerTest::CompilerTest() : intermediates_path_(GetIntermediatesPath()) {
  intermediates_directory_ =
      fml::OpenDirectory(intermediates_path_.c_str(),
                         true,  // create if necessary
                         fml::FilePermission::kReadWrite);
  FML_CHECK(intermediates_directory_.is_valid());
}

CompilerTest::~CompilerTest() {
  intermediates_directory_.reset();

  std::filesystem::remove_all(std::filesystem::path(intermediates_path_));
}

static std::string ReflectionHeaderName(const char* fixture_name) {
  std::stringstream stream;
  stream << fixture_name << ".h";
  return stream.str();
}

static std::string ReflectionCCName(const char* fixture_name) {
  std::stringstream stream;
  stream << fixture_name << ".cc";
  return stream.str();
}

static std::string ReflectionJSONName(const char* fixture_name) {
  std::stringstream stream;
  stream << fixture_name << ".json";
  return stream.str();
}

static std::string SPIRVFileName(const char* fixture_name) {
  std::stringstream stream;
  stream << fixture_name << ".spv";
  return stream.str();
}

static std::string SLFileName(const char* fixture_name,
                              TargetPlatform platform) {
  std::stringstream stream;
  stream << fixture_name << "." << TargetPlatformSLExtension(platform);
  return stream.str();
}

std::unique_ptr<fml::FileMapping> CompilerTest::GetReflectionJson(
    const char* fixture_name) const {
  auto filename = ReflectionJSONName(fixture_name);
  auto fd = fml::OpenFileReadOnly(intermediates_directory_, filename.c_str());
  return fml::FileMapping::CreateReadOnly(fd);
}

std::unique_ptr<fml::FileMapping> CompilerTest::GetShaderFile(
    const char* fixture_name,
    TargetPlatform platform) const {
  auto filename = SLFileName(fixture_name, platform);
  auto fd = fml::OpenFileReadOnly(intermediates_directory_, filename.c_str());
  return fml::FileMapping::CreateReadOnly(fd);
}

bool CompilerTest::CanCompileAndReflect(const char* fixture_name,
                                        SourceType source_type,
                                        SourceLanguage source_language,
                                        const char* entry_point_name) const {
  std::shared_ptr<fml::Mapping> fixture =
      flutter::testing::OpenFixtureAsMapping(fixture_name);
  if (!fixture || !fixture->GetMapping()) {
    VALIDATION_LOG << "Could not find shader in fixtures: " << fixture_name;
    return false;
  }

  SourceOptions source_options(fixture_name, source_type);
  source_options.target_platform = GetParam();
  source_options.source_language = source_language;
  source_options.working_directory = std::make_shared<fml::UniqueFD>(
      flutter::testing::OpenFixturesDirectory());
  source_options.entry_point_name = EntryPointFunctionNameFromSourceName(
      fixture_name, SourceTypeFromFileName(fixture_name), source_language,
      entry_point_name);

  Reflector::Options reflector_options;
  reflector_options.header_file_name = ReflectionHeaderName(fixture_name);
  reflector_options.shader_name = "shader_name";

  Compiler compiler(fixture, source_options, reflector_options);
  if (!compiler.IsValid()) {
    VALIDATION_LOG << "Compilation failed: " << compiler.GetErrorMessages();
    return false;
  }

  auto spirv_assembly = compiler.GetSPIRVAssembly();
  if (!spirv_assembly) {
    VALIDATION_LOG << "No spirv was generated.";
    return false;
  }

  if (!fml::WriteAtomically(intermediates_directory_,
                            SPIRVFileName(fixture_name).c_str(),
                            *spirv_assembly)) {
    VALIDATION_LOG << "Could not write SPIRV intermediates.";
    return false;
  }

  auto sl_source = compiler.GetSLShaderSource();
  if (!sl_source) {
    VALIDATION_LOG << "No SL source was generated.";
    return false;
  }

  if (!fml::WriteAtomically(intermediates_directory_,
                            SLFileName(fixture_name, GetParam()).c_str(),
                            *sl_source)) {
    VALIDATION_LOG << "Could not write SL intermediates.";
    return false;
  }

  if (TargetPlatformNeedsReflection(GetParam())) {
    auto reflector = compiler.GetReflector();
    if (!reflector) {
      VALIDATION_LOG
          << "No reflector was found for target platform SL compiler.";
      return false;
    }

    auto reflection_json = reflector->GetReflectionJSON();
    auto reflection_header = reflector->GetReflectionHeader();
    auto reflection_source = reflector->GetReflectionCC();

    if (!reflection_json) {
      VALIDATION_LOG << "Reflection JSON was not found.";
      return false;
    }

    if (!reflection_header) {
      VALIDATION_LOG << "Reflection header was not found.";
      return false;
    }

    if (!reflection_source) {
      VALIDATION_LOG << "Reflection source was not found.";
      return false;
    }

    if (!fml::WriteAtomically(intermediates_directory_,
                              ReflectionHeaderName(fixture_name).c_str(),
                              *reflection_header)) {
      VALIDATION_LOG << "Could not write reflection header intermediates.";
      return false;
    }

    if (!fml::WriteAtomically(intermediates_directory_,
                              ReflectionCCName(fixture_name).c_str(),
                              *reflection_source)) {
      VALIDATION_LOG << "Could not write reflection CC intermediates.";
      return false;
    }

    if (!fml::WriteAtomically(intermediates_directory_,
                              ReflectionJSONName(fixture_name).c_str(),
                              *reflection_json)) {
      VALIDATION_LOG << "Could not write reflection json intermediates.";
      return false;
    }
  }
  return true;
}

}  // namespace testing
}  // namespace compiler
}  // namespace impeller
