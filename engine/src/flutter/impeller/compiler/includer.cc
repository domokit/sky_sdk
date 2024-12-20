// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/compiler/includer.h"

#include <cstring>

#include "flutter/fml/paths.h"

namespace impeller {
namespace compiler {

Includer::Includer(std::shared_ptr<fml::UniqueFD> working_directory,
                   std::vector<IncludeDir> include_dirs,
                   std::function<void(std::string)> on_file_included)
    : working_directory_(std::move(working_directory)),
      include_dirs_(std::move(include_dirs)),
      on_file_included_(std::move(on_file_included)) {}

// |shaderc::CompileOptions::IncluderInterface|
Includer::~Includer() = default;

std::unique_ptr<fml::FileMapping> Includer::TryOpenMapping(
    const IncludeDir& dir,
    const char* requested_source) {
  if (!dir.dir || !dir.dir->is_valid()) {
    return nullptr;
  }

  if (requested_source == nullptr) {
    return nullptr;
  }

  std::string source(requested_source);
  if (source.empty()) {
    return nullptr;
  }

  auto mapping = fml::FileMapping::CreateReadOnly(*dir.dir, requested_source);
  if (!mapping || !mapping->IsValid()) {
    return nullptr;
  }

  on_file_included_(fml::paths::JoinPaths({dir.name, requested_source}));

  return mapping;
}

std::unique_ptr<fml::FileMapping> Includer::FindFirstMapping(
    const char* requested_source) {
  // Always try the working directory first no matter what the include
  // directories are.
  {
    IncludeDir dir;
    dir.name = ".";
    dir.dir = working_directory_;
    if (auto mapping = TryOpenMapping(dir, requested_source)) {
      return mapping;
    }
  }

  for (const auto& include_dir : include_dirs_) {
    if (auto mapping = TryOpenMapping(include_dir, requested_source)) {
      return mapping;
    }
  }
  return nullptr;
}

shaderc_include_result* Includer::GetInclude(const char* requested_source,
                                             shaderc_include_type type,
                                             const char* requesting_source,
                                             size_t include_depth) {
  auto result = std::make_unique<shaderc_include_result>();

  // Default initialize to failed inclusion.
  result->source_name = "";
  result->source_name_length = 0;

  constexpr const char* kFileNotFoundMessage = "Included file not found.";
  result->content = kFileNotFoundMessage;
  result->content_length = ::strlen(kFileNotFoundMessage);
  result->user_data = nullptr;

  if (!working_directory_ || !working_directory_->is_valid()) {
    return result.release();
  }

  if (requested_source == nullptr) {
    return result.release();
  }

  auto file = FindFirstMapping(requested_source);

  if (!file || file->GetMapping() == nullptr) {
    return result.release();
  }

  auto includer_data =
      std::make_unique<IncluderData>(requested_source, std::move(file));

  result->source_name = includer_data->file_name.c_str();
  result->source_name_length = includer_data->file_name.length();
  result->content = reinterpret_cast<decltype(result->content)>(
      includer_data->mapping->GetMapping());
  result->content_length = includer_data->mapping->GetSize();
  result->user_data = includer_data.release();

  return result.release();
}

void Includer::ReleaseInclude(shaderc_include_result* data) {
  delete reinterpret_cast<IncluderData*>(data->user_data);
  delete data;
}

}  // namespace compiler
}  // namespace impeller
