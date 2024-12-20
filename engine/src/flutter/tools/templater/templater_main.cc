// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// FLUTTER_NOLINT: https://github.com/flutter/flutter/issues/105732

#include <filesystem>
#include <string>
#include <string_view>

#include "flutter/fml/command_line.h"
#include "flutter/fml/file.h"
#include "flutter/fml/logging.h"
#include "flutter/fml/mapping.h"
#include "inja/inja.hpp"

namespace flutter {

bool TemplaterMain(const fml::CommandLine& command_line) {
  std::string input_path;
  std::string output_path;

  if (!command_line.GetOptionValue("templater-input", &input_path)) {
    FML_LOG(ERROR)
        << "Input template path not specified. Use --templater-input.";
    return false;
  }
  if (!command_line.GetOptionValue("templater-output", &output_path)) {
    FML_LOG(ERROR)
        << "Input template path not specified. Use --templater-output.";
    return false;
  }

  auto input = fml::FileMapping::CreateReadOnly(input_path);
  if (!input) {
    FML_LOG(ERROR) << "Could not open input file: " << input_path;
    return false;
  }

  nlohmann::json arguments;
  for (const auto& option : command_line.options()) {
    arguments[option.name] = option.value;
  }
  inja::Environment env;
  auto rendered_template = env.render(
      std::string_view{reinterpret_cast<const char*>(input->GetMapping()),
                       input->GetSize()},
      arguments);
  auto output = fml::NonOwnedMapping{
      reinterpret_cast<const uint8_t*>(rendered_template.data()),
      rendered_template.size()};

  auto current_dir =
      fml::OpenDirectory(std::filesystem::current_path().u8string().c_str(),
                         false, fml::FilePermission::kReadWrite);
  if (!current_dir.is_valid()) {
    FML_LOG(ERROR) << "Could not open current directory.";
    return false;
  }
  if (!fml::WriteAtomically(current_dir, output_path.c_str(), output)) {
    FML_LOG(ERROR) << "Could not write output to path: " << output_path;
    return false;
  }
  return true;
}

}  // namespace flutter

int main(int argc, char const* argv[]) {
  return flutter::TemplaterMain(fml::CommandLineFromArgcArgv(argc, argv))
             ? EXIT_SUCCESS
             : EXIT_FAILURE;
}
