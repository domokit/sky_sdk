// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <filesystem>
#include <iostream>

#include "flutter/fml/command_line.h"
#include "impeller/shader_archive/shader_archive_writer.h"

namespace impeller {

bool Main(const fml::CommandLine& command_line) {
  ShaderArchiveWriter writer;

  std::string output;
  if (!command_line.GetOptionValue("output", &output)) {
    std::cerr << "Output path not specified." << std::endl;
    return false;
  }

  for (const auto& input : command_line.GetOptionValues("input")) {
    if (!writer.AddShaderAtPath(std::string{input})) {
      std::cerr << "Could not add shader at path: " << input << std::endl;
      return false;
    }
  }

  auto archive = writer.CreateMapping();
  if (!archive) {
    std::cerr << "Could not create shader archive." << std::endl;
    return false;
  }

  auto current_directory =
      fml::OpenDirectory(std::filesystem::current_path().string().c_str(),
                         false, fml::FilePermission::kReadWrite);
  auto output_path =
      std::filesystem::absolute(std::filesystem::current_path() / output);
  if (!fml::WriteAtomically(current_directory, output_path.string().c_str(),
                            *archive)) {
    std::cerr << "Could not write shader archive to path " << output
              << std::endl;
    return false;
  }

  return true;
}

}  // namespace impeller

int main(int argc, char const* argv[]) {
  return impeller::Main(fml::CommandLineFromPlatformOrArgcArgv(argc, argv))
             ? EXIT_SUCCESS
             : EXIT_FAILURE;
}
