// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_SHADER_ARCHIVE_SHADER_ARCHIVE_WRITER_H_
#define FLUTTER_IMPELLER_SHADER_ARCHIVE_SHADER_ARCHIVE_WRITER_H_

#include <memory>
#include <string>
#include <vector>

#include "flutter/fml/mapping.h"
#include "impeller/shader_archive/shader_archive_types.h"

namespace impeller {

class ShaderArchiveWriter {
 public:
  ShaderArchiveWriter();

  ~ShaderArchiveWriter();

  [[nodiscard]] bool AddShaderAtPath(const std::string& path);

  [[nodiscard]] bool AddShader(ArchiveShaderType type,
                               std::string name,
                               std::shared_ptr<fml::Mapping> mapping);

  std::shared_ptr<fml::Mapping> CreateMapping() const;

 private:
  struct ShaderDescription {
    ArchiveShaderType type;
    std::string name;
    std::shared_ptr<fml::Mapping> mapping;
  };

  std::vector<ShaderDescription> shader_descriptions_;

  ShaderArchiveWriter(const ShaderArchiveWriter&) = delete;

  ShaderArchiveWriter& operator=(const ShaderArchiveWriter&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_SHADER_ARCHIVE_SHADER_ARCHIVE_WRITER_H_
