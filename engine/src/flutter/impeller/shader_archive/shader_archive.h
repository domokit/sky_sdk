// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_SHADER_ARCHIVE_SHADER_ARCHIVE_H_
#define FLUTTER_IMPELLER_SHADER_ARCHIVE_SHADER_ARCHIVE_H_

#include <memory>
#include <type_traits>
#include <unordered_map>

#include "flutter/fml/hash_combine.h"
#include "flutter/fml/mapping.h"
#include "impeller/shader_archive/shader_archive_types.h"

namespace impeller {

class ShaderArchive {
 public:
  explicit ShaderArchive(std::shared_ptr<fml::Mapping> payload);

  ShaderArchive(ShaderArchive&&);

  ~ShaderArchive();

  bool IsValid() const;

  size_t GetShaderCount() const;

  std::shared_ptr<fml::Mapping> GetMapping(ArchiveShaderType type,
                                           std::string name) const;

  size_t IterateAllShaders(
      const std::function<bool(ArchiveShaderType type,
                               const std::string& name,
                               const std::shared_ptr<fml::Mapping>& mapping)>&)
      const;

 private:
  struct ShaderKey {
    ArchiveShaderType type = ArchiveShaderType::kFragment;
    std::string name;

    struct Hash {
      size_t operator()(const ShaderKey& key) const {
        return fml::HashCombine(
            static_cast<std::underlying_type_t<decltype(key.type)>>(key.type),
            key.name);
      }
    };

    struct Equal {
      bool operator()(const ShaderKey& lhs, const ShaderKey& rhs) const {
        return lhs.type == rhs.type && lhs.name == rhs.name;
      }
    };
  };

  using Shaders = std::unordered_map<ShaderKey,
                                     std::shared_ptr<fml::Mapping>,
                                     ShaderKey::Hash,
                                     ShaderKey::Equal>;

  std::shared_ptr<fml::Mapping> payload_;
  Shaders shaders_;
  bool is_valid_ = false;

  ShaderArchive(const ShaderArchive&) = delete;

  ShaderArchive& operator=(const ShaderArchive&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_SHADER_ARCHIVE_SHADER_ARCHIVE_H_
