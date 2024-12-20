// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_SHADER_KEY_H_
#define FLUTTER_IMPELLER_RENDERER_SHADER_KEY_H_

#include <memory>
#include <string>
#include <unordered_map>

#include "flutter/fml/hash_combine.h"
#include "impeller/core/shader_types.h"

namespace impeller {

struct ShaderKey {
  std::string name;
  ShaderStage stage = ShaderStage::kUnknown;

  ShaderKey(std::string_view p_name, ShaderStage p_stage)
      : name({p_name.data(), p_name.size()}), stage(p_stage) {}

  struct Hash {
    size_t operator()(const ShaderKey& key) const {
      return fml::HashCombine(key.name, key.stage);
    }
  };

  struct Equal {
    constexpr bool operator()(const ShaderKey& k1, const ShaderKey& k2) const {
      return k1.stage == k2.stage && k1.name == k2.name;
    }
  };
};

class ShaderFunction;

using ShaderFunctionMap =
    std::unordered_map<ShaderKey,
                       std::shared_ptr<const ShaderFunction>,
                       ShaderKey::Hash,
                       ShaderKey::Equal>;

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_SHADER_KEY_H_
