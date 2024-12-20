// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/shader_function.h"

namespace impeller {

ShaderFunction::ShaderFunction(UniqueID parent_library_id,
                               std::string name,
                               ShaderStage stage)
    : parent_library_id_(parent_library_id),
      name_(std::move(name)),
      stage_(stage) {}

ShaderFunction::~ShaderFunction() = default;

ShaderStage ShaderFunction::GetStage() const {
  return stage_;
}

const std::string& ShaderFunction::GetName() const {
  return name_;
}

// |Comparable<ShaderFunction>|
std::size_t ShaderFunction::GetHash() const {
  return fml::HashCombine(parent_library_id_, name_, stage_);
}

// |Comparable<ShaderFunction>|
bool ShaderFunction::IsEqual(const ShaderFunction& other) const {
  return parent_library_id_ == other.parent_library_id_ &&
         name_ == other.name_ && stage_ == other.stage_;
}

}  // namespace impeller
