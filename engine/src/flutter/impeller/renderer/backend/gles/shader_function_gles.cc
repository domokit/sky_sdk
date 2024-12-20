// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/backend/gles/shader_function_gles.h"

namespace impeller {

ShaderFunctionGLES::ShaderFunctionGLES(
    UniqueID library_id,
    ShaderStage stage,
    std::string name,
    std::shared_ptr<const fml::Mapping> mapping)
    : ShaderFunction(library_id, std::move(name), stage),
      mapping_(std::move(mapping)) {}

ShaderFunctionGLES::~ShaderFunctionGLES() = default;

const std::shared_ptr<const fml::Mapping>&
ShaderFunctionGLES::GetSourceMapping() const {
  return mapping_;
}

}  // namespace impeller
