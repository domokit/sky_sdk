// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/shader_library.h"

namespace impeller {

ShaderLibrary::ShaderLibrary() = default;

ShaderLibrary::~ShaderLibrary() = default;

void ShaderLibrary::RegisterFunction(
    std::string name,  // NOLINT(performance-unnecessary-value-param)
    ShaderStage stage,
    std::shared_ptr<fml::Mapping>
        code,  // NOLINT(performance-unnecessary-value-param)
    RegistrationCallback
        callback) {  // NOLINT(performance-unnecessary-value-param)
  if (callback) {
    callback(false);
  }
}

}  // namespace impeller
