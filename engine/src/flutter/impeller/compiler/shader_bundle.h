// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_COMPILER_SHADER_BUNDLE_H_
#define FLUTTER_IMPELLER_COMPILER_SHADER_BUNDLE_H_

#include "impeller/compiler/source_options.h"
#include "impeller/compiler/switches.h"
#include "impeller/shader_bundle/shader_bundle_flatbuffers.h"

namespace impeller {
namespace compiler {

/// @brief  Parse a shader bundle configuration from a given JSON string.
///
/// @note   Exposed only for testing purposes. Use `GenerateShaderBundle`
///         directly.
std::optional<ShaderBundleConfig> ParseShaderBundleConfig(
    const std::string& bundle_config_json,
    std::ostream& error_stream);

/// @brief  Parses the JSON shader bundle configuration and invokes the
///         compiler multiple times to produce a shader bundle flatbuffer.
///
/// @note   Exposed only for testing purposes. Use `GenerateShaderBundle`
///         directly.
std::optional<fb::shaderbundle::ShaderBundleT> GenerateShaderBundleFlatbuffer(
    const std::string& bundle_config_json,
    const SourceOptions& options);

/// @brief  Parses the JSON shader bundle configuration and invokes the
///         compiler multiple times to produce a shader bundle flatbuffer, which
///         is then output to the `sl` file.
bool GenerateShaderBundle(Switches& switches);

}  // namespace compiler
}  // namespace impeller

#endif  // FLUTTER_IMPELLER_COMPILER_SHADER_BUNDLE_H_
