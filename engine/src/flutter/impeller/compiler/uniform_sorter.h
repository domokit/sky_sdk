// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_COMPILER_UNIFORM_SORTER_H_
#define FLUTTER_IMPELLER_COMPILER_UNIFORM_SORTER_H_

#include <optional>

#include "impeller/compiler/compiler_backend.h"

#include "spirv_msl.hpp"
#include "spirv_parser.hpp"

namespace impeller {

/// @brief Sorts uniform declarations in an IR according to decoration order.
///
/// The [type_filter] may be optionally supplied to limit which types are
/// returned The [include] value can be set to false change this filter to
/// exclude instead of include.
std::vector<spirv_cross::ID> SortUniforms(
    const spirv_cross::ParsedIR* ir,
    const spirv_cross::Compiler* compiler,
    std::optional<spirv_cross::SPIRType::BaseType> type_filter = std::nullopt,
    bool include = true);

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_COMPILER_UNIFORM_SORTER_H_
