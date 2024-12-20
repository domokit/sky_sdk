// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_SHADER_STAGE_COMPATIBILITY_CHECKER_H_
#define FLUTTER_IMPELLER_RENDERER_SHADER_STAGE_COMPATIBILITY_CHECKER_H_

#include <cstddef>

#include "impeller/core/shader_types.h"

namespace impeller {

//------------------------------------------------------------------------------
/// @brief      Checks, at C++ compile-time, if the two pipeline stages are
///             compatible.
///
///             Stages may be incompatible if the outputs declared in the vertex
///             stage don't line up with the inputs declared in the fragment
///             stage. Additionally, the types of the inputs and outputs need to
///             be identical. Some drivers like the one on the PowerVR GE8320
///             also have bugs the require the precision qualifier of the stage
///             interfaces to match exactly.
///
///             Not ensuring stage compatibility will cause pipeline creation
///             errors that will only be caught at runtime. In addition to the
///             bugs discovered related to precision qualifier, some errors may
///             only manifest at runtime on some devices.
///
///             This static compile-time C++ check ensures that all the possible
///             runtime errors will be caught at build time.
///
///             There is no runtime overhead to using this class.
///
/// @tparam     VertexShaderT    The vertex shader stage metadata.
/// @tparam     FragmentShaderT  The fragment shader stage metadata.
///
template <typename VertexShaderT, typename FragmentShaderT>
class ShaderStageCompatibilityChecker {
 public:
  static constexpr bool CompileTimeStrEqual(const char* str1,
                                            const char* str2) {
    return *str1 == *str2 &&
           (*str1 == '\0' || CompileTimeStrEqual(str1 + 1, str2 + 1));
  }

  /// Returns `true` if the shader input slots for the fragment shader match the
  /// ones declared as outputs in the vertex shader.
  static constexpr bool Check() {
    constexpr size_t num_outputs = VertexShaderT::kAllShaderStageOutputs.size();
    constexpr size_t num_inputs = FragmentShaderT::kAllShaderStageInputs.size();

    if (num_inputs > num_outputs) {
      return false;
    }

    for (size_t i = 0; i < num_inputs; ++i) {
      const ShaderStageIOSlot* input_slot =
          FragmentShaderT::kAllShaderStageInputs[i];
      for (size_t j = 0; j < num_outputs; ++j) {
        const ShaderStageIOSlot* output_slot =
            VertexShaderT::kAllShaderStageOutputs[j];
        if (input_slot->location == output_slot->location) {
          if (!CompileTimeStrEqual(input_slot->name, output_slot->name) ||
              input_slot->set != output_slot->set ||
              input_slot->binding != output_slot->binding ||
              input_slot->type != output_slot->type ||
              input_slot->bit_width != output_slot->bit_width ||
              input_slot->vec_size != output_slot->vec_size ||
              input_slot->columns != output_slot->columns ||
              input_slot->offset != output_slot->offset ||
              input_slot->relaxed_precision != output_slot->relaxed_precision) {
            return false;
          }
        }
      }
    }

    return true;
  }
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_SHADER_STAGE_COMPATIBILITY_CHECKER_H_
