// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_COMPUTE_PIPELINE_BUILDER_H_
#define FLUTTER_IMPELLER_RENDERER_COMPUTE_PIPELINE_BUILDER_H_

#include "impeller/base/strings.h"
#include "impeller/base/validation.h"
#include "impeller/renderer/compute_pipeline_descriptor.h"
#include "impeller/renderer/context.h"
#include "impeller/renderer/shader_library.h"

namespace impeller {

//------------------------------------------------------------------------------
/// @brief      An optional (but highly recommended) utility for creating
///             pipelines from reflected shader information.
///
/// @tparam     Compute_Shader   The reflected compute shader information. Found
///                              in a generated header file called
///                              <shader_name>.comp.h.
///
template <class ComputeShader_>
struct ComputePipelineBuilder {
 public:
  using ComputeShader = ComputeShader_;

  //----------------------------------------------------------------------------
  /// @brief      Create a default pipeline descriptor using the combination
  ///             reflected shader information. The descriptor can be configured
  ///             further before a pipeline state object is created using it.
  ///
  /// @param[in]  context  The context
  ///
  /// @return     If the combination of reflected shader information is
  ///             compatible and the requisite functions can be found in the
  ///             context, a pipeline descriptor.
  ///
  static std::optional<ComputePipelineDescriptor> MakeDefaultPipelineDescriptor(
      const Context& context) {
    ComputePipelineDescriptor desc;
    if (InitializePipelineDescriptorDefaults(context, desc)) {
      return {std::move(desc)};
    }
    return std::nullopt;
  }

  [[nodiscard]] static bool InitializePipelineDescriptorDefaults(
      const Context& context,
      ComputePipelineDescriptor& desc) {
    // Setup debug instrumentation.
    desc.SetLabel(SPrintF("%s Pipeline", ComputeShader::kLabel.data()));

    // Resolve pipeline entrypoints.
    {
      auto compute_function = context.GetShaderLibrary()->GetFunction(
          ComputeShader::kEntrypointName, ShaderStage::kCompute);

      if (!compute_function) {
        VALIDATION_LOG << "Could not resolve compute pipeline entrypoint '"
                       << ComputeShader::kEntrypointName
                       << "' for pipeline named '" << ComputeShader::kLabel
                       << "'.";
        return false;
      }

      if (!desc.RegisterDescriptorSetLayouts(
              ComputeShader::kDescriptorSetLayouts)) {
        VALIDATION_LOG << "Could not configure compute descriptor set layout "
                          "for pipeline named '"
                       << ComputeShader::kLabel << "'.";
        return false;
      }

      desc.SetStageEntrypoint(std::move(compute_function));
    }
    return true;
  }
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_COMPUTE_PIPELINE_BUILDER_H_
