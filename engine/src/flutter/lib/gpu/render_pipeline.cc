// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/lib/gpu/render_pipeline.h"

#include "flutter/lib/gpu/shader.h"
#include "impeller/renderer/pipeline_descriptor.h"

namespace flutter {
namespace gpu {

IMPLEMENT_WRAPPERTYPEINFO(flutter_gpu, RenderPipeline);

RenderPipeline::RenderPipeline(
    fml::RefPtr<flutter::gpu::Shader> vertex_shader,
    fml::RefPtr<flutter::gpu::Shader> fragment_shader)
    : vertex_shader_(std::move(vertex_shader)),
      fragment_shader_(std::move(fragment_shader)) {}

void RenderPipeline::BindToPipelineDescriptor(
    impeller::ShaderLibrary& library,
    impeller::PipelineDescriptor& desc) {
  auto vertex_descriptor = vertex_shader_->CreateVertexDescriptor();
  vertex_descriptor->RegisterDescriptorSetLayouts(
      vertex_shader_->GetDescriptorSetLayouts().data(),
      vertex_shader_->GetDescriptorSetLayouts().size());
  vertex_descriptor->RegisterDescriptorSetLayouts(
      fragment_shader_->GetDescriptorSetLayouts().data(),
      fragment_shader_->GetDescriptorSetLayouts().size());
  desc.SetVertexDescriptor(vertex_descriptor);

  desc.AddStageEntrypoint(vertex_shader_->GetFunctionFromLibrary(library));
  desc.AddStageEntrypoint(fragment_shader_->GetFunctionFromLibrary(library));
}

RenderPipeline::~RenderPipeline() = default;

}  // namespace gpu
}  // namespace flutter

//----------------------------------------------------------------------------
/// Exports
///

Dart_Handle InternalFlutterGpu_RenderPipeline_Initialize(
    Dart_Handle wrapper,
    flutter::gpu::Context* gpu_context,
    flutter::gpu::Shader* vertex_shader,
    flutter::gpu::Shader* fragment_shader) {
  // Lazily register the shaders synchronously if they haven't been already.
  vertex_shader->RegisterSync(*gpu_context);
  fragment_shader->RegisterSync(*gpu_context);

  auto res = fml::MakeRefCounted<flutter::gpu::RenderPipeline>(
      fml::RefPtr<flutter::gpu::Shader>(vertex_shader),  //
      fml::RefPtr<flutter::gpu::Shader>(fragment_shader));
  res->AssociateWithDartWrapper(wrapper);

  return Dart_Null();
}
