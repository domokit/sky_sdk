// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/entity/contents/runtime_effect_contents.h"

#include <algorithm>
#include <future>
#include <memory>

#include "flutter/fml/logging.h"
#include "flutter/fml/make_copyable.h"
#include "impeller/base/validation.h"
#include "impeller/core/formats.h"
#include "impeller/core/runtime_types.h"
#include "impeller/core/shader_types.h"
#include "impeller/entity/contents/content_context.h"
#include "impeller/entity/runtime_effect.vert.h"
#include "impeller/renderer/capabilities.h"
#include "impeller/renderer/pipeline_library.h"
#include "impeller/renderer/render_pass.h"
#include "impeller/renderer/shader_function.h"
#include "impeller/renderer/vertex_descriptor.h"

namespace impeller {

namespace {
constexpr char kPaddingType = 0;
constexpr char kFloatType = 1;
}  // namespace

// static
BufferView RuntimeEffectContents::EmplaceVulkanUniform(
    const std::shared_ptr<const std::vector<uint8_t>>& input_data,
    HostBuffer& host_buffer,
    const RuntimeUniformDescription& uniform) {
  // TODO(jonahwilliams): rewrite this to emplace directly into
  // HostBuffer.
  std::vector<float> uniform_buffer;
  uniform_buffer.reserve(uniform.struct_layout.size());
  size_t uniform_byte_index = 0u;
  for (char byte_type : uniform.struct_layout) {
    if (byte_type == kPaddingType) {
      uniform_buffer.push_back(0.f);
    } else {
      FML_DCHECK(byte_type == kFloatType);
      uniform_buffer.push_back(reinterpret_cast<const float*>(
          input_data->data())[uniform_byte_index++]);
    }
  }
  size_t alignment = std::max(sizeof(float) * uniform_buffer.size(),
                              DefaultUniformAlignment());

  return host_buffer.Emplace(
      reinterpret_cast<const void*>(uniform_buffer.data()),
      sizeof(float) * uniform_buffer.size(), alignment);
}

void RuntimeEffectContents::SetRuntimeStage(
    std::shared_ptr<RuntimeStage> runtime_stage) {
  runtime_stage_ = std::move(runtime_stage);
}

void RuntimeEffectContents::SetUniformData(
    std::shared_ptr<std::vector<uint8_t>> uniform_data) {
  uniform_data_ = std::move(uniform_data);
}

void RuntimeEffectContents::SetTextureInputs(
    std::vector<TextureInput> texture_inputs) {
  texture_inputs_ = std::move(texture_inputs);
}

static ShaderType GetShaderType(RuntimeUniformType type) {
  switch (type) {
    case kSampledImage:
      return ShaderType::kSampledImage;
    case kFloat:
      return ShaderType::kFloat;
    case kStruct:
      return ShaderType::kStruct;
  }
}

static std::unique_ptr<ShaderMetadata> MakeShaderMetadata(
    const RuntimeUniformDescription& uniform) {
  std::unique_ptr<ShaderMetadata> metadata = std::make_unique<ShaderMetadata>();
  metadata->name = uniform.name;
  metadata->members.emplace_back(ShaderStructMemberMetadata{
      .type = GetShaderType(uniform.type),
      .size = uniform.GetSize(),
      .byte_length = uniform.bit_width / 8,
  });

  return metadata;
}

bool RuntimeEffectContents::BootstrapShader(
    const ContentContext& renderer) const {
  if (!RegisterShader(renderer)) {
    return false;
  }
  ContentContextOptions options;
  options.color_attachment_pixel_format =
      renderer.GetContext()->GetCapabilities()->GetDefaultColorFormat();
  CreatePipeline(renderer, options, /*async=*/true);
  return true;
}

bool RuntimeEffectContents::RegisterShader(
    const ContentContext& renderer) const {
  const std::shared_ptr<Context>& context = renderer.GetContext();
  const std::shared_ptr<ShaderLibrary>& library = context->GetShaderLibrary();

  std::shared_ptr<const ShaderFunction> function = library->GetFunction(
      runtime_stage_->GetEntrypoint(), ShaderStage::kFragment);

  //--------------------------------------------------------------------------
  /// Resolve runtime stage function.
  ///

  if (function && runtime_stage_->IsDirty()) {
    renderer.ClearCachedRuntimeEffectPipeline(runtime_stage_->GetEntrypoint());
    context->GetPipelineLibrary()->RemovePipelinesWithEntryPoint(function);
    library->UnregisterFunction(runtime_stage_->GetEntrypoint(),
                                ShaderStage::kFragment);

    function = nullptr;
  }

  if (!function) {
    std::promise<bool> promise;
    auto future = promise.get_future();

    library->RegisterFunction(
        runtime_stage_->GetEntrypoint(),
        ToShaderStage(runtime_stage_->GetShaderStage()),
        runtime_stage_->GetCodeMapping(),
        fml::MakeCopyable([promise = std::move(promise)](bool result) mutable {
          promise.set_value(result);
        }));

    if (!future.get()) {
      VALIDATION_LOG << "Failed to build runtime effect (entry point: "
                     << runtime_stage_->GetEntrypoint() << ")";
      return false;
    }

    function = library->GetFunction(runtime_stage_->GetEntrypoint(),
                                    ShaderStage::kFragment);
    if (!function) {
      VALIDATION_LOG
          << "Failed to fetch runtime effect function immediately after "
             "registering it (entry point: "
          << runtime_stage_->GetEntrypoint() << ")";
      return false;
    }

    runtime_stage_->SetClean();
  }
  return true;
}

std::shared_ptr<Pipeline<PipelineDescriptor>>
RuntimeEffectContents::CreatePipeline(const ContentContext& renderer,
                                      ContentContextOptions options,
                                      bool async) const {
  const std::shared_ptr<Context>& context = renderer.GetContext();
  const std::shared_ptr<ShaderLibrary>& library = context->GetShaderLibrary();
  const std::shared_ptr<const Capabilities>& caps = context->GetCapabilities();
  const auto color_attachment_format = caps->GetDefaultColorFormat();
  const auto stencil_attachment_format = caps->GetDefaultDepthStencilFormat();

  using VS = RuntimeEffectVertexShader;

  PipelineDescriptor desc;
  desc.SetLabel("Runtime Stage");
  desc.AddStageEntrypoint(
      library->GetFunction(VS::kEntrypointName, ShaderStage::kVertex));
  desc.AddStageEntrypoint(library->GetFunction(runtime_stage_->GetEntrypoint(),
                                               ShaderStage::kFragment));

  std::shared_ptr<VertexDescriptor> vertex_descriptor =
      std::make_shared<VertexDescriptor>();
  vertex_descriptor->SetStageInputs(VS::kAllShaderStageInputs,
                                    VS::kInterleavedBufferLayout);
  vertex_descriptor->RegisterDescriptorSetLayouts(VS::kDescriptorSetLayouts);
  vertex_descriptor->RegisterDescriptorSetLayouts(
      runtime_stage_->GetDescriptorSetLayouts().data(),
      runtime_stage_->GetDescriptorSetLayouts().size());
  desc.SetVertexDescriptor(std::move(vertex_descriptor));
  desc.SetColorAttachmentDescriptor(
      0u, {.format = color_attachment_format, .blending_enabled = true});

  desc.SetStencilAttachmentDescriptors(StencilAttachmentDescriptor{});
  desc.SetStencilPixelFormat(stencil_attachment_format);

  desc.SetDepthStencilAttachmentDescriptor(DepthAttachmentDescriptor{});
  desc.SetDepthPixelFormat(stencil_attachment_format);

  options.ApplyToPipelineDescriptor(desc);
  if (async) {
    context->GetPipelineLibrary()->GetPipeline(desc, async);
    return nullptr;
  }

  auto pipeline = context->GetPipelineLibrary()->GetPipeline(desc, async).Get();
  if (!pipeline) {
    VALIDATION_LOG << "Failed to get or create runtime effect pipeline.";
    return nullptr;
  }

  return pipeline;
}

bool RuntimeEffectContents::Render(const ContentContext& renderer,
                                   const Entity& entity,
                                   RenderPass& pass) const {
  const std::shared_ptr<Context>& context = renderer.GetContext();
  const std::shared_ptr<ShaderLibrary>& library = context->GetShaderLibrary();

  //--------------------------------------------------------------------------
  /// Get or register shader. Flutter will do this when the runtime effect
  /// is first loaded, but this check is added to supporting testing of the
  /// Aiks API and non-flutter usage of Impeller.
  ///
  if (!RegisterShader(renderer)) {
    return false;
  }

  //--------------------------------------------------------------------------
  /// Fragment stage uniforms.
  ///
  BindFragmentCallback bind_callback = [this, &renderer,
                                        &context](RenderPass& pass) {
    size_t minimum_sampler_index = 100000000;
    size_t buffer_index = 0;
    size_t buffer_offset = 0;

    for (const auto& uniform : runtime_stage_->GetUniforms()) {
      std::unique_ptr<ShaderMetadata> metadata = MakeShaderMetadata(uniform);
      switch (uniform.type) {
        case kSampledImage: {
          // Sampler uniforms are ordered in the IPLR according to their
          // declaration and the uniform location reflects the correct offset to
          // be mapped to - except that it may include all proceeding float
          // uniforms. For example, a float sampler that comes after 4 float
          // uniforms may have a location of 4. To convert to the actual offset
          // we need to find the largest location assigned to a float uniform
          // and then subtract this from all uniform locations. This is more or
          // less the same operation we previously performed in the shader
          // compiler.
          minimum_sampler_index =
              std::min(minimum_sampler_index, uniform.location);
          break;
        }
        case kFloat: {
          FML_DCHECK(renderer.GetContext()->GetBackendType() !=
                     Context::BackendType::kVulkan)
              << "Uniform " << uniform.name
              << " had unexpected type kFloat for Vulkan backend.";

          size_t alignment =
              std::max(uniform.bit_width / 8, DefaultUniformAlignment());
          BufferView buffer_view = renderer.GetTransientsBuffer().Emplace(
              uniform_data_->data() + buffer_offset, uniform.GetSize(),
              alignment);

          ShaderUniformSlot uniform_slot;
          uniform_slot.name = uniform.name.c_str();
          uniform_slot.ext_res_0 = uniform.location;
          pass.BindDynamicResource(ShaderStage::kFragment,
                                   DescriptorType::kUniformBuffer, uniform_slot,
                                   std::move(metadata), std::move(buffer_view));
          buffer_index++;
          buffer_offset += uniform.GetSize();
          break;
        }
        case kStruct: {
          FML_DCHECK(renderer.GetContext()->GetBackendType() ==
                     Context::BackendType::kVulkan);
          ShaderUniformSlot uniform_slot;
          uniform_slot.binding = uniform.location;
          uniform_slot.name = uniform.name.c_str();

          pass.BindResource(
              ShaderStage::kFragment, DescriptorType::kUniformBuffer,
              uniform_slot, nullptr,
              EmplaceVulkanUniform(uniform_data_,
                                   renderer.GetTransientsBuffer(), uniform));
        }
      }
    }

    size_t sampler_index = 0;
    for (const auto& uniform : runtime_stage_->GetUniforms()) {
      std::unique_ptr<ShaderMetadata> metadata = MakeShaderMetadata(uniform);

      switch (uniform.type) {
        case kSampledImage: {
          FML_DCHECK(sampler_index < texture_inputs_.size());
          auto& input = texture_inputs_[sampler_index];

          raw_ptr<const Sampler> sampler =
              context->GetSamplerLibrary()->GetSampler(
                  input.sampler_descriptor);

          SampledImageSlot image_slot;
          image_slot.name = uniform.name.c_str();
          image_slot.binding = uniform.binding;
          image_slot.texture_index = uniform.location - minimum_sampler_index;
          pass.BindDynamicResource(ShaderStage::kFragment,
                                   DescriptorType::kSampledImage, image_slot,
                                   std::move(metadata), input.texture, sampler);

          sampler_index++;
          break;
        }
        default:
          continue;
      }
    }
    return true;
  };

  /// Now that the descriptor set layouts are known, get the pipeline.
  using VS = RuntimeEffectVertexShader;

  PipelineBuilderCallback pipeline_callback =
      [&](ContentContextOptions options) {
        // Pipeline creation callback for the cache handler to call.
        return renderer.GetCachedRuntimeEffectPipeline(
            runtime_stage_->GetEntrypoint(), options, [&]() {
              return CreatePipeline(renderer, options, /*async=*/false);
            });
      };

  return ColorSourceContents::DrawGeometry<VS>(renderer, entity, pass,
                                               pipeline_callback,
                                               VS::FrameInfo{}, bind_callback);
}

}  // namespace impeller
