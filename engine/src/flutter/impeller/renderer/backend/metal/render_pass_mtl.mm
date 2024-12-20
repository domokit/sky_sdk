// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/backend/metal/render_pass_mtl.h"

#include "flutter/fml/closure.h"
#include "flutter/fml/logging.h"
#include "flutter/fml/make_copyable.h"
#include "fml/status.h"

#include "impeller/base/backend_cast.h"
#include "impeller/core/formats.h"
#include "impeller/core/host_buffer.h"
#include "impeller/core/shader_types.h"
#include "impeller/renderer/backend/metal/context_mtl.h"
#include "impeller/renderer/backend/metal/device_buffer_mtl.h"
#include "impeller/renderer/backend/metal/formats_mtl.h"
#include "impeller/renderer/backend/metal/pipeline_mtl.h"
#include "impeller/renderer/backend/metal/sampler_mtl.h"
#include "impeller/renderer/backend/metal/texture_mtl.h"
#include "impeller/renderer/command.h"
#include "impeller/renderer/vertex_descriptor.h"

namespace impeller {

static bool ConfigureResolveTextureAttachment(
    const Attachment& desc,
    MTLRenderPassAttachmentDescriptor* attachment) {
  bool needs_resolve =
      desc.store_action == StoreAction::kMultisampleResolve ||
      desc.store_action == StoreAction::kStoreAndMultisampleResolve;

  if (needs_resolve && !desc.resolve_texture) {
    VALIDATION_LOG << "Resolve store action specified on attachment but no "
                      "resolve texture was specified.";
    return false;
  }

  if (desc.resolve_texture && !needs_resolve) {
    VALIDATION_LOG << "A resolve texture was specified even though the store "
                      "action doesn't require it.";
    return false;
  }

  if (!desc.resolve_texture) {
    return true;
  }

  attachment.resolveTexture =
      TextureMTL::Cast(*desc.resolve_texture).GetMTLTexture();

  return true;
}

static bool ConfigureAttachment(const Attachment& desc,
                                MTLRenderPassAttachmentDescriptor* attachment) {
  if (!desc.texture) {
    return false;
  }

  attachment.texture = TextureMTL::Cast(*desc.texture).GetMTLTexture();
  attachment.loadAction = ToMTLLoadAction(desc.load_action);
  attachment.storeAction = ToMTLStoreAction(desc.store_action);

  if (!ConfigureResolveTextureAttachment(desc, attachment)) {
    return false;
  }

  return true;
}

static bool ConfigureColorAttachment(
    const ColorAttachment& desc,
    MTLRenderPassColorAttachmentDescriptor* attachment) {
  if (!ConfigureAttachment(desc, attachment)) {
    return false;
  }
  attachment.clearColor = ToMTLClearColor(desc.clear_color);
  return true;
}

static bool ConfigureDepthAttachment(
    const DepthAttachment& desc,
    MTLRenderPassDepthAttachmentDescriptor* attachment) {
  if (!ConfigureAttachment(desc, attachment)) {
    return false;
  }
  attachment.clearDepth = desc.clear_depth;
  return true;
}

static bool ConfigureStencilAttachment(
    const StencilAttachment& desc,
    MTLRenderPassStencilAttachmentDescriptor* attachment) {
  if (!ConfigureAttachment(desc, attachment)) {
    return false;
  }
  attachment.clearStencil = desc.clear_stencil;
  return true;
}

static MTLRenderPassDescriptor* ToMTLRenderPassDescriptor(
    const RenderTarget& desc) {
  auto result = [MTLRenderPassDescriptor renderPassDescriptor];

  bool configured_attachment = desc.IterateAllColorAttachments(
      [&result](size_t index, const ColorAttachment& attachment) -> bool {
        return ConfigureColorAttachment(attachment,
                                        result.colorAttachments[index]);
      });

  if (!configured_attachment) {
    VALIDATION_LOG << "Could not configure color attachments";
    return nil;
  }

  const auto& depth = desc.GetDepthAttachment();

  if (depth.has_value() &&
      !ConfigureDepthAttachment(depth.value(), result.depthAttachment)) {
    VALIDATION_LOG << "Could not configure depth attachment.";
    return nil;
  }

  const auto& stencil = desc.GetStencilAttachment();

  if (stencil.has_value() &&
      !ConfigureStencilAttachment(stencil.value(), result.stencilAttachment)) {
    VALIDATION_LOG << "Could not configure stencil attachment.";
    return nil;
  }

  return result;
}

RenderPassMTL::RenderPassMTL(std::shared_ptr<const Context> context,
                             const RenderTarget& target,
                             id<MTLCommandBuffer> buffer)
    : RenderPass(std::move(context), target),
      buffer_(buffer),
      desc_(ToMTLRenderPassDescriptor(GetRenderTarget())) {
  if (!buffer_ || !desc_ || !render_target_.IsValid()) {
    return;
  }
  encoder_ = [buffer_ renderCommandEncoderWithDescriptor:desc_];

  if (!encoder_) {
    return;
  }
#ifdef IMPELLER_DEBUG
  is_metal_trace_active_ =
      [[MTLCaptureManager sharedCaptureManager] isCapturing];
#endif  // IMPELLER_DEBUG
  pass_bindings_.SetEncoder(encoder_);
  pass_bindings_.SetViewport(
      Viewport{.rect = Rect::MakeSize(GetRenderTargetSize())});
  pass_bindings_.SetScissor(IRect::MakeSize(GetRenderTargetSize()));
  is_valid_ = true;
}

RenderPassMTL::~RenderPassMTL() {
  if (!did_finish_encoding_) {
    [encoder_ endEncoding];
    did_finish_encoding_ = true;
  }
}

bool RenderPassMTL::IsValid() const {
  return is_valid_;
}

void RenderPassMTL::OnSetLabel(std::string_view label) {
#ifdef IMPELLER_DEBUG
  if (label.empty()) {
    return;
  }
  encoder_.label = @(std::string(label).c_str());
#endif  // IMPELLER_DEBUG
}

bool RenderPassMTL::OnEncodeCommands(const Context& context) const {
  did_finish_encoding_ = true;
  [encoder_ endEncoding];
  return true;
}

static bool Bind(PassBindingsCacheMTL& pass,
                 ShaderStage stage,
                 size_t bind_index,
                 const BufferView& view) {
  if (!view.GetBuffer()) {
    return false;
  }

  const DeviceBuffer* device_buffer = view.GetBuffer();
  if (!device_buffer) {
    return false;
  }

  auto buffer = DeviceBufferMTL::Cast(*device_buffer).GetMTLBuffer();
  // The Metal call is a void return and we don't want to make it on nil.
  if (!buffer) {
    return false;
  }

  return pass.SetBuffer(stage, bind_index, view.GetRange().offset, buffer);
}

static bool Bind(PassBindingsCacheMTL& pass,
                 ShaderStage stage,
                 size_t bind_index,
                 raw_ptr<const Sampler> sampler,
                 const Texture& texture) {
  if (!sampler || !texture.IsValid()) {
    return false;
  }

  if (texture.NeedsMipmapGeneration()) {
    // TODO(127697): generate mips when the GPU is available on iOS.
#if !FML_OS_IOS
    VALIDATION_LOG
        << "Texture at binding index " << bind_index
        << " has a mip count > 1, but the mipmap has not been generated.";
    return false;
#endif  // !FML_OS_IOS
  }

  return pass.SetTexture(stage, bind_index,
                         TextureMTL::Cast(texture).GetMTLTexture()) &&
         pass.SetSampler(stage, bind_index,
                         SamplerMTL::Cast(*sampler).GetMTLSamplerState());
}

// |RenderPass|
void RenderPassMTL::SetPipeline(PipelineRef pipeline) {
  const PipelineDescriptor& pipeline_desc = pipeline->GetDescriptor();
  primitive_type_ = pipeline_desc.GetPrimitiveType();
  pass_bindings_.SetRenderPipelineState(
      PipelineMTL::Cast(*pipeline).GetMTLRenderPipelineState());
  pass_bindings_.SetDepthStencilState(
      PipelineMTL::Cast(*pipeline).GetMTLDepthStencilState());

  [encoder_ setFrontFacingWinding:pipeline_desc.GetWindingOrder() ==
                                          WindingOrder::kClockwise
                                      ? MTLWindingClockwise
                                      : MTLWindingCounterClockwise];
  [encoder_ setCullMode:ToMTLCullMode(pipeline_desc.GetCullMode())];
  [encoder_ setTriangleFillMode:ToMTLTriangleFillMode(
                                    pipeline_desc.GetPolygonMode())];
  has_valid_pipeline_ = true;
}

// |RenderPass|
void RenderPassMTL::SetCommandLabel(std::string_view label) {
#ifdef IMPELLER_DEBUG
  if (is_metal_trace_active_) {
    has_label_ = true;
    std::string label_copy(label);
    [encoder_ pushDebugGroup:@(label_copy.c_str())];
  }
#endif  // IMPELLER_DEBUG
}

// |RenderPass|
void RenderPassMTL::SetStencilReference(uint32_t value) {
  [encoder_ setStencilReferenceValue:value];
}

// |RenderPass|
void RenderPassMTL::SetBaseVertex(uint64_t value) {
  base_vertex_ = value;
}

// |RenderPass|
void RenderPassMTL::SetViewport(Viewport viewport) {
  pass_bindings_.SetViewport(viewport);
}

// |RenderPass|
void RenderPassMTL::SetScissor(IRect scissor) {
  pass_bindings_.SetScissor(scissor);
}

// |RenderPass|
void RenderPassMTL::SetElementCount(size_t count) {
  vertex_count_ = count;
}

// |RenderPass|
void RenderPassMTL::SetInstanceCount(size_t count) {
  instance_count_ = count;
}

// |RenderPass|
bool RenderPassMTL::SetVertexBuffer(BufferView vertex_buffers[],
                                    size_t vertex_buffer_count) {
  if (!ValidateVertexBuffers(vertex_buffers, vertex_buffer_count)) {
    return false;
  }

  for (size_t i = 0; i < vertex_buffer_count; i++) {
    if (!Bind(pass_bindings_, ShaderStage::kVertex,
              VertexDescriptor::kReservedVertexBufferIndex - i,
              vertex_buffers[i])) {
      return false;
    }
  }

  return true;
}

// |RenderPass|
bool RenderPassMTL::SetIndexBuffer(BufferView index_buffer,
                                   IndexType index_type) {
  if (!ValidateIndexBuffer(index_buffer, index_type)) {
    return false;
  }

  if (index_type != IndexType::kNone) {
    index_type_ = ToMTLIndexType(index_type);
    index_buffer_ = std::move(index_buffer);
  }

  return true;
}

// |RenderPass|
fml::Status RenderPassMTL::Draw() {
  if (!has_valid_pipeline_) {
    return fml::Status(fml::StatusCode::kCancelled, "Invalid pipeline.");
  }

  if (!index_buffer_) {
    if (instance_count_ != 1u) {
      [encoder_ drawPrimitives:ToMTLPrimitiveType(primitive_type_)
                   vertexStart:base_vertex_
                   vertexCount:vertex_count_
                 instanceCount:instance_count_
                  baseInstance:0u];
    } else {
      [encoder_ drawPrimitives:ToMTLPrimitiveType(primitive_type_)
                   vertexStart:base_vertex_
                   vertexCount:vertex_count_];
    }
  } else {
    id<MTLBuffer> mtl_index_buffer =
        DeviceBufferMTL::Cast(*index_buffer_.GetBuffer()).GetMTLBuffer();
    if (instance_count_ != 1u) {
      [encoder_ drawIndexedPrimitives:ToMTLPrimitiveType(primitive_type_)
                           indexCount:vertex_count_
                            indexType:index_type_
                          indexBuffer:mtl_index_buffer
                    indexBufferOffset:index_buffer_.GetRange().offset
                        instanceCount:instance_count_
                           baseVertex:base_vertex_
                         baseInstance:0u];
    } else {
      [encoder_ drawIndexedPrimitives:ToMTLPrimitiveType(primitive_type_)
                           indexCount:vertex_count_
                            indexType:index_type_
                          indexBuffer:mtl_index_buffer
                    indexBufferOffset:index_buffer_.GetRange().offset];
    }
  }

#ifdef IMPELLER_DEBUG
  if (has_label_) {
    [encoder_ popDebugGroup];
  }
#endif  // IMPELLER_DEBUG

  vertex_count_ = 0u;
  base_vertex_ = 0u;
  instance_count_ = 1u;
  index_buffer_ = {};
  has_valid_pipeline_ = false;
  has_label_ = false;

  return fml::Status();
}

// |RenderPass|
bool RenderPassMTL::BindResource(ShaderStage stage,
                                 DescriptorType type,
                                 const ShaderUniformSlot& slot,
                                 const ShaderMetadata* metadata,
                                 BufferView view) {
  return Bind(pass_bindings_, stage, slot.ext_res_0, view);
}

// |RenderPass|
bool RenderPassMTL::BindDynamicResource(
    ShaderStage stage,
    DescriptorType type,
    const ShaderUniformSlot& slot,
    std::unique_ptr<ShaderMetadata> metadata,
    BufferView view) {
  return Bind(pass_bindings_, stage, slot.ext_res_0, view);
}

// |RenderPass|
bool RenderPassMTL::BindResource(ShaderStage stage,
                                 DescriptorType type,
                                 const SampledImageSlot& slot,
                                 const ShaderMetadata* metadata,
                                 std::shared_ptr<const Texture> texture,
                                 raw_ptr<const Sampler> sampler) {
  if (!texture) {
    return false;
  }
  return Bind(pass_bindings_, stage, slot.texture_index, sampler, *texture);
}

bool RenderPassMTL::BindDynamicResource(
    ShaderStage stage,
    DescriptorType type,
    const SampledImageSlot& slot,
    std::unique_ptr<ShaderMetadata> metadata,
    std::shared_ptr<const Texture> texture,
    raw_ptr<const Sampler> sampler) {
  if (!texture) {
    return false;
  }
  return Bind(pass_bindings_, stage, slot.texture_index, sampler, *texture);
}

}  // namespace impeller
