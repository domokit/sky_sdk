// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/backend/vulkan/render_pass_vk.h"

#include <array>
#include <cstdint>

#include "fml/status.h"
#include "impeller/base/validation.h"
#include "impeller/core/buffer_view.h"
#include "impeller/core/device_buffer.h"
#include "impeller/core/formats.h"
#include "impeller/core/texture.h"
#include "impeller/core/vertex_buffer.h"
#include "impeller/renderer/backend/vulkan/barrier_vk.h"
#include "impeller/renderer/backend/vulkan/command_buffer_vk.h"
#include "impeller/renderer/backend/vulkan/context_vk.h"
#include "impeller/renderer/backend/vulkan/device_buffer_vk.h"
#include "impeller/renderer/backend/vulkan/formats_vk.h"
#include "impeller/renderer/backend/vulkan/pipeline_vk.h"
#include "impeller/renderer/backend/vulkan/render_pass_builder_vk.h"
#include "impeller/renderer/backend/vulkan/sampler_vk.h"
#include "impeller/renderer/backend/vulkan/shared_object_vk.h"
#include "impeller/renderer/backend/vulkan/texture_vk.h"

namespace impeller {

// Warning: if any of the constant values or layouts are changed in the
// framebuffer fetch shader, then this input binding may need to be
// manually changed.
//
// See: impeller/entity/shaders/blending/framebuffer_blend.frag
static constexpr size_t kMagicSubpassInputBinding = 64u;

static vk::ClearColorValue VKClearValueFromColor(Color color) {
  vk::ClearColorValue value;
  value.setFloat32(
      std::array<float, 4>{color.red, color.green, color.blue, color.alpha});
  return value;
}

static vk::ClearDepthStencilValue VKClearValueFromDepthStencil(uint32_t stencil,
                                                               Scalar depth) {
  vk::ClearDepthStencilValue value;
  value.depth = depth;
  value.stencil = stencil;
  return value;
}

static size_t GetVKClearValues(
    const RenderTarget& target,
    std::array<vk::ClearValue, kMaxAttachments>& values) {
  size_t offset = 0u;
  target.IterateAllColorAttachments(
      [&values, &offset](size_t index,
                         const ColorAttachment& attachment) -> bool {
        values.at(offset++) = VKClearValueFromColor(attachment.clear_color);
        if (attachment.resolve_texture) {
          values.at(offset++) = VKClearValueFromColor(attachment.clear_color);
        }
        return true;
      });

  const auto& depth = target.GetDepthAttachment();
  const auto& stencil = target.GetStencilAttachment();

  if (depth.has_value()) {
    values.at(offset++) = VKClearValueFromDepthStencil(
        stencil ? stencil->clear_stencil : 0u, depth->clear_depth);
  } else if (stencil.has_value()) {
    values.at(offset++) = VKClearValueFromDepthStencil(
        stencil->clear_stencil, depth ? depth->clear_depth : 0.0f);
  }
  return offset;
}

SharedHandleVK<vk::RenderPass> RenderPassVK::CreateVKRenderPass(
    const ContextVK& context,
    const SharedHandleVK<vk::RenderPass>& recycled_renderpass,
    const std::shared_ptr<CommandBufferVK>& command_buffer) const {
  RenderPassBuilderVK builder;

  render_target_.IterateAllColorAttachments(
      [&](size_t bind_point, const ColorAttachment& attachment) -> bool {
        builder.SetColorAttachment(
            bind_point,                                               //
            attachment.texture->GetTextureDescriptor().format,        //
            attachment.texture->GetTextureDescriptor().sample_count,  //
            attachment.load_action,                                   //
            attachment.store_action                                   //
        );
        return true;
      });

  if (auto depth = render_target_.GetDepthAttachment(); depth.has_value()) {
    builder.SetDepthStencilAttachment(
        depth->texture->GetTextureDescriptor().format,        //
        depth->texture->GetTextureDescriptor().sample_count,  //
        depth->load_action,                                   //
        depth->store_action                                   //
    );
  } else if (auto stencil = render_target_.GetStencilAttachment();
             stencil.has_value()) {
    builder.SetStencilAttachment(
        stencil->texture->GetTextureDescriptor().format,        //
        stencil->texture->GetTextureDescriptor().sample_count,  //
        stencil->load_action,                                   //
        stencil->store_action                                   //
    );
  }

  if (recycled_renderpass != nullptr) {
    return recycled_renderpass;
  }

  auto pass = builder.Build(context.GetDevice());

  if (!pass) {
    VALIDATION_LOG << "Failed to create render pass for framebuffer.";
    return {};
  }

  context.SetDebugName(pass.get(), debug_label_.c_str());

  return MakeSharedVK(std::move(pass));
}

RenderPassVK::RenderPassVK(const std::shared_ptr<const Context>& context,
                           const RenderTarget& target,
                           std::shared_ptr<CommandBufferVK> command_buffer)
    : RenderPass(context, target), command_buffer_(std::move(command_buffer)) {
  const ColorAttachment& color0 = render_target_.GetColorAttachment(0);
  color_image_vk_ = color0.texture;
  resolve_image_vk_ = color0.resolve_texture;

  const auto& vk_context = ContextVK::Cast(*context);
  command_buffer_vk_ = command_buffer_->GetCommandBuffer();
  render_target_.IterateAllAttachments([&](const auto& attachment) -> bool {
    command_buffer_->Track(attachment.texture);
    command_buffer_->Track(attachment.resolve_texture);
    return true;
  });

  SharedHandleVK<vk::RenderPass> recycled_render_pass;
  SharedHandleVK<vk::Framebuffer> recycled_framebuffer;
  if (resolve_image_vk_) {
    recycled_render_pass =
        TextureVK::Cast(*resolve_image_vk_).GetCachedRenderPass();
    recycled_framebuffer =
        TextureVK::Cast(*resolve_image_vk_).GetCachedFramebuffer();
  }

  const auto& target_size = render_target_.GetRenderTargetSize();

  render_pass_ =
      CreateVKRenderPass(vk_context, recycled_render_pass, command_buffer_);
  if (!render_pass_) {
    VALIDATION_LOG << "Could not create renderpass.";
    is_valid_ = false;
    return;
  }

  auto framebuffer = (recycled_framebuffer == nullptr)
                         ? CreateVKFramebuffer(vk_context, *render_pass_)
                         : recycled_framebuffer;
  if (!framebuffer) {
    VALIDATION_LOG << "Could not create framebuffer.";
    is_valid_ = false;
    return;
  }

  if (!command_buffer_->Track(framebuffer) ||
      !command_buffer_->Track(render_pass_)) {
    is_valid_ = false;
    return;
  }
  if (resolve_image_vk_) {
    TextureVK::Cast(*resolve_image_vk_).SetCachedFramebuffer(framebuffer);
    TextureVK::Cast(*resolve_image_vk_).SetCachedRenderPass(render_pass_);
  }

  std::array<vk::ClearValue, kMaxAttachments> clears;
  size_t clear_count = GetVKClearValues(render_target_, clears);

  vk::RenderPassBeginInfo pass_info;
  pass_info.renderPass = *render_pass_;
  pass_info.framebuffer = *framebuffer;
  pass_info.renderArea.extent.width = static_cast<uint32_t>(target_size.width);
  pass_info.renderArea.extent.height =
      static_cast<uint32_t>(target_size.height);
  pass_info.setPClearValues(clears.data());
  pass_info.setClearValueCount(clear_count);

  command_buffer_vk_.beginRenderPass(pass_info, vk::SubpassContents::eInline);

  if (resolve_image_vk_) {
    TextureVK::Cast(*resolve_image_vk_)
        .SetLayoutWithoutEncoding(vk::ImageLayout::eGeneral);
  }
  if (color_image_vk_) {
    TextureVK::Cast(*color_image_vk_)
        .SetLayoutWithoutEncoding(vk::ImageLayout::eGeneral);
  }

  // Set the initial viewport.
  const auto vp = Viewport{.rect = Rect::MakeSize(target_size)};
  vk::Viewport viewport = vk::Viewport()
                              .setWidth(vp.rect.GetWidth())
                              .setHeight(-vp.rect.GetHeight())
                              .setY(vp.rect.GetHeight())
                              .setMinDepth(0.0f)
                              .setMaxDepth(1.0f);
  command_buffer_vk_.setViewport(0, 1, &viewport);

  // Set the initial scissor.
  const auto sc = IRect::MakeSize(target_size);
  vk::Rect2D scissor =
      vk::Rect2D()
          .setOffset(vk::Offset2D(sc.GetX(), sc.GetY()))
          .setExtent(vk::Extent2D(sc.GetWidth(), sc.GetHeight()));
  command_buffer_vk_.setScissor(0, 1, &scissor);

  // Set the initial stencil reference.
  command_buffer_vk_.setStencilReference(
      vk::StencilFaceFlagBits::eVkStencilFrontAndBack, 0u);

  is_valid_ = true;
}

RenderPassVK::~RenderPassVK() = default;

bool RenderPassVK::IsValid() const {
  return is_valid_;
}

void RenderPassVK::OnSetLabel(std::string_view label) {
#ifdef IMPELLER_DEBUG
  ContextVK::Cast(*context_).SetDebugName(render_pass_->Get(), label.data());
#endif  // IMPELLER_DEBUG
}

SharedHandleVK<vk::Framebuffer> RenderPassVK::CreateVKFramebuffer(
    const ContextVK& context,
    const vk::RenderPass& pass) const {
  vk::FramebufferCreateInfo fb_info;

  fb_info.renderPass = pass;

  const auto target_size = render_target_.GetRenderTargetSize();
  fb_info.width = target_size.width;
  fb_info.height = target_size.height;
  fb_info.layers = 1u;

  std::array<vk::ImageView, kMaxAttachments> attachments;
  size_t count = 0;

  // This bit must be consistent to ensure compatibility with the pass created
  // earlier. Follow this order: Color attachments, then depth-stencil, then
  // stencil.
  render_target_.IterateAllColorAttachments(
      [&attachments, &count](size_t index,
                             const ColorAttachment& attachment) -> bool {
        // The bind point doesn't matter here since that information is present
        // in the render pass.
        attachments[count++] =
            TextureVK::Cast(*attachment.texture).GetRenderTargetView();
        if (attachment.resolve_texture) {
          attachments[count++] = TextureVK::Cast(*attachment.resolve_texture)
                                     .GetRenderTargetView();
        }
        return true;
      });

  if (auto depth = render_target_.GetDepthAttachment(); depth.has_value()) {
    attachments[count++] =
        TextureVK::Cast(*depth->texture).GetRenderTargetView();
  } else if (auto stencil = render_target_.GetStencilAttachment();
             stencil.has_value()) {
    attachments[count++] =
        TextureVK::Cast(*stencil->texture).GetRenderTargetView();
  }

  fb_info.setPAttachments(attachments.data());
  fb_info.setAttachmentCount(count);

  auto [result, framebuffer] =
      context.GetDevice().createFramebufferUnique(fb_info);

  if (result != vk::Result::eSuccess) {
    VALIDATION_LOG << "Could not create framebuffer: " << vk::to_string(result);
    return {};
  }

  return MakeSharedVK(std::move(framebuffer));
}

// |RenderPass|
void RenderPassVK::SetPipeline(PipelineRef pipeline) {
  pipeline_ = pipeline;
  if (!pipeline_) {
    return;
  }

  pipeline_uses_input_attachments_ =
      pipeline_->GetDescriptor().GetVertexDescriptor()->UsesInputAttacments();

  if (pipeline_uses_input_attachments_) {
    if (bound_image_offset_ >= kMaxBindings) {
      pipeline_ = PipelineRef(nullptr);
      return;
    }
    vk::DescriptorImageInfo image_info;
    image_info.imageLayout = vk::ImageLayout::eGeneral;
    image_info.sampler = VK_NULL_HANDLE;
    image_info.imageView = TextureVK::Cast(*color_image_vk_).GetImageView();
    image_workspace_[bound_image_offset_++] = image_info;

    vk::WriteDescriptorSet write_set;
    write_set.dstBinding = kMagicSubpassInputBinding;
    write_set.descriptorCount = 1u;
    write_set.descriptorType = vk::DescriptorType::eInputAttachment;
    write_set.pImageInfo = &image_workspace_[bound_image_offset_ - 1];

    write_workspace_[descriptor_write_offset_++] = write_set;
  }
}

// |RenderPass|
void RenderPassVK::SetCommandLabel(std::string_view label) {
#ifdef IMPELLER_DEBUG
  command_buffer_->PushDebugGroup(label);
  has_label_ = true;
#endif  // IMPELLER_DEBUG
}

// |RenderPass|
void RenderPassVK::SetStencilReference(uint32_t value) {
  command_buffer_vk_.setStencilReference(
      vk::StencilFaceFlagBits::eVkStencilFrontAndBack, value);
}

// |RenderPass|
void RenderPassVK::SetBaseVertex(uint64_t value) {
  base_vertex_ = value;
}

// |RenderPass|
void RenderPassVK::SetViewport(Viewport viewport) {
  vk::Viewport viewport_vk = vk::Viewport()
                                 .setWidth(viewport.rect.GetWidth())
                                 .setHeight(-viewport.rect.GetHeight())
                                 .setY(viewport.rect.GetHeight())
                                 .setMinDepth(0.0f)
                                 .setMaxDepth(1.0f);
  command_buffer_vk_.setViewport(0, 1, &viewport_vk);
}

// |RenderPass|
void RenderPassVK::SetScissor(IRect scissor) {
  vk::Rect2D scissor_vk =
      vk::Rect2D()
          .setOffset(vk::Offset2D(scissor.GetX(), scissor.GetY()))
          .setExtent(vk::Extent2D(scissor.GetWidth(), scissor.GetHeight()));
  command_buffer_vk_.setScissor(0, 1, &scissor_vk);
}

// |RenderPass|
void RenderPassVK::SetElementCount(size_t count) {
  element_count_ = count;
}

// |RenderPass|
void RenderPassVK::SetInstanceCount(size_t count) {
  instance_count_ = count;
}

// |RenderPass|
bool RenderPassVK::SetVertexBuffer(BufferView vertex_buffers[],
                                   size_t vertex_buffer_count) {
  if (!ValidateVertexBuffers(vertex_buffers, vertex_buffer_count)) {
    return false;
  }

  vk::Buffer buffers[kMaxVertexBuffers];
  vk::DeviceSize vertex_buffer_offsets[kMaxVertexBuffers];
  for (size_t i = 0; i < vertex_buffer_count; i++) {
    buffers[i] =
        DeviceBufferVK::Cast(*vertex_buffers[i].GetBuffer()).GetBuffer();
    vertex_buffer_offsets[i] = vertex_buffers[i].GetRange().offset;
    std::shared_ptr<const DeviceBuffer> device_buffer =
        vertex_buffers[i].TakeBuffer();
    if (device_buffer) {
      command_buffer_->Track(device_buffer);
    }
  }

  // Bind the vertex buffers.
  command_buffer_vk_.bindVertexBuffers(0u, vertex_buffer_count, buffers,
                                       vertex_buffer_offsets);

  return true;
}

// |RenderPass|
bool RenderPassVK::SetIndexBuffer(BufferView index_buffer,
                                  IndexType index_type) {
  if (!ValidateIndexBuffer(index_buffer, index_type)) {
    return false;
  }

  if (index_type != IndexType::kNone) {
    has_index_buffer_ = true;

    BufferView index_buffer_view = std::move(index_buffer);
    if (!index_buffer_view) {
      return false;
    }

    if (!index_buffer_view.GetBuffer()) {
      VALIDATION_LOG << "Failed to acquire device buffer"
                     << " for index buffer view";
      return false;
    }

    std::shared_ptr<const DeviceBuffer> index_buffer =
        index_buffer_view.TakeBuffer();
    if (index_buffer && !command_buffer_->Track(index_buffer)) {
      return false;
    }

    vk::Buffer index_buffer_handle =
        DeviceBufferVK::Cast(*index_buffer_view.GetBuffer()).GetBuffer();
    command_buffer_vk_.bindIndexBuffer(index_buffer_handle,
                                       index_buffer_view.GetRange().offset,
                                       ToVKIndexType(index_type));
  } else {
    has_index_buffer_ = false;
  }

  return true;
}

// |RenderPass|
fml::Status RenderPassVK::Draw() {
  if (!pipeline_) {
    return fml::Status(fml::StatusCode::kCancelled,
                       "No valid pipeline is bound to the RenderPass.");
  }

  //----------------------------------------------------------------------------
  /// If there are immutable samplers referenced in the render pass, the base
  /// pipeline variant is no longer valid and needs to be re-constructed to
  /// reference the samplers.
  ///
  /// This is an instance of JIT creation of PSOs that can cause jank. It is
  /// unavoidable because it isn't possible to know all possible combinations of
  /// target YUV conversions. Fortunately, this will only ever happen when
  /// rendering to external textures. Like Android Hardware Buffers on Android.
  ///
  /// Even when JIT creation is unavoidable, pipelines will cache their variants
  /// when able and all pipeline creation will happen via a base pipeline cache
  /// anyway. So the jank can be mostly entirely ameliorated and it should only
  /// ever happen when the first unknown YUV conversion is encountered.
  ///
  /// Jank can be completely eliminated by pre-populating known YUV conversion
  /// pipelines.
  if (immutable_sampler_) {
    std::shared_ptr<Pipeline<PipelineDescriptor>> pipeline_variant =
        PipelineVK::Cast(*pipeline_)
            .CreateVariantForImmutableSamplers(immutable_sampler_);
    if (!pipeline_variant) {
      return fml::Status(
          fml::StatusCode::kAborted,
          "Could not create pipeline variant with immutable sampler.");
    }
    pipeline_ = raw_ptr(pipeline_variant);
  }

  const auto& context_vk = ContextVK::Cast(*context_);
  const auto& pipeline_vk = PipelineVK::Cast(*pipeline_);

  auto descriptor_result = command_buffer_->AllocateDescriptorSets(
      pipeline_vk.GetDescriptorSetLayout(), context_vk);
  if (!descriptor_result.ok()) {
    return fml::Status(fml::StatusCode::kAborted,
                       "Could not allocate descriptor sets.");
  }
  const auto descriptor_set = descriptor_result.value();
  const auto pipeline_layout = pipeline_vk.GetPipelineLayout();
  command_buffer_vk_.bindPipeline(vk::PipelineBindPoint::eGraphics,
                                  pipeline_vk.GetPipeline());

  for (auto i = 0u; i < descriptor_write_offset_; i++) {
    write_workspace_[i].dstSet = descriptor_set;
  }

  context_vk.GetDevice().updateDescriptorSets(descriptor_write_offset_,
                                              write_workspace_.data(), 0u, {});

  command_buffer_vk_.bindDescriptorSets(
      vk::PipelineBindPoint::eGraphics,  // bind point
      pipeline_layout,                   // layout
      0,                                 // first set
      1,                                 // set count
      &descriptor_set,                   // sets
      0,                                 // offset count
      nullptr                            // offsets
  );

  if (pipeline_uses_input_attachments_) {
    InsertBarrierForInputAttachmentRead(
        command_buffer_vk_, TextureVK::Cast(*color_image_vk_).GetImage());
  }

  if (has_index_buffer_) {
    command_buffer_vk_.drawIndexed(element_count_,   // index count
                                   instance_count_,  // instance count
                                   0u,               // first index
                                   base_vertex_,     // vertex offset
                                   0u                // first instance
    );
  } else {
    command_buffer_vk_.draw(element_count_,   // vertex count
                            instance_count_,  // instance count
                            base_vertex_,     // vertex offset
                            0u                // first instance
    );
  }

#ifdef IMPELLER_DEBUG
  if (has_label_) {
    command_buffer_->PopDebugGroup();
  }
#endif  // IMPELLER_DEBUG
  has_label_ = false;
  has_index_buffer_ = false;
  bound_image_offset_ = 0u;
  bound_buffer_offset_ = 0u;
  descriptor_write_offset_ = 0u;
  instance_count_ = 1u;
  base_vertex_ = 0u;
  element_count_ = 0u;
  pipeline_ = PipelineRef(nullptr);
  pipeline_uses_input_attachments_ = false;
  immutable_sampler_ = nullptr;
  return fml::Status();
}

// The RenderPassVK binding methods only need the binding, set, and buffer type
// information.
bool RenderPassVK::BindResource(ShaderStage stage,
                                DescriptorType type,
                                const ShaderUniformSlot& slot,
                                const ShaderMetadata* metadata,
                                BufferView view) {
  return BindResource(slot.binding, type, view);
}

bool RenderPassVK::BindDynamicResource(ShaderStage stage,
                                       DescriptorType type,
                                       const ShaderUniformSlot& slot,
                                       std::unique_ptr<ShaderMetadata> metadata,
                                       BufferView view) {
  return BindResource(slot.binding, type, view);
}

bool RenderPassVK::BindResource(size_t binding,
                                DescriptorType type,
                                BufferView view) {
  if (bound_buffer_offset_ >= kMaxBindings) {
    return false;
  }

  auto buffer = DeviceBufferVK::Cast(*view.GetBuffer()).GetBuffer();
  if (!buffer) {
    return false;
  }

  std::shared_ptr<const DeviceBuffer> device_buffer = view.TakeBuffer();
  if (device_buffer && !command_buffer_->Track(device_buffer)) {
    return false;
  }

  uint32_t offset = view.GetRange().offset;

  vk::DescriptorBufferInfo buffer_info;
  buffer_info.buffer = buffer;
  buffer_info.offset = offset;
  buffer_info.range = view.GetRange().length;
  buffer_workspace_[bound_buffer_offset_++] = buffer_info;

  vk::WriteDescriptorSet write_set;
  write_set.dstBinding = binding;
  write_set.descriptorCount = 1u;
  write_set.descriptorType = ToVKDescriptorType(type);
  write_set.pBufferInfo = &buffer_workspace_[bound_buffer_offset_ - 1];

  write_workspace_[descriptor_write_offset_++] = write_set;
  return true;
}

bool RenderPassVK::BindDynamicResource(ShaderStage stage,
                                       DescriptorType type,
                                       const SampledImageSlot& slot,
                                       std::unique_ptr<ShaderMetadata> metadata,
                                       std::shared_ptr<const Texture> texture,
                                       raw_ptr<const Sampler> sampler) {
  return BindResource(stage, type, slot, nullptr, texture, sampler);
}

bool RenderPassVK::BindResource(ShaderStage stage,
                                DescriptorType type,
                                const SampledImageSlot& slot,
                                const ShaderMetadata* metadata,
                                std::shared_ptr<const Texture> texture,
                                raw_ptr<const Sampler> sampler) {
  if (bound_buffer_offset_ >= kMaxBindings) {
    return false;
  }
  if (!texture || !texture->IsValid() || !sampler) {
    return false;
  }
  const TextureVK& texture_vk = TextureVK::Cast(*texture);
  const SamplerVK& sampler_vk = SamplerVK::Cast(*sampler);

  if (!command_buffer_->Track(texture)) {
    return false;
  }

  if (!immutable_sampler_) {
    immutable_sampler_ = texture_vk.GetImmutableSamplerVariant(sampler_vk);
  }

  vk::DescriptorImageInfo image_info;
  image_info.imageLayout = vk::ImageLayout::eShaderReadOnlyOptimal;
  image_info.sampler = sampler_vk.GetSampler();
  image_info.imageView = texture_vk.GetImageView();
  image_workspace_[bound_image_offset_++] = image_info;

  vk::WriteDescriptorSet write_set;
  write_set.dstBinding = slot.binding;
  write_set.descriptorCount = 1u;
  write_set.descriptorType = vk::DescriptorType::eCombinedImageSampler;
  write_set.pImageInfo = &image_workspace_[bound_image_offset_ - 1];

  write_workspace_[descriptor_write_offset_++] = write_set;
  return true;
}

bool RenderPassVK::OnEncodeCommands(const Context& context) const {
  command_buffer_->GetCommandBuffer().endRenderPass();

  // If this render target will be consumed by a subsequent render pass,
  // perform a layout transition to a shader read state.
  const std::shared_ptr<Texture>& result_texture =
      resolve_image_vk_ ? resolve_image_vk_ : color_image_vk_;
  if (result_texture->GetTextureDescriptor().usage &
      TextureUsage::kShaderRead) {
    BarrierVK barrier;
    barrier.cmd_buffer = command_buffer_vk_;
    barrier.src_access = vk::AccessFlagBits::eColorAttachmentWrite |
                         vk::AccessFlagBits::eTransferWrite;
    barrier.src_stage = vk::PipelineStageFlagBits::eColorAttachmentOutput |
                        vk::PipelineStageFlagBits::eTransfer;
    barrier.dst_access = vk::AccessFlagBits::eShaderRead;
    barrier.dst_stage = vk::PipelineStageFlagBits::eFragmentShader;

    barrier.new_layout = vk::ImageLayout::eShaderReadOnlyOptimal;

    if (!TextureVK::Cast(*result_texture).SetLayout(barrier)) {
      return false;
    }
  }

  return true;
}

}  // namespace impeller
