// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_RENDER_PASS_VK_H_
#define FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_RENDER_PASS_VK_H_

#include "impeller/core/buffer_view.h"
#include "impeller/renderer/backend/vulkan/context_vk.h"
#include "impeller/renderer/backend/vulkan/pipeline_vk.h"
#include "impeller/renderer/backend/vulkan/shared_object_vk.h"
#include "impeller/renderer/command_buffer.h"
#include "impeller/renderer/render_pass.h"
#include "impeller/renderer/render_target.h"

namespace impeller {

class CommandBufferVK;
class SamplerVK;

class RenderPassVK final : public RenderPass {
 public:
  // |RenderPass|
  ~RenderPassVK() override;

 private:
  friend class CommandBufferVK;

  std::shared_ptr<CommandBufferVK> command_buffer_;
  std::string debug_label_;
  SharedHandleVK<vk::RenderPass> render_pass_;
  bool is_valid_ = false;

  vk::CommandBuffer command_buffer_vk_;
  std::shared_ptr<Texture> color_image_vk_;
  std::shared_ptr<Texture> resolve_image_vk_;

  // Per-command state.
  std::array<vk::DescriptorImageInfo, kMaxBindings> image_workspace_;
  std::array<vk::DescriptorBufferInfo, kMaxBindings> buffer_workspace_;
  std::array<vk::WriteDescriptorSet, kMaxBindings + kMaxBindings>
      write_workspace_;
  size_t bound_image_offset_ = 0u;
  size_t bound_buffer_offset_ = 0u;
  size_t descriptor_write_offset_ = 0u;
  size_t instance_count_ = 1u;
  size_t base_vertex_ = 0u;
  size_t element_count_ = 0u;
  bool has_index_buffer_ = false;
  bool has_label_ = false;
  PipelineRef pipeline_ = PipelineRef(nullptr);
  bool pipeline_uses_input_attachments_ = false;
  std::shared_ptr<SamplerVK> immutable_sampler_;

  RenderPassVK(const std::shared_ptr<const Context>& context,
               const RenderTarget& target,
               std::shared_ptr<CommandBufferVK> command_buffer);

  // |RenderPass|
  void SetPipeline(PipelineRef pipeline) override;

  // |RenderPass|
  void SetCommandLabel(std::string_view label) override;

  // |RenderPass|
  void SetStencilReference(uint32_t value) override;

  // |RenderPass|
  void SetBaseVertex(uint64_t value) override;

  // |RenderPass|
  void SetViewport(Viewport viewport) override;

  // |RenderPass|
  void SetScissor(IRect scissor) override;

  // |RenderPass|
  void SetElementCount(size_t count) override;

  // |RenderPass|
  void SetInstanceCount(size_t count) override;

  // |RenderPass|
  bool SetVertexBuffer(BufferView vertex_buffers[],
                       size_t vertex_buffer_count) override;

  // |RenderPass|
  bool SetIndexBuffer(BufferView index_buffer, IndexType index_type) override;

  // |RenderPass|
  fml::Status Draw() override;

  // |ResourceBinder|
  bool BindResource(ShaderStage stage,
                    DescriptorType type,
                    const ShaderUniformSlot& slot,
                    const ShaderMetadata* metadata,
                    BufferView view) override;

  // |ResourceBinder|
  bool BindResource(ShaderStage stage,
                    DescriptorType type,
                    const SampledImageSlot& slot,
                    const ShaderMetadata* metadata,
                    std::shared_ptr<const Texture> texture,
                    raw_ptr<const Sampler> sampler) override;

  // |RenderPass|
  bool BindDynamicResource(ShaderStage stage,
                           DescriptorType type,
                           const ShaderUniformSlot& slot,
                           std::unique_ptr<ShaderMetadata> metadata,
                           BufferView view) override;

  // |RenderPass|
  bool BindDynamicResource(ShaderStage stage,
                           DescriptorType type,
                           const SampledImageSlot& slot,
                           std::unique_ptr<ShaderMetadata> metadata,
                           std::shared_ptr<const Texture> texture,
                           raw_ptr<const Sampler> sampler) override;

  bool BindResource(size_t binding, DescriptorType type, BufferView view);

  // |RenderPass|
  bool IsValid() const override;

  // |RenderPass|
  void OnSetLabel(std::string_view label) override;

  // |RenderPass|
  bool OnEncodeCommands(const Context& context) const override;

  SharedHandleVK<vk::RenderPass> CreateVKRenderPass(
      const ContextVK& context,
      const SharedHandleVK<vk::RenderPass>& recycled_renderpass,
      const std::shared_ptr<CommandBufferVK>& command_buffer) const;

  SharedHandleVK<vk::Framebuffer> CreateVKFramebuffer(
      const ContextVK& context,
      const vk::RenderPass& pass) const;

  RenderPassVK(const RenderPassVK&) = delete;

  RenderPassVK& operator=(const RenderPassVK&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_RENDER_PASS_VK_H_
