// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_COMPUTE_PASS_VK_H_
#define FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_COMPUTE_PASS_VK_H_

#include "impeller/renderer/backend/vulkan/pipeline_vk.h"
#include "impeller/renderer/backend/vulkan/vk.h"
#include "impeller/renderer/compute_pass.h"

namespace impeller {

class CommandBufferVK;

class ComputePassVK final : public ComputePass {
 public:
  // |ComputePass|
  ~ComputePassVK() override;

 private:
  friend class CommandBufferVK;

  std::shared_ptr<CommandBufferVK> command_buffer_;
  std::string label_;
  std::array<uint32_t, 3> max_wg_size_ = {};
  bool is_valid_ = false;

  // Per-command state.
  std::array<vk::DescriptorImageInfo, kMaxBindings> image_workspace_;
  std::array<vk::DescriptorBufferInfo, kMaxBindings> buffer_workspace_;
  std::array<vk::WriteDescriptorSet, kMaxBindings + kMaxBindings>
      write_workspace_;
  size_t bound_image_offset_ = 0u;
  size_t bound_buffer_offset_ = 0u;
  size_t descriptor_write_offset_ = 0u;
  bool has_label_ = false;
  bool pipeline_valid_ = false;
  vk::DescriptorSet descriptor_set_ = {};
  vk::PipelineLayout pipeline_layout_ = {};

  ComputePassVK(std::shared_ptr<const Context> context,
                std::shared_ptr<CommandBufferVK> command_buffer);

  // |ComputePass|
  bool IsValid() const override;

  // |ComputePass|
  void OnSetLabel(const std::string& label) override;

  // |ComputePass|
  bool EncodeCommands() const override;

  // |ComputePass|
  void SetCommandLabel(std::string_view label) override;

  // |ComputePass|
  void SetPipeline(const std::shared_ptr<Pipeline<ComputePipelineDescriptor>>&
                       pipeline) override;

  // |ComputePass|
  void AddBufferMemoryBarrier() override;

  // |ComputePass|
  void AddTextureMemoryBarrier() override;

  // |ComputePass|
  fml::Status Compute(const ISize& grid_size) override;

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

  bool BindResource(size_t binding, DescriptorType type, BufferView view);
};

}  // namespace impeller
#endif  // FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_COMPUTE_PASS_VK_H_
