// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_ENTITY_CONTENTS_TEST_RECORDING_RENDER_PASS_H_
#define FLUTTER_IMPELLER_ENTITY_CONTENTS_TEST_RECORDING_RENDER_PASS_H_

#include "impeller/renderer/render_pass.h"

namespace impeller::testing {

class RecordingRenderPass : public RenderPass {
 public:
  explicit RecordingRenderPass(std::shared_ptr<RenderPass> delegate,
                               const std::shared_ptr<const Context>& context,
                               const RenderTarget& render_target);

  ~RecordingRenderPass() = default;

  const std::vector<Command>& GetCommands() const override { return commands_; }

  // |RenderPass|
  void SetPipeline(PipelineRef pipeline) override;

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
  void SetInstanceCount(size_t count) override;

  // |RenderPass|
  bool SetVertexBuffer(VertexBuffer buffer) override;

  // |RenderPass|
  fml::Status Draw() override;

  bool BindResource(ShaderStage stage,
                    DescriptorType type,
                    const ShaderUniformSlot& slot,
                    const ShaderMetadata* metadata,
                    BufferView view) override;

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

  // |RenderPass|
  void OnSetLabel(std::string_view label) override;

  // |RenderPass|
  bool OnEncodeCommands(const Context& context) const override;

  bool IsValid() const override { return true; }

 private:
  Command pending_;
  std::shared_ptr<RenderPass> delegate_;
  std::vector<Command> commands_;
};

}  // namespace impeller::testing

#endif  // FLUTTER_IMPELLER_ENTITY_CONTENTS_TEST_RECORDING_RENDER_PASS_H_
