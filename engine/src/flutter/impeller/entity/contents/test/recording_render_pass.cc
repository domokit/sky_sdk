// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/entity/contents/test/recording_render_pass.h"

#include <utility>

namespace impeller::testing {

RecordingRenderPass::RecordingRenderPass(
    std::shared_ptr<RenderPass> delegate,
    const std::shared_ptr<const Context>& context,
    const RenderTarget& render_target)
    : RenderPass(context, render_target), delegate_(std::move(delegate)) {}

// |RenderPass|
void RecordingRenderPass::SetPipeline(PipelineRef pipeline) {
  pending_.pipeline = pipeline;
  if (delegate_) {
    delegate_->SetPipeline(pipeline);
  }
}

void RecordingRenderPass::SetCommandLabel(std::string_view label) {
#ifdef IMPELLER_DEBUG
  pending_.label = std::string(label);
#endif  // IMPELLER_DEBUG
  if (delegate_) {
    delegate_->SetCommandLabel(label);
  }
}

// |RenderPass|
void RecordingRenderPass::SetStencilReference(uint32_t value) {
  pending_.stencil_reference = value;
  if (delegate_) {
    delegate_->SetStencilReference(value);
  }
}

// |RenderPass|
void RecordingRenderPass::SetBaseVertex(uint64_t value) {
  pending_.base_vertex = value;
  if (delegate_) {
    delegate_->SetBaseVertex(value);
  }
}

// |RenderPass|
void RecordingRenderPass::SetViewport(Viewport viewport) {
  pending_.viewport = viewport;
  if (delegate_) {
    delegate_->SetViewport(viewport);
  }
}

// |RenderPass|
void RecordingRenderPass::SetScissor(IRect scissor) {
  pending_.scissor = scissor;
  if (delegate_) {
    delegate_->SetScissor(scissor);
  }
}

// |RenderPass|
void RecordingRenderPass::SetInstanceCount(size_t count) {
  pending_.instance_count = count;
  if (delegate_) {
    delegate_->SetInstanceCount(count);
  }
}

// |RenderPass|
bool RecordingRenderPass::SetVertexBuffer(VertexBuffer buffer) {
  if (delegate_) {
    return delegate_->SetVertexBuffer(buffer);
  }
  return true;
}

// |RenderPass|
fml::Status RecordingRenderPass::Draw() {
  commands_.emplace_back(std::move(pending_));
  pending_ = {};
  if (delegate_) {
    return delegate_->Draw();
  }
  return fml::Status();
}

// |RenderPass|
void RecordingRenderPass::OnSetLabel(std::string_view label) {
  return;
}

// |RenderPass|
bool RecordingRenderPass::OnEncodeCommands(const Context& context) const {
  if (delegate_) {
    return delegate_->EncodeCommands();
  }
  return true;
}

// |RenderPass|
bool RecordingRenderPass::BindResource(ShaderStage stage,
                                       DescriptorType type,
                                       const ShaderUniformSlot& slot,
                                       const ShaderMetadata* metadata,
                                       BufferView view) {
  if (delegate_) {
    return delegate_->BindResource(stage, type, slot, metadata, view);
  }
  return true;
}

// |RenderPass|
bool RecordingRenderPass::BindDynamicResource(
    ShaderStage stage,
    DescriptorType type,
    const ShaderUniformSlot& slot,
    std::unique_ptr<ShaderMetadata> metadata,
    BufferView view) {
  if (delegate_) {
    return delegate_->BindDynamicResource(stage, type, slot,
                                          std::move(metadata), view);
  }
  return true;
}

// |RenderPass|
bool RecordingRenderPass::BindDynamicResource(
    ShaderStage stage,
    DescriptorType type,
    const SampledImageSlot& slot,
    std::unique_ptr<ShaderMetadata> metadata,
    std::shared_ptr<const Texture> texture,
    raw_ptr<const Sampler> sampler) {
  if (delegate_) {
    return delegate_->BindDynamicResource(
        stage, type, slot, std::move(metadata), texture, sampler);
  }
  return true;
}

bool RecordingRenderPass::BindResource(ShaderStage stage,
                                       DescriptorType type,
                                       const SampledImageSlot& slot,
                                       const ShaderMetadata* metadata,
                                       std::shared_ptr<const Texture> texture,
                                       raw_ptr<const Sampler> sampler) {
  if (delegate_) {
    return delegate_->BindResource(stage, type, slot, metadata, texture,
                                   sampler);
  }
  return true;
}

}  // namespace impeller::testing
