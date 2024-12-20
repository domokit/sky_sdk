// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/vertex_descriptor.h"

namespace impeller {

VertexDescriptor::VertexDescriptor() = default;

VertexDescriptor::~VertexDescriptor() = default;

void VertexDescriptor::SetStageInputs(
    const ShaderStageIOSlot* const stage_inputs[],
    size_t count,
    const ShaderStageBufferLayout* const stage_layout[],
    size_t layout_count) {
  inputs_.reserve(inputs_.size() + count);
  layouts_.reserve(layouts_.size() + layout_count);
  for (size_t i = 0; i < count; i++) {
    inputs_.emplace_back(*stage_inputs[i]);
  }
  for (size_t i = 0; i < layout_count; i++) {
    layouts_.emplace_back(*stage_layout[i]);
  }
}

void VertexDescriptor::SetStageInputs(
    const std::vector<ShaderStageIOSlot>& inputs,
    const std::vector<ShaderStageBufferLayout>& layout) {
  inputs_.insert(inputs_.end(), inputs.begin(), inputs.end());
  layouts_.insert(layouts_.end(), layout.begin(), layout.end());
}

void VertexDescriptor::RegisterDescriptorSetLayouts(
    const DescriptorSetLayout desc_set_layout[],
    size_t count) {
  desc_set_layouts_.reserve(desc_set_layouts_.size() + count);
  for (size_t i = 0; i < count; i++) {
    uses_input_attachments_ |=
        desc_set_layout[i].descriptor_type == DescriptorType::kInputAttachment;
    desc_set_layouts_.emplace_back(desc_set_layout[i]);
  }
}

// |Comparable<VertexDescriptor>|
size_t VertexDescriptor::GetHash() const {
  auto seed = fml::HashCombine();
  for (const auto& input : inputs_) {
    fml::HashCombineSeed(seed, input.GetHash());
  }
  for (const auto& layout : layouts_) {
    fml::HashCombineSeed(seed, layout.GetHash());
  }
  return seed;
}

// |Comparable<VertexDescriptor>|
bool VertexDescriptor::IsEqual(const VertexDescriptor& other) const {
  return inputs_ == other.inputs_ && layouts_ == other.layouts_;
}

const std::vector<ShaderStageIOSlot>& VertexDescriptor::GetStageInputs() const {
  return inputs_;
}

const std::vector<ShaderStageBufferLayout>& VertexDescriptor::GetStageLayouts()
    const {
  return layouts_;
}

const std::vector<DescriptorSetLayout>&
VertexDescriptor::GetDescriptorSetLayouts() const {
  return desc_set_layouts_;
}

bool VertexDescriptor::UsesInputAttacments() const {
  return uses_input_attachments_;
}

}  // namespace impeller
