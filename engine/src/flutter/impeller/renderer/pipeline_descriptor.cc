// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/pipeline_descriptor.h"

#include <utility>

#include "impeller/base/comparable.h"
#include "impeller/core/formats.h"
#include "impeller/renderer/shader_function.h"
#include "impeller/renderer/shader_library.h"
#include "impeller/renderer/vertex_descriptor.h"

namespace impeller {

PipelineDescriptor::PipelineDescriptor() = default;

PipelineDescriptor::~PipelineDescriptor() = default;

// Comparable<PipelineDescriptor>
std::size_t PipelineDescriptor::GetHash() const {
  auto seed = fml::HashCombine();
  fml::HashCombineSeed(seed, label_);
  fml::HashCombineSeed(seed, sample_count_);
  for (const auto& entry : entrypoints_) {
    fml::HashCombineSeed(seed, entry.first);
    if (auto second = entry.second) {
      fml::HashCombineSeed(seed, second->GetHash());
    }
  }
  for (const auto& des : color_attachment_descriptors_) {
    fml::HashCombineSeed(seed, des.first);
    fml::HashCombineSeed(seed, des.second.Hash());
  }
  if (vertex_descriptor_) {
    fml::HashCombineSeed(seed, vertex_descriptor_->GetHash());
  }
  fml::HashCombineSeed(seed, depth_pixel_format_);
  fml::HashCombineSeed(seed, stencil_pixel_format_);
  fml::HashCombineSeed(seed, depth_attachment_descriptor_);
  fml::HashCombineSeed(seed, front_stencil_attachment_descriptor_);
  fml::HashCombineSeed(seed, back_stencil_attachment_descriptor_);
  fml::HashCombineSeed(seed, winding_order_);
  fml::HashCombineSeed(seed, cull_mode_);
  fml::HashCombineSeed(seed, primitive_type_);
  fml::HashCombineSeed(seed, polygon_mode_);
  return seed;
}

// Comparable<PipelineDescriptor>
bool PipelineDescriptor::IsEqual(const PipelineDescriptor& other) const {
  return label_ == other.label_ && sample_count_ == other.sample_count_ &&
         DeepCompareMap(entrypoints_, other.entrypoints_) &&
         color_attachment_descriptors_ == other.color_attachment_descriptors_ &&
         DeepComparePointer(vertex_descriptor_, other.vertex_descriptor_) &&
         stencil_pixel_format_ == other.stencil_pixel_format_ &&
         depth_pixel_format_ == other.depth_pixel_format_ &&
         depth_attachment_descriptor_ == other.depth_attachment_descriptor_ &&
         front_stencil_attachment_descriptor_ ==
             other.front_stencil_attachment_descriptor_ &&
         back_stencil_attachment_descriptor_ ==
             other.back_stencil_attachment_descriptor_ &&
         winding_order_ == other.winding_order_ &&
         cull_mode_ == other.cull_mode_ &&
         primitive_type_ == other.primitive_type_ &&
         polygon_mode_ == other.polygon_mode_ &&
         specialization_constants_ == other.specialization_constants_;
}

PipelineDescriptor& PipelineDescriptor::SetLabel(std::string_view label) {
  label_ = std::string(label);
  return *this;
}

PipelineDescriptor& PipelineDescriptor::SetSampleCount(SampleCount samples) {
  sample_count_ = samples;
  return *this;
}

PipelineDescriptor& PipelineDescriptor::AddStageEntrypoint(
    std::shared_ptr<const ShaderFunction> function) {
  if (!function) {
    return *this;
  }

  if (function->GetStage() == ShaderStage::kUnknown) {
    return *this;
  }

  entrypoints_[function->GetStage()] = std::move(function);

  return *this;
}

PipelineDescriptor& PipelineDescriptor::SetVertexDescriptor(
    std::shared_ptr<VertexDescriptor> vertex_descriptor) {
  vertex_descriptor_ = std::move(vertex_descriptor);
  return *this;
}

size_t PipelineDescriptor::GetMaxColorAttacmentBindIndex() const {
  size_t max = 0;
  for (const auto& color : color_attachment_descriptors_) {
    max = std::max(color.first, max);
  }
  return max;
}

PipelineDescriptor& PipelineDescriptor::SetColorAttachmentDescriptor(
    size_t index,
    ColorAttachmentDescriptor desc) {
  color_attachment_descriptors_[index] = desc;
  return *this;
}

PipelineDescriptor& PipelineDescriptor::SetColorAttachmentDescriptors(
    std::map<size_t /* index */, ColorAttachmentDescriptor> descriptors) {
  color_attachment_descriptors_ = std::move(descriptors);
  return *this;
}

const ColorAttachmentDescriptor*
PipelineDescriptor::GetColorAttachmentDescriptor(size_t index) const {
  auto found = color_attachment_descriptors_.find(index);
  return found == color_attachment_descriptors_.end() ? nullptr
                                                      : &found->second;
}

const ColorAttachmentDescriptor*
PipelineDescriptor::GetLegacyCompatibleColorAttachment() const {
  // Legacy renderers may only render to a single color attachment at index 0u.
  if (color_attachment_descriptors_.size() != 1u) {
    return nullptr;
  }
  return GetColorAttachmentDescriptor(0u);
}

PipelineDescriptor& PipelineDescriptor::SetDepthPixelFormat(
    PixelFormat format) {
  depth_pixel_format_ = format;
  return *this;
}

PipelineDescriptor& PipelineDescriptor::SetStencilPixelFormat(
    PixelFormat format) {
  stencil_pixel_format_ = format;
  return *this;
}

PipelineDescriptor& PipelineDescriptor::SetDepthStencilAttachmentDescriptor(
    std::optional<DepthAttachmentDescriptor> desc) {
  depth_attachment_descriptor_ = desc;
  return *this;
}

PipelineDescriptor& PipelineDescriptor::SetStencilAttachmentDescriptors(
    std::optional<StencilAttachmentDescriptor> front_and_back) {
  return SetStencilAttachmentDescriptors(front_and_back, front_and_back);
}

PipelineDescriptor& PipelineDescriptor::SetStencilAttachmentDescriptors(
    std::optional<StencilAttachmentDescriptor> front,
    std::optional<StencilAttachmentDescriptor> back) {
  front_stencil_attachment_descriptor_ = front;
  back_stencil_attachment_descriptor_ = back;
  return *this;
}

void PipelineDescriptor::ClearStencilAttachments() {
  back_stencil_attachment_descriptor_.reset();
  front_stencil_attachment_descriptor_.reset();
  SetStencilPixelFormat(impeller::PixelFormat::kUnknown);
}

void PipelineDescriptor::ClearDepthAttachment() {
  depth_attachment_descriptor_.reset();
  SetDepthPixelFormat(impeller::PixelFormat::kUnknown);
}

void PipelineDescriptor::ClearColorAttachment(size_t index) {
  if (color_attachment_descriptors_.find(index) ==
      color_attachment_descriptors_.end()) {
    return;
  }

  color_attachment_descriptors_.erase(index);
}

void PipelineDescriptor::ResetAttachments() {
  color_attachment_descriptors_.clear();
  depth_attachment_descriptor_.reset();
  front_stencil_attachment_descriptor_.reset();
  back_stencil_attachment_descriptor_.reset();
}

PixelFormat PipelineDescriptor::GetStencilPixelFormat() const {
  return stencil_pixel_format_;
}

std::optional<StencilAttachmentDescriptor>
PipelineDescriptor::GetFrontStencilAttachmentDescriptor() const {
  return front_stencil_attachment_descriptor_;
}

std::optional<DepthAttachmentDescriptor>
PipelineDescriptor::GetDepthStencilAttachmentDescriptor() const {
  return depth_attachment_descriptor_;
}

const std::map<size_t /* index */, ColorAttachmentDescriptor>&
PipelineDescriptor::GetColorAttachmentDescriptors() const {
  return color_attachment_descriptors_;
}

const std::shared_ptr<VertexDescriptor>&
PipelineDescriptor::GetVertexDescriptor() const {
  return vertex_descriptor_;
}

const std::map<ShaderStage, std::shared_ptr<const ShaderFunction>>&
PipelineDescriptor::GetStageEntrypoints() const {
  return entrypoints_;
}

std::shared_ptr<const ShaderFunction> PipelineDescriptor::GetEntrypointForStage(
    ShaderStage stage) const {
  if (auto found = entrypoints_.find(stage); found != entrypoints_.end()) {
    return found->second;
  }
  return nullptr;
}

std::string_view PipelineDescriptor::GetLabel() const {
  return label_;
}

PixelFormat PipelineDescriptor::GetDepthPixelFormat() const {
  return depth_pixel_format_;
}

std::optional<StencilAttachmentDescriptor>
PipelineDescriptor::GetBackStencilAttachmentDescriptor() const {
  return back_stencil_attachment_descriptor_;
}

bool PipelineDescriptor::HasStencilAttachmentDescriptors() const {
  return front_stencil_attachment_descriptor_.has_value() ||
         back_stencil_attachment_descriptor_.has_value();
}

void PipelineDescriptor::SetCullMode(CullMode mode) {
  cull_mode_ = mode;
}

CullMode PipelineDescriptor::GetCullMode() const {
  return cull_mode_;
}

void PipelineDescriptor::SetWindingOrder(WindingOrder order) {
  winding_order_ = order;
}

WindingOrder PipelineDescriptor::GetWindingOrder() const {
  return winding_order_;
}

void PipelineDescriptor::SetPrimitiveType(PrimitiveType type) {
  primitive_type_ = type;
}

PrimitiveType PipelineDescriptor::GetPrimitiveType() const {
  return primitive_type_;
}

void PipelineDescriptor::SetPolygonMode(PolygonMode mode) {
  polygon_mode_ = mode;
}

PolygonMode PipelineDescriptor::GetPolygonMode() const {
  return polygon_mode_;
}

void PipelineDescriptor::SetSpecializationConstants(
    std::vector<Scalar> values) {
  specialization_constants_ = std::move(values);
}

const std::vector<Scalar>& PipelineDescriptor::GetSpecializationConstants()
    const {
  return specialization_constants_;
}

}  // namespace impeller
