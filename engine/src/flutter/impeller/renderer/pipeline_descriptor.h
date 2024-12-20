// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_PIPELINE_DESCRIPTOR_H_
#define FLUTTER_IMPELLER_RENDERER_PIPELINE_DESCRIPTOR_H_

#include <map>
#include <memory>
#include <string>

#include "impeller/base/comparable.h"
#include "impeller/core/formats.h"
#include "impeller/core/shader_types.h"
#include "impeller/tessellator/tessellator.h"

namespace impeller {

class ShaderFunction;
class VertexDescriptor;
template <typename T>
class Pipeline;

class PipelineDescriptor final : public Comparable<PipelineDescriptor> {
 public:
  PipelineDescriptor();

  ~PipelineDescriptor();

  PipelineDescriptor& SetLabel(std::string_view label);

  std::string_view GetLabel() const;

  PipelineDescriptor& SetSampleCount(SampleCount samples);

  SampleCount GetSampleCount() const { return sample_count_; }

  PipelineDescriptor& AddStageEntrypoint(
      std::shared_ptr<const ShaderFunction> function);

  const std::map<ShaderStage, std::shared_ptr<const ShaderFunction>>&
  GetStageEntrypoints() const;

  std::shared_ptr<const ShaderFunction> GetEntrypointForStage(
      ShaderStage stage) const;

  PipelineDescriptor& SetVertexDescriptor(
      std::shared_ptr<VertexDescriptor> vertex_descriptor);

  const std::shared_ptr<VertexDescriptor>& GetVertexDescriptor() const;

  size_t GetMaxColorAttacmentBindIndex() const;

  PipelineDescriptor& SetColorAttachmentDescriptor(
      size_t index,
      ColorAttachmentDescriptor desc);

  PipelineDescriptor& SetColorAttachmentDescriptors(
      std::map<size_t /* index */, ColorAttachmentDescriptor> descriptors);

  const ColorAttachmentDescriptor* GetColorAttachmentDescriptor(
      size_t index) const;

  const std::map<size_t /* index */, ColorAttachmentDescriptor>&
  GetColorAttachmentDescriptors() const;

  const ColorAttachmentDescriptor* GetLegacyCompatibleColorAttachment() const;

  PipelineDescriptor& SetDepthStencilAttachmentDescriptor(
      std::optional<DepthAttachmentDescriptor> desc);

  std::optional<DepthAttachmentDescriptor> GetDepthStencilAttachmentDescriptor()
      const;

  PipelineDescriptor& SetStencilAttachmentDescriptors(
      std::optional<StencilAttachmentDescriptor> front_and_back);

  PipelineDescriptor& SetStencilAttachmentDescriptors(
      std::optional<StencilAttachmentDescriptor> front,
      std::optional<StencilAttachmentDescriptor> back);

  void ClearStencilAttachments();

  void ClearDepthAttachment();

  void ClearColorAttachment(size_t index);

  std::optional<StencilAttachmentDescriptor>
  GetFrontStencilAttachmentDescriptor() const;

  std::optional<StencilAttachmentDescriptor>
  GetBackStencilAttachmentDescriptor() const;

  bool HasStencilAttachmentDescriptors() const;

  PipelineDescriptor& SetDepthPixelFormat(PixelFormat format);

  PixelFormat GetDepthPixelFormat() const;

  PipelineDescriptor& SetStencilPixelFormat(PixelFormat format);

  PixelFormat GetStencilPixelFormat() const;

  // Comparable<PipelineDescriptor>
  std::size_t GetHash() const override;

  // Comparable<PipelineDescriptor>
  bool IsEqual(const PipelineDescriptor& other) const override;

  void ResetAttachments();

  void SetCullMode(CullMode mode);

  CullMode GetCullMode() const;

  void SetWindingOrder(WindingOrder order);

  WindingOrder GetWindingOrder() const;

  void SetPrimitiveType(PrimitiveType type);

  PrimitiveType GetPrimitiveType() const;

  void SetPolygonMode(PolygonMode mode);

  PolygonMode GetPolygonMode() const;

  void SetSpecializationConstants(std::vector<Scalar> values);

  const std::vector<Scalar>& GetSpecializationConstants() const;

 private:
  std::string label_;
  SampleCount sample_count_ = SampleCount::kCount1;
  WindingOrder winding_order_ = WindingOrder::kClockwise;
  CullMode cull_mode_ = CullMode::kNone;
  std::map<ShaderStage, std::shared_ptr<const ShaderFunction>> entrypoints_;
  std::map<size_t /* index */, ColorAttachmentDescriptor>
      color_attachment_descriptors_;
  std::shared_ptr<VertexDescriptor> vertex_descriptor_;
  PixelFormat depth_pixel_format_ = PixelFormat::kUnknown;
  PixelFormat stencil_pixel_format_ = PixelFormat::kUnknown;
  std::optional<DepthAttachmentDescriptor> depth_attachment_descriptor_;
  std::optional<StencilAttachmentDescriptor>
      front_stencil_attachment_descriptor_;
  std::optional<StencilAttachmentDescriptor>
      back_stencil_attachment_descriptor_;
  PrimitiveType primitive_type_ = PrimitiveType::kTriangle;
  PolygonMode polygon_mode_ = PolygonMode::kFill;
  std::vector<Scalar> specialization_constants_;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_PIPELINE_DESCRIPTOR_H_
