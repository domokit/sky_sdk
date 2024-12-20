// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/entity/contents/contents.h"
#include <optional>

#include "fml/logging.h"
#include "impeller/base/validation.h"
#include "impeller/core/formats.h"
#include "impeller/entity/contents/anonymous_contents.h"
#include "impeller/entity/contents/content_context.h"
#include "impeller/entity/entity.h"
#include "impeller/renderer/command_buffer.h"
#include "impeller/renderer/render_pass.h"

namespace impeller {

ContentContextOptions OptionsFromPass(const RenderPass& pass) {
  ContentContextOptions opts;
  opts.sample_count = pass.GetSampleCount();
  opts.color_attachment_pixel_format = pass.GetRenderTargetPixelFormat();

  bool has_depth_stencil_attachments =
      pass.HasDepthAttachment() && pass.HasStencilAttachment();
  FML_DCHECK(pass.HasDepthAttachment() == pass.HasStencilAttachment());

  opts.has_depth_stencil_attachments = has_depth_stencil_attachments;
  opts.depth_compare = CompareFunction::kGreaterEqual;
  opts.stencil_mode = ContentContextOptions::StencilMode::kIgnore;
  return opts;
}

ContentContextOptions OptionsFromPassAndEntity(const RenderPass& pass,
                                               const Entity& entity) {
  ContentContextOptions opts = OptionsFromPass(pass);
  opts.blend_mode = entity.GetBlendMode();
  return opts;
}

std::shared_ptr<Contents> Contents::MakeAnonymous(
    Contents::RenderProc render_proc,
    Contents::CoverageProc coverage_proc) {
  return AnonymousContents::Make(std::move(render_proc),
                                 std::move(coverage_proc));
}

Contents::Contents() = default;

Contents::~Contents() = default;

bool Contents::IsOpaque(const Matrix& transform) const {
  return false;
}

std::optional<Snapshot> Contents::RenderToSnapshot(
    const ContentContext& renderer,
    const Entity& entity,
    std::optional<Rect> coverage_limit,
    const std::optional<SamplerDescriptor>& sampler_descriptor,
    bool msaa_enabled,
    int32_t mip_count,
    std::string_view label) const {
  auto coverage = GetCoverage(entity);
  if (!coverage.has_value()) {
    return std::nullopt;
  }

  std::shared_ptr<CommandBuffer> command_buffer =
      renderer.GetContext()->CreateCommandBuffer();
  if (!command_buffer) {
    return std::nullopt;
  }

  // Pad Contents snapshots with 1 pixel borders to ensure correct sampling
  // behavior. Not doing so results in a coverage leak for filters that support
  // customizing the input sampling mode. Snapshots of contents should be
  // theoretically treated as infinite size just like layers.
  coverage = coverage->Expand(1);

  if (coverage_limit.has_value()) {
    coverage = coverage->Intersection(*coverage_limit);
    if (!coverage.has_value()) {
      return std::nullopt;
    }
  }

  ISize subpass_size = ISize::Ceil(coverage->GetSize());
  fml::StatusOr<RenderTarget> render_target = renderer.MakeSubpass(
      label, subpass_size, command_buffer,
      [&contents = *this, &entity, &coverage](const ContentContext& renderer,
                                              RenderPass& pass) -> bool {
        Entity sub_entity;
        sub_entity.SetBlendMode(BlendMode::kSourceOver);
        sub_entity.SetTransform(
            Matrix::MakeTranslation(Vector3(-coverage->GetOrigin())) *
            entity.GetTransform());
        return contents.Render(renderer, sub_entity, pass);
      },
      msaa_enabled, /*depth_stencil_enabled=*/true,
      std::min(mip_count, static_cast<int32_t>(subpass_size.MipCount())));

  if (!render_target.ok()) {
    return std::nullopt;
  }
  if (!renderer.GetContext()->EnqueueCommandBuffer(std::move(command_buffer))) {
    return std::nullopt;
  }

  auto snapshot = Snapshot{
      .texture = render_target.value().GetRenderTargetTexture(),
      .transform = Matrix::MakeTranslation(coverage->GetOrigin()),
  };
  if (sampler_descriptor.has_value()) {
    snapshot.sampler_descriptor = sampler_descriptor.value();
  }

  return snapshot;
}

void Contents::SetInheritedOpacity(Scalar opacity) {
  VALIDATION_LOG << "Contents::SetInheritedOpacity should never be called when "
                    "Contents::CanAcceptOpacity returns false.";
}

std::optional<Color> Contents::AsBackgroundColor(const Entity& entity,
                                                 ISize target_size) const {
  return {};
}

bool Contents::ApplyColorFilter(
    const Contents::ColorFilterProc& color_filter_proc) {
  return false;
}

void Contents::SetCoverageHint(std::optional<Rect> coverage_hint) {
  coverage_hint_ = coverage_hint;
}

const std::optional<Rect>& Contents::GetCoverageHint() const {
  return coverage_hint_;
}

std::optional<Size> Contents::GetColorSourceSize() const {
  return color_source_size_;
};

void Contents::SetColorSourceSize(Size size) {
  color_source_size_ = size;
}

}  // namespace impeller
