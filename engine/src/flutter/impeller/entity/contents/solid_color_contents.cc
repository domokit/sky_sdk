// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "solid_color_contents.h"

#include "impeller/entity/contents/content_context.h"
#include "impeller/entity/entity.h"
#include "impeller/entity/geometry/geometry.h"
#include "impeller/renderer/render_pass.h"

namespace impeller {

SolidColorContents::SolidColorContents() = default;

SolidColorContents::~SolidColorContents() = default;

void SolidColorContents::SetColor(Color color) {
  color_ = color;
}

Color SolidColorContents::GetColor() const {
  return color_.WithAlpha(color_.alpha * GetOpacityFactor());
}

bool SolidColorContents::IsSolidColor() const {
  return true;
}

bool SolidColorContents::IsOpaque(const Matrix& transform) const {
  return GetColor().IsOpaque() && !AppliesAlphaForStrokeCoverage(transform);
}

std::optional<Rect> SolidColorContents::GetCoverage(
    const Entity& entity) const {
  if (GetColor().IsTransparent()) {
    return std::nullopt;
  }

  const Geometry* geometry = GetGeometry();
  if (geometry == nullptr) {
    return std::nullopt;
  }
  return geometry->GetCoverage(entity.GetTransform());
};

bool SolidColorContents::Render(const ContentContext& renderer,
                                const Entity& entity,
                                RenderPass& pass) const {
  using VS = SolidFillPipeline::VertexShader;
  using FS = SolidFillPipeline::FragmentShader;
  auto& host_buffer = renderer.GetTransientsBuffer();

  VS::FrameInfo frame_info;
  FS::FragInfo frag_info;
  frag_info.color = GetColor().Premultiply() *
                    GetGeometry()->ComputeAlphaCoverage(entity.GetTransform());

  PipelineBuilderCallback pipeline_callback =
      [&renderer](ContentContextOptions options) {
        return renderer.GetSolidFillPipeline(options);
      };
  return ColorSourceContents::DrawGeometry<VS>(
      renderer, entity, pass, pipeline_callback, frame_info,
      [&frag_info, &host_buffer](RenderPass& pass) {
        FS::BindFragInfo(pass, host_buffer.EmplaceUniform(frag_info));
        pass.SetCommandLabel("Solid Fill");
        return true;
      });
}

std::optional<Color> SolidColorContents::AsBackgroundColor(
    const Entity& entity,
    ISize target_size) const {
  Rect target_rect = Rect::MakeSize(target_size);
  return GetGeometry()->CoversArea(entity.GetTransform(), target_rect)
             ? GetColor()
             : std::optional<Color>();
}

bool SolidColorContents::ApplyColorFilter(
    const ColorFilterProc& color_filter_proc) {
  color_ = color_filter_proc(color_);
  return true;
}

}  // namespace impeller
