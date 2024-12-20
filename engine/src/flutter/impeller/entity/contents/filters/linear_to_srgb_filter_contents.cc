// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/entity/contents/filters/linear_to_srgb_filter_contents.h"

#include "impeller/entity/contents/anonymous_contents.h"
#include "impeller/entity/contents/content_context.h"
#include "impeller/entity/contents/contents.h"
#include "impeller/geometry/point.h"
#include "impeller/renderer/render_pass.h"
#include "impeller/renderer/vertex_buffer_builder.h"

namespace impeller {

LinearToSrgbFilterContents::LinearToSrgbFilterContents() = default;

LinearToSrgbFilterContents::~LinearToSrgbFilterContents() = default;

std::optional<Entity> LinearToSrgbFilterContents::RenderFilter(
    const FilterInput::Vector& inputs,
    const ContentContext& renderer,
    const Entity& entity,
    const Matrix& effect_transform,
    const Rect& coverage,
    const std::optional<Rect>& coverage_hint) const {
  if (inputs.empty()) {
    return std::nullopt;
  }

  using VS = LinearToSrgbFilterPipeline::VertexShader;
  using FS = LinearToSrgbFilterPipeline::FragmentShader;

  auto input_snapshot =
      inputs[0]->GetSnapshot("LinearToSrgb", renderer, entity);
  if (!input_snapshot.has_value()) {
    return std::nullopt;
  }

  //----------------------------------------------------------------------------
  /// Create AnonymousContents for rendering.
  ///
  RenderProc render_proc = [input_snapshot,
                            absorb_opacity = GetAbsorbOpacity()](
                               const ContentContext& renderer,
                               const Entity& entity, RenderPass& pass) -> bool {
    pass.SetCommandLabel("Linear to sRGB Filter");

    auto options = OptionsFromPassAndEntity(pass, entity);
    options.primitive_type = PrimitiveType::kTriangleStrip;
    pass.SetPipeline(renderer.GetLinearToSrgbFilterPipeline(options));

    auto size = input_snapshot->texture->GetSize();
    std::array<VS::PerVertexData, 4> vertices = {
        VS::PerVertexData{Point(0, 0)},
        VS::PerVertexData{Point(1, 0)},
        VS::PerVertexData{Point(0, 1)},
        VS::PerVertexData{Point(1, 1)},
    };

    auto& host_buffer = renderer.GetTransientsBuffer();
    pass.SetVertexBuffer(CreateVertexBuffer(vertices, host_buffer));

    VS::FrameInfo frame_info;
    frame_info.mvp = Entity::GetShaderTransform(
        entity.GetShaderClipDepth(), pass,
        entity.GetTransform() * input_snapshot->transform *
            Matrix::MakeScale(Vector2(size)));
    frame_info.texture_sampler_y_coord_scale =
        input_snapshot->texture->GetYCoordScale();

    FS::FragInfo frag_info;
    frag_info.input_alpha =
        absorb_opacity == ColorFilterContents::AbsorbOpacity::kYes
            ? input_snapshot->opacity
            : 1.0f;

    FS::BindInputTexture(
        pass, input_snapshot->texture,
        renderer.GetContext()->GetSamplerLibrary()->GetSampler({}));
    FS::BindFragInfo(pass, host_buffer.EmplaceUniform(frag_info));
    VS::BindFrameInfo(pass, host_buffer.EmplaceUniform(frame_info));

    return pass.Draw().ok();
  };

  CoverageProc coverage_proc =
      [coverage](const Entity& entity) -> std::optional<Rect> {
    return coverage.TransformBounds(entity.GetTransform());
  };

  auto contents = AnonymousContents::Make(render_proc, coverage_proc);

  Entity sub_entity;
  sub_entity.SetContents(std::move(contents));
  sub_entity.SetBlendMode(entity.GetBlendMode());
  return sub_entity;
}

}  // namespace impeller
