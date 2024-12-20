// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/entity/contents/filters/color_matrix_filter_contents.h"

#include <optional>

#include "impeller/entity/contents/anonymous_contents.h"
#include "impeller/entity/contents/content_context.h"
#include "impeller/entity/contents/contents.h"
#include "impeller/entity/contents/filters/color_filter_contents.h"
#include "impeller/geometry/point.h"
#include "impeller/geometry/vector.h"
#include "impeller/renderer/render_pass.h"
#include "impeller/renderer/vertex_buffer_builder.h"

namespace impeller {

ColorMatrixFilterContents::ColorMatrixFilterContents() = default;

ColorMatrixFilterContents::~ColorMatrixFilterContents() = default;

void ColorMatrixFilterContents::SetMatrix(const ColorMatrix& matrix) {
  matrix_ = matrix;
}

std::optional<Entity> ColorMatrixFilterContents::RenderFilter(
    const FilterInput::Vector& inputs,
    const ContentContext& renderer,
    const Entity& entity,
    const Matrix& effect_transform,
    const Rect& coverage,
    const std::optional<Rect>& coverage_hint) const {
  using VS = ColorMatrixColorFilterPipeline::VertexShader;
  using FS = ColorMatrixColorFilterPipeline::FragmentShader;

  //----------------------------------------------------------------------------
  /// Handle inputs.
  ///

  if (inputs.empty()) {
    return std::nullopt;
  }

  auto input_snapshot = inputs[0]->GetSnapshot("ColorMatrix", renderer, entity);
  if (!input_snapshot.has_value()) {
    return std::nullopt;
  }

  //----------------------------------------------------------------------------
  /// Create AnonymousContents for rendering.
  ///
  RenderProc render_proc = [input_snapshot, color_matrix = matrix_,
                            absorb_opacity = GetAbsorbOpacity()](
                               const ContentContext& renderer,
                               const Entity& entity, RenderPass& pass) -> bool {
    pass.SetCommandLabel("Color Matrix Filter");

    auto options = OptionsFromPassAndEntity(pass, entity);
    options.primitive_type = PrimitiveType::kTriangleStrip;
    pass.SetPipeline(renderer.GetColorMatrixColorFilterPipeline(options));

    auto size = input_snapshot->texture->GetSize();

    std::array<VS::PerVertexData, 4> vertices = {
        VS::PerVertexData{Point(0, 0)},
        VS::PerVertexData{Point(1, 0)},
        VS::PerVertexData{Point(0, 1)},
        VS::PerVertexData{Point(1, 1)},
    };
    auto& host_buffer = renderer.GetTransientsBuffer();
    pass.SetVertexBuffer(
        CreateVertexBuffer(vertices, renderer.GetTransientsBuffer()));

    VS::FrameInfo frame_info;
    frame_info.mvp = Entity::GetShaderTransform(
        entity.GetShaderClipDepth(), pass,
        entity.GetTransform() * input_snapshot->transform *
            Matrix::MakeScale(Vector2(size)));
    frame_info.texture_sampler_y_coord_scale =
        input_snapshot->texture->GetYCoordScale();

    FS::FragInfo frag_info;
    const float* matrix = color_matrix.array;
    frag_info.color_v = Vector4(matrix[4], matrix[9], matrix[14], matrix[19]);
    // clang-format off
    frag_info.color_m = Matrix(
        matrix[0], matrix[5], matrix[10], matrix[15],
        matrix[1], matrix[6], matrix[11], matrix[16],
        matrix[2], matrix[7], matrix[12], matrix[17],
        matrix[3], matrix[8], matrix[13], matrix[18]
    );
    // clang-format on
    frag_info.input_alpha =
        absorb_opacity == ColorFilterContents::AbsorbOpacity::kYes
            ? input_snapshot->opacity
            : 1.0f;
    raw_ptr<const Sampler> sampler =
        renderer.GetContext()->GetSamplerLibrary()->GetSampler({});
    FS::BindInputTexture(pass, input_snapshot->texture, sampler);
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
