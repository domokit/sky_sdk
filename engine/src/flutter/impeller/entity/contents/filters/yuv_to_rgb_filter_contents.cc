// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/entity/contents/filters/yuv_to_rgb_filter_contents.h"

#include "impeller/core/formats.h"
#include "impeller/entity/contents/anonymous_contents.h"
#include "impeller/entity/contents/content_context.h"
#include "impeller/geometry/matrix.h"
#include "impeller/renderer/render_pass.h"
#include "impeller/renderer/vertex_buffer_builder.h"

namespace impeller {

// clang-format off
constexpr Matrix kMatrixBT601LimitedRange = {
    1.164,  1.164, 1.164, 0.0,
      0.0, -0.392, 2.017, 0.0,
    1.596, -0.813,   0.0, 0.0,
      0.0,    0.0,   0.0, 1.0
};

constexpr Matrix kMatrixBT601FullRange = {
      1.0,    1.0,   1.0, 0.0,
      0.0, -0.344, 1.772, 0.0,
    1.402, -0.714,   0.0, 0.0,
      0.0,    0.0,   0.0, 1.0
};
// clang-format on

YUVToRGBFilterContents::YUVToRGBFilterContents() = default;

YUVToRGBFilterContents::~YUVToRGBFilterContents() = default;

void YUVToRGBFilterContents::SetYUVColorSpace(YUVColorSpace yuv_color_space) {
  yuv_color_space_ = yuv_color_space;
}

std::optional<Entity> YUVToRGBFilterContents::RenderFilter(
    const FilterInput::Vector& inputs,
    const ContentContext& renderer,
    const Entity& entity,
    const Matrix& effect_transform,
    const Rect& coverage,
    const std::optional<Rect>& coverage_hint) const {
  if (inputs.size() < 2) {
    return std::nullopt;
  }

  using VS = YUVToRGBFilterPipeline::VertexShader;
  using FS = YUVToRGBFilterPipeline::FragmentShader;

  auto y_input_snapshot =
      inputs[0]->GetSnapshot("YUVToRGB(Y)", renderer, entity);
  auto uv_input_snapshot =
      inputs[1]->GetSnapshot("YUVToRGB(UV)", renderer, entity);
  if (!y_input_snapshot.has_value() || !uv_input_snapshot.has_value()) {
    return std::nullopt;
  }

  if (y_input_snapshot->texture->GetTextureDescriptor().format !=
          PixelFormat::kR8UNormInt ||
      uv_input_snapshot->texture->GetTextureDescriptor().format !=
          PixelFormat::kR8G8UNormInt) {
    return std::nullopt;
  }

  //----------------------------------------------------------------------------
  /// Create AnonymousContents for rendering.
  ///
  RenderProc render_proc = [y_input_snapshot, uv_input_snapshot,
                            yuv_color_space = yuv_color_space_](
                               const ContentContext& renderer,
                               const Entity& entity, RenderPass& pass) -> bool {
    pass.SetCommandLabel("YUV to RGB Filter");

    auto options = OptionsFromPassAndEntity(pass, entity);
    options.primitive_type = PrimitiveType::kTriangleStrip;
    pass.SetPipeline(renderer.GetYUVToRGBFilterPipeline(options));

    auto size = y_input_snapshot->texture->GetSize();

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
        entity.GetTransform() * y_input_snapshot->transform *
            Matrix::MakeScale(Vector2(size)));
    frame_info.texture_sampler_y_coord_scale =
        y_input_snapshot->texture->GetYCoordScale();

    FS::FragInfo frag_info;
    frag_info.yuv_color_space = static_cast<Scalar>(yuv_color_space);
    switch (yuv_color_space) {
      case YUVColorSpace::kBT601LimitedRange:
        frag_info.matrix = kMatrixBT601LimitedRange;
        break;
      case YUVColorSpace::kBT601FullRange:
        frag_info.matrix = kMatrixBT601FullRange;
        break;
    }

    raw_ptr<const Sampler> sampler =
        renderer.GetContext()->GetSamplerLibrary()->GetSampler({});
    FS::BindYTexture(pass, y_input_snapshot->texture, sampler);
    FS::BindUvTexture(pass, uv_input_snapshot->texture, sampler);

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

std::optional<Rect> YUVToRGBFilterContents::GetFilterSourceCoverage(
    const Matrix& effect_transform,
    const Rect& output_limit) const {
  return output_limit;
}

}  // namespace impeller
