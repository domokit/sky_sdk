// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "linear_gradient_contents.h"

#include "impeller/core/formats.h"
#include "impeller/entity/contents/content_context.h"
#include "impeller/entity/contents/gradient_generator.h"
#include "impeller/entity/entity.h"
#include "impeller/entity/geometry/geometry.h"
#include "impeller/geometry/scalar.h"
#include "impeller/renderer/render_pass.h"
#include "impeller/renderer/vertex_buffer_builder.h"

namespace impeller {

LinearGradientContents::LinearGradientContents() = default;

LinearGradientContents::~LinearGradientContents() = default;

void LinearGradientContents::SetEndPoints(Point start_point, Point end_point) {
  start_point_ = start_point;
  end_point_ = end_point;
}

void LinearGradientContents::SetColors(std::vector<Color> colors) {
  colors_ = std::move(colors);
}

void LinearGradientContents::SetStops(std::vector<Scalar> stops) {
  stops_ = std::move(stops);
}

const std::vector<Color>& LinearGradientContents::GetColors() const {
  return colors_;
}

const std::vector<Scalar>& LinearGradientContents::GetStops() const {
  return stops_;
}

void LinearGradientContents::SetTileMode(Entity::TileMode tile_mode) {
  tile_mode_ = tile_mode;
}

bool LinearGradientContents::IsOpaque(const Matrix& transform) const {
  if (GetOpacityFactor() < 1 || tile_mode_ == Entity::TileMode::kDecal) {
    return false;
  }
  for (auto color : colors_) {
    if (!color.IsOpaque()) {
      return false;
    }
  }
  return !AppliesAlphaForStrokeCoverage(transform);
}

bool LinearGradientContents::CanApplyFastGradient() const {
  if (!GetInverseEffectTransform().IsIdentity()) {
    return false;
  }
  std::optional<Rect> maybe_rect = GetGeometry()->GetCoverage(Matrix());
  if (!maybe_rect.has_value()) {
    return false;
  }
  Rect rect = maybe_rect.value();

  if (ScalarNearlyEqual(start_point_.x, end_point_.x)) {
    // Sort start and end to make on-rect comparisons easier.
    Point start = (start_point_.y < end_point_.y) ? start_point_ : end_point_;
    Point end = (start_point_.y < end_point_.y) ? end_point_ : start_point_;
    // The exact x positon doesn't matter for a vertical gradient, but the y
    // position must be nearly on the rectangle.
    if (ScalarNearlyEqual(start.y, rect.GetTop()) &&
        ScalarNearlyEqual(end.y, rect.GetBottom())) {
      return true;
    }
    return false;
  }

  if (ScalarNearlyEqual(start_point_.y, end_point_.y)) {
    // Sort start and end to make on-rect comparisons easier.
    Point start = (start_point_.x < end_point_.x) ? start_point_ : end_point_;
    Point end = (start_point_.x < end_point_.x) ? end_point_ : start_point_;
    // The exact y positon doesn't matter for a horizontal gradient, but the x
    // position must be nearly on the rectangle.
    if (ScalarNearlyEqual(start.x, rect.GetLeft()) &&
        ScalarNearlyEqual(end.x, rect.GetRight())) {
      return true;
    }
    return false;
  }

  return false;
}

// A much faster (in terms of ALU) linear gradient that uses vertex
// interpolation to perform all color computation. Requires that the geometry of
// the gradient is divided into regions based on the stop values.
// Currently restricted to rect geometry where the start and end points are
// perfectly horizontal/vertical, but could easily be expanded to StC cases
// provided that the start/end are on or outside of the coverage rect.
bool LinearGradientContents::FastLinearGradient(const ContentContext& renderer,
                                                const Entity& entity,
                                                RenderPass& pass) const {
  using VS = FastGradientPipeline::VertexShader;
  using FS = FastGradientPipeline::FragmentShader;

  const Geometry* geometry = GetGeometry();
  bool force_stencil = !geometry->IsAxisAlignedRect();

  auto geom_callback = [&](const ContentContext& renderer, const Entity& entity,
                           RenderPass& pass,
                           const Geometry* geometry) -> GeometryResult {
    // We already know this is an axis aligned rectangle, so the coverage will
    // be approximately the same as the geometry. For non axis-algined
    // rectangles, we can force stencil then cover (not done here). We give an
    // identity transform to avoid double transforming the gradient.
    std::optional<Rect> maybe_rect = geometry->GetCoverage(Matrix());
    if (!maybe_rect.has_value()) {
      return {};
    }
    Rect rect = maybe_rect.value();
    bool horizontal_axis = start_point_.y == end_point_.y;

    // Compute the locations of each breakpoint along the primary axis, then
    // create a rectangle that joins each segment. There will be two triangles
    // between each pair of points.
    VertexBufferBuilder<VS::PerVertexData> vtx_builder;
    vtx_builder.Reserve(6 * (stops_.size() - 1));
    Point prev = start_point_;
    for (auto i = 1u; i < stops_.size(); i++) {
      Scalar t = stops_[i];
      Point current = (1.0 - t) * start_point_ + t * end_point_;
      Rect section = horizontal_axis
                         ? Rect::MakeXYWH(prev.x, rect.GetY(),
                                          current.x - prev.x, rect.GetHeight())

                         : Rect::MakeXYWH(rect.GetX(), prev.y, rect.GetWidth(),
                                          current.y - prev.y);
      vtx_builder.AddVertices({
          {section.GetLeftTop(), colors_[i - 1]},
          {section.GetRightTop(),
           horizontal_axis ? colors_[i] : colors_[i - 1]},
          {section.GetLeftBottom(),
           horizontal_axis ? colors_[i - 1] : colors_[i]},
          {section.GetRightTop(),
           horizontal_axis ? colors_[i] : colors_[i - 1]},
          {section.GetLeftBottom(),
           horizontal_axis ? colors_[i - 1] : colors_[i]},
          {section.GetRightBottom(), colors_[i]},
      });
      prev = current;
    }
    return GeometryResult{
        .type = PrimitiveType::kTriangle,
        .vertex_buffer =
            vtx_builder.CreateVertexBuffer(renderer.GetTransientsBuffer()),
        .transform = entity.GetShaderTransform(pass),
    };
  };

  pass.SetLabel("LinearGradient");

  VS::FrameInfo frame_info;

  PipelineBuilderCallback pipeline_callback =
      [&renderer](ContentContextOptions options) {
        return renderer.GetFastGradientPipeline(options);
      };
  return ColorSourceContents::DrawGeometry<VS>(
      renderer, entity, pass, pipeline_callback, frame_info,
      [this, &renderer, &entity](RenderPass& pass) {
        auto& host_buffer = renderer.GetTransientsBuffer();

        FS::FragInfo frag_info;
        frag_info.alpha =
            GetOpacityFactor() *
            GetGeometry()->ComputeAlphaCoverage(entity.GetTransform());

        FS::BindFragInfo(pass, host_buffer.EmplaceUniform(frag_info));

        return true;
      },
      /*force_stencil=*/force_stencil, geom_callback);
}

#define ARRAY_LEN(a) (sizeof(a) / sizeof(a[0]))
#define UNIFORM_FRAG_INFO(t) \
  t##GradientUniformFillPipeline::FragmentShader::FragInfo
#define UNIFORM_COLOR_SIZE ARRAY_LEN(UNIFORM_FRAG_INFO(Linear)::colors)
#define UNIFORM_STOP_SIZE ARRAY_LEN(UNIFORM_FRAG_INFO(Linear)::stop_pairs)
static_assert(UNIFORM_COLOR_SIZE == kMaxUniformGradientStops);
static_assert(UNIFORM_STOP_SIZE == kMaxUniformGradientStops / 2);

bool LinearGradientContents::Render(const ContentContext& renderer,
                                    const Entity& entity,
                                    RenderPass& pass) const {
  // TODO(148651): The fast path is overly restrictive, following the design in
  // https://github.com/flutter/flutter/issues/148651 support for more cases can
  // be gradually added.
  if (CanApplyFastGradient()) {
    return FastLinearGradient(renderer, entity, pass);
  }
  if (renderer.GetDeviceCapabilities().SupportsSSBO()) {
    return RenderSSBO(renderer, entity, pass);
  }
  if (colors_.size() <= kMaxUniformGradientStops &&
      stops_.size() <= kMaxUniformGradientStops) {
    return RenderUniform(renderer, entity, pass);
  }
  return RenderTexture(renderer, entity, pass);
}

bool LinearGradientContents::RenderTexture(const ContentContext& renderer,
                                           const Entity& entity,
                                           RenderPass& pass) const {
  using VS = LinearGradientFillPipeline::VertexShader;
  using FS = LinearGradientFillPipeline::FragmentShader;

  VS::FrameInfo frame_info;
  frame_info.matrix = GetInverseEffectTransform();

  PipelineBuilderCallback pipeline_callback =
      [&renderer](ContentContextOptions options) {
        return renderer.GetLinearGradientFillPipeline(options);
      };
  return ColorSourceContents::DrawGeometry<VS>(
      renderer, entity, pass, pipeline_callback, frame_info,
      [this, &renderer, &entity](RenderPass& pass) {
        auto gradient_data = CreateGradientBuffer(colors_, stops_);
        auto gradient_texture =
            CreateGradientTexture(gradient_data, renderer.GetContext());
        if (gradient_texture == nullptr) {
          return false;
        }

        FS::FragInfo frag_info;
        frag_info.start_point = start_point_;
        frag_info.end_point = end_point_;
        frag_info.tile_mode = static_cast<Scalar>(tile_mode_);
        frag_info.decal_border_color = decal_border_color_;
        frag_info.texture_sampler_y_coord_scale =
            gradient_texture->GetYCoordScale();
        frag_info.alpha =
            GetOpacityFactor() *
            GetGeometry()->ComputeAlphaCoverage(entity.GetTransform());
        ;
        frag_info.half_texel =
            Vector2(0.5 / gradient_texture->GetSize().width,
                    0.5 / gradient_texture->GetSize().height);

        pass.SetCommandLabel("LinearGradientFill");

        SamplerDescriptor sampler_desc;
        sampler_desc.min_filter = MinMagFilter::kLinear;
        sampler_desc.mag_filter = MinMagFilter::kLinear;

        FS::BindTextureSampler(
            pass, std::move(gradient_texture),
            renderer.GetContext()->GetSamplerLibrary()->GetSampler(
                sampler_desc));
        FS::BindFragInfo(
            pass, renderer.GetTransientsBuffer().EmplaceUniform(frag_info));
        return true;
      });
}

namespace {
Scalar CalculateInverseDotStartToEnd(Point start_point, Point end_point) {
  Point start_to_end = end_point - start_point;
  Scalar dot =
      (start_to_end.x * start_to_end.x + start_to_end.y * start_to_end.y);
  return dot == 0.0f ? 0.0f : 1.0f / dot;
}
}  // namespace

bool LinearGradientContents::RenderSSBO(const ContentContext& renderer,
                                        const Entity& entity,
                                        RenderPass& pass) const {
  using VS = LinearGradientSSBOFillPipeline::VertexShader;
  using FS = LinearGradientSSBOFillPipeline::FragmentShader;

  VS::FrameInfo frame_info;
  frame_info.matrix = GetInverseEffectTransform();

  PipelineBuilderCallback pipeline_callback =
      [&renderer](ContentContextOptions options) {
        return renderer.GetLinearGradientSSBOFillPipeline(options);
      };
  return ColorSourceContents::DrawGeometry<VS>(
      renderer, entity, pass, pipeline_callback, frame_info,
      [this, &renderer, &entity](RenderPass& pass) {
        FS::FragInfo frag_info;
        frag_info.start_point = start_point_;
        frag_info.end_point = end_point_;
        frag_info.tile_mode = static_cast<Scalar>(tile_mode_);
        frag_info.decal_border_color = decal_border_color_;
        frag_info.alpha =
            GetOpacityFactor() *
            GetGeometry()->ComputeAlphaCoverage(entity.GetTransform());
        frag_info.start_to_end = end_point_ - start_point_;
        frag_info.inverse_dot_start_to_end =
            CalculateInverseDotStartToEnd(start_point_, end_point_);

        auto& host_buffer = renderer.GetTransientsBuffer();
        auto colors = CreateGradientColors(colors_, stops_);

        frag_info.colors_length = colors.size();
        auto color_buffer =
            host_buffer.Emplace(colors.data(), colors.size() * sizeof(StopData),
                                DefaultUniformAlignment());

        pass.SetCommandLabel("LinearGradientSSBOFill");

        FS::BindFragInfo(
            pass, renderer.GetTransientsBuffer().EmplaceUniform(frag_info));
        FS::BindColorData(pass, color_buffer);

        return true;
      });
}

bool LinearGradientContents::RenderUniform(const ContentContext& renderer,
                                           const Entity& entity,
                                           RenderPass& pass) const {
  using VS = LinearGradientUniformFillPipeline::VertexShader;
  using FS = LinearGradientUniformFillPipeline::FragmentShader;

  VS::FrameInfo frame_info;
  frame_info.matrix = GetInverseEffectTransform();

  PipelineBuilderCallback pipeline_callback =
      [&renderer](ContentContextOptions options) {
        return renderer.GetLinearGradientUniformFillPipeline(options);
      };
  return ColorSourceContents::DrawGeometry<VS>(
      renderer, entity, pass, pipeline_callback, frame_info,
      [this, &renderer, &entity](RenderPass& pass) {
        FS::FragInfo frag_info;
        frag_info.start_point = start_point_;
        frag_info.start_to_end = end_point_ - start_point_;
        frag_info.alpha =
            GetOpacityFactor() *
            GetGeometry()->ComputeAlphaCoverage(entity.GetTransform());
        frag_info.tile_mode = static_cast<Scalar>(tile_mode_);
        frag_info.colors_length = PopulateUniformGradientColors(
            colors_, stops_, frag_info.colors, frag_info.stop_pairs);
        frag_info.inverse_dot_start_to_end =
            CalculateInverseDotStartToEnd(start_point_, end_point_);
        frag_info.decal_border_color = decal_border_color_;

        pass.SetCommandLabel("LinearGradientUniformFill");

        FS::BindFragInfo(
            pass, renderer.GetTransientsBuffer().EmplaceUniform(frag_info));

        return true;
      });
}

bool LinearGradientContents::ApplyColorFilter(
    const ColorFilterProc& color_filter_proc) {
  for (Color& color : colors_) {
    color = color_filter_proc(color);
  }
  decal_border_color_ = color_filter_proc(decal_border_color_);
  return true;
}

}  // namespace impeller
