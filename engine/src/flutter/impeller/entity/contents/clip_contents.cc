// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <cmath>
#include <optional>

#include "fml/logging.h"
#include "impeller/core/formats.h"
#include "impeller/core/vertex_buffer.h"
#include "impeller/entity/contents/clip_contents.h"
#include "impeller/entity/contents/content_context.h"
#include "impeller/entity/entity.h"
#include "impeller/renderer/render_pass.h"
#include "impeller/renderer/vertex_buffer_builder.h"

namespace impeller {

static Scalar GetShaderClipDepth(uint32_t clip_depth) {
  // Draw the clip at the max of the clip entity's depth slice, so that other
  // draw calls with this same depth value will be culled even if they have a
  // perspective transform.
  return std::nextafterf(Entity::GetShaderClipDepth(clip_depth + 1), 0.0f);
}

/*******************************************************************************
 ******* ClipContents
 ******************************************************************************/

ClipContents::ClipContents(Rect coverage_rect, bool is_axis_aligned_rect)
    : coverage_rect_(coverage_rect),
      is_axis_aligned_rect_(is_axis_aligned_rect) {}

ClipContents::~ClipContents() = default;

void ClipContents::SetGeometry(GeometryResult clip_geometry) {
  clip_geometry_ = std::move(clip_geometry);
}

void ClipContents::SetClipOperation(Entity::ClipOperation clip_op) {
  clip_op_ = clip_op;
}

ClipCoverage ClipContents::GetClipCoverage(
    const std::optional<Rect>& current_clip_coverage) const {
  if (!current_clip_coverage.has_value()) {
    return {.coverage = std::nullopt};
  }
  switch (clip_op_) {
    case Entity::ClipOperation::kDifference:
      // This can be optimized further by considering cases when the bounds of
      // the current stencil will shrink.
      return {
          .is_difference_or_non_square = true,  //
          .coverage = current_clip_coverage     //
      };
    case Entity::ClipOperation::kIntersect:
      if (coverage_rect_.IsEmpty() || !current_clip_coverage.has_value()) {
        return {.coverage = std::nullopt};
      }
      return {
          .is_difference_or_non_square = !is_axis_aligned_rect_,            //
          .coverage = current_clip_coverage->Intersection(coverage_rect_),  //
      };
  }
  FML_UNREACHABLE();
}

bool ClipContents::Render(const ContentContext& renderer,
                          RenderPass& pass,
                          uint32_t clip_depth) const {
  if (!clip_geometry_.vertex_buffer) {
    return true;
  }

  using VS = ClipPipeline::VertexShader;

  VS::FrameInfo info;
  info.depth = GetShaderClipDepth(clip_depth);

  auto options = OptionsFromPass(pass);
  options.blend_mode = BlendMode::kDestination;

  pass.SetStencilReference(0);

  /// Stencil preparation draw.

  options.depth_write_enabled = false;
  options.primitive_type = clip_geometry_.type;
  pass.SetVertexBuffer(clip_geometry_.vertex_buffer);
  switch (clip_geometry_.mode) {
    case GeometryResult::Mode::kNonZero:
      pass.SetCommandLabel("Clip stencil preparation (NonZero)");
      options.stencil_mode =
          ContentContextOptions::StencilMode::kStencilNonZeroFill;
      break;
    case GeometryResult::Mode::kEvenOdd:
      pass.SetCommandLabel("Clip stencil preparation (EvenOdd)");
      options.stencil_mode =
          ContentContextOptions::StencilMode::kStencilEvenOddFill;
      break;
    case GeometryResult::Mode::kNormal:
    case GeometryResult::Mode::kPreventOverdraw:
      pass.SetCommandLabel("Clip stencil preparation (Increment)");
      options.stencil_mode =
          ContentContextOptions::StencilMode::kOverdrawPreventionIncrement;
      break;
  }
  pass.SetPipeline(renderer.GetClipPipeline(options));

  info.mvp = clip_geometry_.transform;
  VS::BindFrameInfo(pass, renderer.GetTransientsBuffer().EmplaceUniform(info));

  if (!pass.Draw().ok()) {
    return false;
  }

  /// Write depth.

  options.depth_write_enabled = true;
  options.primitive_type = PrimitiveType::kTriangleStrip;
  Rect cover_area;
  switch (clip_op_) {
    case Entity::ClipOperation::kIntersect:
      pass.SetCommandLabel("Intersect Clip");
      options.stencil_mode =
          ContentContextOptions::StencilMode::kCoverCompareInverted;
      cover_area = Rect::MakeSize(pass.GetRenderTargetSize());
      break;
    case Entity::ClipOperation::kDifference:
      pass.SetCommandLabel("Difference Clip");
      options.stencil_mode = ContentContextOptions::StencilMode::kCoverCompare;
      cover_area = coverage_rect_;
      break;
  }
  auto points = cover_area.GetPoints();
  pass.SetVertexBuffer(
      CreateVertexBuffer(points, renderer.GetTransientsBuffer()));

  pass.SetPipeline(renderer.GetClipPipeline(options));

  info.mvp = pass.GetOrthographicTransform();
  VS::BindFrameInfo(pass, renderer.GetTransientsBuffer().EmplaceUniform(info));

  return pass.Draw().ok();
}

/*******************************************************************************
 ******* ClipRestoreContents
 ******************************************************************************/

bool RenderClipRestore(const ContentContext& renderer,
                       RenderPass& pass,
                       uint32_t clip_depth,
                       std::optional<Rect> restore_coverage) {
  using VS = ClipPipeline::VertexShader;

  pass.SetCommandLabel("Restore Clip");
  auto options = OptionsFromPass(pass);
  options.blend_mode = BlendMode::kDestination;
  options.stencil_mode =
      ContentContextOptions::StencilMode::kOverdrawPreventionRestore;
  options.primitive_type = PrimitiveType::kTriangleStrip;
  pass.SetPipeline(renderer.GetClipPipeline(options));
  pass.SetStencilReference(0);

  // Create a rect that covers either the given restore area, or the whole
  // render target texture.
  auto ltrb =
      restore_coverage.value_or(Rect::MakeSize(pass.GetRenderTargetSize()))
          .GetLTRB();

  std::array<VS::PerVertexData, 4> vertices = {
      VS::PerVertexData{Point(ltrb[0], ltrb[1])},
      VS::PerVertexData{Point(ltrb[2], ltrb[1])},
      VS::PerVertexData{Point(ltrb[0], ltrb[3])},
      VS::PerVertexData{Point(ltrb[2], ltrb[3])},
  };
  pass.SetVertexBuffer(
      CreateVertexBuffer(vertices, renderer.GetTransientsBuffer()));

  VS::FrameInfo info;
  info.depth = GetShaderClipDepth(clip_depth);
  info.mvp = pass.GetOrthographicTransform();
  VS::BindFrameInfo(pass, renderer.GetTransientsBuffer().EmplaceUniform(info));

  return pass.Draw().ok();
}

}  // namespace impeller
