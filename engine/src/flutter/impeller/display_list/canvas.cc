// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/display_list/canvas.h"

#include <memory>
#include <optional>
#include <unordered_map>
#include <utility>

#include "display_list/effects/dl_color_source.h"
#include "display_list/effects/dl_image_filter.h"
#include "flutter/fml/logging.h"
#include "flutter/fml/trace_event.h"
#include "impeller/base/validation.h"
#include "impeller/core/formats.h"
#include "impeller/display_list/color_filter.h"
#include "impeller/display_list/image_filter.h"
#include "impeller/display_list/skia_conversions.h"
#include "impeller/entity/contents/atlas_contents.h"
#include "impeller/entity/contents/clip_contents.h"
#include "impeller/entity/contents/color_source_contents.h"
#include "impeller/entity/contents/content_context.h"
#include "impeller/entity/contents/filters/filter_contents.h"
#include "impeller/entity/contents/framebuffer_blend_contents.h"
#include "impeller/entity/contents/solid_rrect_blur_contents.h"
#include "impeller/entity/contents/text_contents.h"
#include "impeller/entity/contents/texture_contents.h"
#include "impeller/entity/contents/vertices_contents.h"
#include "impeller/entity/geometry/circle_geometry.h"
#include "impeller/entity/geometry/cover_geometry.h"
#include "impeller/entity/geometry/ellipse_geometry.h"
#include "impeller/entity/geometry/fill_path_geometry.h"
#include "impeller/entity/geometry/geometry.h"
#include "impeller/entity/geometry/line_geometry.h"
#include "impeller/entity/geometry/point_field_geometry.h"
#include "impeller/entity/geometry/rect_geometry.h"
#include "impeller/entity/geometry/round_rect_geometry.h"
#include "impeller/entity/geometry/round_superellipse_geometry.h"
#include "impeller/entity/geometry/stroke_path_geometry.h"
#include "impeller/entity/save_layer_utils.h"
#include "impeller/geometry/color.h"
#include "impeller/geometry/constants.h"
#include "impeller/geometry/path_builder.h"

namespace impeller {

namespace {

static bool UseColorSourceContents(
    const std::shared_ptr<VerticesGeometry>& vertices,
    const Paint& paint) {
  // If there are no vertex color or texture coordinates. Or if there
  // are vertex coordinates but its just a color.
  if (vertices->HasVertexColors()) {
    return false;
  }
  if (vertices->HasTextureCoordinates() && !paint.color_source) {
    return true;
  }
  return !vertices->HasTextureCoordinates();
}

static void SetClipScissor(std::optional<Rect> clip_coverage,
                           RenderPass& pass,
                           Point global_pass_position) {
  // Set the scissor to the clip coverage area. We do this prior to rendering
  // the clip itself and all its contents.
  IRect scissor;
  if (clip_coverage.has_value()) {
    clip_coverage = clip_coverage->Shift(-global_pass_position);
    scissor = IRect::RoundOut(clip_coverage.value());
    // The scissor rect must not exceed the size of the render target.
    scissor = scissor.Intersection(IRect::MakeSize(pass.GetRenderTargetSize()))
                  .value_or(IRect());
  }
  pass.SetScissor(scissor);
}

static void ApplyFramebufferBlend(Entity& entity) {
  auto src_contents = entity.GetContents();
  auto contents = std::make_shared<FramebufferBlendContents>();
  contents->SetChildContents(src_contents);
  contents->SetBlendMode(entity.GetBlendMode());
  entity.SetContents(std::move(contents));
  entity.SetBlendMode(BlendMode::kSource);
}

/// @brief Create the subpass restore contents, appling any filters or opacity
///        from the provided paint object.
static std::shared_ptr<Contents> CreateContentsForSubpassTarget(
    const Paint& paint,
    const std::shared_ptr<Texture>& target,
    const Matrix& effect_transform) {
  auto contents = TextureContents::MakeRect(Rect::MakeSize(target->GetSize()));
  contents->SetTexture(target);
  contents->SetLabel("Subpass");
  contents->SetSourceRect(Rect::MakeSize(target->GetSize()));
  contents->SetOpacity(paint.color.alpha);
  contents->SetDeferApplyingOpacity(true);

  return paint.WithFiltersForSubpassTarget(std::move(contents),
                                           effect_transform);
}

static const constexpr RenderTarget::AttachmentConfig kDefaultStencilConfig =
    RenderTarget::AttachmentConfig{
        .storage_mode = StorageMode::kDeviceTransient,
        .load_action = LoadAction::kDontCare,
        .store_action = StoreAction::kDontCare,
    };

static std::unique_ptr<EntityPassTarget> CreateRenderTarget(
    ContentContext& renderer,
    ISize size,
    const Color& clear_color) {
  const std::shared_ptr<Context>& context = renderer.GetContext();

  /// All of the load/store actions are managed by `InlinePassContext` when
  /// `RenderPasses` are created, so we just set them to `kDontCare` here.
  /// What's important is the `StorageMode` of the textures, which cannot be
  /// changed for the lifetime of the textures.

  RenderTarget target;
  if (context->GetCapabilities()->SupportsOffscreenMSAA()) {
    target = renderer.GetRenderTargetCache()->CreateOffscreenMSAA(
        /*context=*/*context,
        /*size=*/size,
        /*mip_count=*/1,
        /*label=*/"EntityPass",
        /*color_attachment_config=*/
        RenderTarget::AttachmentConfigMSAA{
            .storage_mode = StorageMode::kDeviceTransient,
            .resolve_storage_mode = StorageMode::kDevicePrivate,
            .load_action = LoadAction::kDontCare,
            .store_action = StoreAction::kMultisampleResolve,
            .clear_color = clear_color},
        /*stencil_attachment_config=*/kDefaultStencilConfig);
  } else {
    target = renderer.GetRenderTargetCache()->CreateOffscreen(
        *context,  // context
        size,      // size
        /*mip_count=*/1,
        "EntityPass",  // label
        RenderTarget::AttachmentConfig{
            .storage_mode = StorageMode::kDevicePrivate,
            .load_action = LoadAction::kDontCare,
            .store_action = StoreAction::kDontCare,
            .clear_color = clear_color,
        },                     // color_attachment_config
        kDefaultStencilConfig  //
    );
  }

  return std::make_unique<EntityPassTarget>(
      target, renderer.GetDeviceCapabilities().SupportsReadFromResolve(),
      renderer.GetDeviceCapabilities().SupportsImplicitResolvingMSAA());
}

}  // namespace

Canvas::Canvas(ContentContext& renderer,
               const RenderTarget& render_target,
               bool requires_readback)
    : renderer_(renderer),
      render_target_(render_target),
      requires_readback_(requires_readback),
      clip_coverage_stack_(EntityPassClipStack(
          Rect::MakeSize(render_target.GetRenderTargetSize()))) {
  Initialize(std::nullopt);
  SetupRenderPass();
}

Canvas::Canvas(ContentContext& renderer,
               const RenderTarget& render_target,
               bool requires_readback,
               Rect cull_rect)
    : renderer_(renderer),
      render_target_(render_target),
      requires_readback_(requires_readback),
      clip_coverage_stack_(EntityPassClipStack(
          Rect::MakeSize(render_target.GetRenderTargetSize()))) {
  Initialize(cull_rect);
  SetupRenderPass();
}

Canvas::Canvas(ContentContext& renderer,
               const RenderTarget& render_target,
               bool requires_readback,
               IRect cull_rect)
    : renderer_(renderer),
      render_target_(render_target),
      requires_readback_(requires_readback),
      clip_coverage_stack_(EntityPassClipStack(
          Rect::MakeSize(render_target.GetRenderTargetSize()))) {
  Initialize(Rect::MakeLTRB(cull_rect.GetLeft(), cull_rect.GetTop(),
                            cull_rect.GetRight(), cull_rect.GetBottom()));
  SetupRenderPass();
}

void Canvas::Initialize(std::optional<Rect> cull_rect) {
  initial_cull_rect_ = cull_rect;
  transform_stack_.emplace_back(CanvasStackEntry{
      .clip_depth = kMaxDepth,
  });
  FML_DCHECK(GetSaveCount() == 1u);
}

void Canvas::Reset() {
  current_depth_ = 0u;
  transform_stack_ = {};
}

void Canvas::Concat(const Matrix& transform) {
  transform_stack_.back().transform = GetCurrentTransform() * transform;
}

void Canvas::PreConcat(const Matrix& transform) {
  transform_stack_.back().transform = transform * GetCurrentTransform();
}

void Canvas::ResetTransform() {
  transform_stack_.back().transform = {};
}

void Canvas::Transform(const Matrix& transform) {
  Concat(transform);
}

const Matrix& Canvas::GetCurrentTransform() const {
  return transform_stack_.back().transform;
}

void Canvas::Translate(const Vector3& offset) {
  Concat(Matrix::MakeTranslation(offset));
}

void Canvas::Scale(const Vector2& scale) {
  Concat(Matrix::MakeScale(scale));
}

void Canvas::Scale(const Vector3& scale) {
  Concat(Matrix::MakeScale(scale));
}

void Canvas::Skew(Scalar sx, Scalar sy) {
  Concat(Matrix::MakeSkew(sx, sy));
}

void Canvas::Rotate(Radians radians) {
  Concat(Matrix::MakeRotationZ(radians));
}

Point Canvas::GetGlobalPassPosition() const {
  if (save_layer_state_.empty()) {
    return Point(0, 0);
  }
  return save_layer_state_.back().coverage.GetOrigin();
}

// clip depth of the previous save or 0.
size_t Canvas::GetClipHeightFloor() const {
  if (transform_stack_.size() > 1) {
    return transform_stack_[transform_stack_.size() - 2].clip_height;
  }
  return 0;
}

size_t Canvas::GetSaveCount() const {
  return transform_stack_.size();
}

bool Canvas::IsSkipping() const {
  return transform_stack_.back().skipping;
}

void Canvas::RestoreToCount(size_t count) {
  while (GetSaveCount() > count) {
    if (!Restore()) {
      return;
    }
  }
}

void Canvas::DrawPath(const Path& path, const Paint& paint) {
  Entity entity;
  entity.SetTransform(GetCurrentTransform());
  entity.SetBlendMode(paint.blend_mode);

  if (paint.style == Paint::Style::kFill) {
    FillPathGeometry geom(path);
    AddRenderEntityWithFiltersToCurrentPass(entity, &geom, paint);
  } else {
    StrokePathGeometry geom(path, paint.stroke_width, paint.stroke_miter,
                            paint.stroke_cap, paint.stroke_join);
    AddRenderEntityWithFiltersToCurrentPass(entity, &geom, paint);
  }
}

void Canvas::DrawPaint(const Paint& paint) {
  Entity entity;
  entity.SetTransform(GetCurrentTransform());
  entity.SetBlendMode(paint.blend_mode);

  CoverGeometry geom;
  AddRenderEntityWithFiltersToCurrentPass(entity, &geom, paint);
}

bool Canvas::AttemptDrawBlurredRRect(const Rect& rect,
                                     Size corner_radii,
                                     const Paint& paint) {
  if (paint.style != Paint::Style::kFill) {
    return false;
  }

  if (paint.color_source) {
    return false;
  }

  if (!paint.mask_blur_descriptor.has_value()) {
    return false;
  }

  // A blur sigma that is not positive enough should not result in a blur.
  if (paint.mask_blur_descriptor->sigma.sigma <= kEhCloseEnough) {
    return false;
  }

  // The current rrect blur math doesn't work on ovals.
  if (fabsf(corner_radii.width - corner_radii.height) > kEhCloseEnough) {
    return false;
  }

  // For symmetrically mask blurred solid RRects, absorb the mask blur and use
  // a faster SDF approximation.
  Color rrect_color = paint.color;
  if (paint.invert_colors) {
    rrect_color = rrect_color.ApplyColorMatrix(kColorInversion);
  }
  if (paint.color_filter) {
    rrect_color = GetCPUColorFilterProc(paint.color_filter)(rrect_color);
  }

  Paint rrect_paint = {.mask_blur_descriptor = paint.mask_blur_descriptor};

  // In some cases, we need to render the mask blur to a separate layer.
  //
  //   1. If the blur style is normal, we'll be drawing using one draw call and
  //      no clips. And so we can just wrap the RRect contents with the
  //      ImageFilter, which will get applied to the result as per usual.
  //
  //   2. If the blur style is solid, we combine the non-blurred RRect with the
  //      blurred RRect via two separate draw calls, and so we need to defer any
  //      fancy blending, translucency, or image filtering until after these two
  //      draws have been combined in a separate layer.
  //
  //   3. If the blur style is outer or inner, we apply the blur style via a
  //      clip. The ImageFilter needs to be applied to the mask blurred result.
  //      And so if there's an ImageFilter, we need to defer applying it until
  //      after the clipped RRect blur has been drawn to a separate texture.
  //      However, since there's only one draw call that produces color, we
  //      don't need to worry about the blend mode or translucency (unlike with
  //      BlurStyle::kSolid).
  //
  if ((paint.mask_blur_descriptor->style !=
           FilterContents::BlurStyle::kNormal &&
       paint.image_filter) ||
      (paint.mask_blur_descriptor->style == FilterContents::BlurStyle::kSolid &&
       (!rrect_color.IsOpaque() ||
        paint.blend_mode != BlendMode::kSourceOver))) {
    Rect render_bounds = rect;
    if (paint.mask_blur_descriptor->style !=
        FilterContents::BlurStyle::kInner) {
      render_bounds =
          render_bounds.Expand(paint.mask_blur_descriptor->sigma.sigma * 4.0);
    }
    // Defer the alpha, blend mode, and image filter to a separate layer.
    SaveLayer(
        Paint{
            .color = Color::White().WithAlpha(rrect_color.alpha),
            .image_filter = paint.image_filter,
            .blend_mode = paint.blend_mode,
        },
        render_bounds, nullptr, ContentBoundsPromise::kContainsContents, 1u);
    rrect_paint.color = rrect_color.WithAlpha(1);
  } else {
    rrect_paint.color = rrect_color;
    rrect_paint.blend_mode = paint.blend_mode;
    rrect_paint.image_filter = paint.image_filter;
    Save(1u);
  }

  auto draw_blurred_rrect = [this, &rect, &corner_radii, &rrect_paint]() {
    auto contents = std::make_shared<SolidRRectBlurContents>();

    contents->SetColor(rrect_paint.color);
    contents->SetSigma(rrect_paint.mask_blur_descriptor->sigma);
    contents->SetRRect(rect, corner_radii);

    Entity blurred_rrect_entity;
    blurred_rrect_entity.SetTransform(GetCurrentTransform());
    blurred_rrect_entity.SetBlendMode(rrect_paint.blend_mode);

    rrect_paint.mask_blur_descriptor = std::nullopt;
    blurred_rrect_entity.SetContents(
        rrect_paint.WithFilters(std::move(contents)));
    AddRenderEntityToCurrentPass(blurred_rrect_entity);
  };

  switch (rrect_paint.mask_blur_descriptor->style) {
    case FilterContents::BlurStyle::kNormal: {
      draw_blurred_rrect();
      break;
    }
    case FilterContents::BlurStyle::kSolid: {
      // First, draw the blurred RRect.
      draw_blurred_rrect();
      // Then, draw the non-blurred RRect on top.
      Entity entity;
      entity.SetTransform(GetCurrentTransform());
      entity.SetBlendMode(rrect_paint.blend_mode);

      RoundRectGeometry geom(rect, corner_radii);
      AddRenderEntityWithFiltersToCurrentPass(entity, &geom, rrect_paint,
                                              /*reuse_depth=*/true);
      break;
    }
    case FilterContents::BlurStyle::kOuter: {
      RoundRectGeometry geom(rect, corner_radii);
      ClipGeometry(geom, Entity::ClipOperation::kDifference);
      draw_blurred_rrect();
      break;
    }
    case FilterContents::BlurStyle::kInner: {
      RoundRectGeometry geom(rect, corner_radii);
      ClipGeometry(geom, Entity::ClipOperation::kIntersect);
      draw_blurred_rrect();
      break;
    }
  }

  Restore();

  return true;
}

void Canvas::DrawLine(const Point& p0,
                      const Point& p1,
                      const Paint& paint,
                      bool reuse_depth) {
  Entity entity;
  entity.SetTransform(GetCurrentTransform());
  entity.SetBlendMode(paint.blend_mode);

  LineGeometry geom(p0, p1, paint.stroke_width, paint.stroke_cap);
  AddRenderEntityWithFiltersToCurrentPass(entity, &geom, paint,
                                          /*reuse_depth=*/reuse_depth);
}

void Canvas::DrawRect(const Rect& rect, const Paint& paint) {
  if (paint.style == Paint::Style::kStroke) {
    DrawPath(PathBuilder{}.AddRect(rect).TakePath(), paint);
    return;
  }

  if (AttemptDrawBlurredRRect(rect, {}, paint)) {
    return;
  }

  Entity entity;
  entity.SetTransform(GetCurrentTransform());
  entity.SetBlendMode(paint.blend_mode);

  RectGeometry geom(rect);
  AddRenderEntityWithFiltersToCurrentPass(entity, &geom, paint);
}

void Canvas::DrawOval(const Rect& rect, const Paint& paint) {
  // TODO(jonahwilliams): This additional condition avoids an assert in the
  // stroke circle geometry generator. I need to verify the condition that this
  // assert prevents.
  if (rect.IsSquare() && (paint.style == Paint::Style::kFill ||
                          (paint.style == Paint::Style::kStroke &&
                           paint.stroke_width < rect.GetWidth()))) {
    // Circles have slightly less overhead and can do stroking
    DrawCircle(rect.GetCenter(), rect.GetWidth() * 0.5f, paint);
    return;
  }

  if (paint.style == Paint::Style::kStroke) {
    // No stroked ellipses yet
    DrawPath(PathBuilder{}.AddOval(rect).TakePath(), paint);
    return;
  }

  if (AttemptDrawBlurredRRect(rect, rect.GetSize() * 0.5f, paint)) {
    return;
  }

  Entity entity;
  entity.SetTransform(GetCurrentTransform());
  entity.SetBlendMode(paint.blend_mode);

  EllipseGeometry geom(rect);
  AddRenderEntityWithFiltersToCurrentPass(entity, &geom, paint);
}

void Canvas::DrawRoundRect(const RoundRect& round_rect, const Paint& paint) {
  auto& rect = round_rect.GetBounds();
  auto& radii = round_rect.GetRadii();
  if (radii.AreAllCornersSame()) {
    if (AttemptDrawBlurredRRect(rect, radii.top_left, paint)) {
      return;
    }

    if (paint.style == Paint::Style::kFill) {
      Entity entity;
      entity.SetTransform(GetCurrentTransform());
      entity.SetBlendMode(paint.blend_mode);

      RoundRectGeometry geom(rect, radii.top_left);
      AddRenderEntityWithFiltersToCurrentPass(entity, &geom, paint);
      return;
    }
  }

  auto path = PathBuilder{}
                  .SetConvexity(Convexity::kConvex)
                  .AddRoundRect(round_rect)
                  .SetBounds(rect)
                  .TakePath();
  DrawPath(path, paint);
}

void Canvas::DrawCircle(const Point& center,
                        Scalar radius,
                        const Paint& paint) {
  Size half_size(radius, radius);
  if (AttemptDrawBlurredRRect(
          Rect::MakeOriginSize(center - half_size, half_size * 2),
          {radius, radius}, paint)) {
    return;
  }

  Entity entity;
  entity.SetTransform(GetCurrentTransform());
  entity.SetBlendMode(paint.blend_mode);

  if (paint.style == Paint::Style::kStroke) {
    CircleGeometry geom(center, radius, paint.stroke_width);
    AddRenderEntityWithFiltersToCurrentPass(entity, &geom, paint);
  } else {
    CircleGeometry geom(center, radius);
    AddRenderEntityWithFiltersToCurrentPass(entity, &geom, paint);
  }
}

void Canvas::ClipGeometry(const Geometry& geometry,
                          Entity::ClipOperation clip_op,
                          bool is_aa) {
  if (IsSkipping()) {
    return;
  }

  // Ideally the clip depth would be greater than the current rendering
  // depth because any rendering calls that follow this clip operation will
  // pre-increment the depth and then be rendering above our clip depth,
  // but that case will be caught by the CHECK in AddRenderEntity above.
  // In practice we sometimes have a clip set with no rendering after it
  // and in such cases the current depth will equal the clip depth.
  // Eventually the DisplayList should optimize these out, but it is hard
  // to know if a clip will actually be used in advance of storing it in
  // the DisplayList buffer.
  // See https://github.com/flutter/flutter/issues/147021
  FML_DCHECK(current_depth_ <= transform_stack_.back().clip_depth)
      << current_depth_ << " <=? " << transform_stack_.back().clip_depth;
  uint32_t clip_depth = transform_stack_.back().clip_depth;

  const Matrix clip_transform =
      Matrix::MakeTranslation(Vector3(-GetGlobalPassPosition())) *
      GetCurrentTransform();

  std::optional<Rect> clip_coverage = geometry.GetCoverage(clip_transform);
  if (!clip_coverage.has_value()) {
    return;
  }

  ClipContents clip_contents(
      clip_coverage.value(),
      /*is_axis_aligned_rect=*/geometry.IsAxisAlignedRect() &&
          GetCurrentTransform().IsTranslationScaleOnly());
  clip_contents.SetClipOperation(clip_op);

  EntityPassClipStack::ClipStateResult clip_state_result =
      clip_coverage_stack_.RecordClip(
          clip_contents,                                     //
          /*transform=*/clip_transform,                      //
          /*global_pass_position=*/GetGlobalPassPosition(),  //
          /*clip_depth=*/clip_depth,                         //
          /*clip_height_floor=*/GetClipHeightFloor(),        //
          /*is_aa=*/is_aa);

  if (clip_state_result.clip_did_change) {
    // We only need to update the pass scissor if the clip state has changed.
    SetClipScissor(clip_coverage_stack_.CurrentClipCoverage(),
                   *render_passes_.back().inline_pass_context->GetRenderPass(),
                   GetGlobalPassPosition());
  }

  ++transform_stack_.back().clip_height;
  ++transform_stack_.back().num_clips;

  if (!clip_state_result.should_render) {
    return;
  }

  // Note: this is a bit of a hack. Its not possible to construct a geometry
  // result without begninning the render pass. We should refactor the geometry
  // objects so that they only need a reference to the render pass size and/or
  // orthographic transform.
  Entity entity;
  entity.SetTransform(clip_transform);
  entity.SetClipDepth(clip_depth);

  GeometryResult geometry_result = geometry.GetPositionBuffer(
      renderer_,                                                   //
      entity,                                                      //
      *render_passes_.back().inline_pass_context->GetRenderPass()  //
  );
  clip_contents.SetGeometry(geometry_result);
  clip_coverage_stack_.GetLastReplayResult().clip_contents.SetGeometry(
      geometry_result);

  clip_contents.Render(
      renderer_, *render_passes_.back().inline_pass_context->GetRenderPass(),
      clip_depth);
}

void Canvas::DrawPoints(const Point points[],
                        uint32_t count,
                        Scalar radius,
                        const Paint& paint,
                        PointStyle point_style) {
  if (radius <= 0) {
    return;
  }

  Entity entity;
  entity.SetTransform(GetCurrentTransform());
  entity.SetBlendMode(paint.blend_mode);

  PointFieldGeometry geom(points, count, radius,
                          /*round=*/point_style == PointStyle::kRound);
  AddRenderEntityWithFiltersToCurrentPass(entity, &geom, paint);
}

void Canvas::DrawImage(const std::shared_ptr<Texture>& image,
                       Point offset,
                       const Paint& paint,
                       const SamplerDescriptor& sampler) {
  if (!image) {
    return;
  }

  const auto source = Rect::MakeSize(image->GetSize());
  const auto dest = source.Shift(offset);

  DrawImageRect(image, source, dest, paint, sampler);
}

void Canvas::DrawImageRect(const std::shared_ptr<Texture>& image,
                           Rect source,
                           Rect dest,
                           const Paint& paint,
                           const SamplerDescriptor& sampler,
                           SourceRectConstraint src_rect_constraint) {
  if (!image || source.IsEmpty() || dest.IsEmpty()) {
    return;
  }

  auto size = image->GetSize();

  if (size.IsEmpty()) {
    return;
  }

  std::optional<Rect> clipped_source =
      source.Intersection(Rect::MakeSize(size));
  if (!clipped_source) {
    return;
  }
  if (*clipped_source != source) {
    Scalar sx = dest.GetWidth() / source.GetWidth();
    Scalar sy = dest.GetHeight() / source.GetHeight();
    Scalar tx = dest.GetLeft() - source.GetLeft() * sx;
    Scalar ty = dest.GetTop() - source.GetTop() * sy;
    Matrix src_to_dest = Matrix::MakeTranslateScale({sx, sy, 1}, {tx, ty, 0});
    dest = clipped_source->TransformBounds(src_to_dest);
  }

  auto texture_contents = TextureContents::MakeRect(dest);
  texture_contents->SetTexture(image);
  texture_contents->SetSourceRect(*clipped_source);
  texture_contents->SetStrictSourceRect(src_rect_constraint ==
                                        SourceRectConstraint::kStrict);
  texture_contents->SetSamplerDescriptor(sampler);
  texture_contents->SetOpacity(paint.color.alpha);
  texture_contents->SetDeferApplyingOpacity(paint.HasColorFilter());

  Entity entity;
  entity.SetBlendMode(paint.blend_mode);
  entity.SetTransform(GetCurrentTransform());

  if (!paint.mask_blur_descriptor.has_value()) {
    entity.SetContents(paint.WithFilters(std::move(texture_contents)));
    AddRenderEntityToCurrentPass(entity);
    return;
  }

  RectGeometry out_rect(Rect{});

  entity.SetContents(paint.WithFilters(
      paint.mask_blur_descriptor->CreateMaskBlur(texture_contents, &out_rect)));
  AddRenderEntityToCurrentPass(entity);
}

size_t Canvas::GetClipHeight() const {
  return transform_stack_.back().clip_height;
}

void Canvas::DrawVertices(const std::shared_ptr<VerticesGeometry>& vertices,
                          BlendMode blend_mode,
                          const Paint& paint) {
  // Override the blend mode with kDestination in order to match the behavior
  // of Skia's SK_LEGACY_IGNORE_DRAW_VERTICES_BLEND_WITH_NO_SHADER flag, which
  // is enabled when the Flutter engine builds Skia.
  if (!paint.color_source) {
    blend_mode = BlendMode::kDestination;
  }

  Entity entity;
  entity.SetTransform(GetCurrentTransform());
  entity.SetBlendMode(paint.blend_mode);

  // If there are no vertex colors.
  if (UseColorSourceContents(vertices, paint)) {
    AddRenderEntityWithFiltersToCurrentPass(entity, vertices.get(), paint);
    return;
  }

  // If the blend mode is destination don't bother to bind or create a texture.
  if (blend_mode == BlendMode::kDestination) {
    auto contents = std::make_shared<VerticesSimpleBlendContents>();
    contents->SetBlendMode(blend_mode);
    contents->SetAlpha(paint.color.alpha);
    contents->SetGeometry(vertices);
    entity.SetContents(paint.WithFilters(std::move(contents)));
    AddRenderEntityToCurrentPass(entity);
    return;
  }

  // If there is a texture, use this directly. Otherwise render the color
  // source to a texture.
  if (paint.color_source &&
      paint.color_source->type() == flutter::DlColorSourceType::kImage) {
    const flutter::DlImageColorSource* image_color_source =
        paint.color_source->asImage();
    FML_DCHECK(image_color_source &&
               image_color_source->image()->impeller_texture());
    auto texture = image_color_source->image()->impeller_texture();
    auto x_tile_mode = static_cast<Entity::TileMode>(
        image_color_source->horizontal_tile_mode());
    auto y_tile_mode =
        static_cast<Entity::TileMode>(image_color_source->vertical_tile_mode());
    auto sampler_descriptor =
        skia_conversions::ToSamplerDescriptor(image_color_source->sampling());
    auto effect_transform = image_color_source->matrix();

    auto contents = std::make_shared<VerticesSimpleBlendContents>();
    contents->SetBlendMode(blend_mode);
    contents->SetAlpha(paint.color.alpha);
    contents->SetGeometry(vertices);
    contents->SetEffectTransform(effect_transform);
    contents->SetTexture(texture);
    contents->SetTileMode(x_tile_mode, y_tile_mode);
    contents->SetSamplerDescriptor(sampler_descriptor);

    entity.SetContents(paint.WithFilters(std::move(contents)));
    AddRenderEntityToCurrentPass(entity);
    return;
  }

  auto src_paint = paint;
  src_paint.color = paint.color.WithAlpha(1.0);

  std::shared_ptr<ColorSourceContents> src_contents =
      src_paint.CreateContents();
  src_contents->SetGeometry(vertices.get());

  // If the color source has an intrinsic size, then we use that to
  // create the src contents as a simplification. Otherwise we use
  // the extent of the texture coordinates to determine how large
  // the src contents should be. If neither has a value we fall back
  // to using the geometry coverage data.
  Rect src_coverage;
  auto size = src_contents->GetColorSourceSize();
  if (size.has_value()) {
    src_coverage = Rect::MakeXYWH(0, 0, size->width, size->height);
  } else {
    auto cvg = vertices->GetCoverage(Matrix{});
    FML_CHECK(cvg.has_value());
    src_coverage =
        // Covered by FML_CHECK.
        // NOLINTNEXTLINE(bugprone-unchecked-optional-access)
        vertices->GetTextureCoordinateCoverge().value_or(cvg.value());
  }
  src_contents = src_paint.CreateContents();

  clip_geometry_.push_back(Geometry::MakeRect(Rect::Round(src_coverage)));
  src_contents->SetGeometry(clip_geometry_.back().get());

  auto contents = std::make_shared<VerticesSimpleBlendContents>();
  contents->SetBlendMode(blend_mode);
  contents->SetAlpha(paint.color.alpha);
  contents->SetGeometry(vertices);
  contents->SetLazyTextureCoverage(src_coverage);
  contents->SetLazyTexture(
      [src_contents, src_coverage](const ContentContext& renderer) {
        // Applying the src coverage as the coverage limit prevents the 1px
        // coverage pad from adding a border that is picked up by developer
        // specified UVs.
        return src_contents
            ->RenderToSnapshot(renderer, {}, Rect::Round(src_coverage))
            ->texture;
      });
  entity.SetContents(paint.WithFilters(std::move(contents)));
  AddRenderEntityToCurrentPass(entity);
}

void Canvas::DrawAtlas(const std::shared_ptr<AtlasContents>& atlas_contents,
                       const Paint& paint) {
  atlas_contents->SetAlpha(paint.color.alpha);

  Entity entity;
  entity.SetTransform(GetCurrentTransform());
  entity.SetBlendMode(paint.blend_mode);
  entity.SetContents(paint.WithFilters(atlas_contents));

  AddRenderEntityToCurrentPass(entity);
}

/// Compositor Functionality
/////////////////////////////////////////

void Canvas::SetupRenderPass() {
  renderer_.GetRenderTargetCache()->Start();
  ColorAttachment color0 = render_target_.GetColorAttachment(0);

  auto& stencil_attachment = render_target_.GetStencilAttachment();
  auto& depth_attachment = render_target_.GetDepthAttachment();
  if (!stencil_attachment.has_value() || !depth_attachment.has_value()) {
    // Setup a new root stencil with an optimal configuration if one wasn't
    // provided by the caller.
    render_target_.SetupDepthStencilAttachments(
        *renderer_.GetContext(),
        *renderer_.GetContext()->GetResourceAllocator(),
        color0.texture->GetSize(),
        renderer_.GetContext()->GetCapabilities()->SupportsOffscreenMSAA(),
        "ImpellerOnscreen", kDefaultStencilConfig);
  }

  // Set up the clear color of the root pass.
  color0.clear_color = Color::BlackTransparent();
  render_target_.SetColorAttachment(color0, 0);

  // If requires_readback is true, then there is a backdrop filter or emulated
  // advanced blend in the first save layer. This requires a readback, which
  // isn't supported by onscreen textures. To support this, we immediately begin
  // a second save layer with the same dimensions as the onscreen. When
  // rendering is completed, we must blit this saveLayer to the onscreen.
  if (requires_readback_) {
    auto entity_pass_target =
        CreateRenderTarget(renderer_,                  //
                           color0.texture->GetSize(),  //
                           /*clear_color=*/Color::BlackTransparent());
    render_passes_.push_back(
        LazyRenderingConfig(renderer_, std::move(entity_pass_target)));
  } else {
    auto entity_pass_target = std::make_unique<EntityPassTarget>(
        render_target_,                                                    //
        renderer_.GetDeviceCapabilities().SupportsReadFromResolve(),       //
        renderer_.GetDeviceCapabilities().SupportsImplicitResolvingMSAA()  //
    );
    render_passes_.push_back(
        LazyRenderingConfig(renderer_, std::move(entity_pass_target)));
  }
}

void Canvas::SkipUntilMatchingRestore(size_t total_content_depth) {
  auto entry = CanvasStackEntry{};
  entry.skipping = true;
  entry.clip_depth = current_depth_ + total_content_depth;
  transform_stack_.push_back(entry);
}

void Canvas::Save(uint32_t total_content_depth) {
  if (IsSkipping()) {
    return SkipUntilMatchingRestore(total_content_depth);
  }

  auto entry = CanvasStackEntry{};
  entry.transform = transform_stack_.back().transform;
  entry.clip_depth = current_depth_ + total_content_depth;
  entry.distributed_opacity = transform_stack_.back().distributed_opacity;
  FML_DCHECK(entry.clip_depth <= transform_stack_.back().clip_depth)
      << entry.clip_depth << " <=? " << transform_stack_.back().clip_depth
      << " after allocating " << total_content_depth;
  entry.clip_height = transform_stack_.back().clip_height;
  entry.rendering_mode = Entity::RenderingMode::kDirect;
  transform_stack_.push_back(entry);
}

std::optional<Rect> Canvas::GetLocalCoverageLimit() const {
  if (!clip_coverage_stack_.HasCoverage()) {
    // The current clip is empty. This means the pass texture won't be
    // visible, so skip it.
    return std::nullopt;
  }

  auto maybe_current_clip_coverage = clip_coverage_stack_.CurrentClipCoverage();
  if (!maybe_current_clip_coverage.has_value()) {
    return std::nullopt;
  }

  auto current_clip_coverage = maybe_current_clip_coverage.value();

  // The maximum coverage of the subpass. Subpasses textures should never
  // extend outside the parent pass texture or the current clip coverage.
  std::optional<Rect> maybe_coverage_limit =
      Rect::MakeOriginSize(GetGlobalPassPosition(),
                           Size(render_passes_.back()
                                    .inline_pass_context->GetTexture()
                                    ->GetSize()))
          .Intersection(current_clip_coverage);

  if (!maybe_coverage_limit.has_value() || maybe_coverage_limit->IsEmpty()) {
    return std::nullopt;
  }

  return maybe_coverage_limit->Intersection(
      Rect::MakeSize(render_target_.GetRenderTargetSize()));
}

void Canvas::SaveLayer(const Paint& paint,
                       std::optional<Rect> bounds,
                       const flutter::DlImageFilter* backdrop_filter,
                       ContentBoundsPromise bounds_promise,
                       uint32_t total_content_depth,
                       bool can_distribute_opacity,
                       std::optional<int64_t> backdrop_id) {
  TRACE_EVENT0("flutter", "Canvas::saveLayer");
  if (IsSkipping()) {
    return SkipUntilMatchingRestore(total_content_depth);
  }

  auto maybe_coverage_limit = GetLocalCoverageLimit();
  if (!maybe_coverage_limit.has_value()) {
    return SkipUntilMatchingRestore(total_content_depth);
  }
  auto coverage_limit = maybe_coverage_limit.value();

  if (can_distribute_opacity && !backdrop_filter &&
      Paint::CanApplyOpacityPeephole(paint) &&
      bounds_promise != ContentBoundsPromise::kMayClipContents) {
    Save(total_content_depth);
    transform_stack_.back().distributed_opacity *= paint.color.alpha;
    return;
  }

  std::shared_ptr<FilterContents> filter_contents = paint.WithImageFilter(
      Rect(), transform_stack_.back().transform,
      Entity::RenderingMode::kSubpassPrependSnapshotTransform);

  std::optional<Rect> maybe_subpass_coverage = ComputeSaveLayerCoverage(
      bounds.value_or(Rect::MakeMaximum()),
      transform_stack_.back().transform,  //
      coverage_limit,                     //
      filter_contents,                    //
      /*flood_output_coverage=*/
      Entity::IsBlendModeDestructive(paint.blend_mode),  //
      /*flood_input_coverage=*/!!backdrop_filter ||
          (paint.color_filter &&
           paint.color_filter->modifies_transparent_black())  //
  );

  if (!maybe_subpass_coverage.has_value()) {
    return SkipUntilMatchingRestore(total_content_depth);
  }

  auto subpass_coverage = maybe_subpass_coverage.value();

  // When an image filter is present, clamp to avoid flicking due to nearest
  // sampled image. For other cases, round out to ensure than any geometry is
  // not cut off.
  //
  // See also this bug: https://github.com/flutter/flutter/issues/144213
  //
  // TODO(jonahwilliams): this could still round out for filters that use decal
  // sampling mode.
  ISize subpass_size;
  bool did_round_out = false;
  if (paint.image_filter) {
    subpass_size = ISize(subpass_coverage.GetSize());
  } else {
    did_round_out = true;
    subpass_size =
        static_cast<ISize>(IRect::RoundOut(subpass_coverage).GetSize());
  }
  if (subpass_size.IsEmpty()) {
    return SkipUntilMatchingRestore(total_content_depth);
  }

  // When there are scaling filters present, these contents may exceed the
  // maximum texture size. Perform a clamp here, which may cause rendering
  // artifacts.
  subpass_size = subpass_size.Min(renderer_.GetContext()
                                      ->GetCapabilities()
                                      ->GetMaximumRenderPassAttachmentSize());

  // Backdrop filter state, ignored if there is no BDF.
  std::shared_ptr<FilterContents> backdrop_filter_contents;
  Point local_position = Point(0, 0);
  if (backdrop_filter) {
    local_position = subpass_coverage.GetOrigin() - GetGlobalPassPosition();
    Canvas::BackdropFilterProc backdrop_filter_proc =
        [backdrop_filter = backdrop_filter](
            const FilterInput::Ref& input, const Matrix& effect_transform,
            Entity::RenderingMode rendering_mode) {
          auto filter = WrapInput(backdrop_filter, input);
          filter->SetEffectTransform(effect_transform);
          filter->SetRenderingMode(rendering_mode);
          return filter;
        };

    std::shared_ptr<Texture> input_texture;

    // If the backdrop ID is not nullopt and there is more than one usage
    // of it in the current scene, cache the backdrop texture and remove it from
    // the current entity pass flip.
    bool will_cache_backdrop_texture = false;
    BackdropData* backdrop_data = nullptr;
    // If we've reached this point, there is at least one backdrop filter. But
    // potentially more if there is a backdrop id. We may conditionally set this
    // to a higher value in the if block below.
    size_t backdrop_count = 1;
    if (backdrop_id.has_value()) {
      std::unordered_map<int64_t, BackdropData>::iterator backdrop_data_it =
          backdrop_data_.find(backdrop_id.value());
      if (backdrop_data_it != backdrop_data_.end()) {
        backdrop_data = &backdrop_data_it->second;
        will_cache_backdrop_texture =
            backdrop_data_it->second.backdrop_count > 1;
        backdrop_count = backdrop_data_it->second.backdrop_count;
      }
    }

    if (!will_cache_backdrop_texture || !backdrop_data->texture_slot) {
      backdrop_count_ -= backdrop_count;

      // The onscreen texture can be flipped to if:
      // 1. The device supports framebuffer fetch
      // 2. There are no more backdrop filters
      // 3. The current render pass is for the onscreen pass.
      const bool should_use_onscreen =
          renderer_.GetDeviceCapabilities().SupportsFramebufferFetch() &&
          backdrop_count_ == 0 && render_passes_.size() == 1u;
      input_texture = FlipBackdrop(
          GetGlobalPassPosition(),                                //
          /*should_remove_texture=*/will_cache_backdrop_texture,  //
          /*should_use_onscreen=*/should_use_onscreen             //
      );
      if (!input_texture) {
        // Validation failures are logged in FlipBackdrop.
        return;
      }

      if (will_cache_backdrop_texture) {
        backdrop_data->texture_slot = input_texture;
      }
    } else {
      input_texture = backdrop_data->texture_slot;
    }

    backdrop_filter_contents = backdrop_filter_proc(
        FilterInput::Make(std::move(input_texture)),
        transform_stack_.back().transform.Basis(),
        // When the subpass has a translation that means the math with
        // the snapshot has to be different.
        transform_stack_.back().transform.HasTranslation()
            ? Entity::RenderingMode::kSubpassPrependSnapshotTransform
            : Entity::RenderingMode::kSubpassAppendSnapshotTransform);

    if (will_cache_backdrop_texture) {
      FML_DCHECK(backdrop_data);
      // If all filters on the shared backdrop layer are equal, process the
      // layer once.
      if (backdrop_data->all_filters_equal &&
          !backdrop_data->shared_filter_snapshot.has_value()) {
        // TODO(157110): compute minimum input hint.
        backdrop_data->shared_filter_snapshot =
            backdrop_filter_contents->RenderToSnapshot(renderer_, {});
      }

      std::optional<Snapshot> maybe_snapshot =
          backdrop_data->shared_filter_snapshot;
      if (maybe_snapshot.has_value()) {
        Snapshot snapshot = maybe_snapshot.value();
        std::shared_ptr<TextureContents> contents = TextureContents::MakeRect(
            subpass_coverage.Shift(-GetGlobalPassPosition()));
        auto scaled =
            subpass_coverage.TransformBounds(snapshot.transform.Invert());
        contents->SetTexture(snapshot.texture);
        contents->SetSourceRect(scaled);
        contents->SetSamplerDescriptor(snapshot.sampler_descriptor);

        // This backdrop entity sets a depth value as it is written to the newly
        // flipped backdrop and not into a new saveLayer.
        Entity backdrop_entity;
        backdrop_entity.SetContents(std::move(contents));
        backdrop_entity.SetClipDepth(++current_depth_);
        backdrop_entity.SetBlendMode(paint.blend_mode);

        backdrop_entity.Render(renderer_, GetCurrentRenderPass());
        Save(0);
        return;
      }
    }
  }

  // When applying a save layer, absorb any pending distributed opacity.
  Paint paint_copy = paint;
  paint_copy.color.alpha *= transform_stack_.back().distributed_opacity;
  transform_stack_.back().distributed_opacity = 1.0;

  render_passes_.push_back(
      LazyRenderingConfig(renderer_,                                    //
                          CreateRenderTarget(renderer_,                 //
                                             subpass_size,              //
                                             Color::BlackTransparent()  //
                                             )));
  save_layer_state_.push_back(SaveLayerState{paint_copy, subpass_coverage});

  CanvasStackEntry entry;
  entry.transform = transform_stack_.back().transform;
  entry.clip_depth = current_depth_ + total_content_depth;
  FML_DCHECK(entry.clip_depth <= transform_stack_.back().clip_depth)
      << entry.clip_depth << " <=? " << transform_stack_.back().clip_depth
      << " after allocating " << total_content_depth;
  entry.clip_height = transform_stack_.back().clip_height;
  entry.rendering_mode = Entity::RenderingMode::kSubpassAppendSnapshotTransform;
  entry.did_round_out = did_round_out;
  transform_stack_.emplace_back(entry);

  // Start non-collapsed subpasses with a fresh clip coverage stack limited by
  // the subpass coverage. This is important because image filters applied to
  // save layers may transform the subpass texture after it's rendered,
  // causing parent clip coverage to get misaligned with the actual area that
  // the subpass will affect in the parent pass.
  clip_coverage_stack_.PushSubpass(subpass_coverage, GetClipHeight());

  if (!backdrop_filter_contents) {
    return;
  }

  // Render the backdrop entity.
  Entity backdrop_entity;
  backdrop_entity.SetContents(std::move(backdrop_filter_contents));
  backdrop_entity.SetTransform(
      Matrix::MakeTranslation(Vector3(-local_position)));
  backdrop_entity.SetClipDepth(std::numeric_limits<uint32_t>::max());
  backdrop_entity.Render(renderer_, GetCurrentRenderPass());
}

bool Canvas::Restore() {
  FML_DCHECK(transform_stack_.size() > 0);
  if (transform_stack_.size() == 1) {
    return false;
  }

  // This check is important to make sure we didn't exceed the depth
  // that the clips were rendered at while rendering any of the
  // rendering ops. It is OK for the current depth to equal the
  // outgoing clip depth because that means the clipping would have
  // been successful up through the last rendering op, but it cannot
  // be greater.
  // Also, we bump the current rendering depth to the outgoing clip
  // depth so that future rendering operations are not clipped by
  // any of the pixels set by the expiring clips. It is OK for the
  // estimates used to determine the clip depth in save/saveLayer
  // to be overly conservative, but we need to jump the depth to
  // the clip depth so that the next rendering op will get a
  // larger depth (it will pre-increment the current_depth_ value).
  FML_DCHECK(current_depth_ <= transform_stack_.back().clip_depth)
      << current_depth_ << " <=? " << transform_stack_.back().clip_depth;
  current_depth_ = transform_stack_.back().clip_depth;

  if (IsSkipping()) {
    transform_stack_.pop_back();
    return true;
  }

  if (transform_stack_.back().rendering_mode ==
          Entity::RenderingMode::kSubpassAppendSnapshotTransform ||
      transform_stack_.back().rendering_mode ==
          Entity::RenderingMode::kSubpassPrependSnapshotTransform) {
    auto lazy_render_pass = std::move(render_passes_.back());
    render_passes_.pop_back();
    // Force the render pass to be constructed if it never was.
    lazy_render_pass.inline_pass_context->GetRenderPass();

    SaveLayerState save_layer_state = save_layer_state_.back();
    save_layer_state_.pop_back();
    auto global_pass_position = GetGlobalPassPosition();

    std::shared_ptr<Contents> contents = CreateContentsForSubpassTarget(
        save_layer_state.paint,                                    //
        lazy_render_pass.inline_pass_context->GetTexture(),        //
        Matrix::MakeTranslation(Vector3{-global_pass_position}) *  //
            transform_stack_.back().transform                      //
    );

    lazy_render_pass.inline_pass_context->EndPass();

    // Round the subpass texture position for pixel alignment with the parent
    // pass render target. By default, we draw subpass textures with nearest
    // sampling, so aligning here is important for avoiding visual nearest
    // sampling errors caused by limited floating point precision when
    // straddling a half pixel boundary.
    Point subpass_texture_position;
    if (transform_stack_.back().did_round_out) {
      // Subpass coverage was rounded out, origin potentially moved "down" by
      // as much as a pixel.
      subpass_texture_position =
          (save_layer_state.coverage.GetOrigin() - global_pass_position)
              .Floor();
    } else {
      // Subpass coverage was truncated. Pick the closest phyiscal pixel.
      subpass_texture_position =
          (save_layer_state.coverage.GetOrigin() - global_pass_position)
              .Round();
    }

    Entity element_entity;
    element_entity.SetClipDepth(++current_depth_);
    element_entity.SetContents(std::move(contents));
    element_entity.SetBlendMode(save_layer_state.paint.blend_mode);
    element_entity.SetTransform(
        Matrix::MakeTranslation(Vector3(subpass_texture_position)));

    if (element_entity.GetBlendMode() > Entity::kLastPipelineBlendMode) {
      if (renderer_.GetDeviceCapabilities().SupportsFramebufferFetch()) {
        ApplyFramebufferBlend(element_entity);
      } else {
        // End the active pass and flush the buffer before rendering "advanced"
        // blends. Advanced blends work by binding the current render target
        // texture as an input ("destination"), blending with a second texture
        // input ("source"), writing the result to an intermediate texture, and
        // finally copying the data from the intermediate texture back to the
        // render target texture. And so all of the commands that have written
        // to the render target texture so far need to execute before it's bound
        // for blending (otherwise the blend pass will end up executing before
        // all the previous commands in the active pass).
        auto input_texture = FlipBackdrop(GetGlobalPassPosition());
        if (!input_texture) {
          return false;
        }

        FilterInput::Vector inputs = {
            FilterInput::Make(input_texture,
                              element_entity.GetTransform().Invert()),
            FilterInput::Make(element_entity.GetContents())};
        auto contents = ColorFilterContents::MakeBlend(
            element_entity.GetBlendMode(), inputs);
        contents->SetCoverageHint(element_entity.GetCoverage());
        element_entity.SetContents(std::move(contents));
        element_entity.SetBlendMode(BlendMode::kSource);
      }
    }

    element_entity.Render(
        renderer_,                                                   //
        *render_passes_.back().inline_pass_context->GetRenderPass()  //
    );
    clip_coverage_stack_.PopSubpass();
    transform_stack_.pop_back();

    // We don't need to restore clips if a saveLayer was performed, as the clip
    // state is per render target, and no more rendering operations will be
    // performed as the render target workloaded is completed in the restore.
    return true;
  }

  size_t num_clips = transform_stack_.back().num_clips;
  transform_stack_.pop_back();

  if (num_clips > 0) {
    EntityPassClipStack::ClipStateResult clip_state_result =
        clip_coverage_stack_.RecordRestore(GetGlobalPassPosition(),
                                           GetClipHeight());

    // Clip restores are never required with depth based clipping.
    FML_DCHECK(!clip_state_result.should_render);
    if (clip_state_result.clip_did_change) {
      // We only need to update the pass scissor if the clip state has changed.
      SetClipScissor(
          clip_coverage_stack_.CurrentClipCoverage(),                   //
          *render_passes_.back().inline_pass_context->GetRenderPass(),  //
          GetGlobalPassPosition()                                       //
      );
    }
  }

  return true;
}

void Canvas::DrawTextFrame(const std::shared_ptr<TextFrame>& text_frame,
                           Point position,
                           const Paint& paint) {
  Entity entity;
  entity.SetClipDepth(GetClipHeight());
  entity.SetBlendMode(paint.blend_mode);

  auto text_contents = std::make_shared<TextContents>();
  text_contents->SetTextFrame(text_frame);
  text_contents->SetForceTextColor(paint.mask_blur_descriptor.has_value());
  text_contents->SetScale(GetCurrentTransform().GetMaxBasisLengthXY());
  text_contents->SetColor(paint.color);
  text_contents->SetOffset(position);
  text_contents->SetTextProperties(paint.color,                           //
                                   paint.style == Paint::Style::kStroke,  //
                                   paint.stroke_width,                    //
                                   paint.stroke_cap,                      //
                                   paint.stroke_join,                     //
                                   paint.stroke_miter                     //
  );

  entity.SetTransform(GetCurrentTransform() *
                      Matrix::MakeTranslation(position));

  // TODO(bdero): This mask blur application is a hack. It will always wind up
  //              doing a gaussian blur that affects the color source itself
  //              instead of just the mask. The color filter text support
  //              needs to be reworked in order to interact correctly with
  //              mask filters.
  //              https://github.com/flutter/flutter/issues/133297
  entity.SetContents(paint.WithFilters(paint.WithMaskBlur(
      std::move(text_contents), true, GetCurrentTransform())));

  AddRenderEntityToCurrentPass(entity, false);
}

void Canvas::AddRenderEntityWithFiltersToCurrentPass(Entity& entity,
                                                     const Geometry* geometry,
                                                     const Paint& paint,
                                                     bool reuse_depth) {
  std::shared_ptr<ColorSourceContents> contents = paint.CreateContents();
  if (!paint.color_filter && !paint.invert_colors && !paint.image_filter &&
      !paint.mask_blur_descriptor.has_value()) {
    contents->SetGeometry(geometry);
    entity.SetContents(std::move(contents));
    AddRenderEntityToCurrentPass(entity, reuse_depth);
    return;
  }

  // Attempt to apply the color filter on the CPU first.
  // Note: This is not just an optimization; some color sources rely on
  //       CPU-applied color filters to behave properly.
  bool needs_color_filter = paint.color_filter || paint.invert_colors;
  if (needs_color_filter &&
      contents->ApplyColorFilter([&](Color color) -> Color {
        if (paint.color_filter) {
          color = GetCPUColorFilterProc(paint.color_filter)(color);
        }
        if (paint.invert_colors) {
          color = color.ApplyColorMatrix(kColorInversion);
        }
        return color;
      })) {
    needs_color_filter = false;
  }

  bool can_apply_mask_filter = geometry->CanApplyMaskFilter();
  contents->SetGeometry(geometry);

  if (can_apply_mask_filter && paint.mask_blur_descriptor.has_value()) {
    // If there's a mask blur and we need to apply the color filter on the GPU,
    // we need to be careful to only apply the color filter to the source
    // colors. CreateMaskBlur is able to handle this case.
    RectGeometry out_rect(Rect{});
    auto filter_contents = paint.mask_blur_descriptor->CreateMaskBlur(
        contents, needs_color_filter ? paint.color_filter : nullptr,
        needs_color_filter ? paint.invert_colors : false, &out_rect);
    entity.SetContents(std::move(filter_contents));
    AddRenderEntityToCurrentPass(entity, reuse_depth);
    return;
  }

  std::shared_ptr<Contents> contents_copy = std::move(contents);

  // Image input types will directly set their color filter,
  // if any. See `TiledTextureContents.SetColorFilter`.
  if (needs_color_filter &&
      (!paint.color_source ||
       paint.color_source->type() != flutter::DlColorSourceType::kImage)) {
    if (paint.color_filter) {
      contents_copy = WrapWithGPUColorFilter(
          paint.color_filter, FilterInput::Make(std::move(contents_copy)),
          ColorFilterContents::AbsorbOpacity::kYes);
    }
    if (paint.invert_colors) {
      contents_copy =
          WrapWithInvertColors(FilterInput::Make(std::move(contents_copy)),
                               ColorFilterContents::AbsorbOpacity::kYes);
    }
  }

  if (paint.image_filter) {
    std::shared_ptr<FilterContents> filter = WrapInput(
        paint.image_filter, FilterInput::Make(std::move(contents_copy)));
    filter->SetRenderingMode(Entity::RenderingMode::kDirect);
    entity.SetContents(filter);
    AddRenderEntityToCurrentPass(entity, reuse_depth);
    return;
  }

  entity.SetContents(std::move(contents_copy));
  AddRenderEntityToCurrentPass(entity, reuse_depth);
}

void Canvas::AddRenderEntityToCurrentPass(Entity& entity, bool reuse_depth) {
  if (IsSkipping()) {
    return;
  }

  entity.SetTransform(
      Matrix::MakeTranslation(Vector3(-GetGlobalPassPosition())) *
      entity.GetTransform());
  entity.SetInheritedOpacity(transform_stack_.back().distributed_opacity);
  if (entity.GetBlendMode() == BlendMode::kSourceOver &&
      entity.GetContents()->IsOpaque(entity.GetTransform())) {
    entity.SetBlendMode(BlendMode::kSource);
  }

  // If the entity covers the current render target and is a solid color, then
  // conditionally update the backdrop color to its solid color value blended
  // with the current backdrop.
  if (render_passes_.back().IsApplyingClearColor()) {
    std::optional<Color> maybe_color = entity.AsBackgroundColor(
        render_passes_.back().inline_pass_context->GetTexture()->GetSize());
    if (maybe_color.has_value()) {
      Color color = maybe_color.value();
      RenderTarget& render_target = render_passes_.back()
                                        .inline_pass_context->GetPassTarget()
                                        .GetRenderTarget();
      ColorAttachment attachment = render_target.GetColorAttachment(0);
      // Attachment.clear color needs to be premultiplied at all times, but the
      // Color::Blend function requires unpremultiplied colors.
      attachment.clear_color = attachment.clear_color.Unpremultiply()
                                   .Blend(color, entity.GetBlendMode())
                                   .Premultiply();
      render_target.SetColorAttachment(attachment, 0u);
      return;
    }
  }

  if (!reuse_depth) {
    ++current_depth_;
  }
  // We can render at a depth up to and including the depth of the currently
  // active clips and we will still be clipped out, but we cannot render at
  // a depth that is greater than the current clips or we will not be clipped.
  FML_DCHECK(current_depth_ <= transform_stack_.back().clip_depth)
      << current_depth_ << " <=? " << transform_stack_.back().clip_depth;
  entity.SetClipDepth(current_depth_);

  if (entity.GetBlendMode() > Entity::kLastPipelineBlendMode) {
    if (renderer_.GetDeviceCapabilities().SupportsFramebufferFetch()) {
      ApplyFramebufferBlend(entity);
    } else {
      // End the active pass and flush the buffer before rendering "advanced"
      // blends. Advanced blends work by binding the current render target
      // texture as an input ("destination"), blending with a second texture
      // input ("source"), writing the result to an intermediate texture, and
      // finally copying the data from the intermediate texture back to the
      // render target texture. And so all of the commands that have written
      // to the render target texture so far need to execute before it's bound
      // for blending (otherwise the blend pass will end up executing before
      // all the previous commands in the active pass).
      auto input_texture = FlipBackdrop(GetGlobalPassPosition());
      if (!input_texture) {
        return;
      }

      // The coverage hint tells the rendered Contents which portion of the
      // rendered output will actually be used, and so we set this to the
      // current clip coverage (which is the max clip bounds). The contents may
      // optionally use this hint to avoid unnecessary rendering work.
      auto element_coverage_hint = entity.GetContents()->GetCoverageHint();
      entity.GetContents()->SetCoverageHint(Rect::Intersection(
          element_coverage_hint, clip_coverage_stack_.CurrentClipCoverage()));

      FilterInput::Vector inputs = {
          FilterInput::Make(input_texture, entity.GetTransform().Invert()),
          FilterInput::Make(entity.GetContents())};
      auto contents =
          ColorFilterContents::MakeBlend(entity.GetBlendMode(), inputs);
      entity.SetContents(std::move(contents));
      entity.SetBlendMode(BlendMode::kSource);
    }
  }

  const std::shared_ptr<RenderPass>& result =
      render_passes_.back().inline_pass_context->GetRenderPass();
  if (!result) {
    // Failure to produce a render pass should be explained by specific errors
    // in `InlinePassContext::GetRenderPass()`, so avoid log spam and don't
    // append a validation log here.
    return;
  }

  entity.Render(renderer_, *result);
}

RenderPass& Canvas::GetCurrentRenderPass() const {
  return *render_passes_.back().inline_pass_context->GetRenderPass();
}

void Canvas::SetBackdropData(
    std::unordered_map<int64_t, BackdropData> backdrop_data,
    size_t backdrop_count) {
  backdrop_data_ = std::move(backdrop_data);
  backdrop_count_ = backdrop_count;
}

std::shared_ptr<Texture> Canvas::FlipBackdrop(Point global_pass_position,
                                              bool should_remove_texture,
                                              bool should_use_onscreen) {
  LazyRenderingConfig rendering_config = std::move(render_passes_.back());
  render_passes_.pop_back();

  // If the very first thing we render in this EntityPass is a subpass that
  // happens to have a backdrop filter or advanced blend, than that backdrop
  // filter/blend will sample from an uninitialized texture.
  //
  // By calling `pass_context.GetRenderPass` here, we force the texture to pass
  // through at least one RenderPass with the correct clear configuration before
  // any sampling occurs.
  //
  // In cases where there are no contents, we
  // could instead check the clear color and initialize a 1x2 CPU texture
  // instead of ending the pass.
  rendering_config.inline_pass_context->GetRenderPass();
  if (!rendering_config.inline_pass_context->EndPass()) {
    VALIDATION_LOG
        << "Failed to end the current render pass in order to read from "
           "the backdrop texture and apply an advanced blend or backdrop "
           "filter.";
    // Note: adding this render pass ensures there are no later crashes from
    // unbalanced save layers. Ideally, this method would return false and the
    // renderer could handle that by terminating dispatch.
    render_passes_.push_back(LazyRenderingConfig(
        renderer_, std::move(rendering_config.entity_pass_target),
        std::move(rendering_config.inline_pass_context)));
    return nullptr;
  }

  const std::shared_ptr<Texture>& input_texture =
      rendering_config.inline_pass_context->GetTexture();

  if (!input_texture) {
    VALIDATION_LOG << "Failed to fetch the color texture in order to "
                      "apply an advanced blend or backdrop filter.";

    // Note: see above.
    render_passes_.push_back(LazyRenderingConfig(
        renderer_, std::move(rendering_config.entity_pass_target),
        std::move(rendering_config.inline_pass_context)));
    return nullptr;
  }

  if (should_use_onscreen) {
    ColorAttachment color0 = render_target_.GetColorAttachment(0);
    // When MSAA is being used, we end up overriding the entire backdrop by
    // drawing the previous pass texture, and so we don't have to clear it and
    // can use kDontCare.
    color0.load_action = color0.resolve_texture != nullptr
                             ? LoadAction::kDontCare
                             : LoadAction::kLoad;
    render_target_.SetColorAttachment(color0, 0);

    auto entity_pass_target = std::make_unique<EntityPassTarget>(
        render_target_,                                                    //
        renderer_.GetDeviceCapabilities().SupportsReadFromResolve(),       //
        renderer_.GetDeviceCapabilities().SupportsImplicitResolvingMSAA()  //
    );
    render_passes_.push_back(
        LazyRenderingConfig(renderer_, std::move(entity_pass_target)));
    requires_readback_ = false;
  } else {
    render_passes_.push_back(LazyRenderingConfig(
        renderer_, std::move(rendering_config.entity_pass_target),
        std::move(rendering_config.inline_pass_context)));
    // If the current texture is being cached for a BDF we need to ensure we
    // don't recycle it during recording; remove it from the entity pass target.
    if (should_remove_texture) {
      render_passes_.back().entity_pass_target->RemoveSecondary();
    }
  }
  RenderPass& current_render_pass =
      *render_passes_.back().inline_pass_context->GetRenderPass();

  // Eagerly restore the BDF contents.

  // If the pass context returns a backdrop texture, we need to draw it to the
  // current pass. We do this because it's faster and takes significantly less
  // memory than storing/loading large MSAA textures. Also, it's not possible
  // to blit the non-MSAA resolve texture of the previous pass to MSAA
  // textures (let alone a transient one).
  Rect size_rect = Rect::MakeSize(input_texture->GetSize());
  auto msaa_backdrop_contents = TextureContents::MakeRect(size_rect);
  msaa_backdrop_contents->SetStencilEnabled(false);
  msaa_backdrop_contents->SetLabel("MSAA backdrop");
  msaa_backdrop_contents->SetSourceRect(size_rect);
  msaa_backdrop_contents->SetTexture(input_texture);

  Entity msaa_backdrop_entity;
  msaa_backdrop_entity.SetContents(std::move(msaa_backdrop_contents));
  msaa_backdrop_entity.SetBlendMode(BlendMode::kSource);
  msaa_backdrop_entity.SetClipDepth(std::numeric_limits<uint32_t>::max());
  if (!msaa_backdrop_entity.Render(renderer_, current_render_pass)) {
    VALIDATION_LOG << "Failed to render MSAA backdrop entity.";
    return nullptr;
  }

  // Restore any clips that were recorded before the backdrop filter was
  // applied.
  auto& replay_entities = clip_coverage_stack_.GetReplayEntities();
  for (const auto& replay : replay_entities) {
    SetClipScissor(replay.clip_coverage, current_render_pass,
                   global_pass_position);
    if (!replay.clip_contents.Render(renderer_, current_render_pass,
                                     replay.clip_depth)) {
      VALIDATION_LOG << "Failed to render entity for clip restore.";
    }
  }

  return input_texture;
}

bool Canvas::BlitToOnscreen() {
  auto command_buffer = renderer_.GetContext()->CreateCommandBuffer();
  command_buffer->SetLabel("EntityPass Root Command Buffer");
  auto offscreen_target = render_passes_.back()
                              .inline_pass_context->GetPassTarget()
                              .GetRenderTarget();

  if (renderer_.GetContext()
          ->GetCapabilities()
          ->SupportsTextureToTextureBlits()) {
    auto blit_pass = command_buffer->CreateBlitPass();
    blit_pass->AddCopy(offscreen_target.GetRenderTargetTexture(),
                       render_target_.GetRenderTargetTexture());
    if (!blit_pass->EncodeCommands(
            renderer_.GetContext()->GetResourceAllocator())) {
      VALIDATION_LOG << "Failed to encode root pass blit command.";
      return false;
    }
    if (!renderer_.GetContext()->EnqueueCommandBuffer(
            std::move(command_buffer))) {
      return false;
    }
  } else {
    auto render_pass = command_buffer->CreateRenderPass(render_target_);
    render_pass->SetLabel("EntityPass Root Render Pass");

    {
      auto size_rect = Rect::MakeSize(offscreen_target.GetRenderTargetSize());
      auto contents = TextureContents::MakeRect(size_rect);
      contents->SetTexture(offscreen_target.GetRenderTargetTexture());
      contents->SetSourceRect(size_rect);
      contents->SetLabel("Root pass blit");

      Entity entity;
      entity.SetContents(contents);
      entity.SetBlendMode(BlendMode::kSource);

      if (!entity.Render(renderer_, *render_pass)) {
        VALIDATION_LOG << "Failed to render EntityPass root blit.";
        return false;
      }
    }

    if (!render_pass->EncodeCommands()) {
      VALIDATION_LOG << "Failed to encode root pass command buffer.";
      return false;
    }
    if (!renderer_.GetContext()->EnqueueCommandBuffer(
            std::move(command_buffer))) {
      return false;
    }
  }
  return true;
}

void Canvas::EndReplay() {
  FML_DCHECK(render_passes_.size() == 1u);
  render_passes_.back().inline_pass_context->GetRenderPass();
  render_passes_.back().inline_pass_context->EndPass();
  backdrop_data_.clear();

  // If requires_readback_ was true, then we rendered to an offscreen texture
  // instead of to the onscreen provided in the render target. Now we need to
  // draw or blit the offscreen back to the onscreen.
  if (requires_readback_) {
    BlitToOnscreen();
  }

  if (!renderer_.GetContext()->FlushCommandBuffers()) {
    // Not much we can do.
    VALIDATION_LOG << "Failed to submit command buffers";
  }
  render_passes_.clear();
  renderer_.GetRenderTargetCache()->End();
  clip_geometry_.clear();

  Reset();
  Initialize(initial_cull_rect_);
}

}  // namespace impeller
