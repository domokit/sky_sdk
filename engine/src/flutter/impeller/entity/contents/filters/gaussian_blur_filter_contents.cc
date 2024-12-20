// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/entity/contents/filters/gaussian_blur_filter_contents.h"

#include <cmath>

#include "flutter/fml/make_copyable.h"
#include "impeller/entity/contents/clip_contents.h"
#include "impeller/entity/contents/content_context.h"
#include "impeller/entity/entity.h"
#include "impeller/entity/texture_downsample.frag.h"
#include "impeller/entity/texture_fill.frag.h"
#include "impeller/entity/texture_fill.vert.h"
#include "impeller/geometry/color.h"
#include "impeller/renderer/render_pass.h"
#include "impeller/renderer/vertex_buffer_builder.h"

namespace impeller {

using GaussianBlurVertexShader = GaussianBlurPipeline::VertexShader;
using GaussianBlurFragmentShader = GaussianBlurPipeline::FragmentShader;

namespace {

constexpr Scalar kMaxSigma = 500.0f;

SamplerDescriptor MakeSamplerDescriptor(MinMagFilter filter,
                                        SamplerAddressMode address_mode) {
  SamplerDescriptor sampler_desc;
  sampler_desc.min_filter = filter;
  sampler_desc.mag_filter = filter;
  sampler_desc.width_address_mode = address_mode;
  sampler_desc.height_address_mode = address_mode;
  return sampler_desc;
}

void SetTileMode(SamplerDescriptor* descriptor,
                 const ContentContext& renderer,
                 Entity::TileMode tile_mode) {
  switch (tile_mode) {
    case Entity::TileMode::kDecal:
      if (renderer.GetDeviceCapabilities().SupportsDecalSamplerAddressMode()) {
        descriptor->width_address_mode = SamplerAddressMode::kDecal;
        descriptor->height_address_mode = SamplerAddressMode::kDecal;
      }
      break;
    case Entity::TileMode::kClamp:
      descriptor->width_address_mode = SamplerAddressMode::kClampToEdge;
      descriptor->height_address_mode = SamplerAddressMode::kClampToEdge;
      break;
    case Entity::TileMode::kMirror:
      descriptor->width_address_mode = SamplerAddressMode::kMirror;
      descriptor->height_address_mode = SamplerAddressMode::kMirror;
      break;
    case Entity::TileMode::kRepeat:
      descriptor->width_address_mode = SamplerAddressMode::kRepeat;
      descriptor->height_address_mode = SamplerAddressMode::kRepeat;
      break;
  }
}

Vector2 Clamp(Vector2 vec2, Scalar min, Scalar max) {
  return Vector2(std::clamp(vec2.x, /*lo=*/min, /*hi=*/max),
                 std::clamp(vec2.y, /*lo=*/min, /*hi=*/max));
}

Vector2 ExtractScale(const Matrix& matrix) {
  Vector2 entity_scale_x = matrix * Vector2(1.0, 0.0);
  Vector2 entity_scale_y = matrix * Vector2(0.0, 1.0);
  return Vector2(entity_scale_x.GetLength(), entity_scale_y.GetLength());
}

struct BlurInfo {
  /// The scalar that is used to get from source space to unrotated local space.
  Vector2 source_space_scalar;
  /// The translation that is used to get from  source space to unrotated local
  /// space.
  Vector2 source_space_offset;
  /// Sigma when considering an entity's scale and the effect transform.
  Vector2 scaled_sigma;
  /// Blur radius in source pixels based on scaled_sigma.
  Vector2 blur_radius;
  /// The halo padding in source space.
  Vector2 padding;
  /// Padding in unrotated local space.
  Vector2 local_padding;
};

/// Calculates sigma derivatives necessary for rendering or calculating
/// coverage.
BlurInfo CalculateBlurInfo(const Entity& entity,
                           const Matrix& effect_transform,
                           Vector2 sigma) {
  // Source space here is scaled by the entity's transform. This is a
  // requirement for text to be rendered correctly. You can think of this as
  // "scaled source space" or "un-rotated local space". The entity's rotation is
  // applied to the result of the blur as part of the result's transform.
  const Vector2 source_space_scalar =
      ExtractScale(entity.GetTransform().Basis());
  const Vector2 source_space_offset =
      Vector2(entity.GetTransform().m[12], entity.GetTransform().m[13]);

  Vector2 scaled_sigma =
      (effect_transform.Basis() * Matrix::MakeScale(source_space_scalar) *  //
       Vector2(GaussianBlurFilterContents::ScaleSigma(sigma.x),
               GaussianBlurFilterContents::ScaleSigma(sigma.y)))
          .Abs();
  scaled_sigma = Clamp(scaled_sigma, 0, kMaxSigma);
  Vector2 blur_radius =
      Vector2(GaussianBlurFilterContents::CalculateBlurRadius(scaled_sigma.x),
              GaussianBlurFilterContents::CalculateBlurRadius(scaled_sigma.y));
  Vector2 padding(ceil(blur_radius.x), ceil(blur_radius.y));
  Vector2 local_padding =
      (Matrix::MakeScale(source_space_scalar) * padding).Abs();
  return {
      .source_space_scalar = source_space_scalar,
      .source_space_offset = source_space_offset,
      .scaled_sigma = scaled_sigma,
      .blur_radius = blur_radius,
      .padding = padding,
      .local_padding = local_padding,
  };
}

/// Perform FilterInput::GetSnapshot with safety checks.
std::optional<Snapshot> GetSnapshot(const std::shared_ptr<FilterInput>& input,
                                    const ContentContext& renderer,
                                    const Entity& entity,
                                    const std::optional<Rect>& coverage_hint) {
  std::optional<Snapshot> input_snapshot =
      input->GetSnapshot("GaussianBlur", renderer, entity,
                         /*coverage_limit=*/coverage_hint);
  if (!input_snapshot.has_value()) {
    return std::nullopt;
  }

  return input_snapshot;
}

/// Returns `rect` relative to `reference`, where Rect::MakeXYWH(0,0,1,1) will
/// be returned when `rect` == `reference`.
Rect MakeReferenceUVs(const Rect& reference, const Rect& rect) {
  Rect result = Rect::MakeOriginSize(rect.GetOrigin() - reference.GetOrigin(),
                                     rect.GetSize());
  return result.Scale(1.0f / Vector2(reference.GetSize()));
}

Quad CalculateSnapshotUVs(
    const Snapshot& input_snapshot,
    const std::optional<Rect>& source_expanded_coverage_hint) {
  std::optional<Rect> input_snapshot_coverage = input_snapshot.GetCoverage();
  Quad blur_uvs = {Point(0, 0), Point(1, 0), Point(0, 1), Point(1, 1)};
  FML_DCHECK(input_snapshot.transform.IsTranslationScaleOnly());
  if (source_expanded_coverage_hint.has_value() &&
      input_snapshot_coverage.has_value()) {
    // Only process the uvs where the blur is happening, not the whole texture.
    std::optional<Rect> uvs =
        MakeReferenceUVs(input_snapshot_coverage.value(),
                         source_expanded_coverage_hint.value())
            .Intersection(Rect::MakeSize(Size(1, 1)));
    FML_DCHECK(uvs.has_value());
    if (uvs.has_value()) {
      blur_uvs[0] = uvs->GetLeftTop();
      blur_uvs[1] = uvs->GetRightTop();
      blur_uvs[2] = uvs->GetLeftBottom();
      blur_uvs[3] = uvs->GetRightBottom();
    }
  }
  return blur_uvs;
}

Scalar CeilToDivisible(Scalar val, Scalar divisor) {
  if (divisor == 0.0f) {
    return val;
  }

  Scalar remainder = fmod(val, divisor);
  if (remainder != 0.0f) {
    return val + (divisor - remainder);
  } else {
    return val;
  }
}

Scalar FloorToDivisible(Scalar val, Scalar divisor) {
  if (divisor == 0.0f) {
    return val;
  }

  Scalar remainder = fmod(val, divisor);
  if (remainder != 0.0f) {
    return val - remainder;
  } else {
    return val;
  }
}

struct DownsamplePassArgs {
  /// The output size of the down-sampling pass.
  ISize subpass_size;
  /// The UVs that will be used for drawing to the down-sampling pass.
  /// This effectively is chopping out a region of the input.
  Quad uvs;
  /// The effective scalar of the down-sample pass.
  /// This isn't usually exactly as we'd calculate because it has to be rounded
  /// to integer boundaries for generating the texture for the output.
  Vector2 effective_scalar;
  /// Transforms from unrotated local space to position the output from the
  /// down-sample pass.
  /// This can differ if we request a coverage hint but it is rejected, as is
  /// the case with backdrop filters.
  Matrix transform;
};

/// Calculates info required for the down-sampling pass.
DownsamplePassArgs CalculateDownsamplePassArgs(
    Vector2 scaled_sigma,
    Vector2 padding,
    const Snapshot& input_snapshot,
    const std::optional<Rect>& source_expanded_coverage_hint,
    const std::shared_ptr<FilterInput>& input,
    const Entity& snapshot_entity) {
  Scalar desired_scalar =
      std::min(GaussianBlurFilterContents::CalculateScale(scaled_sigma.x),
               GaussianBlurFilterContents::CalculateScale(scaled_sigma.y));

  // TODO(jonahwilliams): If desired_scalar is 1.0 and we fully acquired the
  // gutter from the expanded_coverage_hint, we can skip the downsample pass.
  // pass.
  Vector2 downsample_scalar(desired_scalar, desired_scalar);
  // TODO(gaaclarke): The padding could be removed if we know it's not needed or
  //   resized to account for the expanded_clip_coverage. There doesn't appear
  //   to be the math to make those calculations though. The following
  //   optimization works, but causes a shimmer as a result of
  //   https://github.com/flutter/flutter/issues/140193 so it isn't applied.
  //
  //   !input_snapshot->GetCoverage()->Expand(-local_padding)
  //     .Contains(coverage_hint.value()))

  std::optional<Rect> snapshot_coverage = input_snapshot.GetCoverage();
  if (input_snapshot.transform.Equals(snapshot_entity.GetTransform()) &&
      source_expanded_coverage_hint.has_value() &&
      snapshot_coverage.has_value() &&
      snapshot_coverage->Contains(source_expanded_coverage_hint.value())) {
    // If the snapshot's transform is the identity transform and we have
    // coverage hint that fits inside of the snapshots coverage that means the
    // coverage hint was ignored so we will trim out the area we are interested
    // in the down-sample pass. This usually means we have a backdrop image
    // filter.
    //
    // The region we cut out will be aligned with the down-sample divisor to
    // avoid pixel alignment problems that create shimmering.
    int32_t divisor = std::round(1.0f / desired_scalar);
    Rect aligned_coverage_hint = Rect::MakeLTRB(
        FloorToDivisible(source_expanded_coverage_hint->GetLeft(), divisor),
        FloorToDivisible(source_expanded_coverage_hint->GetTop(), divisor),
        source_expanded_coverage_hint->GetRight(),
        source_expanded_coverage_hint->GetBottom());
    aligned_coverage_hint = Rect::MakeXYWH(
        aligned_coverage_hint.GetX(), aligned_coverage_hint.GetY(),
        CeilToDivisible(aligned_coverage_hint.GetWidth(), divisor),
        CeilToDivisible(aligned_coverage_hint.GetHeight(), divisor));
    ISize source_size = ISize(aligned_coverage_hint.GetSize().width,
                              aligned_coverage_hint.GetSize().height);
    Vector2 downsampled_size = source_size * downsample_scalar;
    Scalar int_part;
    FML_DCHECK(std::modf(downsampled_size.x, &int_part) == 0.0f);
    FML_DCHECK(std::modf(downsampled_size.y, &int_part) == 0.0f);
    (void)int_part;
    ISize subpass_size = ISize(downsampled_size.x, downsampled_size.y);
    Vector2 effective_scalar = Vector2(subpass_size) / source_size;
    FML_DCHECK(effective_scalar == downsample_scalar);

    Quad uvs = CalculateSnapshotUVs(input_snapshot, aligned_coverage_hint);
    return {
        .subpass_size = subpass_size,
        .uvs = uvs,
        .effective_scalar = effective_scalar,
        .transform = Matrix::MakeTranslation(
            {aligned_coverage_hint.GetX(), aligned_coverage_hint.GetY(), 0})};
  } else {
    //////////////////////////////////////////////////////////////////////////////
    auto input_snapshot_size = input_snapshot.texture->GetSize();
    Rect source_rect = Rect::MakeSize(input_snapshot_size);
    Rect source_rect_padded = source_rect.Expand(padding);
    Vector2 downsampled_size = source_rect_padded.GetSize() * downsample_scalar;
    ISize subpass_size =
        ISize(ceil(downsampled_size.x), ceil(downsampled_size.y));
    Vector2 divisible_size(CeilToDivisible(source_rect_padded.GetSize().width,
                                           1.0 / downsample_scalar.x),
                           CeilToDivisible(source_rect_padded.GetSize().height,
                                           1.0 / downsample_scalar.y));
    // Only make the padding divisible if we already have padding.  If we don't
    // have padding adding more can add artifacts to hard blur edges.
    Vector2 divisible_padding(
        padding.x > 0
            ? padding.x +
                  (divisible_size.x - source_rect_padded.GetSize().width) / 2.0
            : 0.f,
        padding.y > 0
            ? padding.y +
                  (divisible_size.y - source_rect_padded.GetSize().height) / 2.0
            : 0.f);
    source_rect_padded = source_rect.Expand(divisible_padding);

    Vector2 effective_scalar =
        Vector2(subpass_size) / source_rect_padded.GetSize();
    Quad uvs = GaussianBlurFilterContents::CalculateUVs(
        input, snapshot_entity, source_rect_padded, input_snapshot_size);
    return {
        .subpass_size = subpass_size,
        .uvs = uvs,
        .effective_scalar = effective_scalar,
        .transform = input_snapshot.transform *
                     Matrix::MakeTranslation(-divisible_padding),
    };
  }
}

/// Makes a subpass that will render the scaled down input and add the
/// transparent gutter required for the blur halo.
fml::StatusOr<RenderTarget> MakeDownsampleSubpass(
    const ContentContext& renderer,
    const std::shared_ptr<CommandBuffer>& command_buffer,
    const std::shared_ptr<Texture>& input_texture,
    const SamplerDescriptor& sampler_descriptor,
    const DownsamplePassArgs& pass_args,
    Entity::TileMode tile_mode) {
  using VS = TextureFillVertexShader;

  // If the texture already had mip levels generated, then we can use the
  // original downsample shader.
  if (pass_args.effective_scalar.x >= 0.5f ||
      (!input_texture->NeedsMipmapGeneration() &&
       input_texture->GetTextureDescriptor().mip_count > 1)) {
    ContentContext::SubpassCallback subpass_callback =
        [&](const ContentContext& renderer, RenderPass& pass) {
          HostBuffer& host_buffer = renderer.GetTransientsBuffer();

          pass.SetCommandLabel("Gaussian blur downsample");
          auto pipeline_options = OptionsFromPass(pass);
          pipeline_options.primitive_type = PrimitiveType::kTriangleStrip;
          pass.SetPipeline(renderer.GetTexturePipeline(pipeline_options));

          TextureFillVertexShader::FrameInfo frame_info;
          frame_info.mvp = Matrix::MakeOrthographic(ISize(1, 1));
          frame_info.texture_sampler_y_coord_scale =
              input_texture->GetYCoordScale();

          TextureFillFragmentShader::FragInfo frag_info;
          frag_info.alpha = 1.0;

          const Quad& uvs = pass_args.uvs;
          std::array<VS::PerVertexData, 4> vertices = {
              VS::PerVertexData{Point(0, 0), uvs[0]},
              VS::PerVertexData{Point(1, 0), uvs[1]},
              VS::PerVertexData{Point(0, 1), uvs[2]},
              VS::PerVertexData{Point(1, 1), uvs[3]},
          };
          pass.SetVertexBuffer(CreateVertexBuffer(vertices, host_buffer));

          SamplerDescriptor linear_sampler_descriptor = sampler_descriptor;
          SetTileMode(&linear_sampler_descriptor, renderer, tile_mode);
          linear_sampler_descriptor.mag_filter = MinMagFilter::kLinear;
          linear_sampler_descriptor.min_filter = MinMagFilter::kLinear;
          TextureFillVertexShader::BindFrameInfo(
              pass, host_buffer.EmplaceUniform(frame_info));
          TextureFillFragmentShader::BindFragInfo(
              pass, host_buffer.EmplaceUniform(frag_info));
          TextureFillFragmentShader::BindTextureSampler(
              pass, input_texture,
              renderer.GetContext()->GetSamplerLibrary()->GetSampler(
                  linear_sampler_descriptor));

          return pass.Draw().ok();
        };
    return renderer.MakeSubpass("Gaussian Blur Filter", pass_args.subpass_size,
                                command_buffer, subpass_callback);
  } else {
    // This assumes we don't scale below 1/16.
    Scalar edge = 1.0;
    Scalar ratio = 0.25;
    if (pass_args.effective_scalar.x <= 0.0625f) {
      edge = 7.0;
      ratio = 1.0f / 64.0f;
    } else if (pass_args.effective_scalar.x <= 0.125f) {
      edge = 3.0;
      ratio = 1.0f / 16.0f;
    }
    ContentContext::SubpassCallback subpass_callback =
        [&](const ContentContext& renderer, RenderPass& pass) {
          HostBuffer& host_buffer = renderer.GetTransientsBuffer();

          pass.SetCommandLabel("Gaussian blur downsample");
          auto pipeline_options = OptionsFromPass(pass);
          pipeline_options.primitive_type = PrimitiveType::kTriangleStrip;
#ifdef IMPELLER_ENABLE_OPENGLES
          // The GLES backend conditionally supports decal tile mode, while
          // decal is always supported for Vulkan and Metal.
          if (renderer.GetDeviceCapabilities()
                  .SupportsDecalSamplerAddressMode() ||
              tile_mode != Entity::TileMode::kDecal) {
            pass.SetPipeline(renderer.GetDownsamplePipeline(pipeline_options));
          } else {
            pass.SetPipeline(
                renderer.GetDownsampleTextureGlesPipeline(pipeline_options));
          }
#else
          pass.SetPipeline(renderer.GetDownsamplePipeline(pipeline_options));
#endif  // IMPELLER_ENABLE_OPENGLES

          TextureFillVertexShader::FrameInfo frame_info;
          frame_info.mvp = Matrix::MakeOrthographic(ISize(1, 1));
          frame_info.texture_sampler_y_coord_scale =
              input_texture->GetYCoordScale();

          TextureDownsampleFragmentShader::FragInfo frag_info;
          frag_info.edge = edge;
          frag_info.ratio = ratio;
          frag_info.pixel_size = Vector2(1.0f / Size(input_texture->GetSize()));

          const Quad& uvs = pass_args.uvs;
          std::array<VS::PerVertexData, 4> vertices = {
              VS::PerVertexData{Point(0, 0), uvs[0]},
              VS::PerVertexData{Point(1, 0), uvs[1]},
              VS::PerVertexData{Point(0, 1), uvs[2]},
              VS::PerVertexData{Point(1, 1), uvs[3]},
          };
          pass.SetVertexBuffer(CreateVertexBuffer(vertices, host_buffer));

          SamplerDescriptor linear_sampler_descriptor = sampler_descriptor;
          SetTileMode(&linear_sampler_descriptor, renderer, tile_mode);
          linear_sampler_descriptor.mag_filter = MinMagFilter::kLinear;
          linear_sampler_descriptor.min_filter = MinMagFilter::kLinear;
          TextureFillVertexShader::BindFrameInfo(
              pass, host_buffer.EmplaceUniform(frame_info));
          TextureDownsampleFragmentShader::BindFragInfo(
              pass, host_buffer.EmplaceUniform(frag_info));
          TextureDownsampleFragmentShader::BindTextureSampler(
              pass, input_texture,
              renderer.GetContext()->GetSamplerLibrary()->GetSampler(
                  linear_sampler_descriptor));

          return pass.Draw().ok();
        };
    return renderer.MakeSubpass("Gaussian Blur Filter", pass_args.subpass_size,
                                command_buffer, subpass_callback);
  }
}

fml::StatusOr<RenderTarget> MakeBlurSubpass(
    const ContentContext& renderer,
    const std::shared_ptr<CommandBuffer>& command_buffer,
    const RenderTarget& input_pass,
    const SamplerDescriptor& sampler_descriptor,
    Entity::TileMode tile_mode,
    const BlurParameters& blur_info,
    std::optional<RenderTarget> destination_target,
    const Quad& blur_uvs) {
  using VS = GaussianBlurVertexShader;

  if (blur_info.blur_sigma < kEhCloseEnough) {
    return input_pass;
  }

  const std::shared_ptr<Texture>& input_texture =
      input_pass.GetRenderTargetTexture();

  // TODO(gaaclarke): This blurs the whole image, but because we know the clip
  //                  region we could focus on just blurring that.
  ISize subpass_size = input_texture->GetSize();
  ContentContext::SubpassCallback subpass_callback =
      [&](const ContentContext& renderer, RenderPass& pass) {
        GaussianBlurVertexShader::FrameInfo frame_info;
        frame_info.mvp = Matrix::MakeOrthographic(ISize(1, 1)),
        frame_info.texture_sampler_y_coord_scale =
            input_texture->GetYCoordScale();

        HostBuffer& host_buffer = renderer.GetTransientsBuffer();

        ContentContextOptions options = OptionsFromPass(pass);
        options.primitive_type = PrimitiveType::kTriangleStrip;
        pass.SetPipeline(renderer.GetGaussianBlurPipeline(options));

        std::array<VS::PerVertexData, 4> vertices = {
            VS::PerVertexData{blur_uvs[0], blur_uvs[0]},
            VS::PerVertexData{blur_uvs[1], blur_uvs[1]},
            VS::PerVertexData{blur_uvs[2], blur_uvs[2]},
            VS::PerVertexData{blur_uvs[3], blur_uvs[3]},
        };
        pass.SetVertexBuffer(CreateVertexBuffer(vertices, host_buffer));

        SamplerDescriptor linear_sampler_descriptor = sampler_descriptor;
        linear_sampler_descriptor.mag_filter = MinMagFilter::kLinear;
        linear_sampler_descriptor.min_filter = MinMagFilter::kLinear;
        GaussianBlurFragmentShader::BindTextureSampler(
            pass, input_texture,
            renderer.GetContext()->GetSamplerLibrary()->GetSampler(
                linear_sampler_descriptor));
        GaussianBlurVertexShader::BindFrameInfo(
            pass, host_buffer.EmplaceUniform(frame_info));
        GaussianBlurFragmentShader::BindKernelSamples(
            pass, host_buffer.EmplaceUniform(
                      LerpHackKernelSamples(GenerateBlurInfo(blur_info))));
        return pass.Draw().ok();
      };
  if (destination_target.has_value()) {
    return renderer.MakeSubpass("Gaussian Blur Filter",
                                destination_target.value(), command_buffer,
                                subpass_callback);
  } else {
    return renderer.MakeSubpass("Gaussian Blur Filter", subpass_size,
                                command_buffer, subpass_callback);
  }
}

int ScaleBlurRadius(Scalar radius, Scalar scalar) {
  return static_cast<int>(std::round(radius * scalar));
}

Entity ApplyClippedBlurStyle(Entity::ClipOperation clip_operation,
                             const Entity& entity,
                             const std::shared_ptr<FilterInput>& input,
                             const Snapshot& input_snapshot,
                             Entity blur_entity,
                             const Geometry* geometry) {
  Matrix entity_transform = entity.GetTransform();
  Matrix blur_transform = blur_entity.GetTransform();

  auto renderer =
      fml::MakeCopyable([blur_entity = blur_entity.Clone(), clip_operation,
                         entity_transform, blur_transform, geometry](
                            const ContentContext& renderer,
                            const Entity& entity, RenderPass& pass) mutable {
        Entity clipper;
        clipper.SetClipDepth(entity.GetClipDepth());
        clipper.SetTransform(entity.GetTransform() * entity_transform);

        auto geom_result = geometry->GetPositionBuffer(renderer, clipper, pass);

        ClipContents clip_contents(geometry->GetCoverage(clipper.GetTransform())
                                       .value_or(Rect::MakeLTRB(0, 0, 0, 0)),
                                   /*is_axis_aligned_rect=*/false);
        clip_contents.SetClipOperation(clip_operation);
        clip_contents.SetGeometry(std::move(geom_result));

        if (!clip_contents.Render(renderer, pass, entity.GetClipDepth())) {
          return false;
        }
        blur_entity.SetClipDepth(entity.GetClipDepth());
        blur_entity.SetTransform(entity.GetTransform() * blur_transform);

        return blur_entity.Render(renderer, pass);
      });
  auto coverage =
      fml::MakeCopyable([blur_entity = std::move(blur_entity),
                         blur_transform](const Entity& entity) mutable {
        blur_entity.SetTransform(entity.GetTransform() * blur_transform);
        return blur_entity.GetCoverage();
      });
  Entity result;
  result.SetContents(Contents::MakeAnonymous(renderer, coverage));
  return result;
}

Entity ApplyBlurStyle(FilterContents::BlurStyle blur_style,
                      const Entity& entity,
                      const std::shared_ptr<FilterInput>& input,
                      const Snapshot& input_snapshot,
                      Entity blur_entity,
                      const Geometry* geometry,
                      Vector2 source_space_scalar,
                      Vector2 source_space_offset) {
  switch (blur_style) {
    case FilterContents::BlurStyle::kNormal:
      return blur_entity;
    case FilterContents::BlurStyle::kInner:
      return ApplyClippedBlurStyle(Entity::ClipOperation::kIntersect, entity,
                                   input, input_snapshot,
                                   std::move(blur_entity), geometry);
      break;
    case FilterContents::BlurStyle::kOuter:
      return ApplyClippedBlurStyle(Entity::ClipOperation::kDifference, entity,
                                   input, input_snapshot,
                                   std::move(blur_entity), geometry);
    case FilterContents::BlurStyle::kSolid: {
      Entity snapshot_entity =
          Entity::FromSnapshot(input_snapshot, entity.GetBlendMode());
      Entity result;
      Matrix blurred_transform = blur_entity.GetTransform();
      Matrix snapshot_transform =
          entity.GetTransform() *  //
          Matrix::MakeScale(1.f / source_space_scalar) *
          Matrix::MakeTranslation(-1 * source_space_offset) *
          input_snapshot.transform;
      result.SetContents(Contents::MakeAnonymous(
          fml::MakeCopyable([blur_entity = blur_entity.Clone(),
                             blurred_transform, snapshot_transform,
                             snapshot_entity = std::move(snapshot_entity)](
                                const ContentContext& renderer,
                                const Entity& entity,
                                RenderPass& pass) mutable {
            snapshot_entity.SetTransform(entity.GetTransform() *
                                         snapshot_transform);
            snapshot_entity.SetClipDepth(entity.GetClipDepth());
            if (!snapshot_entity.Render(renderer, pass)) {
              return false;
            }
            blur_entity.SetClipDepth(entity.GetClipDepth());
            blur_entity.SetTransform(entity.GetTransform() * blurred_transform);
            return blur_entity.Render(renderer, pass);
          }),
          fml::MakeCopyable([blur_entity = blur_entity.Clone(),
                             blurred_transform](const Entity& entity) mutable {
            blur_entity.SetTransform(entity.GetTransform() * blurred_transform);
            return blur_entity.GetCoverage();
          })));
      return result;
    }
  }
}
}  // namespace

GaussianBlurFilterContents::GaussianBlurFilterContents(
    Scalar sigma_x,
    Scalar sigma_y,
    Entity::TileMode tile_mode,
    BlurStyle mask_blur_style,
    const Geometry* mask_geometry)
    : sigma_(sigma_x, sigma_y),
      tile_mode_(tile_mode),
      mask_blur_style_(mask_blur_style),
      mask_geometry_(mask_geometry) {
  // This is supposed to be enforced at a higher level.
  FML_DCHECK(mask_blur_style == BlurStyle::kNormal || mask_geometry);
}

// This value was extracted from Skia, see:
//  * https://github.com/google/skia/blob/d29cc3fe182f6e8a8539004a6a4ee8251677a6fd/src/gpu/ganesh/GrBlurUtils.cpp#L2561-L2576
//  * https://github.com/google/skia/blob/d29cc3fe182f6e8a8539004a6a4ee8251677a6fd/src/gpu/BlurUtils.h#L57
Scalar GaussianBlurFilterContents::CalculateScale(Scalar sigma) {
  if (sigma <= 4) {
    return 1.0;
  }
  Scalar raw_result = 4.0 / sigma;
  // Round to the nearest 1/(2^n) to get the best quality down scaling.
  Scalar exponent = round(log2f(raw_result));
  // Don't scale down below 1/16th to preserve signal.
  exponent = std::max(-4.0f, exponent);
  Scalar rounded = powf(2.0f, exponent);
  Scalar result = rounded;
  // Extend the range of the 1/8th downsample based on the effective kernel size
  // for the blur.
  if (rounded < 0.125f) {
    Scalar rounded_plus = powf(2.0f, exponent + 1);
    Scalar blur_radius = CalculateBlurRadius(sigma);
    int kernel_size_plus = (ScaleBlurRadius(blur_radius, rounded_plus) * 2) + 1;
    // This constant was picked by looking at the results to make sure no
    // shimmering was introduced at the highest sigma values that downscale to
    // 1/16th.
    static constexpr int32_t kEighthDownsampleKernalWidthMax = 41;
    result = kernel_size_plus <= kEighthDownsampleKernalWidthMax ? rounded_plus
                                                                 : rounded;
  }
  return result;
};

std::optional<Rect> GaussianBlurFilterContents::GetFilterSourceCoverage(
    const Matrix& effect_transform,
    const Rect& output_limit) const {
  Vector2 scaled_sigma = {ScaleSigma(sigma_.x), ScaleSigma(sigma_.y)};
  Vector2 blur_radius = {CalculateBlurRadius(scaled_sigma.x),
                         CalculateBlurRadius(scaled_sigma.y)};
  Vector3 blur_radii =
      effect_transform.Basis() * Vector3{blur_radius.x, blur_radius.y, 0.0};
  return output_limit.Expand(Point(blur_radii.x, blur_radii.y));
}

std::optional<Rect> GaussianBlurFilterContents::GetFilterCoverage(
    const FilterInput::Vector& inputs,
    const Entity& entity,
    const Matrix& effect_transform) const {
  if (inputs.empty()) {
    return {};
  }
  std::optional<Rect> input_coverage = inputs[0]->GetCoverage(entity);
  if (!input_coverage.has_value()) {
    return {};
  }

  BlurInfo blur_info = CalculateBlurInfo(entity, effect_transform, sigma_);
  return input_coverage.value().Expand(
      Point(blur_info.local_padding.x, blur_info.local_padding.y));
}

// A brief overview how this works:
// 1) Snapshot the filter input.
// 2) Perform downsample pass. This also inserts the gutter around the input
//    snapshot since the blur can render outside the bounds of the snapshot.
// 3) Perform 1D horizontal blur pass.
// 4) Perform 1D vertical blur pass.
// 5) Apply the blur style to the blur result. This may just mask the output or
//    draw the original snapshot over the result.
std::optional<Entity> GaussianBlurFilterContents::RenderFilter(
    const FilterInput::Vector& inputs,
    const ContentContext& renderer,
    const Entity& entity,
    const Matrix& effect_transform,
    const Rect& coverage,
    const std::optional<Rect>& coverage_hint) const {
  if (inputs.empty()) {
    return std::nullopt;
  }

  BlurInfo blur_info = CalculateBlurInfo(entity, effect_transform, sigma_);

  // Apply as much of the desired padding as possible from the source. This may
  // be ignored so must be accounted for in the downsample pass by adding a
  // transparent gutter.
  std::optional<Rect> expanded_coverage_hint;
  if (coverage_hint.has_value()) {
    expanded_coverage_hint = coverage_hint->Expand(blur_info.local_padding);
  }

  Entity snapshot_entity = entity.Clone();
  snapshot_entity.SetTransform(
      Matrix::MakeTranslation(blur_info.source_space_offset) *
      Matrix::MakeScale(blur_info.source_space_scalar));

  std::optional<Rect> source_expanded_coverage_hint;
  if (expanded_coverage_hint.has_value()) {
    source_expanded_coverage_hint = expanded_coverage_hint->TransformBounds(
        Matrix::MakeTranslation(blur_info.source_space_offset) *
        Matrix::MakeScale(blur_info.source_space_scalar) *
        entity.GetTransform().Invert());
  }

  std::optional<Snapshot> input_snapshot = GetSnapshot(
      inputs[0], renderer, snapshot_entity, source_expanded_coverage_hint);
  if (!input_snapshot.has_value()) {
    return std::nullopt;
  }

  if (blur_info.scaled_sigma.x < kEhCloseEnough &&
      blur_info.scaled_sigma.y < kEhCloseEnough) {
    Entity result =
        Entity::FromSnapshot(input_snapshot.value(),
                             entity.GetBlendMode());  // No blur to render.
    result.SetTransform(
        entity.GetTransform() *
        Matrix::MakeScale(1.f / blur_info.source_space_scalar) *
        Matrix::MakeTranslation(-1 * blur_info.source_space_offset) *
        input_snapshot->transform);
    return result;
  }

  // Note: The code below uses three different command buffers when it would be
  // possible to combine the operations into a single buffer. From testing and
  // user bug reports (see https://github.com/flutter/flutter/issues/154046 ),
  // this sometimes causes deviceLost errors on older Adreno devices. Breaking
  // the work up into three different command buffers seems to prevent this
  // crash.
  std::shared_ptr<CommandBuffer> command_buffer_1 =
      renderer.GetContext()->CreateCommandBuffer();
  if (!command_buffer_1) {
    return std::nullopt;
  }

  DownsamplePassArgs downsample_pass_args = CalculateDownsamplePassArgs(
      blur_info.scaled_sigma, blur_info.padding, input_snapshot.value(),
      source_expanded_coverage_hint, inputs[0], snapshot_entity);

  fml::StatusOr<RenderTarget> pass1_out = MakeDownsampleSubpass(
      renderer, command_buffer_1, input_snapshot->texture,
      input_snapshot->sampler_descriptor, downsample_pass_args, tile_mode_);

  if (!pass1_out.ok()) {
    return std::nullopt;
  }

  Vector2 pass1_pixel_size =
      1.0 / Vector2(pass1_out.value().GetRenderTargetTexture()->GetSize());

  Quad blur_uvs = {Point(0, 0), Point(1, 0), Point(0, 1), Point(1, 1)};

  std::shared_ptr<CommandBuffer> command_buffer_2 =
      renderer.GetContext()->CreateCommandBuffer();
  if (!command_buffer_2) {
    return std::nullopt;
  }

  fml::StatusOr<RenderTarget> pass2_out = MakeBlurSubpass(
      renderer, command_buffer_2, /*input_pass=*/pass1_out.value(),
      input_snapshot->sampler_descriptor, tile_mode_,
      BlurParameters{
          .blur_uv_offset = Point(0.0, pass1_pixel_size.y),
          .blur_sigma = blur_info.scaled_sigma.y *
                        downsample_pass_args.effective_scalar.y,
          .blur_radius = ScaleBlurRadius(
              blur_info.blur_radius.y, downsample_pass_args.effective_scalar.y),
          .step_size = 1,
      },
      /*destination_target=*/std::nullopt, blur_uvs);

  if (!pass2_out.ok()) {
    return std::nullopt;
  }

  std::shared_ptr<CommandBuffer> command_buffer_3 =
      renderer.GetContext()->CreateCommandBuffer();
  if (!command_buffer_3) {
    return std::nullopt;
  }

  // Only ping pong if the first pass actually created a render target.
  auto pass3_destination = pass2_out.value().GetRenderTargetTexture() !=
                                   pass1_out.value().GetRenderTargetTexture()
                               ? std::optional<RenderTarget>(pass1_out.value())
                               : std::optional<RenderTarget>(std::nullopt);

  fml::StatusOr<RenderTarget> pass3_out = MakeBlurSubpass(
      renderer, command_buffer_3, /*input_pass=*/pass2_out.value(),
      input_snapshot->sampler_descriptor, tile_mode_,
      BlurParameters{
          .blur_uv_offset = Point(pass1_pixel_size.x, 0.0),
          .blur_sigma = blur_info.scaled_sigma.x *
                        downsample_pass_args.effective_scalar.x,
          .blur_radius = ScaleBlurRadius(
              blur_info.blur_radius.x, downsample_pass_args.effective_scalar.x),
          .step_size = 1,
      },
      pass3_destination, blur_uvs);

  if (!pass3_out.ok()) {
    return std::nullopt;
  }

  if (!(renderer.GetContext()->EnqueueCommandBuffer(
            std::move(command_buffer_1)) &&
        renderer.GetContext()->EnqueueCommandBuffer(
            std::move(command_buffer_2)) &&
        renderer.GetContext()->EnqueueCommandBuffer(
            std::move(command_buffer_3)))) {
    return std::nullopt;
  }

  // The ping-pong approach requires that each render pass output has the same
  // size.
  FML_DCHECK((pass1_out.value().GetRenderTargetSize() ==
              pass2_out.value().GetRenderTargetSize()) &&
             (pass2_out.value().GetRenderTargetSize() ==
              pass3_out.value().GetRenderTargetSize()));

  SamplerDescriptor sampler_desc = MakeSamplerDescriptor(
      MinMagFilter::kLinear, SamplerAddressMode::kClampToEdge);

  Entity blur_output_entity = Entity::FromSnapshot(
      Snapshot{.texture = pass3_out.value().GetRenderTargetTexture(),
               .transform =
                   entity.GetTransform() *                                   //
                   Matrix::MakeScale(1.f / blur_info.source_space_scalar) *  //
                   Matrix::MakeTranslation(-1 * blur_info.source_space_offset) *
                   downsample_pass_args.transform *  //
                   Matrix::MakeScale(1 / downsample_pass_args.effective_scalar),
               .sampler_descriptor = sampler_desc,
               .opacity = input_snapshot->opacity},
      entity.GetBlendMode());

  return ApplyBlurStyle(mask_blur_style_, entity, inputs[0],
                        input_snapshot.value(), std::move(blur_output_entity),
                        mask_geometry_, blur_info.source_space_scalar,
                        blur_info.source_space_offset);
}

Scalar GaussianBlurFilterContents::CalculateBlurRadius(Scalar sigma) {
  return static_cast<Radius>(Sigma(sigma)).radius;
}

Quad GaussianBlurFilterContents::CalculateUVs(
    const std::shared_ptr<FilterInput>& filter_input,
    const Entity& entity,
    const Rect& source_rect,
    const ISize& texture_size) {
  Matrix input_transform = filter_input->GetLocalTransform(entity);
  Quad coverage_quad = source_rect.GetTransformedPoints(input_transform);

  Matrix uv_transform = Matrix::MakeScale(
      {1.0f / texture_size.width, 1.0f / texture_size.height, 1.0f});
  return uv_transform.Transform(coverage_quad);
}

// This function was calculated by observing Skia's behavior. Its blur at 500
// seemed to be 0.15.  Since we clamp at 500 I solved the quadratic equation
// that puts the minima there and a f(0)=1.
Scalar GaussianBlurFilterContents::ScaleSigma(Scalar sigma) {
  // Limit the kernel size to 1000x1000 pixels, like Skia does.
  Scalar clamped = std::min(sigma, kMaxSigma);
  constexpr Scalar a = 3.4e-06;
  constexpr Scalar b = -3.4e-3;
  constexpr Scalar c = 1.f;
  Scalar scalar = c + b * clamped + a * clamped * clamped;
  return clamped * scalar;
}

KernelSamples GenerateBlurInfo(BlurParameters parameters) {
  KernelSamples result;
  result.sample_count =
      ((2 * parameters.blur_radius) / parameters.step_size) + 1;

  // Chop off the last samples if the radius >= 3 where they account for < 1.56%
  // of the result.
  int x_offset = 0;
  if (parameters.blur_radius >= 3) {
    result.sample_count -= 2;
    x_offset = 1;
  }

  // This is a safe-guard to make sure we don't overflow the fragment shader.
  // The kernel size is multiplied by 2 since we'll use the lerp hack on the
  // result. In practice this isn't throwing away much data since the blur radii
  // are around 53 before the down-sampling and max sigma of 500 kick in.
  //
  // TODO(https://github.com/flutter/flutter/issues/150462): Come up with a more
  // wholistic remedy for this.  A proper downsample size should not make this
  // required. Or we can increase the kernel size.
  if (result.sample_count > KernelSamples::kMaxKernelSize) {
    result.sample_count = KernelSamples::kMaxKernelSize;
  }

  Scalar tally = 0.0f;
  for (int i = 0; i < result.sample_count; ++i) {
    int x = x_offset + (i * parameters.step_size) - parameters.blur_radius;
    result.samples[i] = KernelSample{
        .uv_offset = parameters.blur_uv_offset * x,
        .coefficient = expf(-0.5f * (x * x) /
                            (parameters.blur_sigma * parameters.blur_sigma)) /
                       (sqrtf(2.0f * M_PI) * parameters.blur_sigma),
    };
    tally += result.samples[i].coefficient;
  }

  // Make sure everything adds up to 1.
  for (auto& sample : result.samples) {
    sample.coefficient /= tally;
  }

  return result;
}

// This works by shrinking the kernel size by 2 and relying on lerp to read
// between the samples.
GaussianBlurPipeline::FragmentShader::KernelSamples LerpHackKernelSamples(
    KernelSamples parameters) {
  GaussianBlurPipeline::FragmentShader::KernelSamples result = {};
  result.sample_count = ((parameters.sample_count - 1) / 2) + 1;
  int32_t middle = result.sample_count / 2;
  int32_t j = 0;
  FML_DCHECK(result.sample_count <= kGaussianBlurMaxKernelSize);
  static_assert(sizeof(result.sample_data) ==
                sizeof(std::array<Vector4, kGaussianBlurMaxKernelSize>));

  for (int i = 0; i < result.sample_count; i++) {
    if (i == middle) {
      result.sample_data[i].x = parameters.samples[j].uv_offset.x;
      result.sample_data[i].y = parameters.samples[j].uv_offset.y;
      result.sample_data[i].z = parameters.samples[j].coefficient;
      j++;
    } else {
      KernelSample left = parameters.samples[j];
      KernelSample right = parameters.samples[j + 1];

      result.sample_data[i].z = left.coefficient + right.coefficient;

      Point uv = (left.uv_offset * left.coefficient +
                  right.uv_offset * right.coefficient) /
                 (left.coefficient + right.coefficient);
      result.sample_data[i].x = uv.x;
      result.sample_data[i].y = uv.y;
      j += 2;
    }
  }

  return result;
}

}  // namespace impeller
