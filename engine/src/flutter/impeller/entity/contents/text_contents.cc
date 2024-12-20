// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/entity/contents/text_contents.h"

#include <cstring>
#include <optional>
#include <utility>

#include "impeller/core/buffer_view.h"
#include "impeller/core/formats.h"
#include "impeller/core/sampler_descriptor.h"
#include "impeller/entity/contents/content_context.h"
#include "impeller/entity/entity.h"
#include "impeller/geometry/color.h"
#include "impeller/geometry/point.h"
#include "impeller/renderer/render_pass.h"
#include "impeller/typographer/glyph_atlas.h"

namespace impeller {

TextContents::TextContents() = default;

TextContents::~TextContents() = default;

void TextContents::SetTextFrame(const std::shared_ptr<TextFrame>& frame) {
  frame_ = frame;
}

void TextContents::SetColor(Color color) {
  color_ = color;
}

Color TextContents::GetColor() const {
  return color_.WithAlpha(color_.alpha * inherited_opacity_);
}

void TextContents::SetInheritedOpacity(Scalar opacity) {
  inherited_opacity_ = opacity;
}

void TextContents::SetOffset(Vector2 offset) {
  offset_ = offset;
}

void TextContents::SetForceTextColor(bool value) {
  force_text_color_ = value;
}

std::optional<Rect> TextContents::GetCoverage(const Entity& entity) const {
  return frame_->GetBounds().TransformBounds(entity.GetTransform());
}

void TextContents::SetTextProperties(Color color,
                                     bool stroke,
                                     Scalar stroke_width,
                                     Cap stroke_cap,
                                     Join stroke_join,
                                     Scalar stroke_miter) {
  if (frame_->HasColor()) {
    // Alpha is always applied when rendering, remove it here so
    // we do not double-apply the alpha.
    properties_.color = color.WithAlpha(1.0);
  }
  if (stroke) {
    properties_.stroke = true;
    properties_.stroke_width = stroke_width;
    properties_.stroke_cap = stroke_cap;
    properties_.stroke_join = stroke_join;
    properties_.stroke_miter = stroke_miter;
  }
}

bool TextContents::Render(const ContentContext& renderer,
                          const Entity& entity,
                          RenderPass& pass) const {
  auto color = GetColor();
  if (color.IsTransparent()) {
    return true;
  }

  auto type = frame_->GetAtlasType();
  const std::shared_ptr<GlyphAtlas>& atlas =
      renderer.GetLazyGlyphAtlas()->CreateOrGetGlyphAtlas(
          *renderer.GetContext(), renderer.GetTransientsBuffer(), type);

  if (!atlas || !atlas->IsValid()) {
    VALIDATION_LOG << "Cannot render glyphs without prepared atlas.";
    return false;
  }
  if (!frame_->IsFrameComplete()) {
    VALIDATION_LOG << "Failed to find font glyph bounds.";
    return false;
  }

  // Information shared by all glyph draw calls.
  pass.SetCommandLabel("TextFrame");
  auto opts = OptionsFromPassAndEntity(pass, entity);
  opts.primitive_type = PrimitiveType::kTriangle;
  pass.SetPipeline(renderer.GetGlyphAtlasPipeline(opts));

  using VS = GlyphAtlasPipeline::VertexShader;
  using FS = GlyphAtlasPipeline::FragmentShader;

  // Common vertex uniforms for all glyphs.
  VS::FrameInfo frame_info;
  frame_info.mvp =
      Entity::GetShaderTransform(entity.GetShaderClipDepth(), pass, Matrix());
  ISize atlas_size = atlas->GetTexture()->GetSize();
  bool is_translation_scale = entity.GetTransform().IsTranslationScaleOnly();
  Matrix entity_transform = entity.GetTransform();
  Matrix basis_transform = entity_transform.Basis();

  VS::BindFrameInfo(pass,
                    renderer.GetTransientsBuffer().EmplaceUniform(frame_info));

  FS::FragInfo frag_info;
  frag_info.use_text_color = force_text_color_ ? 1.0 : 0.0;
  frag_info.text_color = ToVector(color.Premultiply());
  frag_info.is_color_glyph = type == GlyphAtlas::Type::kColorBitmap;

  FS::BindFragInfo(pass,
                   renderer.GetTransientsBuffer().EmplaceUniform(frag_info));

  SamplerDescriptor sampler_desc;
  if (is_translation_scale) {
    sampler_desc.min_filter = MinMagFilter::kNearest;
    sampler_desc.mag_filter = MinMagFilter::kNearest;
  } else {
    // Currently, we only propagate the scale of the transform to the atlas
    // renderer, so if the transform has more than just a translation, we turn
    // on linear sampling to prevent crunchiness caused by the pixel grid not
    // being perfectly aligned.
    // The downside is that this slightly over-blurs rotated/skewed text.
    sampler_desc.min_filter = MinMagFilter::kLinear;
    sampler_desc.mag_filter = MinMagFilter::kLinear;
  }

  // No mipmaps for glyph atlas (glyphs are generated at exact scales).
  sampler_desc.mip_filter = MipFilter::kBase;

  FS::BindGlyphAtlasSampler(
      pass,                 // command
      atlas->GetTexture(),  // texture
      renderer.GetContext()->GetSamplerLibrary()->GetSampler(
          sampler_desc)  // sampler
  );

  // Common vertex information for all glyphs.
  // All glyphs are given the same vertex information in the form of a
  // unit-sized quad. The size of the glyph is specified in per instance data
  // and the vertex shader uses this to size the glyph correctly. The
  // interpolated vertex information is also used in the fragment shader to
  // sample from the glyph atlas.

  constexpr std::array<Point, 6> unit_points = {Point{0, 0}, Point{1, 0},
                                                Point{0, 1}, Point{1, 0},
                                                Point{0, 1}, Point{1, 1}};

  auto& host_buffer = renderer.GetTransientsBuffer();
  size_t vertex_count = 0;
  for (const auto& run : frame_->GetRuns()) {
    vertex_count += run.GetGlyphPositions().size();
  }
  vertex_count *= 6;

  BufferView buffer_view = host_buffer.Emplace(
      vertex_count * sizeof(VS::PerVertexData), alignof(VS::PerVertexData),
      [&](uint8_t* contents) {
        VS::PerVertexData vtx;
        VS::PerVertexData* vtx_contents =
            reinterpret_cast<VS::PerVertexData*>(contents);
        size_t i = 0u;
        size_t bounds_offset = 0u;
        for (const TextRun& run : frame_->GetRuns()) {
          const Font& font = run.GetFont();
          Scalar rounded_scale = TextFrame::RoundScaledFontSize(scale_);
          FontGlyphAtlas* font_atlas = nullptr;

          // Adjust glyph position based on the subpixel rounding
          // used by the font.
          Point subpixel_adjustment(0.5, 0.5);
          switch (font.GetAxisAlignment()) {
            case AxisAlignment::kNone:
              break;
            case AxisAlignment::kX:
              subpixel_adjustment.x = 0.125;
              break;
            case AxisAlignment::kY:
              subpixel_adjustment.y = 0.125;
              break;
            case AxisAlignment::kAll:
              subpixel_adjustment.x = 0.125;
              subpixel_adjustment.y = 0.125;
              break;
          }

          Point screen_offset = (entity_transform * Point(0, 0));
          for (const TextRun::GlyphPosition& glyph_position :
               run.GetGlyphPositions()) {
            const FrameBounds& frame_bounds =
                frame_->GetFrameBounds(bounds_offset);
            bounds_offset++;
            auto atlas_glyph_bounds = frame_bounds.atlas_bounds;
            auto glyph_bounds = frame_bounds.glyph_bounds;

            // If frame_bounds.is_placeholder is true, this is the first frame
            // the glyph has been rendered and so its atlas position was not
            // known when the glyph was recorded. Perform a slow lookup into the
            // glyph atlas hash table.
            if (frame_bounds.is_placeholder) {
              if (!font_atlas) {
                font_atlas = atlas->GetOrCreateFontGlyphAtlas(
                    ScaledFont{font, rounded_scale});
              }

              if (!font_atlas) {
                VALIDATION_LOG << "Could not find font in the atlas.";
                continue;
              }
              Point subpixel = TextFrame::ComputeSubpixelPosition(
                  glyph_position, font.GetAxisAlignment(), offset_,
                  rounded_scale);

              std::optional<FrameBounds> maybe_atlas_glyph_bounds =
                  font_atlas->FindGlyphBounds(SubpixelGlyph{
                      glyph_position.glyph,  //
                      subpixel,              //
                      GetGlyphProperties()   //
                  });
              if (!maybe_atlas_glyph_bounds.has_value()) {
                VALIDATION_LOG << "Could not find glyph position in the atlas.";
                continue;
              }
              atlas_glyph_bounds =
                  maybe_atlas_glyph_bounds.value().atlas_bounds;
            }

            Rect scaled_bounds = glyph_bounds.Scale(1.0 / rounded_scale);
            // For each glyph, we compute two rectangles. One for the vertex
            // positions and one for the texture coordinates (UVs). The atlas
            // glyph bounds are used to compute UVs in cases where the
            // destination and source sizes may differ due to clamping the sizes
            // of large glyphs.
            Point uv_origin =
                (atlas_glyph_bounds.GetLeftTop() - Point(0.5, 0.5)) /
                atlas_size;
            Point uv_size =
                (atlas_glyph_bounds.GetSize() + Point(1, 1)) / atlas_size;

            Point unrounded_glyph_position =
                basis_transform *
                (glyph_position.position + scaled_bounds.GetLeftTop());

            Point screen_glyph_position =
                (screen_offset + unrounded_glyph_position + subpixel_adjustment)
                    .Floor();

            for (const Point& point : unit_points) {
              Point position;
              if (is_translation_scale) {
                position = (screen_glyph_position +
                            (basis_transform * point * scaled_bounds.GetSize()))
                               .Round();
              } else {
                position = entity_transform * (glyph_position.position +
                                               scaled_bounds.GetLeftTop() +
                                               point * scaled_bounds.GetSize());
              }
              vtx.uv = uv_origin + (uv_size * point);
              vtx.position = position;
              vtx_contents[i++] = vtx;
            }
          }
        }
      });

  pass.SetVertexBuffer(std::move(buffer_view));
  pass.SetIndexBuffer({}, IndexType::kNone);
  pass.SetElementCount(vertex_count);

  return pass.Draw().ok();
}

std::optional<GlyphProperties> TextContents::GetGlyphProperties() const {
  return (properties_.stroke || frame_->HasColor())
             ? std::optional<GlyphProperties>(properties_)
             : std::nullopt;
}

}  // namespace impeller
