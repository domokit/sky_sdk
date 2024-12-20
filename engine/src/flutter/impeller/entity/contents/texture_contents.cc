// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/entity/contents/texture_contents.h"

#include <memory>
#include <optional>
#include <utility>

#include "impeller/core/formats.h"
#include "impeller/entity/contents/content_context.h"
#include "impeller/entity/entity.h"
#include "impeller/entity/texture_fill.frag.h"
#include "impeller/entity/texture_fill.vert.h"
#include "impeller/entity/texture_fill_strict_src.frag.h"
#include "impeller/entity/tiled_texture_fill_external.frag.h"
#include "impeller/geometry/constants.h"
#include "impeller/renderer/render_pass.h"
#include "impeller/renderer/vertex_buffer_builder.h"

namespace impeller {

TextureContents::TextureContents() = default;

TextureContents::~TextureContents() = default;

std::shared_ptr<TextureContents> TextureContents::MakeRect(Rect destination) {
  auto contents = std::make_shared<TextureContents>();
  contents->destination_rect_ = destination;
  return contents;
}

void TextureContents::SetLabel(std::string_view label) {
  label_ = label;
}

void TextureContents::SetDestinationRect(Rect rect) {
  destination_rect_ = rect;
}

void TextureContents::SetTexture(std::shared_ptr<Texture> texture) {
  texture_ = std::move(texture);
}

std::shared_ptr<Texture> TextureContents::GetTexture() const {
  return texture_;
}

void TextureContents::SetOpacity(Scalar opacity) {
  opacity_ = opacity;
}

void TextureContents::SetStencilEnabled(bool enabled) {
  stencil_enabled_ = enabled;
}

void TextureContents::SetInheritedOpacity(Scalar opacity) {
  inherited_opacity_ = opacity;
}

Scalar TextureContents::GetOpacity() const {
  return opacity_ * inherited_opacity_;
}

std::optional<Rect> TextureContents::GetCoverage(const Entity& entity) const {
  if (GetOpacity() == 0) {
    return std::nullopt;
  }
  return destination_rect_.TransformBounds(entity.GetTransform());
};

std::optional<Snapshot> TextureContents::RenderToSnapshot(
    const ContentContext& renderer,
    const Entity& entity,
    std::optional<Rect> coverage_limit,
    const std::optional<SamplerDescriptor>& sampler_descriptor,
    bool msaa_enabled,
    int32_t mip_count,
    std::string_view label) const {
  // Passthrough textures that have simple rectangle paths and complete source
  // rects.
  auto bounds = destination_rect_;
  auto opacity = GetOpacity();
  if (source_rect_ == Rect::MakeSize(texture_->GetSize()) &&
      (opacity >= 1 - kEhCloseEnough || defer_applying_opacity_)) {
    auto scale = Vector2(bounds.GetSize() / Size(texture_->GetSize()));
    return Snapshot{
        .texture = texture_,
        .transform = entity.GetTransform() *
                     Matrix::MakeTranslation(bounds.GetOrigin()) *
                     Matrix::MakeScale(scale),
        .sampler_descriptor = sampler_descriptor.value_or(sampler_descriptor_),
        .opacity = opacity};
  }
  return Contents::RenderToSnapshot(
      renderer,                                          // renderer
      entity,                                            // entity
      std::nullopt,                                      // coverage_limit
      sampler_descriptor.value_or(sampler_descriptor_),  // sampler_descriptor
      true,                                              // msaa_enabled
      /*mip_count=*/mip_count,
      label);  // label
}

bool TextureContents::Render(const ContentContext& renderer,
                             const Entity& entity,
                             RenderPass& pass) const {
  using VS = TextureFillVertexShader;
  using FS = TextureFillFragmentShader;
  using FSStrict = TextureFillStrictSrcFragmentShader;

  if (destination_rect_.IsEmpty() || source_rect_.IsEmpty() ||
      texture_ == nullptr || texture_->GetSize().IsEmpty()) {
    return true;  // Nothing to render.
  }

#ifdef IMPELLER_ENABLE_OPENGLES
  using FSExternal = TiledTextureFillExternalFragmentShader;
  bool is_external_texture =
      texture_->GetTextureDescriptor().type == TextureType::kTextureExternalOES;
#endif  // IMPELLER_ENABLE_OPENGLES

  auto texture_coords =
      Rect::MakeSize(texture_->GetSize()).Project(source_rect_);
  auto& host_buffer = renderer.GetTransientsBuffer();

  std::array<VS::PerVertexData, 4> vertices = {
      VS::PerVertexData{destination_rect_.GetLeftTop(),
                        texture_coords.GetLeftTop()},
      VS::PerVertexData{destination_rect_.GetRightTop(),
                        texture_coords.GetRightTop()},
      VS::PerVertexData{destination_rect_.GetLeftBottom(),
                        texture_coords.GetLeftBottom()},
      VS::PerVertexData{destination_rect_.GetRightBottom(),
                        texture_coords.GetRightBottom()},
  };
  auto vertex_buffer = CreateVertexBuffer(vertices, host_buffer);

  VS::FrameInfo frame_info;
  frame_info.mvp = entity.GetShaderTransform(pass);
  frame_info.texture_sampler_y_coord_scale = texture_->GetYCoordScale();

#ifdef IMPELLER_DEBUG
  if (label_.empty()) {
    pass.SetCommandLabel("Texture Fill");
  } else {
    pass.SetCommandLabel("Texture Fill: " + label_);
  }
#endif  // IMPELLER_DEBUG

  auto pipeline_options = OptionsFromPassAndEntity(pass, entity);
  if (!stencil_enabled_) {
    pipeline_options.stencil_mode = ContentContextOptions::StencilMode::kIgnore;
  }
  pipeline_options.primitive_type = PrimitiveType::kTriangleStrip;

  pipeline_options.depth_write_enabled =
      stencil_enabled_ && pipeline_options.blend_mode == BlendMode::kSource;

#ifdef IMPELLER_ENABLE_OPENGLES
  if (is_external_texture) {
    pass.SetPipeline(
        renderer.GetTiledTextureExternalPipeline(pipeline_options));
  } else {
    pass.SetPipeline(
        strict_source_rect_enabled_
            ? renderer.GetTextureStrictSrcPipeline(pipeline_options)
            : renderer.GetTexturePipeline(pipeline_options));
  }
#else
  pass.SetPipeline(strict_source_rect_enabled_
                       ? renderer.GetTextureStrictSrcPipeline(pipeline_options)
                       : renderer.GetTexturePipeline(pipeline_options));
#endif  // IMPELLER_ENABLE_OPENGLES

  pass.SetVertexBuffer(vertex_buffer);
  VS::BindFrameInfo(pass, host_buffer.EmplaceUniform(frame_info));

  if (strict_source_rect_enabled_) {
    // For a strict source rect, shrink the texture coordinate range by half a
    // texel to ensure that linear filtering does not sample anything outside
    // the source rect bounds.
    auto strict_texture_coords =
        Rect::MakeSize(texture_->GetSize()).Project(source_rect_.Expand(-0.5));

    FSStrict::FragInfo frag_info;
    frag_info.source_rect = Vector4(strict_texture_coords.GetLTRB());
    frag_info.alpha = GetOpacity();
    FSStrict::BindFragInfo(pass, host_buffer.EmplaceUniform((frag_info)));
    FSStrict::BindTextureSampler(
        pass, texture_,
        renderer.GetContext()->GetSamplerLibrary()->GetSampler(
            sampler_descriptor_));
#ifdef IMPELLER_ENABLE_OPENGLES
  } else if (is_external_texture) {
    FSExternal::FragInfo frag_info;
    frag_info.x_tile_mode =
        static_cast<Scalar>(sampler_descriptor_.width_address_mode);
    frag_info.y_tile_mode =
        static_cast<Scalar>(sampler_descriptor_.height_address_mode);
    frag_info.alpha = GetOpacity();
    FSExternal::BindFragInfo(pass, host_buffer.EmplaceUniform(frag_info));

    SamplerDescriptor sampler_desc;
    // OES_EGL_image_external states that only CLAMP_TO_EDGE is valid, so
    // we emulate all other tile modes here by remapping the texture
    // coordinates.
    sampler_desc.width_address_mode = SamplerAddressMode::kClampToEdge;
    sampler_desc.height_address_mode = SamplerAddressMode::kClampToEdge;
    FSExternal::BindSAMPLEREXTERNALOESTextureSampler(
        pass, texture_,
        renderer.GetContext()->GetSamplerLibrary()->GetSampler(sampler_desc));
#endif  //  IMPELLER_ENABLE_OPENGLES
  } else {
    FS::FragInfo frag_info;
    frag_info.alpha = GetOpacity();
    FS::BindFragInfo(pass, host_buffer.EmplaceUniform((frag_info)));
    FS::BindTextureSampler(
        pass, texture_,
        renderer.GetContext()->GetSamplerLibrary()->GetSampler(
            sampler_descriptor_));
  }
  return pass.Draw().ok();
}

void TextureContents::SetSourceRect(const Rect& source_rect) {
  source_rect_ = source_rect;
}

const Rect& TextureContents::GetSourceRect() const {
  return source_rect_;
}

void TextureContents::SetStrictSourceRect(bool strict) {
  strict_source_rect_enabled_ = strict;
}

bool TextureContents::GetStrictSourceRect() const {
  return strict_source_rect_enabled_;
}

void TextureContents::SetSamplerDescriptor(const SamplerDescriptor& desc) {
  sampler_descriptor_ = desc;
}

const SamplerDescriptor& TextureContents::GetSamplerDescriptor() const {
  return sampler_descriptor_;
}

void TextureContents::SetDeferApplyingOpacity(bool defer_applying_opacity) {
  defer_applying_opacity_ = defer_applying_opacity;
}

}  // namespace impeller
