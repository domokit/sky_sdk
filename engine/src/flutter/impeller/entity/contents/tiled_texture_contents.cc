// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/entity/contents/tiled_texture_contents.h"

#include "fml/logging.h"
#include "impeller/entity/contents/content_context.h"
#include "impeller/entity/tiled_texture_fill.frag.h"
#include "impeller/entity/tiled_texture_fill_external.frag.h"
#include "impeller/renderer/render_pass.h"

namespace impeller {

static std::optional<SamplerAddressMode> TileModeToAddressMode(
    Entity::TileMode tile_mode,
    const Capabilities& capabilities) {
  switch (tile_mode) {
    case Entity::TileMode::kClamp:
      return SamplerAddressMode::kClampToEdge;
      break;
    case Entity::TileMode::kMirror:
      return SamplerAddressMode::kMirror;
      break;
    case Entity::TileMode::kRepeat:
      return SamplerAddressMode::kRepeat;
      break;
    case Entity::TileMode::kDecal:
      if (capabilities.SupportsDecalSamplerAddressMode()) {
        return SamplerAddressMode::kDecal;
      }
      return std::nullopt;
  }
}

TiledTextureContents::TiledTextureContents() = default;

TiledTextureContents::~TiledTextureContents() = default;

void TiledTextureContents::SetTexture(std::shared_ptr<Texture> texture) {
  texture_ = std::move(texture);
}

void TiledTextureContents::SetTileModes(Entity::TileMode x_tile_mode,
                                        Entity::TileMode y_tile_mode) {
  x_tile_mode_ = x_tile_mode;
  y_tile_mode_ = y_tile_mode;
}

void TiledTextureContents::SetSamplerDescriptor(const SamplerDescriptor& desc) {
  sampler_descriptor_ = desc;
}

void TiledTextureContents::SetColorFilter(ColorFilterProc color_filter) {
  color_filter_ = std::move(color_filter);
}

std::shared_ptr<Texture> TiledTextureContents::CreateFilterTexture(
    const ContentContext& renderer) const {
  if (!color_filter_) {
    return nullptr;
  }
  auto color_filter_contents = color_filter_(FilterInput::Make(texture_));
  auto snapshot = color_filter_contents->RenderToSnapshot(
      renderer,      // renderer
      Entity(),      // entity
      std::nullopt,  // coverage_limit
      std::nullopt,  // sampler_descriptor
      true,          // msaa_enabled
      /*mip_count=*/1,
      "TiledTextureContents Snapshot");  // label
  if (snapshot.has_value()) {
    return snapshot.value().texture;
  }
  return nullptr;
}

SamplerDescriptor TiledTextureContents::CreateSamplerDescriptor(
    const Capabilities& capabilities) const {
  SamplerDescriptor descriptor = sampler_descriptor_;
  auto width_mode = TileModeToAddressMode(x_tile_mode_, capabilities);
  auto height_mode = TileModeToAddressMode(y_tile_mode_, capabilities);
  if (width_mode.has_value()) {
    descriptor.width_address_mode = width_mode.value();
  }
  if (height_mode.has_value()) {
    descriptor.height_address_mode = height_mode.value();
  }
  return descriptor;
}

bool TiledTextureContents::UsesEmulatedTileMode(
    const Capabilities& capabilities) const {
  return !TileModeToAddressMode(x_tile_mode_, capabilities).has_value() ||
         !TileModeToAddressMode(y_tile_mode_, capabilities).has_value();
}

// |Contents|
bool TiledTextureContents::IsOpaque(const Matrix& transform) const {
  if (GetOpacityFactor() < 1 || x_tile_mode_ == Entity::TileMode::kDecal ||
      y_tile_mode_ == Entity::TileMode::kDecal) {
    return false;
  }
  if (color_filter_) {
    return false;
  }
  return texture_->IsOpaque() && !AppliesAlphaForStrokeCoverage(transform);
}

bool TiledTextureContents::Render(const ContentContext& renderer,
                                  const Entity& entity,
                                  RenderPass& pass) const {
  if (texture_ == nullptr) {
    return true;
  }

  using VS = TextureUvFillVertexShader;
  using FS = TiledTextureFillFragmentShader;

  const auto texture_size = texture_->GetSize();
  if (texture_size.IsEmpty()) {
    return true;
  }

  VS::FrameInfo frame_info;
  frame_info.texture_sampler_y_coord_scale = texture_->GetYCoordScale();
  frame_info.uv_transform =
      Rect::MakeSize(texture_size).GetNormalizingTransform() *
      GetInverseEffectTransform();

#ifdef IMPELLER_ENABLE_OPENGLES
  using FSExternal = TiledTextureFillExternalFragmentShader;
  if (texture_->GetTextureDescriptor().type ==
      TextureType::kTextureExternalOES) {
    return ColorSourceContents::DrawGeometry<VS>(
        renderer, entity, pass,
        [&renderer](ContentContextOptions options) {
          return renderer.GetTiledTextureUvExternalPipeline(options);
        },
        frame_info,
        [this, &renderer](RenderPass& pass) {
          auto& host_buffer = renderer.GetTransientsBuffer();
#ifdef IMPELLER_DEBUG
          pass.SetCommandLabel("TextureFill External");
#endif  // IMPELLER_DEBUG

          FML_DCHECK(!color_filter_);
          FSExternal::FragInfo frag_info;
          frag_info.x_tile_mode =
              static_cast<Scalar>(sampler_descriptor_.width_address_mode);
          frag_info.y_tile_mode =
              static_cast<Scalar>(sampler_descriptor_.height_address_mode);
          frag_info.alpha = GetOpacityFactor();
          FSExternal::BindFragInfo(pass, host_buffer.EmplaceUniform(frag_info));

          SamplerDescriptor sampler_desc;
          // OES_EGL_image_external states that only CLAMP_TO_EDGE is valid,
          // so we emulate all other tile modes here by remapping the texture
          // coordinates.
          sampler_desc.width_address_mode = SamplerAddressMode::kClampToEdge;
          sampler_desc.height_address_mode = SamplerAddressMode::kClampToEdge;
          FSExternal::BindSAMPLEREXTERNALOESTextureSampler(
              pass, texture_,
              renderer.GetContext()->GetSamplerLibrary()->GetSampler(
                  sampler_desc));
          return true;
        });
  }
#endif  // IMPELLER_ENABLE_OPENGLES

  PipelineBuilderCallback pipeline_callback =
      [&renderer](ContentContextOptions options) {
        return renderer.GetTiledTexturePipeline(options);
      };
  return ColorSourceContents::DrawGeometry<VS>(
      renderer, entity, pass, pipeline_callback, frame_info,
      [this, &renderer, &entity](RenderPass& pass) {
        auto& host_buffer = renderer.GetTransientsBuffer();
#ifdef IMPELLER_DEBUG
        pass.SetCommandLabel("TextureFill");
#endif  // IMPELLER_DEBUG

        FS::FragInfo frag_info;
        frag_info.x_tile_mode = static_cast<Scalar>(x_tile_mode_);
        frag_info.y_tile_mode = static_cast<Scalar>(y_tile_mode_);
        frag_info.alpha =
            GetOpacityFactor() *
            GetGeometry()->ComputeAlphaCoverage(entity.GetTransform());
        FS::BindFragInfo(pass, host_buffer.EmplaceUniform(frag_info));

        if (color_filter_) {
          auto filtered_texture = CreateFilterTexture(renderer);
          if (!filtered_texture) {
            return false;
          }
          FS::BindTextureSampler(
              pass, filtered_texture,
              renderer.GetContext()->GetSamplerLibrary()->GetSampler(
                  CreateSamplerDescriptor(renderer.GetDeviceCapabilities())));
        } else {
          FS::BindTextureSampler(
              pass, texture_,
              renderer.GetContext()->GetSamplerLibrary()->GetSampler(
                  CreateSamplerDescriptor(renderer.GetDeviceCapabilities())));
        }

        return true;
      });
}

std::optional<Snapshot> TiledTextureContents::RenderToSnapshot(
    const ContentContext& renderer,
    const Entity& entity,
    std::optional<Rect> coverage_limit,
    const std::optional<SamplerDescriptor>& sampler_descriptor,
    bool msaa_enabled,
    int32_t mip_count,
    std::string_view label) const {
  std::optional<Rect> geometry_coverage = GetGeometry()->GetCoverage({});
  if (GetInverseEffectTransform().IsIdentity() &&
      GetGeometry()->IsAxisAlignedRect() &&
      (!geometry_coverage.has_value() ||
       Rect::MakeSize(texture_->GetSize())
           .Contains(geometry_coverage.value()))) {
    auto coverage = GetCoverage(entity);
    if (!coverage.has_value()) {
      return std::nullopt;
    }
    auto scale = Vector2(coverage->GetSize() / Size(texture_->GetSize()));

    return Snapshot{
        .texture = texture_,
        .transform = Matrix::MakeTranslation(coverage->GetOrigin()) *
                     Matrix::MakeScale(scale),
        .sampler_descriptor = sampler_descriptor.value_or(sampler_descriptor_),
        .opacity = GetOpacityFactor(),
    };
  }

  return Contents::RenderToSnapshot(
      renderer,                                          // renderer
      entity,                                            // entity
      std::nullopt,                                      // coverage_limit
      sampler_descriptor.value_or(sampler_descriptor_),  // sampler_descriptor
      true,                                              // msaa_enabled
      /*mip_count=*/1,
      label);  // label
}

}  // namespace impeller
