// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/entity/entity.h"

#include <algorithm>
#include <optional>

#include "impeller/entity/contents/content_context.h"
#include "impeller/entity/contents/texture_contents.h"
#include "impeller/geometry/color.h"
#include "impeller/geometry/vector.h"
#include "impeller/renderer/render_pass.h"

namespace impeller {

Entity Entity::FromSnapshot(const Snapshot& snapshot, BlendMode blend_mode) {
  auto texture_rect = Rect::MakeSize(snapshot.texture->GetSize());

  auto contents = TextureContents::MakeRect(texture_rect);
  contents->SetTexture(snapshot.texture);
  contents->SetSamplerDescriptor(snapshot.sampler_descriptor);
  contents->SetSourceRect(texture_rect);
  contents->SetOpacity(snapshot.opacity);

  Entity entity;
  entity.SetBlendMode(blend_mode);
  entity.SetTransform(snapshot.transform);
  entity.SetContents(contents);
  return entity;
}

Entity::Entity() = default;

Entity::~Entity() = default;

Entity::Entity(Entity&&) = default;

Entity::Entity(const Entity&) = default;

Entity& Entity::operator=(Entity&&) = default;

const Matrix& Entity::GetTransform() const {
  return transform_;
}

Matrix Entity::GetShaderTransform(const RenderPass& pass) const {
  return Entity::GetShaderTransform(GetShaderClipDepth(), pass, transform_);
}

Matrix Entity::GetShaderTransform(Scalar shader_clip_depth,
                                  const RenderPass& pass,
                                  const Matrix& transform) {
  return Matrix::MakeTranslateScale({1, 1, Entity::kDepthEpsilon},
                                    {0, 0, shader_clip_depth}) *
         pass.GetOrthographicTransform() * transform;
}

void Entity::SetTransform(const Matrix& transform) {
  transform_ = transform;
}

std::optional<Rect> Entity::GetCoverage() const {
  if (!contents_) {
    return std::nullopt;
  }

  return contents_->GetCoverage(*this);
}

void Entity::SetContents(std::shared_ptr<Contents> contents) {
  contents_ = std::move(contents);
}

const std::shared_ptr<Contents>& Entity::GetContents() const {
  return contents_;
}

void Entity::SetClipDepth(uint32_t clip_depth) {
  clip_depth_ = clip_depth;
}

uint32_t Entity::GetClipDepth() const {
  return clip_depth_;
}

Scalar Entity::GetShaderClipDepth() const {
  return Entity::GetShaderClipDepth(clip_depth_);
}

Scalar Entity::GetShaderClipDepth(uint32_t clip_depth) {
  Scalar result = std::clamp(clip_depth * kDepthEpsilon, 0.0f, 1.0f);
  return std::min(result, 1.0f - kDepthEpsilon);
}

void Entity::SetBlendMode(BlendMode blend_mode) {
  blend_mode_ = blend_mode;
}

BlendMode Entity::GetBlendMode() const {
  return blend_mode_;
}

bool Entity::SetInheritedOpacity(Scalar alpha) {
  if (alpha >= 1.0) {
    return true;
  }
  if (blend_mode_ == BlendMode::kSource &&
      contents_->IsOpaque(GetTransform())) {
    blend_mode_ = BlendMode::kSourceOver;
  }
  contents_->SetInheritedOpacity(alpha);
  return true;
}

std::optional<Color> Entity::AsBackgroundColor(ISize target_size) const {
  return contents_->AsBackgroundColor(*this, target_size);
}

/// @brief  Returns true if the blend mode is "destructive", meaning that even
///         fully transparent source colors would result in the destination
///         getting changed.
///
///         This is useful for determining if EntityPass textures can be
///         shrinkwrapped to their Entities' coverage; they can be shrinkwrapped
///         if all of the contained Entities have non-destructive blends.
bool Entity::IsBlendModeDestructive(BlendMode blend_mode) {
  switch (blend_mode) {
    case BlendMode::kClear:
    case BlendMode::kSource:
    case BlendMode::kSourceIn:
    case BlendMode::kDestinationIn:
    case BlendMode::kSourceOut:
    case BlendMode::kDestinationOut:
    case BlendMode::kDestinationATop:
    case BlendMode::kXor:
    case BlendMode::kModulate:
      return true;
    default:
      return false;
  }
}

bool Entity::Render(const ContentContext& renderer,
                    RenderPass& parent_pass) const {
  if (!contents_) {
    return true;
  }

  if (!contents_->GetCoverageHint().has_value()) {
    contents_->SetCoverageHint(
        Rect::MakeSize(parent_pass.GetRenderTargetSize()));
  }

  return contents_->Render(renderer, *this, parent_pass);
}

Entity Entity::Clone() const {
  return Entity(*this);
}

}  // namespace impeller
