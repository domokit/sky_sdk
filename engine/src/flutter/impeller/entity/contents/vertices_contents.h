// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_ENTITY_CONTENTS_VERTICES_CONTENTS_H_
#define FLUTTER_IMPELLER_ENTITY_CONTENTS_VERTICES_CONTENTS_H_

#include <memory>

#include "impeller/core/sampler_descriptor.h"
#include "impeller/entity/contents/contents.h"
#include "impeller/entity/entity.h"
#include "impeller/entity/geometry/vertices_geometry.h"
#include "impeller/geometry/color.h"

namespace impeller {

/// A vertices contents for (optional) per-color vertices + texture and any
/// blend mode.
class VerticesSimpleBlendContents final : public Contents {
 public:
  VerticesSimpleBlendContents();

  ~VerticesSimpleBlendContents() override;

  using LazyTexture =
      std::function<std::shared_ptr<Texture>(const ContentContext& renderer)>;

  void SetGeometry(std::shared_ptr<VerticesGeometry> geometry);

  void SetAlpha(Scalar alpha);

  void SetBlendMode(BlendMode blend_mode);

  void SetTexture(std::shared_ptr<Texture> texture);

  void SetLazyTexture(const LazyTexture& lazy_texture);

  void SetSamplerDescriptor(const SamplerDescriptor& descriptor);

  void SetTileMode(Entity::TileMode tile_mode_x, Entity::TileMode tile_mode_y);

  void SetEffectTransform(Matrix transform);

  void SetLazyTextureCoverage(Rect rect);

  // |Contents|
  std::optional<Rect> GetCoverage(const Entity& entity) const override;

  // |Contents|
  bool Render(const ContentContext& renderer,
              const Entity& entity,
              RenderPass& pass) const override;

 private:
  Scalar alpha_ = 1.0;
  std::shared_ptr<VerticesGeometry> geometry_;
  std::shared_ptr<Texture> texture_;
  BlendMode blend_mode_ = BlendMode::kSource;
  SamplerDescriptor descriptor_ = {};
  Entity::TileMode tile_mode_x_ = Entity::TileMode::kClamp;
  Entity::TileMode tile_mode_y_ = Entity::TileMode::kClamp;
  Matrix inverse_matrix_ = {};
  std::optional<Rect> lazy_texture_coverage_;
  LazyTexture lazy_texture_;

  VerticesSimpleBlendContents(const VerticesSimpleBlendContents&) = delete;

  VerticesSimpleBlendContents& operator=(const VerticesSimpleBlendContents&) =
      delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_ENTITY_CONTENTS_VERTICES_CONTENTS_H_
