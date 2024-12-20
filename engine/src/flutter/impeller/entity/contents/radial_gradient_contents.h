// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_ENTITY_CONTENTS_RADIAL_GRADIENT_CONTENTS_H_
#define FLUTTER_IMPELLER_ENTITY_CONTENTS_RADIAL_GRADIENT_CONTENTS_H_

#include <vector>

#include "impeller/entity/contents/color_source_contents.h"
#include "impeller/entity/entity.h"
#include "impeller/geometry/color.h"
#include "impeller/geometry/point.h"

namespace impeller {

class RadialGradientContents final : public ColorSourceContents {
 public:
  RadialGradientContents();

  ~RadialGradientContents() override;

  // |Contents|
  bool IsOpaque(const Matrix& transform) const override;

  // |Contents|
  bool Render(const ContentContext& renderer,
              const Entity& entity,
              RenderPass& pass) const override;

  // |Contents|
  [[nodiscard]] bool ApplyColorFilter(
      const ColorFilterProc& color_filter_proc) override;

  void SetCenterAndRadius(Point center, Scalar radius);

  void SetColors(std::vector<Color> colors);

  void SetStops(std::vector<Scalar> stops);

  const std::vector<Color>& GetColors() const;

  const std::vector<Scalar>& GetStops() const;

  void SetTileMode(Entity::TileMode tile_mode);

 private:
  bool RenderTexture(const ContentContext& renderer,
                     const Entity& entity,
                     RenderPass& pass) const;

  bool RenderSSBO(const ContentContext& renderer,
                  const Entity& entity,
                  RenderPass& pass) const;

  bool RenderUniform(const ContentContext& renderer,
                     const Entity& entity,
                     RenderPass& pass) const;

  Point center_;
  Scalar radius_;
  std::vector<Color> colors_;
  std::vector<Scalar> stops_;
  Entity::TileMode tile_mode_;
  Color decal_border_color_ = Color::BlackTransparent();

  RadialGradientContents(const RadialGradientContents&) = delete;

  RadialGradientContents& operator=(const RadialGradientContents&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_ENTITY_CONTENTS_RADIAL_GRADIENT_CONTENTS_H_
