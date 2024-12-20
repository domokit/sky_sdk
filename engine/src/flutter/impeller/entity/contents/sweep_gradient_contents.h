// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_ENTITY_CONTENTS_SWEEP_GRADIENT_CONTENTS_H_
#define FLUTTER_IMPELLER_ENTITY_CONTENTS_SWEEP_GRADIENT_CONTENTS_H_

#include <vector>

#include "impeller/entity/contents/color_source_contents.h"
#include "impeller/entity/entity.h"
#include "impeller/geometry/color.h"
#include "impeller/geometry/point.h"
#include "impeller/geometry/scalar.h"

namespace impeller {

class SweepGradientContents final : public ColorSourceContents {
 public:
  SweepGradientContents();

  ~SweepGradientContents() override;

  // |Contents|
  bool IsOpaque(const Matrix& transform) const override;

  // |Contents|
  bool Render(const ContentContext& renderer,
              const Entity& entity,
              RenderPass& pass) const override;

  // |Contents|
  [[nodiscard]] bool ApplyColorFilter(
      const ColorFilterProc& color_filter_proc) override;

  void SetCenterAndAngles(Point center, Degrees start_angle, Degrees end_angle);

  void SetColors(std::vector<Color> colors);

  void SetStops(std::vector<Scalar> stops);

  void SetTileMode(Entity::TileMode tile_mode);

  const std::vector<Color>& GetColors() const;

  const std::vector<Scalar>& GetStops() const;

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
  Scalar bias_;
  Scalar scale_;
  std::vector<Color> colors_;
  std::vector<Scalar> stops_;
  Entity::TileMode tile_mode_;
  Color decal_border_color_ = Color::BlackTransparent();

  SweepGradientContents(const SweepGradientContents&) = delete;

  SweepGradientContents& operator=(const SweepGradientContents&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_ENTITY_CONTENTS_SWEEP_GRADIENT_CONTENTS_H_
