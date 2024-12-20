// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_ENTITY_CONTENTS_FILTERS_MORPHOLOGY_FILTER_CONTENTS_H_
#define FLUTTER_IMPELLER_ENTITY_CONTENTS_FILTERS_MORPHOLOGY_FILTER_CONTENTS_H_

#include <memory>
#include <optional>
#include "impeller/entity/contents/filters/filter_contents.h"
#include "impeller/entity/contents/filters/inputs/filter_input.h"

namespace impeller {

class DirectionalMorphologyFilterContents final : public FilterContents {
 public:
  DirectionalMorphologyFilterContents();

  ~DirectionalMorphologyFilterContents() override;

  void SetRadius(Radius radius);

  void SetDirection(Vector2 direction);

  void SetMorphType(MorphType morph_type);

  // |FilterContents|
  std::optional<Rect> GetFilterCoverage(
      const FilterInput::Vector& inputs,
      const Entity& entity,
      const Matrix& effect_transform) const override;

  // |FilterContents|
  std::optional<Rect> GetFilterSourceCoverage(
      const Matrix& effect_transform,
      const Rect& output_limit) const override;

 private:
  // |FilterContents|
  std::optional<Entity> RenderFilter(
      const FilterInput::Vector& input_textures,
      const ContentContext& renderer,
      const Entity& entity,
      const Matrix& effect_transform,
      const Rect& coverage,
      const std::optional<Rect>& coverage_hint) const override;

  Radius radius_;
  Vector2 direction_;
  MorphType morph_type_;

  DirectionalMorphologyFilterContents(
      const DirectionalMorphologyFilterContents&) = delete;

  DirectionalMorphologyFilterContents& operator=(
      const DirectionalMorphologyFilterContents&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_ENTITY_CONTENTS_FILTERS_MORPHOLOGY_FILTER_CONTENTS_H_
