// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_ENTITY_CONTENTS_FILTERS_MATRIX_FILTER_CONTENTS_H_
#define FLUTTER_IMPELLER_ENTITY_CONTENTS_FILTERS_MATRIX_FILTER_CONTENTS_H_

#include "impeller/entity/contents/filters/filter_contents.h"
#include "impeller/entity/contents/filters/inputs/filter_input.h"

namespace impeller {

class MatrixFilterContents final : public FilterContents {
 public:
  MatrixFilterContents();

  ~MatrixFilterContents() override;

  void SetMatrix(Matrix matrix);

  // |FilterContents|
  void SetRenderingMode(Entity::RenderingMode rendering_mode) override;

  void SetSamplerDescriptor(const SamplerDescriptor& desc);

  // |FilterContents|
  std::optional<Rect> GetFilterCoverage(
      const FilterInput::Vector& inputs,
      const Entity& entity,
      const Matrix& effect_transform) const override;

 private:
  // |FilterContents|
  std::optional<Entity> RenderFilter(
      const FilterInput::Vector& input_textures,
      const ContentContext& renderer,
      const Entity& entity,
      const Matrix& effect_transform,
      const Rect& coverage,
      const std::optional<Rect>& coverage_hint) const override;

  // |FilterContents|
  std::optional<Rect> GetFilterSourceCoverage(
      const Matrix& effect_transform,
      const Rect& output_limit) const override;

  Matrix matrix_;
  SamplerDescriptor sampler_descriptor_ = {};
  Entity::RenderingMode rendering_mode_ = Entity::RenderingMode::kDirect;

  MatrixFilterContents(const MatrixFilterContents&) = delete;

  MatrixFilterContents& operator=(const MatrixFilterContents&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_ENTITY_CONTENTS_FILTERS_MATRIX_FILTER_CONTENTS_H_
