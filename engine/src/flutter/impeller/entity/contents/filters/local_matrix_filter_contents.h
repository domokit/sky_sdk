// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_ENTITY_CONTENTS_FILTERS_LOCAL_MATRIX_FILTER_CONTENTS_H_
#define FLUTTER_IMPELLER_ENTITY_CONTENTS_FILTERS_LOCAL_MATRIX_FILTER_CONTENTS_H_

#include "impeller/entity/contents/filters/filter_contents.h"
#include "impeller/entity/contents/filters/inputs/filter_input.h"

namespace impeller {

class LocalMatrixFilterContents final : public FilterContents {
 public:
  LocalMatrixFilterContents();

  ~LocalMatrixFilterContents() override;

  void SetMatrix(Matrix matrix);

  // |FilterContents|
  Matrix GetLocalTransform(const Matrix& parent_transform) const override;

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

  Matrix matrix_;

  LocalMatrixFilterContents(const LocalMatrixFilterContents&) = delete;

  LocalMatrixFilterContents& operator=(const LocalMatrixFilterContents&) =
      delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_ENTITY_CONTENTS_FILTERS_LOCAL_MATRIX_FILTER_CONTENTS_H_
