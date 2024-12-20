// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_ENTITY_CONTENTS_FILTERS_COLOR_MATRIX_FILTER_CONTENTS_H_
#define FLUTTER_IMPELLER_ENTITY_CONTENTS_FILTERS_COLOR_MATRIX_FILTER_CONTENTS_H_

#include <memory>
#include <optional>

#include "impeller/entity/contents/filters/color_filter_contents.h"
#include "impeller/entity/contents/filters/inputs/filter_input.h"

namespace impeller {

// Look at example at: https://github.com/flutter/impeller/pull/132

class ColorMatrixFilterContents final : public ColorFilterContents {
 public:
  ColorMatrixFilterContents();

  ~ColorMatrixFilterContents() override;

  void SetMatrix(const ColorMatrix& matrix);

 private:
  // |FilterContents|
  std::optional<Entity> RenderFilter(
      const FilterInput::Vector& input_textures,
      const ContentContext& renderer,
      const Entity& entity,
      const Matrix& effect_transform,
      const Rect& coverage,
      const std::optional<Rect>& coverage_hint) const override;

  ColorMatrix matrix_;

  ColorMatrixFilterContents(const ColorMatrixFilterContents&) = delete;

  ColorMatrixFilterContents& operator=(const ColorMatrixFilterContents&) =
      delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_ENTITY_CONTENTS_FILTERS_COLOR_MATRIX_FILTER_CONTENTS_H_
