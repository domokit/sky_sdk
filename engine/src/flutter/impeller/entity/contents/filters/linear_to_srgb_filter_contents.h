// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_ENTITY_CONTENTS_FILTERS_LINEAR_TO_SRGB_FILTER_CONTENTS_H_
#define FLUTTER_IMPELLER_ENTITY_CONTENTS_FILTERS_LINEAR_TO_SRGB_FILTER_CONTENTS_H_

#include "impeller/entity/contents/filters/color_filter_contents.h"
#include "impeller/entity/contents/filters/inputs/filter_input.h"

namespace impeller {

class LinearToSrgbFilterContents final : public ColorFilterContents {
 public:
  LinearToSrgbFilterContents();

  ~LinearToSrgbFilterContents() override;

 private:
  // |FilterContents|
  std::optional<Entity> RenderFilter(
      const FilterInput::Vector& input_textures,
      const ContentContext& renderer,
      const Entity& entity,
      const Matrix& effect_transform,
      const Rect& coverage,
      const std::optional<Rect>& coverage_hint) const override;

  LinearToSrgbFilterContents(const LinearToSrgbFilterContents&) = delete;

  LinearToSrgbFilterContents& operator=(const LinearToSrgbFilterContents&) =
      delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_ENTITY_CONTENTS_FILTERS_LINEAR_TO_SRGB_FILTER_CONTENTS_H_
