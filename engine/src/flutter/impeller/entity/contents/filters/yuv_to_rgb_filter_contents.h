// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_ENTITY_CONTENTS_FILTERS_YUV_TO_RGB_FILTER_CONTENTS_H_
#define FLUTTER_IMPELLER_ENTITY_CONTENTS_FILTERS_YUV_TO_RGB_FILTER_CONTENTS_H_

#include "impeller/entity/contents/filters/filter_contents.h"

namespace impeller {

class YUVToRGBFilterContents final : public FilterContents {
 public:
  YUVToRGBFilterContents();

  ~YUVToRGBFilterContents() override;

  void SetYUVColorSpace(YUVColorSpace yuv_color_space);

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

  YUVColorSpace yuv_color_space_ = YUVColorSpace::kBT601LimitedRange;

  YUVToRGBFilterContents(const YUVToRGBFilterContents&) = delete;

  YUVToRGBFilterContents& operator=(const YUVToRGBFilterContents&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_ENTITY_CONTENTS_FILTERS_YUV_TO_RGB_FILTER_CONTENTS_H_
