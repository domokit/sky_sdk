// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_ENTITY_CONTENTS_FILTERS_COLOR_FILTER_CONTENTS_H_
#define FLUTTER_IMPELLER_ENTITY_CONTENTS_FILTERS_COLOR_FILTER_CONTENTS_H_

#include "impeller/entity/contents/filters/filter_contents.h"

namespace impeller {

class ColorFilterContents : public FilterContents {
 public:
  enum class AbsorbOpacity {
    kYes,
    kNo,
  };

  /// @brief the [inputs] are expected to be in the order of dst, src.
  static std::shared_ptr<ColorFilterContents> MakeBlend(
      BlendMode blend_mode,
      FilterInput::Vector inputs,
      std::optional<Color> foreground_color = std::nullopt);

  static std::shared_ptr<ColorFilterContents> MakeColorMatrix(
      FilterInput::Ref input,
      const ColorMatrix& color_matrix);

  static std::shared_ptr<ColorFilterContents> MakeLinearToSrgbFilter(
      FilterInput::Ref input);

  static std::shared_ptr<ColorFilterContents> MakeSrgbToLinearFilter(
      FilterInput::Ref input);

  ColorFilterContents();

  ~ColorFilterContents() override;

  void SetAbsorbOpacity(AbsorbOpacity absorb_opacity);

  AbsorbOpacity GetAbsorbOpacity() const;

  /// @brief Sets an alpha that is applied to the final blended result.
  void SetAlpha(Scalar alpha);

  std::optional<Scalar> GetAlpha() const;

  // |FilterContents|
  std::optional<Rect> GetFilterSourceCoverage(
      const Matrix& effect_transform,
      const Rect& output_limit) const override;

 private:
  AbsorbOpacity absorb_opacity_ = AbsorbOpacity::kNo;
  std::optional<Scalar> alpha_;

  ColorFilterContents(const ColorFilterContents&) = delete;

  ColorFilterContents& operator=(const ColorFilterContents&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_ENTITY_CONTENTS_FILTERS_COLOR_FILTER_CONTENTS_H_
