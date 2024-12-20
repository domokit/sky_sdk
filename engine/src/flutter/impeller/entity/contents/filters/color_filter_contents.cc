// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/entity/contents/filters/color_filter_contents.h"

#include <utility>

#include "impeller/base/validation.h"
#include "impeller/entity/contents/filters/blend_filter_contents.h"
#include "impeller/entity/contents/filters/color_matrix_filter_contents.h"
#include "impeller/entity/contents/filters/linear_to_srgb_filter_contents.h"
#include "impeller/entity/contents/filters/srgb_to_linear_filter_contents.h"

namespace impeller {

std::shared_ptr<ColorFilterContents> ColorFilterContents::MakeBlend(
    BlendMode blend_mode,
    FilterInput::Vector inputs,
    std::optional<Color> foreground_color) {
  if (blend_mode > Entity::kLastAdvancedBlendMode) {
    VALIDATION_LOG << "Invalid blend mode " << static_cast<int>(blend_mode)
                   << " passed to ColorFilterContents::MakeBlend.";
    return nullptr;
  }

  size_t total_inputs = inputs.size() + (foreground_color.has_value() ? 1 : 0);
  if (total_inputs < 2 || blend_mode <= Entity::kLastPipelineBlendMode) {
    auto blend = std::make_shared<BlendFilterContents>();
    blend->SetInputs(inputs);
    blend->SetBlendMode(blend_mode);
    blend->SetForegroundColor(foreground_color);
    return blend;
  }

  auto blend_input = inputs[0];
  std::shared_ptr<BlendFilterContents> new_blend;
  for (auto in_i = inputs.begin() + 1; in_i < inputs.end(); in_i++) {
    new_blend = std::make_shared<BlendFilterContents>();
    new_blend->SetInputs({blend_input, *in_i});
    new_blend->SetBlendMode(blend_mode);
    if (in_i < inputs.end() - 1 || foreground_color.has_value()) {
      blend_input = FilterInput::Make(
          std::static_pointer_cast<FilterContents>(new_blend));
    }
  }

  if (foreground_color.has_value()) {
    new_blend = std::make_shared<BlendFilterContents>();
    new_blend->SetInputs({blend_input});
    new_blend->SetBlendMode(blend_mode);
    new_blend->SetForegroundColor(foreground_color);
  }

  return new_blend;
}

std::shared_ptr<ColorFilterContents> ColorFilterContents::MakeColorMatrix(
    FilterInput::Ref input,
    const ColorMatrix& color_matrix) {
  auto filter = std::make_shared<ColorMatrixFilterContents>();
  filter->SetInputs({std::move(input)});
  filter->SetMatrix(color_matrix);
  return filter;
}

std::shared_ptr<ColorFilterContents>
ColorFilterContents::MakeLinearToSrgbFilter(FilterInput::Ref input) {
  auto filter = std::make_shared<LinearToSrgbFilterContents>();
  filter->SetInputs({std::move(input)});
  return filter;
}

std::shared_ptr<ColorFilterContents>
ColorFilterContents::MakeSrgbToLinearFilter(FilterInput::Ref input) {
  auto filter = std::make_shared<SrgbToLinearFilterContents>();
  filter->SetInputs({std::move(input)});
  return filter;
}

ColorFilterContents::ColorFilterContents() = default;

ColorFilterContents::~ColorFilterContents() = default;

void ColorFilterContents::SetAbsorbOpacity(AbsorbOpacity absorb_opacity) {
  absorb_opacity_ = absorb_opacity;
}

ColorFilterContents::AbsorbOpacity ColorFilterContents::GetAbsorbOpacity()
    const {
  return absorb_opacity_;
}

void ColorFilterContents::SetAlpha(Scalar alpha) {
  alpha_ = alpha;
}

std::optional<Scalar> ColorFilterContents::GetAlpha() const {
  return alpha_;
}

std::optional<Rect> ColorFilterContents::GetFilterSourceCoverage(
    const Matrix& effect_transform,
    const Rect& output_limit) const {
  return output_limit;
}

}  // namespace impeller
