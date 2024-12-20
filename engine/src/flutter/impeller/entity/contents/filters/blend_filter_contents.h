// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_ENTITY_CONTENTS_FILTERS_BLEND_FILTER_CONTENTS_H_
#define FLUTTER_IMPELLER_ENTITY_CONTENTS_FILTERS_BLEND_FILTER_CONTENTS_H_

#include <optional>
#include "impeller/entity/contents/filters/color_filter_contents.h"
#include "impeller/entity/contents/filters/inputs/filter_input.h"
#include "impeller/geometry/color.h"

namespace impeller {

constexpr std::array<std::array<Scalar, 5>, 15> kPorterDuffCoefficients = {{
    {0, 0, 0, 0, 0},    // Clear
    {1, 0, 0, 0, 0},    // Source
    {0, 0, 1, 0, 0},    // Destination
    {1, 0, 1, -1, 0},   // SourceOver
    {1, -1, 1, 0, 0},   // DestinationOver
    {0, 1, 0, 0, 0},    // SourceIn
    {0, 0, 0, 1, 0},    // DestinationIn
    {1, -1, 0, 0, 0},   // SourceOut
    {0, 0, 1, -1, 0},   // DestinationOut
    {0, 1, 1, -1, 0},   // SourceATop
    {1, -1, 0, 1, 0},   // DestinationATop
    {1, -1, 1, -1, 0},  // Xor
    {1, 0, 1, 0, 0},    // Plus
    {0, 0, 0, 0, 1},    // Modulate
    {0, 0, 1, 0, -1},   // Screen
}};

std::optional<BlendMode> InvertPorterDuffBlend(BlendMode blend_mode);

class BlendFilterContents : public ColorFilterContents {
 public:
  using AdvancedBlendProc = std::function<std::optional<Entity>(
      const FilterInput::Vector& inputs,
      const ContentContext& renderer,
      const Entity& entity,
      const Rect& coverage,
      BlendMode blend_mode,
      std::optional<Color> foreground_color,
      ColorFilterContents::AbsorbOpacity absorb_opacity,
      std::optional<Scalar> alpha)>;

  BlendFilterContents();

  ~BlendFilterContents() override;

  void SetBlendMode(BlendMode blend_mode);

  /// @brief  Sets a source color which is blended after all of the inputs have
  ///         been blended.
  void SetForegroundColor(std::optional<Color> color);

 private:
  // |FilterContents|
  std::optional<Entity> RenderFilter(
      const FilterInput::Vector& inputs,
      const ContentContext& renderer,
      const Entity& entity,
      const Matrix& effect_transform,
      const Rect& coverage,
      const std::optional<Rect>& coverage_hint) const override;

  /// @brief Optimized advanced blend that avoids a second subpass when there is
  ///        only a single input and a foreground color.
  ///
  /// These contents cannot absorb opacity.
  std::optional<Entity> CreateForegroundAdvancedBlend(
      const std::shared_ptr<FilterInput>& input,
      const ContentContext& renderer,
      const Entity& entity,
      const Rect& coverage,
      Color foreground_color,
      BlendMode blend_mode,
      std::optional<Scalar> alpha,
      ColorFilterContents::AbsorbOpacity absorb_opacity) const;

  /// @brief Implements the advanced blends filters in terms of the framebuffer
  ///        blend filters.
  ///
  ///        This requires device support for frameuffer fetch,
  ///        `Capabilities::SupportsFramebufferFetch` must be true
  ///        This allows a substantial reduction in the number of bootstrapped
  ///        shaders.
  std::optional<Entity> CreateFramebufferAdvancedBlend(
      const FilterInput::Vector& inputs,
      const ContentContext& renderer,
      const Entity& entity,
      const Rect& coverage,
      std::optional<Color> foreground_color,
      BlendMode blend_mode,
      std::optional<Scalar> alpha,
      ColorFilterContents::AbsorbOpacity absorb_opacity) const;

  /// @brief Optimized porter-duff blend that avoids a second subpass when there
  ///        is only a single input and a foreground color.
  ///
  /// These contents cannot absorb opacity.
  std::optional<Entity> CreateForegroundPorterDuffBlend(
      const std::shared_ptr<FilterInput>& input,
      const ContentContext& renderer,
      const Entity& entity,
      const Rect& coverage,
      Color foreground_color,
      BlendMode blend_mode,
      std::optional<Scalar> alpha,
      ColorFilterContents::AbsorbOpacity absorb_opacity) const;

  BlendMode blend_mode_ = BlendMode::kSourceOver;
  AdvancedBlendProc advanced_blend_proc_;
  std::optional<Color> foreground_color_;

  BlendFilterContents(const BlendFilterContents&) = delete;

  BlendFilterContents& operator=(const BlendFilterContents&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_ENTITY_CONTENTS_FILTERS_BLEND_FILTER_CONTENTS_H_
