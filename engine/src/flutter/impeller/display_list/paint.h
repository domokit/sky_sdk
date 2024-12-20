// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_DISPLAY_LIST_PAINT_H_
#define FLUTTER_IMPELLER_DISPLAY_LIST_PAINT_H_

#include <memory>

#include "display_list/effects/dl_color_filter.h"
#include "display_list/effects/dl_color_source.h"
#include "display_list/effects/dl_image_filter.h"
#include "impeller/display_list/color_filter.h"
#include "impeller/display_list/image_filter.h"
#include "impeller/entity/contents/color_source_contents.h"
#include "impeller/entity/contents/contents.h"
#include "impeller/entity/contents/filters/color_filter_contents.h"
#include "impeller/entity/contents/filters/filter_contents.h"
#include "impeller/entity/contents/texture_contents.h"
#include "impeller/entity/entity.h"
#include "impeller/entity/geometry/geometry.h"
#include "impeller/geometry/color.h"

namespace impeller {

struct Paint {
  using ImageFilterProc = std::function<std::shared_ptr<FilterContents>(
      FilterInput::Ref,
      const Matrix& effect_transform,
      Entity::RenderingMode rendering_mode)>;
  using MaskFilterProc = std::function<std::shared_ptr<FilterContents>(
      FilterInput::Ref,
      bool is_solid_color,
      const Matrix& effect_transform)>;
  using ColorSourceProc = std::function<std::shared_ptr<ColorSourceContents>()>;

  /// @brief Whether or not a save layer with the provided paint can perform the
  ///        opacity peephole optimization.
  static bool CanApplyOpacityPeephole(const Paint& paint) {
    return paint.blend_mode == BlendMode::kSourceOver &&
           paint.invert_colors == false &&
           !paint.mask_blur_descriptor.has_value() &&
           paint.image_filter == nullptr && paint.color_filter == nullptr;
  }

  enum class Style {
    kFill,
    kStroke,
  };

  struct MaskBlurDescriptor {
    FilterContents::BlurStyle style;
    Sigma sigma;
    /// Text mask blurs need to not apply the CTM to the blur kernel.
    /// See: https://github.com/flutter/flutter/issues/115112
    bool respect_ctm = true;

    std::shared_ptr<FilterContents> CreateMaskBlur(
        std::shared_ptr<ColorSourceContents> color_source_contents,
        const flutter::DlColorFilter* color_filter,
        bool invert_colors,
        RectGeometry* rect_geom) const;

    std::shared_ptr<FilterContents> CreateMaskBlur(
        std::shared_ptr<TextureContents> texture_contents,
        RectGeometry* rect_geom) const;

    std::shared_ptr<FilterContents> CreateMaskBlur(
        const FilterInput::Ref& input,
        bool is_solid_color,
        const Matrix& ctm) const;
  };

  Color color = Color::Black();
  const flutter::DlColorSource* color_source = nullptr;
  const flutter::DlColorFilter* color_filter = nullptr;
  const flutter::DlImageFilter* image_filter = nullptr;

  Scalar stroke_width = 0.0;
  Cap stroke_cap = Cap::kButt;
  Join stroke_join = Join::kMiter;
  Scalar stroke_miter = 4.0;
  Style style = Style::kFill;
  BlendMode blend_mode = BlendMode::kSourceOver;
  bool invert_colors = false;

  std::optional<MaskBlurDescriptor> mask_blur_descriptor;

  /// @brief      Wrap this paint's configured filters to the given contents.
  /// @param[in]  input           The contents to wrap with paint's filters.
  /// @return     The filter-wrapped contents. If there are no filters that need
  ///             to be wrapped for the current paint configuration, the
  ///             original contents is returned.
  std::shared_ptr<Contents> WithFilters(std::shared_ptr<Contents> input) const;

  /// @brief      Wrap this paint's configured filters to the given contents of
  ///             subpass target.
  /// @param[in]  input  The contents of subpass target to wrap with paint's
  ///                    filters.
  ///
  /// @return     The filter-wrapped contents. If there are no filters that need
  ///             to be wrapped for the current paint configuration, the
  ///             original contents is returned.
  std::shared_ptr<Contents> WithFiltersForSubpassTarget(
      std::shared_ptr<Contents> input,
      const Matrix& effect_transform = Matrix()) const;

  /// @brief   Whether this paint has a color filter that can apply opacity
  bool HasColorFilter() const;

  std::shared_ptr<ColorSourceContents> CreateContents() const;

  std::shared_ptr<Contents> WithMaskBlur(std::shared_ptr<Contents> input,
                                         bool is_solid_color,
                                         const Matrix& ctm) const;

  std::shared_ptr<FilterContents> WithImageFilter(
      const FilterInput::Variant& input,
      const Matrix& effect_transform,
      Entity::RenderingMode rendering_mode) const;

 private:
  std::shared_ptr<Contents> WithColorFilter(
      std::shared_ptr<Contents> input,
      ColorFilterContents::AbsorbOpacity absorb_opacity =
          ColorFilterContents::AbsorbOpacity::kNo) const;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_DISPLAY_LIST_PAINT_H_
