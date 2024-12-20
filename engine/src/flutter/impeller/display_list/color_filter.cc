// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/display_list/color_filter.h"

#include "display_list/effects/dl_color_filters.h"
#include "fml/logging.h"
#include "impeller/display_list/skia_conversions.h"
#include "impeller/entity/contents/filters/color_filter_contents.h"
#include "impeller/entity/contents/filters/inputs/filter_input.h"
#include "impeller/geometry/color.h"

namespace impeller {

std::shared_ptr<ColorFilterContents> WrapWithInvertColors(
    const std::shared_ptr<FilterInput>& input,
    ColorFilterContents::AbsorbOpacity absorb_opacity) {
  auto filter = ColorFilterContents::MakeColorMatrix({input}, kColorInversion);
  filter->SetAbsorbOpacity(absorb_opacity);
  return filter;
}

std::shared_ptr<ColorFilterContents> WrapWithGPUColorFilter(
    const flutter::DlColorFilter* filter,
    const std::shared_ptr<FilterInput>& input,
    ColorFilterContents::AbsorbOpacity absorb_opacity) {
  FML_DCHECK(filter);

  switch (filter->type()) {
    case flutter::DlColorFilterType::kBlend: {
      const flutter::DlBlendColorFilter* blend_filter = filter->asBlend();
      FML_DCHECK(blend_filter);

      auto filter = ColorFilterContents::MakeBlend(
          static_cast<BlendMode>(blend_filter->mode()), {input},
          skia_conversions::ToColor(blend_filter->color()));
      filter->SetAbsorbOpacity(absorb_opacity);
      return filter;
    }
    case flutter::DlColorFilterType::kMatrix: {
      const flutter::DlMatrixColorFilter* matrix_filter = filter->asMatrix();
      FML_DCHECK(matrix_filter);

      impeller::ColorMatrix color_matrix;
      matrix_filter->get_matrix(color_matrix.array);
      auto filter = ColorFilterContents::MakeColorMatrix({input}, color_matrix);
      filter->SetAbsorbOpacity(absorb_opacity);
      return filter;
    }
    case flutter::DlColorFilterType::kSrgbToLinearGamma: {
      auto filter = ColorFilterContents::MakeSrgbToLinearFilter({input});
      filter->SetAbsorbOpacity(absorb_opacity);
      return filter;
    }
    case flutter::DlColorFilterType::kLinearToSrgbGamma: {
      auto filter = ColorFilterContents::MakeLinearToSrgbFilter({input});
      filter->SetAbsorbOpacity(absorb_opacity);
      return filter;
    }
  }

  FML_UNREACHABLE();
}

ColorFilterProc GetCPUColorFilterProc(const flutter::DlColorFilter* filter) {
  FML_DCHECK(filter);

  switch (filter->type()) {
    case flutter::DlColorFilterType::kBlend: {
      const flutter::DlBlendColorFilter* blend_filter = filter->asBlend();
      FML_DCHECK(blend_filter);

      return [filter_blend_mode = static_cast<BlendMode>(blend_filter->mode()),
              filter_color = skia_conversions::ToColor(blend_filter->color())](
                 Color color) {
        return color.Blend(filter_color, filter_blend_mode);
      };
    }
    case flutter::DlColorFilterType::kMatrix: {
      const flutter::DlMatrixColorFilter* matrix_filter = filter->asMatrix();
      FML_DCHECK(matrix_filter);

      impeller::ColorMatrix color_matrix;
      matrix_filter->get_matrix(color_matrix.array);
      return [color_matrix = color_matrix](Color color) {
        return color.ApplyColorMatrix(color_matrix);
      };
    }
    case flutter::DlColorFilterType::kSrgbToLinearGamma: {
      return [](Color color) { return color.SRGBToLinear(); };
    }

    case flutter::DlColorFilterType::kLinearToSrgbGamma: {
      return [](Color color) { return color.LinearToSRGB(); };
    }
  }

  FML_UNREACHABLE();
}

}  // namespace impeller
