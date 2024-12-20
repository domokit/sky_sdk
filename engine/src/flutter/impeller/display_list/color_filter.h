// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_DISPLAY_LIST_COLOR_FILTER_H_
#define FLUTTER_IMPELLER_DISPLAY_LIST_COLOR_FILTER_H_

#include "display_list/effects/dl_color_filter.h"
#include "impeller/entity/contents/filters/color_filter_contents.h"
#include "impeller/geometry/color.h"

namespace impeller {

/// A color matrix which inverts colors.
// clang-format off
static const constexpr ColorMatrix kColorInversion = {
  .array = {
    -1.0,    0,    0, 1.0, 0, //
       0, -1.0,    0, 1.0, 0, //
       0,    0, -1.0, 1.0, 0, //
     1.0,  1.0,  1.0, 1.0, 0  //
  }
};

std::shared_ptr<ColorFilterContents> WrapWithInvertColors(
    const std::shared_ptr<FilterInput>& input,
    ColorFilterContents::AbsorbOpacity absorb_opacity);

std::shared_ptr<ColorFilterContents> WrapWithGPUColorFilter(
    const flutter::DlColorFilter* filter,
    const std::shared_ptr<FilterInput>& input,
    ColorFilterContents::AbsorbOpacity absorb_opacity);

/// A procedure that filters a given unpremultiplied color to produce a new
/// unpremultiplied color.
using ColorFilterProc = std::function<Color(Color)>;

ColorFilterProc GetCPUColorFilterProc(const flutter::DlColorFilter* filter);

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_DISPLAY_LIST_COLOR_FILTER_H_
