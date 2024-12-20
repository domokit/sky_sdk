// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/display_list/skia/dl_sk_paint_dispatcher.h"

#include <math.h>
#include <optional>
#include <type_traits>

#include "flutter/display_list/dl_blend_mode.h"
#include "flutter/display_list/skia/dl_sk_conversions.h"
#include "flutter/fml/logging.h"

#include "third_party/skia/include/core/SkColorFilter.h"

namespace flutter {

// clang-format off
constexpr float kInvertColorMatrix[20] = {
  -1.0,    0,    0, 1.0, 0,
     0, -1.0,    0, 1.0, 0,
     0,    0, -1.0, 1.0, 0,
   1.0,  1.0,  1.0, 1.0, 0
};
// clang-format on

void DlSkPaintDispatchHelper::save_opacity(SkScalar child_opacity) {
  save_stack_.emplace_back(opacity_);
  set_opacity(child_opacity);
}
void DlSkPaintDispatchHelper::restore_opacity() {
  if (save_stack_.empty()) {
    return;
  }
  set_opacity(save_stack_.back().opacity);
  save_stack_.pop_back();
}

void DlSkPaintDispatchHelper::setAntiAlias(bool aa) {
  paint_.setAntiAlias(aa);
}
void DlSkPaintDispatchHelper::setInvertColors(bool invert) {
  invert_colors_ = invert;
  paint_.setColorFilter(makeColorFilter());
}
void DlSkPaintDispatchHelper::setStrokeCap(DlStrokeCap cap) {
  paint_.setStrokeCap(ToSk(cap));
}
void DlSkPaintDispatchHelper::setStrokeJoin(DlStrokeJoin join) {
  paint_.setStrokeJoin(ToSk(join));
}
void DlSkPaintDispatchHelper::setDrawStyle(DlDrawStyle style) {
  paint_.setStyle(ToSk(style));
}
void DlSkPaintDispatchHelper::setStrokeWidth(SkScalar width) {
  paint_.setStrokeWidth(width);
}
void DlSkPaintDispatchHelper::setStrokeMiter(SkScalar limit) {
  paint_.setStrokeMiter(limit);
}
void DlSkPaintDispatchHelper::setColor(DlColor color) {
  current_color_ = ToSk(color);
  paint_.setColor(ToSk(color));
  if (has_opacity()) {
    paint_.setAlphaf(paint_.getAlphaf() * opacity());
  }
}
void DlSkPaintDispatchHelper::setBlendMode(DlBlendMode mode) {
  paint_.setBlendMode(ToSk(mode));
}
void DlSkPaintDispatchHelper::setColorSource(const DlColorSource* source) {
  // On the Impeller backend, we only support dithering of *gradients*, and
  // so we need to set the dither flag whenever we render a gradient.
  //
  // In this method we can determine whether or not the source is a gradient,
  // but we don't have the other half of the information which is what
  // rendering op is being performed. So, we simply record whether the
  // source is a gradient here and let the |paint()| method figure out
  // the rest (i.e. whether the color source will be used).
  color_source_gradient_ = source && source->isGradient();
  paint_.setShader(ToSk(source));
}
void DlSkPaintDispatchHelper::setImageFilter(const DlImageFilter* filter) {
  paint_.setImageFilter(ToSk(filter));
}
void DlSkPaintDispatchHelper::setColorFilter(const DlColorFilter* filter) {
  sk_color_filter_ = ToSk(filter);
  paint_.setColorFilter(makeColorFilter());
}
void DlSkPaintDispatchHelper::setMaskFilter(const DlMaskFilter* filter) {
  paint_.setMaskFilter(ToSk(filter));
}

sk_sp<SkColorFilter> DlSkPaintDispatchHelper::makeColorFilter() const {
  if (!invert_colors_) {
    return sk_color_filter_;
  }
  sk_sp<SkColorFilter> invert_filter =
      SkColorFilters::Matrix(kInvertColorMatrix);
  if (sk_color_filter_) {
    invert_filter = invert_filter->makeComposed(sk_color_filter_);
  }
  return invert_filter;
}

}  // namespace flutter
