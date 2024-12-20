// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/toolkit/interop/paint.h"

namespace impeller::interop {

Paint::Paint() = default;

Paint::~Paint() = default;

const flutter::DlPaint& Paint::GetPaint() const {
  return paint_;
}

void Paint::SetColor(flutter::DlColor color) {
  paint_.setColor(color);
}

void Paint::SetBlendMode(BlendMode mode) {
  paint_.setBlendMode(ToDisplayListType(mode));
}

void Paint::SetDrawStyle(flutter::DlDrawStyle style) {
  paint_.setDrawStyle(style);
}

void Paint::SetStrokeCap(flutter::DlStrokeCap stroke_cap) {
  paint_.setStrokeCap(stroke_cap);
}

void Paint::SetStrokeJoin(flutter::DlStrokeJoin stroke_join) {
  paint_.setStrokeJoin(stroke_join);
}

void Paint::SetStrokeWidth(Scalar width) {
  paint_.setStrokeWidth(width);
}

void Paint::SetStrokeMiter(Scalar miter) {
  paint_.setStrokeMiter(miter);
}

void Paint::SetColorFilter(const ColorFilter& filter) {
  paint_.setColorFilter(filter.GetColorFilter());
}

void Paint::SetColorSource(const ColorSource& source) {
  paint_.setColorSource(source.GetColorSource());
}

void Paint::SetImageFilter(const ImageFilter& filter) {
  paint_.setImageFilter(filter.GetImageFilter());
}

void Paint::SetMaskFilter(const MaskFilter& filter) {
  paint_.setMaskFilter(filter.GetMaskFilter());
}

}  // namespace impeller::interop
