// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/toolkit/interop/paragraph_style.h"

namespace impeller::interop {

ParagraphStyle::ParagraphStyle() = default;

ParagraphStyle::~ParagraphStyle() = default;

void ParagraphStyle::SetFontWeight(txt::FontWeight weight) {
  style_.font_weight = weight;
}

void ParagraphStyle::SetFontStyle(txt::FontStyle style) {
  style_.font_style = style;
}

void ParagraphStyle::SetFontFamily(std::string family) {
  style_.font_family = std::move(family);
}

void ParagraphStyle::SetFontSize(double size) {
  style_.font_size = size;
}

void ParagraphStyle::SetHeight(double height) {
  style_.height = height;
  style_.has_height_override = (height != 0.0);
}

void ParagraphStyle::SetTextAlignment(txt::TextAlign alignment) {
  style_.text_align = alignment;
}

void ParagraphStyle::SetTextDirection(txt::TextDirection direction) {
  style_.text_direction = direction;
}

void ParagraphStyle::SetMaxLines(size_t max_lines) {
  style_.max_lines = max_lines;
}

void ParagraphStyle::SetLocale(std::string locale) {
  style_.locale = std::move(locale);
}

void ParagraphStyle::SetForeground(ScopedObject<Paint> paint) {
  foreground_ = std::move(paint);
}

void ParagraphStyle::SetBackground(ScopedObject<Paint> paint) {
  background_ = std::move(paint);
}

txt::TextStyle ParagraphStyle::CreateTextStyle() const {
  auto style = style_.GetTextStyle();

  if (foreground_) {
    style.foreground = foreground_->GetPaint();
  }
  if (background_) {
    style.background = background_->GetPaint();
  }
  return style;
}

const txt::ParagraphStyle& ParagraphStyle::GetParagraphStyle() const {
  return style_;
}

}  // namespace impeller::interop
