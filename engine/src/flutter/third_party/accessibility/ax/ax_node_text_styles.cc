// Copyright 2019 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "ax_node_text_styles.h"

constexpr int kUnsetValue = -1;

namespace ui {
AXNodeTextStyles::AXNodeTextStyles()
    : background_color(kUnsetValue),
      color(kUnsetValue),
      invalid_state(kUnsetValue),
      overline_style(kUnsetValue),
      strikethrough_style(kUnsetValue),
      text_direction(kUnsetValue),
      text_position(kUnsetValue),
      text_style(kUnsetValue),
      underline_style(kUnsetValue),
      font_size(kUnsetValue),
      font_weight(kUnsetValue) {}

AXNodeTextStyles::AXNodeTextStyles(AXNodeTextStyles&& other)
    : background_color(other.background_color),
      color(other.color),
      invalid_state(other.invalid_state),
      overline_style(other.overline_style),
      strikethrough_style(other.strikethrough_style),
      text_direction(other.text_direction),
      text_position(other.text_position),
      text_style(other.text_style),
      underline_style(other.underline_style),
      font_size(other.font_size),
      font_weight(other.font_weight),
      font_family(std::move(other.font_family)) {}

AXNodeTextStyles& AXNodeTextStyles::operator=(AXNodeTextStyles&& other) {
  background_color = other.background_color;
  color = other.color;
  invalid_state = other.invalid_state;
  overline_style = other.overline_style;
  strikethrough_style = other.strikethrough_style;
  text_direction = other.text_direction;
  text_position = other.text_position;
  text_style = other.text_style;
  underline_style = other.underline_style;
  font_size = other.font_size;
  font_weight = other.font_weight;
  font_family = other.font_family;

  return *this;
}

bool AXNodeTextStyles::operator==(const AXNodeTextStyles& other) const {
  return (background_color == other.background_color && color == other.color &&
          invalid_state == other.invalid_state &&
          overline_style == other.overline_style &&
          strikethrough_style == other.strikethrough_style &&
          text_direction == other.text_direction &&
          text_position == other.text_position &&
          font_size == other.font_size && font_weight == other.font_weight &&
          text_style == other.text_style &&
          underline_style == other.underline_style &&
          font_family == other.font_family);
}

bool AXNodeTextStyles::operator!=(const AXNodeTextStyles& other) const {
  return !operator==(other);
}

bool AXNodeTextStyles::IsUnset() const {
  return (background_color == kUnsetValue && invalid_state == kUnsetValue &&
          overline_style == kUnsetValue && strikethrough_style == kUnsetValue &&
          text_position == kUnsetValue && font_size == kUnsetValue &&
          font_weight == kUnsetValue && text_style == kUnsetValue &&
          underline_style == kUnsetValue && font_family.length() == 0);
}

}  // namespace ui
