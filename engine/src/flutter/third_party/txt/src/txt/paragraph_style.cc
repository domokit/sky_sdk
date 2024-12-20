/*
 * Copyright 2017 Google, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "paragraph_style.h"

#include <vector>

namespace txt {

TextStyle ParagraphStyle::GetTextStyle() const {
  TextStyle result;
  result.font_weight = font_weight;
  result.font_style = font_style;
  result.font_families = std::vector<std::string>({font_family});
  if (font_size >= 0) {
    result.font_size = font_size;
  }
  result.locale = locale;
  result.height = height;
  result.has_height_override = has_height_override;
  return result;
}

bool ParagraphStyle::unlimited_lines() const {
  return max_lines == std::numeric_limits<size_t>::max();
};

bool ParagraphStyle::ellipsized() const {
  return !ellipsis.empty();
}

TextAlign ParagraphStyle::effective_align() const {
  if (text_align == TextAlign::start) {
    return (text_direction == TextDirection::ltr) ? TextAlign::left
                                                  : TextAlign::right;
  } else if (text_align == TextAlign::end) {
    return (text_direction == TextDirection::ltr) ? TextAlign::right
                                                  : TextAlign::left;
  } else {
    return text_align;
  }
}

}  // namespace txt
