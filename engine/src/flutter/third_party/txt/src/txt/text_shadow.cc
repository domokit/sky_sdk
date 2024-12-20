/*
 * Copyright 2018 Google, Inc.
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

#include "text_shadow.h"
#include "third_party/skia/include/core/SkColor.h"

namespace txt {

TextShadow::TextShadow() {}
TextShadow::TextShadow(SkColor color, SkPoint offset, double blur_sigma)
    : color(color), offset(offset), blur_sigma(blur_sigma) {}

bool TextShadow::operator==(const TextShadow& other) const {
  if (color != other.color)
    return false;
  if (offset != other.offset)
    return false;
  if (blur_sigma != other.blur_sigma)
    return false;

  return true;
}

bool TextShadow::operator!=(const TextShadow& other) const {
  return !(*this == other);
}

bool TextShadow::hasShadow() const {
  if (!offset.isZero())
    return true;
  if (blur_sigma > 0.5)
    return true;

  return false;
}

}  // namespace txt
