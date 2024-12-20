// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "keyboard_utils.h"

#include "flutter/fml/logging.h"

namespace flutter {

std::u16string EncodeUtf16(char32_t character) {
  // Algorithm: https://en.wikipedia.org/wiki/UTF-16#Description
  std::u16string result;
  // Invalid value.
  FML_DCHECK(!(character >= 0xD800 && character <= 0xDFFF) &&
             !(character > 0x10FFFF));
  if ((character >= 0xD800 && character <= 0xDFFF) || (character > 0x10FFFF)) {
    return result;
  }
  if (character <= 0xD7FF || (character >= 0xE000 && character <= 0xFFFF)) {
    result.push_back((char16_t)character);
    return result;
  }
  uint32_t remnant = character - 0x10000;
  result.push_back((remnant >> 10) + 0xD800);
  result.push_back((remnant & 0x3FF) + 0xDC00);
  return result;
}

}  // namespace flutter
