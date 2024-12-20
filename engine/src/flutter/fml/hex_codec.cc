// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/fml/base32.h"

#include <cstdint>  // uint8_t
#include <string>

namespace fml {

static constexpr char kEncoding[] = "0123456789abcdef";

std::string HexEncode(std::string_view input) {
  std::string result;
  result.reserve(input.size() * 2);
  for (char c : input) {
    uint8_t b = static_cast<uint8_t>(c);
    result.push_back(kEncoding[b >> 4]);
    result.push_back(kEncoding[b & 0xF]);
  }
  return result;
}

}  // namespace fml
