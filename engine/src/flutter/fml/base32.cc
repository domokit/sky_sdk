// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/fml/base32.h"

#include <cstdint>  // uint8_t
#include <limits>
#include <string>

namespace fml {

static constexpr char kEncoding[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ234567";

std::pair<bool, std::string> Base32Encode(std::string_view input) {
  if (input.empty()) {
    return {true, ""};
  }

  if (input.size() > std::numeric_limits<size_t>::max() / 8) {
    return {false, ""};
  }

  std::string output;
  const size_t encoded_length = (input.size() * 8 + 4) / 5;
  output.reserve(encoded_length);

  Base32EncodeConverter converter;
  converter.Append(static_cast<uint8_t>(input[0]));
  size_t next_byte_index = 1;

  while (converter.CanExtract()) {
    output.push_back(kEncoding[converter.Extract()]);
    if (converter.CanAppend() && next_byte_index < input.size()) {
      converter.Append(static_cast<uint8_t>(input[next_byte_index++]));
    }
  }

  if (converter.BitsAvailable() > 0) {
    output.push_back(kEncoding[converter.Peek()]);
  }

  return {true, output};
}

static constexpr signed char kDecodeMap[] = {
    // starting from ASCII 50 '2'
    26, 27, 28, 29, 30, 31, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, 0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 12,
    13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25};

static constexpr int kDecodeMapSize =
    sizeof(kDecodeMap) / sizeof(kDecodeMap[0]);

std::pair<bool, std::string> Base32Decode(const std::string& input) {
  std::string result;
  Base32DecodeConverter converter;
  for (char c : input) {
    int map_index = c - '2';
    if (map_index < 0 || map_index >= kDecodeMapSize ||
        kDecodeMap[map_index] == -1) {
      return {false, result};
    }
    converter.Append(kDecodeMap[map_index]);
    if (converter.CanExtract()) {
      result.push_back(converter.Extract());
    }
  }
  if (converter.Peek() != 0) {
    // The padding should always be zero. Return false if not.
    return {false, result};
  }
  return {true, result};
}

}  // namespace fml
