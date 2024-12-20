// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_CORE_RANGE_H_
#define FLUTTER_IMPELLER_CORE_RANGE_H_

#include <algorithm>
#include <cstddef>

namespace impeller {

struct Range {
  size_t offset = 0;
  size_t length = 0;

  constexpr Range() {}

  constexpr Range(size_t p_offset, size_t p_length)
      : offset(p_offset), length(p_length) {}

  constexpr bool operator==(const Range& o) const {
    return offset == o.offset && length == o.length;
  }

  /// @brief Create a new range that is a union of this range and other.
  constexpr Range Merge(const Range& other) {
    if (other.length == 0) {
      return *this;
    }
    if (length == 0) {
      return other;
    }
    auto end_offset = std::max(offset + length, other.offset + other.length);
    auto start_offset = std::min(offset, other.offset);
    return Range{start_offset, end_offset - start_offset};
  }
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_CORE_RANGE_H_
