// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_TYPOGRAPHER_GLYPH_H_
#define FLUTTER_IMPELLER_TYPOGRAPHER_GLYPH_H_

#include <cstdint>
#include <functional>

namespace impeller {

//------------------------------------------------------------------------------
/// @brief      The glyph index in the typeface.
///
struct Glyph {
  enum class Type : uint8_t {
    kPath,
    kBitmap,
  };

  uint16_t index = 0;

  //------------------------------------------------------------------------------
  /// @brief  Whether the glyph is a path or a bitmap.
  ///
  Type type = Type::kPath;

  Glyph(uint16_t p_index, Type p_type) : index(p_index), type(p_type) {}
};

// Many Glyph instances are instantiated, so care should be taken when
// increasing the size.
static_assert(sizeof(Glyph) == 4);

}  // namespace impeller

template <>
struct std::hash<impeller::Glyph> {
  constexpr std::size_t operator()(const impeller::Glyph& g) const {
    static_assert(sizeof(g.index) == 2);
    static_assert(sizeof(g.type) == 1);
    return (static_cast<size_t>(g.type) << 16) | g.index;
  }
};

template <>
struct std::equal_to<impeller::Glyph> {
  constexpr bool operator()(const impeller::Glyph& lhs,
                            const impeller::Glyph& rhs) const {
    return lhs.index == rhs.index && lhs.type == rhs.type;
  }
};

template <>
struct std::less<impeller::Glyph> {
  constexpr bool operator()(const impeller::Glyph& lhs,
                            const impeller::Glyph& rhs) const {
    return lhs.index < rhs.index;
  }
};

#endif  // FLUTTER_IMPELLER_TYPOGRAPHER_GLYPH_H_
