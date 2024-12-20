// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_GEOMETRY_SIZE_H_
#define FLUTTER_IMPELLER_GEOMETRY_SIZE_H_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <ostream>
#include <string>

#include "impeller/geometry/scalar.h"

namespace impeller {

#define ONLY_ON_FLOAT_M(Modifiers, Return) \
  template <typename U = T>                \
  Modifiers std::enable_if_t<std::is_floating_point_v<U>, Return>
#define ONLY_ON_FLOAT(Return) DL_ONLY_ON_FLOAT_M(, Return)

template <class T>
struct TSize {
  using Type = T;

  Type width = {};
  Type height = {};

  constexpr TSize() {}

  constexpr TSize(Type width, Type height) : width(width), height(height) {}

  constexpr explicit TSize(Type dimension)
      : width(dimension), height(dimension) {}

  template <class U>
  explicit constexpr TSize(const TSize<U>& other)
      : TSize(static_cast<Type>(other.width), static_cast<Type>(other.height)) {
  }

  static constexpr TSize MakeWH(Type width, Type height) {
    return TSize{width, height};
  }

  static constexpr TSize Infinite() {
    return TSize{std::numeric_limits<Type>::max(),
                 std::numeric_limits<Type>::max()};
  }

  constexpr TSize operator*(Scalar scale) const {
    return {width * scale, height * scale};
  }

  template <class U, class = std::enable_if_t<std::is_arithmetic_v<U>>>
  inline TSize operator*=(U scale) {
    width *= static_cast<Type>(scale);
    height *= static_cast<Type>(scale);
    return *this;
  }

  constexpr TSize operator/(Scalar scale) const {
    return {static_cast<Scalar>(width) / scale,
            static_cast<Scalar>(height) / scale};
  }

  constexpr TSize operator/(const TSize& s) const {
    return {width / s.width, height / s.height};
  }

  constexpr bool operator==(const TSize& s) const {
    return s.width == width && s.height == height;
  }

  constexpr bool operator!=(const TSize& s) const {
    return s.width != width || s.height != height;
  }

  constexpr TSize operator+(const TSize& s) const {
    return {width + s.width, height + s.height};
  }

  constexpr TSize operator-(const TSize& s) const {
    return {width - s.width, height - s.height};
  }

  constexpr TSize operator-() const { return {-width, -height}; }

  constexpr TSize Min(const TSize& o) const {
    return {
        std::min(width, o.width),
        std::min(height, o.height),
    };
  }

  constexpr TSize Max(const TSize& o) const {
    return {
        std::max(width, o.width),
        std::max(height, o.height),
    };
  }

  constexpr Type MaxDimension() const { return std::max(width, height); }

  constexpr TSize Abs() const { return {std::fabs(width), std::fabs(height)}; }

  constexpr TSize Floor() const {
    return {std::floor(width), std::floor(height)};
  }

  constexpr TSize Ceil() const { return {std::ceil(width), std::ceil(height)}; }

  constexpr TSize Round() const {
    return {std::round(width), std::round(height)};
  }

  constexpr Type Area() const { return width * height; }

  /// Returns true if either of the width or height are 0, negative, or NaN.
  constexpr bool IsEmpty() const { return !(width > 0 && height > 0); }

  ONLY_ON_FLOAT_M(constexpr, bool)
  IsFinite() const { return std::isfinite(width) && std::isfinite(height); }

  constexpr bool IsSquare() const { return width == height; }

  template <class U>
  static constexpr TSize Ceil(const TSize<U>& other) {
    return TSize{static_cast<Type>(std::ceil(other.width)),
                 static_cast<Type>(std::ceil(other.height))};
  }

  constexpr size_t MipCount() const {
    constexpr size_t minimum_mip = 1u;
    if (IsEmpty()) {
      return minimum_mip;
    }
    size_t result = std::max(ceil(log2(width)), ceil(log2(height)));
    return std::max(result, minimum_mip);
  }
};

// RHS algebraic operations with arithmetic types.

template <class T, class U, class = std::enable_if_t<std::is_arithmetic_v<U>>>
constexpr TSize<T> operator*(U s, const TSize<T>& p) {
  return p * s;
}

template <class T, class U, class = std::enable_if_t<std::is_arithmetic_v<U>>>
constexpr TSize<T> operator/(U s, const TSize<T>& p) {
  return {static_cast<T>(s) / p.width, static_cast<T>(s) / p.height};
}

using Size = TSize<Scalar>;
using ISize32 = TSize<int32_t>;
using ISize64 = TSize<int64_t>;
using ISize = ISize64;

static_assert(sizeof(Size) == 2 * sizeof(Scalar));

}  // namespace impeller

namespace std {

template <class T>
inline std::ostream& operator<<(std::ostream& out,
                                const impeller::TSize<T>& s) {
  out << "(" << s.width << ", " << s.height << ")";
  return out;
}

}  // namespace std

#endif  // FLUTTER_IMPELLER_GEOMETRY_SIZE_H_
