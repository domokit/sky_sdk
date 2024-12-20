// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_GEOMETRY_HALF_H_
#define FLUTTER_IMPELLER_GEOMETRY_HALF_H_

#include <cstdint>

#include "flutter/fml/build_config.h"

#include "impeller/geometry/color.h"
#include "impeller/geometry/point.h"
#include "impeller/geometry/scalar.h"
#include "impeller/geometry/vector.h"

// NOLINTBEGIN(google-explicit-constructor)

#if defined(FML_OS_MACOSX) || defined(FML_OS_IOS) || \
    defined(FML_OS_IOS_SIMULATOR)
using InternalHalf = _Float16;
#else
using InternalHalf = uint16_t;
#endif

namespace impeller {

/// @brief Convert a scalar to a half precision float.
///
/// See also: https://clang.llvm.org/docs/LanguageExtensions.html
/// This is not currently supported on Windows toolchains.
inline constexpr InternalHalf ScalarToHalf(Scalar f) {
#ifdef FML_OS_WIN
  return static_cast<InternalHalf>(0);
#else
  return static_cast<InternalHalf>(f);
#endif
}

/// @brief A storage only class for half precision floating point.
struct Half {
  InternalHalf x = 0;

  constexpr Half() {}

  constexpr Half(double value) : x(ScalarToHalf(static_cast<Scalar>(value))) {}

  constexpr Half(Scalar value) : x(ScalarToHalf(value)) {}

  constexpr Half(int value) : x(ScalarToHalf(static_cast<Scalar>(value))) {}

  constexpr Half(InternalHalf x) : x(x) {}

  constexpr bool operator==(const Half& v) const { return v.x == x; }

  constexpr bool operator!=(const Half& v) const { return v.x != x; }
};

/// @brief A storage only class for half precision floating point vector 4.
struct HalfVector4 {
  union {
    struct {
      InternalHalf x = 0;
      InternalHalf y = 0;
      InternalHalf z = 0;
      InternalHalf w = 0;
    };
    InternalHalf e[4];
  };

  constexpr HalfVector4() {}

  constexpr HalfVector4(const Color& a)
      : x(ScalarToHalf(a.red)),
        y(ScalarToHalf(a.green)),
        z(ScalarToHalf(a.blue)),
        w(ScalarToHalf(a.alpha)) {}

  constexpr HalfVector4(const Vector4& a)
      : x(ScalarToHalf(a.x)),
        y(ScalarToHalf(a.y)),
        z(ScalarToHalf(a.z)),
        w(ScalarToHalf(a.w)) {}

  constexpr HalfVector4(InternalHalf x,
                        InternalHalf y,
                        InternalHalf z,
                        InternalHalf w)
      : x(x), y(y), z(z), w(w) {}

  constexpr bool operator==(const HalfVector4& v) const {
    return v.x == x && v.y == y && v.z == z && v.w == w;
  }

  constexpr bool operator!=(const HalfVector4& v) const {
    return v.x != x || v.y != y || v.z != z || v.w != w;
  }
};

/// @brief A storage only class for half precision floating point vector 3.
struct HalfVector3 {
  union {
    struct {
      InternalHalf x = 0;
      InternalHalf y = 0;
      InternalHalf z = 0;
    };
    InternalHalf e[3];
  };

  constexpr HalfVector3() {}

  constexpr HalfVector3(const Vector3& a)
      : x(ScalarToHalf(a.x)), y(ScalarToHalf(a.y)), z(ScalarToHalf(a.z)) {}

  constexpr HalfVector3(InternalHalf x, InternalHalf y, InternalHalf z)
      : x(x), y(y), z(z) {}

  constexpr bool operator==(const HalfVector3& v) const {
    return v.x == x && v.y == y && v.z == z;
  }

  constexpr bool operator!=(const HalfVector3& v) const {
    return v.x != x || v.y != y || v.z != z;
  }
};

/// @brief A storage only class for half precision floating point vector 2.
struct HalfVector2 {
  union {
    struct {
      InternalHalf x = 0;
      InternalHalf y = 0;
    };
    InternalHalf e[2];
  };

  constexpr HalfVector2() {}

  constexpr HalfVector2(const Vector2& a)
      : x(ScalarToHalf(a.x)), y(ScalarToHalf(a.y)) {}

  constexpr HalfVector2(InternalHalf x, InternalHalf y) : x(x), y(y){};

  constexpr bool operator==(const HalfVector2& v) const {
    return v.x == x && v.y == y;
  }

  constexpr bool operator!=(const HalfVector2& v) const {
    return v.x != x || v.y != y;
  }
};

static_assert(sizeof(Half) == sizeof(uint16_t));
static_assert(sizeof(HalfVector2) == 2 * sizeof(Half));
static_assert(sizeof(HalfVector3) == 3 * sizeof(Half));
static_assert(sizeof(HalfVector4) == 4 * sizeof(Half));

}  // namespace impeller

namespace std {

inline std::ostream& operator<<(std::ostream& out, const impeller::Half& p) {
  out << "(" << static_cast<impeller::Scalar>(p.x) << ")";
  return out;
}

inline std::ostream& operator<<(std::ostream& out,
                                const impeller::HalfVector2& p) {
  out << "(" << static_cast<impeller::Scalar>(p.x) << ", "
      << static_cast<impeller::Scalar>(p.y) << ")";
  return out;
}

inline std::ostream& operator<<(std::ostream& out,
                                const impeller::HalfVector3& p) {
  out << "(" << static_cast<impeller::Scalar>(p.x) << ", "
      << static_cast<impeller::Scalar>(p.y) << ", "
      << static_cast<impeller::Scalar>(p.z) << ")";
  return out;
}

inline std::ostream& operator<<(std::ostream& out,
                                const impeller::HalfVector4& p) {
  out << "(" << static_cast<impeller::Scalar>(p.x) << ", "
      << static_cast<impeller::Scalar>(p.y) << ", "
      << static_cast<impeller::Scalar>(p.z) << ", "
      << static_cast<impeller::Scalar>(p.w) << ")";
  return out;
}

// NOLINTEND(google-explicit-constructor)

}  // namespace std

#endif  // FLUTTER_IMPELLER_GEOMETRY_HALF_H_
