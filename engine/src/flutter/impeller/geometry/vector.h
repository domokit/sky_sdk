// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_GEOMETRY_VECTOR_H_
#define FLUTTER_IMPELLER_GEOMETRY_VECTOR_H_

#include <cmath>
#include <string>

#include "impeller/geometry/color.h"
#include "impeller/geometry/point.h"
#include "impeller/geometry/scalar.h"
#include "impeller/geometry/size.h"

namespace impeller {

// NOLINTBEGIN(google-explicit-constructor)

struct Vector3 {
  union {
    struct {
      Scalar x = 0.0f;
      Scalar y = 0.0f;
      Scalar z = 0.0f;
    };
    Scalar e[3];
  };

  constexpr Vector3(){};

  constexpr Vector3(const Color& c) : x(c.red), y(c.green), z(c.blue) {}

  constexpr Vector3(const Point& p) : x(p.x), y(p.y) {}

  constexpr Vector3(const Size& s) : x(s.width), y(s.height) {}

  constexpr Vector3(Scalar x, Scalar y) : x(x), y(y) {}

  constexpr Vector3(Scalar x, Scalar y, Scalar z) : x(x), y(y), z(z) {}

  /**
   *  The length (or magnitude of the vector).
   *
   *  @return the calculated length.
   */
  constexpr Scalar GetLength() const { return sqrt(x * x + y * y + z * z); }

  constexpr Vector3 Normalize() const {
    const auto len = GetLength();
    return {x / len, y / len, z / len};
  }

  constexpr Scalar Dot(const Vector3& other) const {
    return ((x * other.x) + (y * other.y) + (z * other.z));
  }

  constexpr Vector3 Abs() const {
    return {std::fabs(x), std::fabs(y), std::fabs(z)};
  }

  constexpr Vector3 Cross(const Vector3& other) const {
    return {
        (y * other.z) - (z * other.y),  //
        (z * other.x) - (x * other.z),  //
        (x * other.y) - (y * other.x)   //
    };
  }

  constexpr Vector3 Min(const Vector3& p) const {
    return {std::min(x, p.x), std::min(y, p.y), std::min(z, p.z)};
  }

  constexpr Vector3 Max(const Vector3& p) const {
    return {std::max(x, p.x), std::max(y, p.y), std::max(z, p.z)};
  }

  constexpr Vector3 Floor() const {
    return {std::floor(x), std::floor(y), std::floor(z)};
  }

  constexpr Vector3 Ceil() const {
    return {std::ceil(x), std::ceil(y), std::ceil(z)};
  }

  constexpr Vector3 Round() const {
    return {std::round(x), std::round(y), std::round(z)};
  }

  constexpr bool operator==(const Vector3& v) const {
    return v.x == x && v.y == y && v.z == z;
  }

  constexpr bool operator!=(const Vector3& v) const {
    return v.x != x || v.y != y || v.z != z;
  }

  constexpr Vector3 operator+=(const Vector3& p) {
    x += p.x;
    y += p.y;
    z += p.z;
    return *this;
  }

  constexpr Vector3 operator-=(const Vector3& p) {
    x -= p.x;
    y -= p.y;
    z -= p.z;
    return *this;
  }

  constexpr Vector3 operator*=(const Vector3& p) {
    x *= p.x;
    y *= p.y;
    z *= p.z;
    return *this;
  }

  template <class U, class = std::enable_if_t<std::is_arithmetic_v<U>>>
  constexpr Vector3 operator*=(U scale) {
    x *= scale;
    y *= scale;
    z *= scale;
    return *this;
  }

  constexpr Vector3 operator/=(const Vector3& p) {
    x /= p.x;
    y /= p.y;
    z /= p.z;
    return *this;
  }

  template <class U, class = std::enable_if_t<std::is_arithmetic_v<U>>>
  constexpr Vector3 operator/=(U scale) {
    x /= scale;
    y /= scale;
    z /= scale;
    return *this;
  }

  constexpr Vector3 operator-() const { return Vector3(-x, -y, -z); }

  constexpr Vector3 operator+(const Vector3& v) const {
    return Vector3(x + v.x, y + v.y, z + v.z);
  }

  constexpr Vector3 operator-(const Vector3& v) const {
    return Vector3(x - v.x, y - v.y, z - v.z);
  }

  constexpr Vector3 operator+(Scalar s) const {
    return Vector3(x + s, y + s, z + s);
  }

  constexpr Vector3 operator-(Scalar s) const {
    return Vector3(x - s, y - s, z - s);
  }

  constexpr Vector3 operator*(const Vector3& v) const {
    return Vector3(x * v.x, y * v.y, z * v.z);
  }

  template <class U, class = std::enable_if_t<std::is_arithmetic_v<U>>>
  constexpr Vector3 operator*(U scale) const {
    return Vector3(x * scale, y * scale, z * scale);
  }

  constexpr Vector3 operator/(const Vector3& v) const {
    return Vector3(x / v.x, y / v.y, z / v.z);
  }

  template <class U, class = std::enable_if_t<std::is_arithmetic_v<U>>>
  constexpr Vector3 operator/(U scale) const {
    return Vector3(x / scale, y / scale, z / scale);
  }

  constexpr Vector3 Lerp(const Vector3& v, Scalar t) const {
    return *this + (v - *this) * t;
  }

  /**
   *  Make a linear combination of two vectors and return the result.
   *
   *  @param a      the first vector.
   *  @param aScale the scale to use for the first vector.
   *  @param b      the second vector.
   *  @param bScale the scale to use for the second vector.
   *
   *  @return the combined vector.
   */
  static constexpr Vector3 Combine(const Vector3& a,
                                   Scalar aScale,
                                   const Vector3& b,
                                   Scalar bScale) {
    return {
        aScale * a.x + bScale * b.x,  //
        aScale * a.y + bScale * b.y,  //
        aScale * a.z + bScale * b.z,  //
    };
  }

  std::string ToString() const;
};

// RHS algebraic operations with arithmetic types.

template <class U, class = std::enable_if_t<std::is_arithmetic_v<U>>>
constexpr Vector3 operator*(U s, const Vector3& p) {
  return p * s;
}

template <class U, class = std::enable_if_t<std::is_arithmetic_v<U>>>
constexpr Vector3 operator+(U s, const Vector3& p) {
  return p + s;
}

template <class U, class = std::enable_if_t<std::is_arithmetic_v<U>>>
constexpr Vector3 operator-(U s, const Vector3& p) {
  return -p + s;
}

template <class U, class = std::enable_if_t<std::is_arithmetic_v<U>>>
constexpr Vector3 operator/(U s, const Vector3& p) {
  return {
      static_cast<Scalar>(s) / p.x,
      static_cast<Scalar>(s) / p.y,
      static_cast<Scalar>(s) / p.z,
  };
}

struct Vector4 {
  union {
    struct {
      Scalar x = 0.0f;
      Scalar y = 0.0f;
      Scalar z = 0.0f;
      Scalar w = 1.0f;
    };
    Scalar e[4];
  };

  constexpr Vector4() {}

  constexpr Vector4(const Color& c)
      : x(c.red), y(c.green), z(c.blue), w(c.alpha) {}

  constexpr Vector4(Scalar x, Scalar y, Scalar z, Scalar w)
      : x(x), y(y), z(z), w(w) {}

  constexpr Vector4(const Vector3& v) : x(v.x), y(v.y), z(v.z) {}

  constexpr Vector4(const Point& p) : x(p.x), y(p.y) {}

  constexpr Vector4(std::array<Scalar, 4> values)
      : x(values[0]), y(values[1]), z(values[2]), w(values[3]) {}

  constexpr bool IsFinite() const {
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z) &&
           std::isfinite(w);
  }

  Vector4 Normalize() const {
    const Scalar inverse = 1.0f / sqrt(x * x + y * y + z * z + w * w);
    return Vector4(x * inverse, y * inverse, z * inverse, w * inverse);
  }

  constexpr bool operator==(const Vector4& v) const {
    return (x == v.x) && (y == v.y) && (z == v.z) && (w == v.w);
  }

  constexpr bool operator!=(const Vector4& v) const {
    return (x != v.x) || (y != v.y) || (z != v.z) || (w != v.w);
  }

  constexpr Vector4 operator+(const Vector4& v) const {
    return Vector4(x + v.x, y + v.y, z + v.z, w + v.w);
  }

  constexpr Vector4 operator-(const Vector4& v) const {
    return Vector4(x - v.x, y - v.y, z - v.z, w - v.w);
  }

  constexpr Vector4 operator*(Scalar f) const {
    return Vector4(x * f, y * f, z * f, w * f);
  }

  constexpr Vector4 operator*(const Vector4& v) const {
    return Vector4(x * v.x, y * v.y, z * v.z, w * v.w);
  }

  constexpr Vector4 Min(const Vector4& p) const {
    return {std::min(x, p.x), std::min(y, p.y), std::min(z, p.z),
            std::min(w, p.w)};
  }

  constexpr Vector4 Max(const Vector4& p) const {
    return {std::max(x, p.x), std::max(y, p.y), std::max(z, p.z),
            std::max(w, p.w)};
  }

  constexpr Vector4 Floor() const {
    return {std::floor(x), std::floor(y), std::floor(z), std::floor(w)};
  }

  constexpr Vector4 Ceil() const {
    return {std::ceil(x), std::ceil(y), std::ceil(z), std::ceil(w)};
  }

  constexpr Vector4 Round() const {
    return {std::round(x), std::round(y), std::round(z), std::round(w)};
  }

  constexpr Vector4 Lerp(const Vector4& v, Scalar t) const {
    return *this + (v - *this) * t;
  }

  constexpr Vector2 xy() const { return Vector2(x, y); }

  std::string ToString() const;
};

static_assert(sizeof(Vector3) == 3 * sizeof(Scalar));
static_assert(sizeof(Vector4) == 4 * sizeof(Scalar));

}  // namespace impeller

namespace std {

inline std::ostream& operator<<(std::ostream& out, const impeller::Vector3& p) {
  out << "(" << p.x << ", " << p.y << ", " << p.z << ")";
  return out;
}

inline std::ostream& operator<<(std::ostream& out, const impeller::Vector4& p) {
  out << "(" << p.x << ", " << p.y << ", " << p.z << ", " << p.w << ")";
  return out;
}

// NOLINTEND(google-explicit-constructor)

}  // namespace std

#endif  // FLUTTER_IMPELLER_GEOMETRY_VECTOR_H_
