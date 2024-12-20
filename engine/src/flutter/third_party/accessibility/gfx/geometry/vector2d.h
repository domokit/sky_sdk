// Copyright (c) 2012 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// Defines a simple integer vector class.  This class is used to indicate a
// distance in two dimensions between two points. Subtracting two points should
// produce a vector, and adding a vector to a point produces the point at the
// vector's distance from the original point.

#ifndef UI_GFX_GEOMETRY_VECTOR2D_H_
#define UI_GFX_GEOMETRY_VECTOR2D_H_

#include <cstdint>
#include <iosfwd>
#include <string>

#include "gfx/gfx_export.h"
#include "vector2d_f.h"

namespace gfx {

class GFX_EXPORT Vector2d {
 public:
  constexpr Vector2d() : x_(0), y_(0) {}
  constexpr Vector2d(int x, int y) : x_(x), y_(y) {}

  constexpr int x() const { return x_; }
  void set_x(int x) { x_ = x; }

  constexpr int y() const { return y_; }
  void set_y(int y) { y_ = y; }

  // True if both components of the vector are 0.
  bool IsZero() const;

  // Add the components of the |other| vector to the current vector.
  void Add(const Vector2d& other);
  // Subtract the components of the |other| vector from the current vector.
  void Subtract(const Vector2d& other);

  constexpr bool operator==(const Vector2d& other) const {
    return x_ == other.x_ && y_ == other.y_;
  }
  void operator+=(const Vector2d& other) { Add(other); }
  void operator-=(const Vector2d& other) { Subtract(other); }

  void SetToMin(const Vector2d& other) {
    x_ = x_ <= other.x_ ? x_ : other.x_;
    y_ = y_ <= other.y_ ? y_ : other.y_;
  }

  void SetToMax(const Vector2d& other) {
    x_ = x_ >= other.x_ ? x_ : other.x_;
    y_ = y_ >= other.y_ ? y_ : other.y_;
  }

  // Gives the square of the diagonal length of the vector. Since this is
  // cheaper to compute than Length(), it is useful when you want to compare
  // relative lengths of different vectors without needing the actual lengths.
  int64_t LengthSquared() const;
  // Gives the diagonal length of the vector.
  float Length() const;

  std::string ToString() const;

  operator Vector2dF() const {
    return Vector2dF(static_cast<float>(x()), static_cast<float>(y()));
  }

 private:
  int x_;
  int y_;
};

inline constexpr Vector2d operator-(const Vector2d& v) {
  return Vector2d(-v.x(), -v.y());
}

inline Vector2d operator+(const Vector2d& lhs, const Vector2d& rhs) {
  Vector2d result = lhs;
  result.Add(rhs);
  return result;
}

inline Vector2d operator-(const Vector2d& lhs, const Vector2d& rhs) {
  Vector2d result = lhs;
  result.Add(-rhs);
  return result;
}

// This is declared here for use in gtest-based unit tests but is defined in
// the //ui/gfx:test_support target. Depend on that to use this in your unit
// test. This should not be used in production code - call ToString() instead.
void PrintTo(const Vector2d& vector, ::std::ostream* os);

}  // namespace gfx

#endif  // UI_GFX_GEOMETRY_VECTOR2D_H_
