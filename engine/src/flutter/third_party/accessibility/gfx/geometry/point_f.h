// Copyright (c) 2012 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef UI_GFX_GEOMETRY_POINT_F_H_
#define UI_GFX_GEOMETRY_POINT_F_H_

#include <iosfwd>
#include <string>
#include <tuple>

#include "gfx/gfx_export.h"
#include "point.h"
#include "vector2d_f.h"

namespace gfx {

// A floating version of gfx::Point.
class GFX_EXPORT PointF {
 public:
  constexpr PointF() : x_(0.f), y_(0.f) {}
  constexpr PointF(float x, float y) : x_(x), y_(y) {}

  constexpr explicit PointF(const Point& p)
      : PointF(static_cast<float>(p.x()), static_cast<float>(p.y())) {}

  constexpr float x() const { return x_; }
  constexpr float y() const { return y_; }
  void set_x(float x) { x_ = x; }
  void set_y(float y) { y_ = y; }

  void SetPoint(float x, float y) {
    x_ = x;
    y_ = y;
  }

  void Offset(float delta_x, float delta_y) {
    x_ += delta_x;
    y_ += delta_y;
  }

  void operator+=(const Vector2dF& vector) {
    x_ += vector.x();
    y_ += vector.y();
  }

  void operator-=(const Vector2dF& vector) {
    x_ -= vector.x();
    y_ -= vector.y();
  }

  void SetToMin(const PointF& other);
  void SetToMax(const PointF& other);

  bool IsOrigin() const { return x_ == 0 && y_ == 0; }

  Vector2dF OffsetFromOrigin() const { return Vector2dF(x_, y_); }

  // A point is less than another point if its y-value is closer
  // to the origin. If the y-values are the same, then point with
  // the x-value closer to the origin is considered less than the
  // other.
  // This comparison is required to use PointF in sets, or sorted
  // vectors.
  bool operator<(const PointF& rhs) const {
    return std::tie(y_, x_) < std::tie(rhs.y_, rhs.x_);
  }

  void Scale(float scale) { Scale(scale, scale); }

  void Scale(float x_scale, float y_scale) {
    SetPoint(x() * x_scale, y() * y_scale);
  }

  // Returns a string representation of point.
  std::string ToString() const;

 private:
  float x_;
  float y_;
};

inline bool operator==(const PointF& lhs, const PointF& rhs) {
  return lhs.x() == rhs.x() && lhs.y() == rhs.y();
}

inline bool operator!=(const PointF& lhs, const PointF& rhs) {
  return !(lhs == rhs);
}

inline PointF operator+(const PointF& lhs, const Vector2dF& rhs) {
  PointF result(lhs);
  result += rhs;
  return result;
}

inline PointF operator-(const PointF& lhs, const Vector2dF& rhs) {
  PointF result(lhs);
  result -= rhs;
  return result;
}

inline Vector2dF operator-(const PointF& lhs, const PointF& rhs) {
  return Vector2dF(lhs.x() - rhs.x(), lhs.y() - rhs.y());
}

inline PointF PointAtOffsetFromOrigin(const Vector2dF& offset_from_origin) {
  return PointF(offset_from_origin.x(), offset_from_origin.y());
}

GFX_EXPORT PointF ScalePoint(const PointF& p, float x_scale, float y_scale);

inline PointF ScalePoint(const PointF& p, float scale) {
  return ScalePoint(p, scale, scale);
}

// This is declared here for use in gtest-based unit tests but is defined in
// the //ui/gfx:test_support target. Depend on that to use this in your unit
// test. This should not be used in production code - call ToString() instead.
void PrintTo(const PointF& point, ::std::ostream* os);

}  // namespace gfx

#endif  // UI_GFX_GEOMETRY_POINT_F_H_
