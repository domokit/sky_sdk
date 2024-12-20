// Copyright (c) 2012 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef UI_GFX_GEOMETRY_POINT_H_
#define UI_GFX_GEOMETRY_POINT_H_

#include <iosfwd>
#include <string>
#include <tuple>

#include "ax_build/build_config.h"
#include "base/numerics/clamped_math.h"
#include "gfx/gfx_export.h"
#include "vector2d.h"

#if defined(OS_WIN)
typedef unsigned long DWORD;
typedef struct tagPOINT POINT;
#elif defined(OS_APPLE)
typedef struct CGPoint CGPoint;
#endif

namespace gfx {

// A point has an x and y coordinate.
class GFX_EXPORT Point {
 public:
  constexpr Point() : x_(0), y_(0) {}
  constexpr Point(int x, int y) : x_(x), y_(y) {}
#if defined(OS_WIN)
  // |point| is a DWORD value that contains a coordinate.  The x-coordinate is
  // the low-order short and the y-coordinate is the high-order short.  This
  // value is commonly acquired from GetMessagePos/GetCursorPos.
  explicit Point(DWORD point);
  explicit Point(const POINT& point);
  Point& operator=(const POINT& point);
#elif defined(OS_APPLE)
  explicit Point(const CGPoint& point);
#endif

#if defined(OS_WIN)
  POINT ToPOINT() const;
#elif defined(OS_APPLE)
  CGPoint ToCGPoint() const;
#endif

  constexpr int x() const { return x_; }
  constexpr int y() const { return y_; }
  void set_x(int x) { x_ = x; }
  void set_y(int y) { y_ = y; }

  void SetPoint(int x, int y) {
    x_ = x;
    y_ = y;
  }

  void Offset(int delta_x, int delta_y) {
    x_ = base::ClampAdd(x_, delta_x);
    y_ = base::ClampAdd(y_, delta_y);
  }

  void operator+=(const Vector2d& vector) {
    x_ = base::ClampAdd(x_, vector.x());
    y_ = base::ClampAdd(y_, vector.y());
  }

  void operator-=(const Vector2d& vector) {
    x_ = base::ClampSub(x_, vector.x());
    y_ = base::ClampSub(y_, vector.y());
  }

  void SetToMin(const Point& other);
  void SetToMax(const Point& other);

  bool IsOrigin() const { return x_ == 0 && y_ == 0; }

  Vector2d OffsetFromOrigin() const { return Vector2d(x_, y_); }

  // A point is less than another point if its y-value is closer
  // to the origin. If the y-values are the same, then point with
  // the x-value closer to the origin is considered less than the
  // other.
  // This comparison is required to use Point in sets, or sorted
  // vectors.
  bool operator<(const Point& rhs) const { return std::tie(y_, x_) < std::tie(rhs.y_, rhs.x_); }

  // Returns a string representation of point.
  std::string ToString() const;

 private:
  int x_;
  int y_;
};

inline bool operator==(const Point& lhs, const Point& rhs) {
  return lhs.x() == rhs.x() && lhs.y() == rhs.y();
}

inline bool operator!=(const Point& lhs, const Point& rhs) {
  return !(lhs == rhs);
}

inline Point operator+(const Point& lhs, const Vector2d& rhs) {
  Point result(lhs);
  result += rhs;
  return result;
}

inline Point operator-(const Point& lhs, const Vector2d& rhs) {
  Point result(lhs);
  result -= rhs;
  return result;
}

inline Vector2d operator-(const Point& lhs, const Point& rhs) {
  return Vector2d(base::ClampSub(lhs.x(), rhs.x()), base::ClampSub(lhs.y(), rhs.y()));
}

inline Point PointAtOffsetFromOrigin(const Vector2d& offset_from_origin) {
  return Point(offset_from_origin.x(), offset_from_origin.y());
}

// This is declared here for use in gtest-based unit tests but is defined in
// the //ui/gfx:test_support target. Depend on that to use this in your unit
// test. This should not be used in production code - call ToString() instead.
void PrintTo(const Point& point, ::std::ostream* os);

// Helper methods to scale a gfx::Point to a new gfx::Point.
GFX_EXPORT Point ScaleToCeiledPoint(const Point& point, float x_scale, float y_scale);
GFX_EXPORT Point ScaleToCeiledPoint(const Point& point, float x_scale);
GFX_EXPORT Point ScaleToFlooredPoint(const Point& point, float x_scale, float y_scale);
GFX_EXPORT Point ScaleToFlooredPoint(const Point& point, float x_scale);
GFX_EXPORT Point ScaleToRoundedPoint(const Point& point, float x_scale, float y_scale);
GFX_EXPORT Point ScaleToRoundedPoint(const Point& point, float x_scale);

}  // namespace gfx

#endif  // UI_GFX_GEOMETRY_POINT_H_
