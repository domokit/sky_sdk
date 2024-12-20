// Copyright 2014 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "gfx/test/gfx_util.h"

#include <iomanip>
#include <sstream>
#include <string>

#include "gfx/geometry/point.h"
#include "gfx/geometry/point_f.h"
#include "gfx/geometry/rect.h"
#include "gfx/geometry/rect_f.h"
#include "gfx/geometry/size.h"
#include "gfx/geometry/size_f.h"
#include "gfx/geometry/vector2d.h"
#include "gfx/geometry/vector2d_f.h"
#include "gfx/transform.h"

namespace gfx {

namespace {

bool FloatAlmostEqual(float a, float b) {
  // FloatLE is the gtest predicate for less than or almost equal to.
  return ::testing::FloatLE("a", "b", a, b) &&
         ::testing::FloatLE("b", "a", b, a);
}

}  // namespace

::testing::AssertionResult AssertPointFloatEqual(const char* lhs_expr,
                                                 const char* rhs_expr,
                                                 const PointF& lhs,
                                                 const PointF& rhs) {
  if (FloatAlmostEqual(lhs.x(), rhs.x()) &&
      FloatAlmostEqual(lhs.y(), rhs.y())) {
    return ::testing::AssertionSuccess();
  }
  return ::testing::AssertionFailure()
         << "Value of: " << rhs_expr << "\n  Actual: " << rhs.ToString()
         << "\nExpected: " << lhs_expr << "\nWhich is: " << lhs.ToString();
}

::testing::AssertionResult AssertRectFloatEqual(const char* lhs_expr,
                                                const char* rhs_expr,
                                                const RectF& lhs,
                                                const RectF& rhs) {
  if (FloatAlmostEqual(lhs.x(), rhs.x()) &&
      FloatAlmostEqual(lhs.y(), rhs.y()) &&
      FloatAlmostEqual(lhs.width(), rhs.width()) &&
      FloatAlmostEqual(lhs.height(), rhs.height())) {
    return ::testing::AssertionSuccess();
  }
  return ::testing::AssertionFailure()
         << "Value of: " << rhs_expr << "\n  Actual: " << rhs.ToString()
         << "\nExpected: " << lhs_expr << "\nWhich is: " << lhs.ToString();
}

::testing::AssertionResult AssertSizeFFloatEqual(const char* lhs_expr,
                                                 const char* rhs_expr,
                                                 const SizeF& lhs,
                                                 const SizeF& rhs) {
  if (FloatAlmostEqual(lhs.width(), rhs.width()) &&
      FloatAlmostEqual(lhs.height(), rhs.height())) {
    return ::testing::AssertionSuccess();
  }
  return ::testing::AssertionFailure()
         << "Value of: " << rhs_expr << "\n  Actual: " << rhs.ToString()
         << "\nExpected: " << lhs_expr << "\nWhich is: " << lhs.ToString();
}

void PrintTo(const Point& point, ::std::ostream* os) {
  *os << point.ToString();
}

void PrintTo(const PointF& point, ::std::ostream* os) {
  *os << point.ToString();
}

void PrintTo(const Rect& rect, ::std::ostream* os) {
  *os << rect.ToString();
}

void PrintTo(const RectF& rect, ::std::ostream* os) {
  *os << rect.ToString();
}

void PrintTo(const Size& size, ::std::ostream* os) {
  *os << size.ToString();
}

void PrintTo(const SizeF& size, ::std::ostream* os) {
  *os << size.ToString();
}

void PrintTo(const Transform& transform, ::std::ostream* os) {
  *os << transform.ToString();
}

void PrintTo(const Vector2d& vector, ::std::ostream* os) {
  *os << vector.ToString();
}

void PrintTo(const Vector2dF& vector, ::std::ostream* os) {
  *os << vector.ToString();
}

}  // namespace gfx
