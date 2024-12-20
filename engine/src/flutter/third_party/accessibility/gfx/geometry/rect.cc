// Copyright (c) 2012 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "rect.h"

#include <algorithm>

#if defined(OS_WIN)
#include <windows.h>
#elif defined(OS_IOS)
#include <CoreGraphics/CoreGraphics.h>
#elif defined(OS_APPLE)
#include <ApplicationServices/ApplicationServices.h>
#endif

#include "ax_build/build_config.h"
#include "base/logging.h"
#include "base/numerics/clamped_math.h"
#include "base/string_utils.h"
#include "insets.h"

namespace gfx {

#if defined(OS_WIN)
Rect::Rect(const RECT& r)
    : origin_(r.left, r.top),
      size_(std::abs(r.right - r.left), std::abs(r.bottom - r.top)) {}
#elif defined(OS_APPLE)
Rect::Rect(const CGRect& r)
    : origin_(r.origin.x, r.origin.y), size_(r.size.width, r.size.height) {}
#endif

#if defined(OS_WIN)
RECT Rect::ToRECT() const {
  RECT r;
  r.left = x();
  r.right = right();
  r.top = y();
  r.bottom = bottom();
  return r;
}
#elif defined(OS_APPLE)
CGRect Rect::ToCGRect() const {
  return CGRectMake(x(), y(), width(), height());
}
#endif

void AdjustAlongAxis(int dst_origin, int dst_size, int* origin, int* size) {
  *size = std::min(dst_size, *size);
  if (*origin < dst_origin)
    *origin = dst_origin;
  else
    *origin = std::min(dst_origin + dst_size, *origin + *size) - *size;
}

}  // namespace gfx

namespace gfx {

// This is the per-axis heuristic for picking the most useful origin and
// width/height to represent the input range.
static void SaturatedClampRange(int min, int max, int* origin, int* span) {
  if (max < min) {
    *span = 0;
    *origin = min;
    return;
  }

  int effective_span = base::ClampSub(max, min);
  int span_loss = base::ClampSub(max, min + effective_span);

  // If the desired width is within the limits of ints, we can just
  // use the simple computations to represent the range precisely.
  if (span_loss == 0) {
    *span = effective_span;
    *origin = min;
    return;
  }

  // Now we have to approximate. If one of min or max is close enough
  // to zero we choose to represent that one precisely. The other side is
  // probably practically "infinite", so we move it.
  constexpr unsigned kMaxDimension = std::numeric_limits<int>::max() / 2;
  if (base::SafeUnsignedAbs(max) < kMaxDimension) {
    // Maintain origin + span == max.
    *span = effective_span;
    *origin = max - effective_span;
  } else if (base::SafeUnsignedAbs(min) < kMaxDimension) {
    // Maintain origin == min.
    *span = effective_span;
    *origin = min;
  } else {
    // Both are big, so keep the center.
    *span = effective_span;
    *origin = min + span_loss / 2;
  }
}

void Rect::SetByBounds(int left, int top, int right, int bottom) {
  int x, y;
  int width, height;
  SaturatedClampRange(left, right, &x, &width);
  SaturatedClampRange(top, bottom, &y, &height);
  origin_.SetPoint(x, y);
  size_.SetSize(width, height);
}

void Rect::Inset(const Insets& insets) {
  Inset(insets.left(), insets.top(), insets.right(), insets.bottom());
}

void Rect::Inset(int left, int top, int right, int bottom) {
  origin_ += Vector2d(left, top);
  // left+right might overflow/underflow, but width() - (left+right) might
  // overflow as well.
  set_width(base::ClampSub(width(), base::ClampAdd(left, right)));
  set_height(base::ClampSub(height(), base::ClampAdd(top, bottom)));
}

void Rect::Offset(int horizontal, int vertical) {
  origin_ += Vector2d(horizontal, vertical);
  // Ensure that width and height remain valid.
  set_width(width());
  set_height(height());
}

void Rect::operator+=(const Vector2d& offset) {
  origin_ += offset;
  // Ensure that width and height remain valid.
  set_width(width());
  set_height(height());
}

void Rect::operator-=(const Vector2d& offset) {
  origin_ -= offset;
}

Insets Rect::InsetsFrom(const Rect& inner) const {
  return Insets(inner.y() - y(), inner.x() - x(), bottom() - inner.bottom(),
                right() - inner.right());
}

bool Rect::operator<(const Rect& other) const {
  if (origin_ == other.origin_) {
    if (width() == other.width()) {
      return height() < other.height();
    } else {
      return width() < other.width();
    }
  } else {
    return origin_ < other.origin_;
  }
}

bool Rect::Contains(int point_x, int point_y) const {
  return (point_x >= x()) && (point_x < right()) && (point_y >= y()) &&
         (point_y < bottom());
}

bool Rect::Contains(const Rect& rect) const {
  return (rect.x() >= x() && rect.right() <= right() && rect.y() >= y() &&
          rect.bottom() <= bottom());
}

bool Rect::Intersects(const Rect& rect) const {
  return !(IsEmpty() || rect.IsEmpty() || rect.x() >= right() ||
           rect.right() <= x() || rect.y() >= bottom() || rect.bottom() <= y());
}

void Rect::Intersect(const Rect& rect) {
  if (IsEmpty() || rect.IsEmpty()) {
    SetRect(0, 0, 0, 0);  // Throws away empty position.
    return;
  }

  int left = std::max(x(), rect.x());
  int top = std::max(y(), rect.y());
  int new_right = std::min(right(), rect.right());
  int new_bottom = std::min(bottom(), rect.bottom());

  if (left >= new_right || top >= new_bottom) {
    SetRect(0, 0, 0, 0);  // Throws away empty position.
    return;
  }

  SetByBounds(left, top, new_right, new_bottom);
}

void Rect::Union(const Rect& rect) {
  if (IsEmpty()) {
    *this = rect;
    return;
  }
  if (rect.IsEmpty())
    return;

  SetByBounds(std::min(x(), rect.x()), std::min(y(), rect.y()),
              std::max(right(), rect.right()),
              std::max(bottom(), rect.bottom()));
}

void Rect::Subtract(const Rect& rect) {
  if (!Intersects(rect))
    return;
  if (rect.Contains(*this)) {
    SetRect(0, 0, 0, 0);
    return;
  }

  int rx = x();
  int ry = y();
  int rr = right();
  int rb = bottom();

  if (rect.y() <= y() && rect.bottom() >= bottom()) {
    // complete intersection in the y-direction
    if (rect.x() <= x()) {
      rx = rect.right();
    } else if (rect.right() >= right()) {
      rr = rect.x();
    }
  } else if (rect.x() <= x() && rect.right() >= right()) {
    // complete intersection in the x-direction
    if (rect.y() <= y()) {
      ry = rect.bottom();
    } else if (rect.bottom() >= bottom()) {
      rb = rect.y();
    }
  }
  SetByBounds(rx, ry, rr, rb);
}

void Rect::AdjustToFit(const Rect& rect) {
  int new_x = x();
  int new_y = y();
  int new_width = width();
  int new_height = height();
  AdjustAlongAxis(rect.x(), rect.width(), &new_x, &new_width);
  AdjustAlongAxis(rect.y(), rect.height(), &new_y, &new_height);
  SetRect(new_x, new_y, new_width, new_height);
}

Point Rect::CenterPoint() const {
  return Point(x() + width() / 2, y() + height() / 2);
}

void Rect::ClampToCenteredSize(const Size& size) {
  int new_width = std::min(width(), size.width());
  int new_height = std::min(height(), size.height());
  int new_x = x() + (width() - new_width) / 2;
  int new_y = y() + (height() - new_height) / 2;
  SetRect(new_x, new_y, new_width, new_height);
}

void Rect::Transpose() {
  SetRect(y(), x(), height(), width());
}

void Rect::SplitVertically(Rect* left_half, Rect* right_half) const {
  BASE_DCHECK(left_half);
  BASE_DCHECK(right_half);

  left_half->SetRect(x(), y(), width() / 2, height());
  right_half->SetRect(left_half->right(), y(), width() - left_half->width(),
                      height());
}

bool Rect::SharesEdgeWith(const Rect& rect) const {
  return (y() == rect.y() && height() == rect.height() &&
          (x() == rect.right() || right() == rect.x())) ||
         (x() == rect.x() && width() == rect.width() &&
          (y() == rect.bottom() || bottom() == rect.y()));
}

int Rect::ManhattanDistanceToPoint(const Point& point) const {
  int x_distance =
      std::max<int>(0, std::max(x() - point.x(), point.x() - right()));
  int y_distance =
      std::max<int>(0, std::max(y() - point.y(), point.y() - bottom()));

  return x_distance + y_distance;
}

int Rect::ManhattanInternalDistance(const Rect& rect) const {
  Rect c(*this);
  c.Union(rect);

  int x = std::max(0, c.width() - width() - rect.width() + 1);
  int y = std::max(0, c.height() - height() - rect.height() + 1);
  return x + y;
}

std::string Rect::ToString() const {
  return base::StringPrintf("%s %s", origin().ToString().c_str(),
                            size().ToString().c_str());
}

bool Rect::ApproximatelyEqual(const Rect& rect, int tolerance) const {
  return std::abs(x() - rect.x()) <= tolerance &&
         std::abs(y() - rect.y()) <= tolerance &&
         std::abs(right() - rect.right()) <= tolerance &&
         std::abs(bottom() - rect.bottom()) <= tolerance;
}

Rect operator+(const Rect& lhs, const Vector2d& rhs) {
  Rect result(lhs);
  result += rhs;
  return result;
}

Rect operator-(const Rect& lhs, const Vector2d& rhs) {
  Rect result(lhs);
  result -= rhs;
  return result;
}

Rect IntersectRects(const Rect& a, const Rect& b) {
  Rect result = a;
  result.Intersect(b);
  return result;
}

Rect UnionRects(const Rect& a, const Rect& b) {
  Rect result = a;
  result.Union(b);
  return result;
}

Rect SubtractRects(const Rect& a, const Rect& b) {
  Rect result = a;
  result.Subtract(b);
  return result;
}

Rect BoundingRect(const Point& p1, const Point& p2) {
  Rect result;
  result.SetByBounds(std::min(p1.x(), p2.x()), std::min(p1.y(), p2.y()),
                     std::max(p1.x(), p2.x()), std::max(p1.y(), p2.y()));
  return result;
}

}  // namespace gfx
