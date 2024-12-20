// Copyright (c) 2012 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "point.h"

#include "ax_build/build_config.h"
#include "base/string_utils.h"
#include "point_conversions.h"
#include "point_f.h"

#if defined(OS_WIN)
#include <windows.h>
#elif defined(OS_IOS)
#include <CoreGraphics/CoreGraphics.h>
#elif defined(OS_APPLE)
#include <ApplicationServices/ApplicationServices.h>
#endif

namespace gfx {

#if defined(OS_WIN)
Point::Point(DWORD point) {
  POINTS points = MAKEPOINTS(point);
  x_ = points.x;
  y_ = points.y;
}

Point::Point(const POINT& point) : x_(point.x), y_(point.y) {}

Point& Point::operator=(const POINT& point) {
  x_ = point.x;
  y_ = point.y;
  return *this;
}
#elif defined(OS_APPLE)
Point::Point(const CGPoint& point) : x_(point.x), y_(point.y) {}
#endif

#if defined(OS_WIN)
POINT Point::ToPOINT() const {
  POINT p;
  p.x = x();
  p.y = y();
  return p;
}
#elif defined(OS_APPLE)
CGPoint Point::ToCGPoint() const {
  return CGPointMake(x(), y());
}
#endif

void Point::SetToMin(const Point& other) {
  x_ = x_ <= other.x_ ? x_ : other.x_;
  y_ = y_ <= other.y_ ? y_ : other.y_;
}

void Point::SetToMax(const Point& other) {
  x_ = x_ >= other.x_ ? x_ : other.x_;
  y_ = y_ >= other.y_ ? y_ : other.y_;
}

std::string Point::ToString() const {
  return base::StringPrintf("%d,%d", x(), y());
}

Point ScaleToCeiledPoint(const Point& point, float x_scale, float y_scale) {
  if (x_scale == 1.f && y_scale == 1.f)
    return point;
  return ToCeiledPoint(ScalePoint(gfx::PointF(point), x_scale, y_scale));
}

Point ScaleToCeiledPoint(const Point& point, float scale) {
  if (scale == 1.f)
    return point;
  return ToCeiledPoint(ScalePoint(gfx::PointF(point), scale, scale));
}

Point ScaleToFlooredPoint(const Point& point, float x_scale, float y_scale) {
  if (x_scale == 1.f && y_scale == 1.f)
    return point;
  return ToFlooredPoint(ScalePoint(gfx::PointF(point), x_scale, y_scale));
}

Point ScaleToFlooredPoint(const Point& point, float scale) {
  if (scale == 1.f)
    return point;
  return ToFlooredPoint(ScalePoint(gfx::PointF(point), scale, scale));
}

Point ScaleToRoundedPoint(const Point& point, float x_scale, float y_scale) {
  if (x_scale == 1.f && y_scale == 1.f)
    return point;
  return ToRoundedPoint(ScalePoint(gfx::PointF(point), x_scale, y_scale));
}

Point ScaleToRoundedPoint(const Point& point, float scale) {
  if (scale == 1.f)
    return point;
  return ToRoundedPoint(ScalePoint(gfx::PointF(point), scale, scale));
}

}  // namespace gfx
