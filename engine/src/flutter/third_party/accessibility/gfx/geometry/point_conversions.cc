// Copyright (c) 2012 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "point_conversions.h"

#include "base/numerics/safe_conversions.h"

namespace gfx {

Point ToFlooredPoint(const PointF& point) {
  return Point(base::ClampFloor(point.x()), base::ClampFloor(point.y()));
}

Point ToCeiledPoint(const PointF& point) {
  return Point(base::ClampCeil(point.x()), base::ClampCeil(point.y()));
}

Point ToRoundedPoint(const PointF& point) {
  return Point(base::ClampRound(point.x()), base::ClampRound(point.y()));
}

}  // namespace gfx
