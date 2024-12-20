// Copyright (c) 2012 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef UI_GFX_GEOMETRY_POINT_CONVERSIONS_H_
#define UI_GFX_GEOMETRY_POINT_CONVERSIONS_H_

#include "point.h"
#include "point_f.h"

namespace gfx {

// Returns a Point with each component from the input PointF floored.
GFX_EXPORT Point ToFlooredPoint(const PointF& point);

// Returns a Point with each component from the input PointF ceiled.
GFX_EXPORT Point ToCeiledPoint(const PointF& point);

// Returns a Point with each component from the input PointF rounded.
GFX_EXPORT Point ToRoundedPoint(const PointF& point);

}  // namespace gfx

#endif  // UI_GFX_GEOMETRY_POINT_CONVERSIONS_H_
