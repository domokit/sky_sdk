// Copyright (c) 2012 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef UI_GFX_GEOMETRY_VECTOR2D_CONVERSIONS_H_
#define UI_GFX_GEOMETRY_VECTOR2D_CONVERSIONS_H_

#include "vector2d.h"
#include "vector2d_f.h"

namespace gfx {

// Returns a Vector2d with each component from the input Vector2dF floored.
GFX_EXPORT Vector2d ToFlooredVector2d(const Vector2dF& vector2d);

// Returns a Vector2d with each component from the input Vector2dF ceiled.
GFX_EXPORT Vector2d ToCeiledVector2d(const Vector2dF& vector2d);

// Returns a Vector2d with each component from the input Vector2dF rounded.
GFX_EXPORT Vector2d ToRoundedVector2d(const Vector2dF& vector2d);

}  // namespace gfx

#endif  // UI_GFX_GEOMETRY_VECTOR2D_CONVERSIONS_H_
