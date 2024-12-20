// Copyright (c) 2012 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef UI_GFX_GEOMETRY_SIZE_CONVERSIONS_H_
#define UI_GFX_GEOMETRY_SIZE_CONVERSIONS_H_

#include "size.h"
#include "size_f.h"

namespace gfx {

// Returns a Size with each component from the input SizeF floored.
GFX_EXPORT Size ToFlooredSize(const SizeF& size);

// Returns a Size with each component from the input SizeF ceiled.
GFX_EXPORT Size ToCeiledSize(const SizeF& size);

// Returns a Size with each component from the input SizeF rounded.
GFX_EXPORT Size ToRoundedSize(const SizeF& size);

}  // namespace gfx

#endif  // UI_GFX_GEOMETRY_SIZE_CONVERSIONS_H_
