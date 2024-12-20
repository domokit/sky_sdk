// Copyright (c) 2009 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "insets.h"

#include "base/string_utils.h"
#include "vector2d.h"

namespace gfx {

std::string Insets::ToString() const {
  // Print members in the same order of the constructor parameters.
  return base::StringPrintf("%d,%d,%d,%d", top(), left(), bottom(), right());
}

Insets Insets::Offset(const gfx::Vector2d& vector) const {
  return gfx::Insets(base::ClampAdd(top(), vector.y()),
                     base::ClampAdd(left(), vector.x()),
                     base::ClampSub(bottom(), vector.y()),
                     base::ClampSub(right(), vector.x()));
}

}  // namespace gfx
