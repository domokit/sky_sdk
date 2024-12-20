// Copyright (c) 2012 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "size_conversions.h"

#include "base/numerics/safe_conversions.h"

namespace gfx {

Size ToFlooredSize(const SizeF& size) {
  return Size(base::ClampFloor(size.width()), base::ClampFloor(size.height()));
}

Size ToCeiledSize(const SizeF& size) {
  return Size(base::ClampCeil(size.width()), base::ClampCeil(size.height()));
}

Size ToRoundedSize(const SizeF& size) {
  return Size(base::ClampRound(size.width()), base::ClampRound(size.height()));
}

}  // namespace gfx
