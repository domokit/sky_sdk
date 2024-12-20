// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/flow/paint_region.h"

namespace flutter {

DlRect PaintRegion::ComputeBounds() const {
  DlRect res;
  for (const auto& r : *this) {
    res = res.Union(r);
  }
  return res;
}

}  // namespace flutter
