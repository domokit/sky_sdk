// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/lib/ui/window/pointer_data.h"

#include <cstring>

namespace flutter {

// The number of fields of PointerData.
//
// If kPointerDataFieldCount changes, update the corresponding values to:
//
//  * _kPointerDataFieldCount in platform_dispatcher.dart
//  * POINTER_DATA_FIELD_COUNT in AndroidTouchProcessor.java
//
// (This is a centralized list of all locations that should be kept up-to-date.)
static constexpr int kPointerDataFieldCount = 36;
static constexpr int kBytesPerField = sizeof(int64_t);

static_assert(sizeof(PointerData) == kBytesPerField * kPointerDataFieldCount,
              "PointerData has the wrong size");

void PointerData::Clear() {
  memset(this, 0, sizeof(PointerData));
}

}  // namespace flutter
