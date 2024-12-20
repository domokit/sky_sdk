// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "clock.h"

#include <zircon/syscalls.h>

uint64_t zircon_dart_clock_get_monotonic() {
  return zx_clock_get_monotonic();
}
