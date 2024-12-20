// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_FUCHSIA_DART_PKG_ZIRCON_FFI_CLOCK_H_
#define FLUTTER_SHELL_PLATFORM_FUCHSIA_DART_PKG_ZIRCON_FFI_CLOCK_H_

#include "macros.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

ZIRCON_FFI_EXPORT uint64_t zircon_dart_clock_get_monotonic();

#ifdef __cplusplus
}
#endif

#endif  // FLUTTER_SHELL_PLATFORM_FUCHSIA_DART_PKG_ZIRCON_FFI_CLOCK_H_
