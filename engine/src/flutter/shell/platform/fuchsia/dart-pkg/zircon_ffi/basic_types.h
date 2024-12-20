// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_FUCHSIA_DART_PKG_ZIRCON_FFI_BASIC_TYPES_H_
#define FLUTTER_SHELL_PLATFORM_FUCHSIA_DART_PKG_ZIRCON_FFI_BASIC_TYPES_H_

#include "macros.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct zircon_dart_byte_array_t {
  uint8_t* data;
  uint32_t length;
} zircon_dart_byte_array_t;

ZIRCON_FFI_EXPORT zircon_dart_byte_array_t* zircon_dart_byte_array_create(
    uint32_t size);

ZIRCON_FFI_EXPORT void zircon_dart_byte_array_set_value(
    zircon_dart_byte_array_t* arr,
    uint32_t index,
    uint8_t value);

ZIRCON_FFI_EXPORT void zircon_dart_byte_array_free(
    zircon_dart_byte_array_t* arr);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // FLUTTER_SHELL_PLATFORM_FUCHSIA_DART_PKG_ZIRCON_FFI_BASIC_TYPES_H_
