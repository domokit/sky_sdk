// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_FUCHSIA_DART_PKG_ZIRCON_FFI_CHANNEL_H_
#define FLUTTER_SHELL_PLATFORM_FUCHSIA_DART_PKG_ZIRCON_FFI_CHANNEL_H_

#include "basic_types.h"
#include "handle.h"
#include "macros.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

ZIRCON_FFI_EXPORT zircon_dart_handle_pair_t* zircon_dart_channel_create(
    uint32_t options);

ZIRCON_FFI_EXPORT int32_t zircon_dart_channel_write(
    zircon_dart_handle_t* handle,
    zircon_dart_byte_array_t* bytes,
    zircon_dart_handle_list_t* handles);

#ifdef __cplusplus
}
#endif

#endif  // FLUTTER_SHELL_PLATFORM_FUCHSIA_DART_PKG_ZIRCON_FFI_CHANNEL_H_
