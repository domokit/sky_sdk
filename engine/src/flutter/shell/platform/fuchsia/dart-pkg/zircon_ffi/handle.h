// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_FUCHSIA_DART_PKG_ZIRCON_FFI_HANDLE_H_
#define FLUTTER_SHELL_PLATFORM_FUCHSIA_DART_PKG_ZIRCON_FFI_HANDLE_H_

#include "macros.h"

#include "include/dart_api_dl.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct zircon_dart_handle_t {
  uint32_t handle;
} zircon_dart_handle_t;

typedef struct zircon_dart_handle_pair_t {
  zircon_dart_handle_t* left;
  zircon_dart_handle_t* right;
} zircon_dart_handle_pair_t;

typedef struct zircon_dart_handle_list_t {
  // data is of type `std::vector<zircon_handle_t*>*`.
  void* data;
  uint32_t size;
} zircon_dart_handle_list_t;

// Creates a list.
ZIRCON_FFI_EXPORT zircon_dart_handle_list_t* zircon_dart_handle_list_create();

// Appends to the list.
ZIRCON_FFI_EXPORT void zircon_dart_handle_list_append(
    zircon_dart_handle_list_t* list,
    zircon_dart_handle_t* handle);

// Frees the list, all the handles passed here must have been released.
ZIRCON_FFI_EXPORT void zircon_dart_handle_list_free(
    zircon_dart_handle_list_t* list);

// Returns 1 if the handle is valid.
ZIRCON_FFI_EXPORT int32_t zircon_dart_handle_is_valid(
    zircon_dart_handle_t* handle);

// Closes the handle, but doesn't release any ffi-associated memory. Returns 1
// on success.
ZIRCON_FFI_EXPORT int32_t zircon_dart_handle_close(
    zircon_dart_handle_t* handle);

// Closes the zircon handle if valid and frees the memory.
ZIRCON_FFI_EXPORT void zircon_dart_handle_free(zircon_dart_handle_t* handle);

// Attach a finalizer for pointer to object, such that `finalizer(pointer)` will
// be called when `object` is collected by the Dart garbage collector.
//
// The external_allocation_size is used by the Dart garbage collector as a hint
// about the size of the external allocation.
//
// Returns 1 on success.

ZIRCON_FFI_EXPORT int zircon_dart_handle_pair_attach_finalizer(
    Dart_Handle object,
    void* pointer,
    intptr_t external_allocation_size);

ZIRCON_FFI_EXPORT int zircon_dart_handle_attach_finalizer(
    Dart_Handle object,
    void* pointer,
    intptr_t external_allocation_size);

// ZIRCON_FFI_EXPORT zircon_dart_handle_t* zircon_dart_duplicate_handle(
//     zircon_dart_handle_t* handle,
//     uint32_t rights);

#ifdef __cplusplus
}
#endif

#endif  // FLUTTER_SHELL_PLATFORM_FUCHSIA_DART_PKG_ZIRCON_FFI_HANDLE_H_
