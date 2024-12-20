// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "handle.h"

#include "flutter/fml/logging.h"

#include <iostream>
#include <vector>

#include <zircon/syscalls.h>

static void HandleFree(void* isolate_callback_data, void* peer) {
  FML_CHECK(peer);
  zircon_dart_handle_t* handle = reinterpret_cast<zircon_dart_handle_t*>(peer);
  zircon_dart_handle_free(handle);
}

static void HandlePairFree(void* isolate_callback_data, void* peer) {
  FML_CHECK(peer);
  zircon_dart_handle_pair_t* handle_pair =
      reinterpret_cast<zircon_dart_handle_pair_t*>(peer);
  free(handle_pair);
}

void zircon_dart_handle_free(zircon_dart_handle_t* handle) {
  FML_CHECK(handle);
  if (handle->handle != ZX_HANDLE_INVALID) {
    zircon_dart_handle_close(handle);
  }
  free(handle);
}

int32_t zircon_dart_handle_close(zircon_dart_handle_t* handle) {
  FML_CHECK(handle->handle != ZX_HANDLE_INVALID);
  zx_status_t status = zx_handle_close(handle->handle);
  handle->handle = ZX_HANDLE_INVALID;
  if (status == ZX_OK) {
    return 1;
  } else {
    return 0;
  }
}

int32_t zircon_dart_handle_is_valid(zircon_dart_handle_t* handle) {
  if (!handle || (handle->handle == ZX_HANDLE_INVALID)) {
    return 0;
  } else {
    return 1;
  }
}

int zircon_dart_handle_attach_finalizer(Dart_Handle object,
                                        void* pointer,
                                        intptr_t external_allocation_size) {
  Dart_FinalizableHandle weak_handle = Dart_NewFinalizableHandle_DL(
      object, pointer, external_allocation_size, HandleFree);

  if (weak_handle == nullptr) {
    FML_LOG(ERROR) << "Unable to attach finalizer: " << std::hex << pointer;
    return -1;
  }

  return 1;
}

int zircon_dart_handle_pair_attach_finalizer(
    Dart_Handle object,
    void* pointer,
    intptr_t external_allocation_size) {
  Dart_FinalizableHandle weak_handle = Dart_NewFinalizableHandle_DL(
      object, pointer, external_allocation_size, HandlePairFree);

  if (weak_handle == nullptr) {
    FML_LOG(ERROR) << "Unable to attach finalizer: " << std::hex << pointer;
    return -1;
  }

  return 1;
}

// zircon handle list methods.
using HandleVector = std::vector<zircon_dart_handle_t*>;
using HandleVectorPtr = HandleVector*;

zircon_dart_handle_list_t* zircon_dart_handle_list_create() {
  zircon_dart_handle_list_t* result = static_cast<zircon_dart_handle_list_t*>(
      malloc(sizeof(zircon_dart_handle_list_t)));
  result->size = 0;
  result->data = new HandleVector();
  return result;
}

void zircon_dart_handle_list_append(zircon_dart_handle_list_t* list,
                                    zircon_dart_handle_t* handle) {
  FML_CHECK(list);
  FML_CHECK(handle);
  list->size++;
  auto data = reinterpret_cast<HandleVectorPtr>(list->data);
  data->push_back(handle);
}

void zircon_dart_handle_list_free(zircon_dart_handle_list_t* list) {
  auto data = reinterpret_cast<HandleVectorPtr>(list->data);
  data->clear();
  delete data;
  free(list);
}
