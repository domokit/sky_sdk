// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "basic_types.h"

#include <cstdint>
#include <cstdlib>

#include "flutter/fml/logging.h"

zircon_dart_byte_array_t* zircon_dart_byte_array_create(uint32_t size) {
  zircon_dart_byte_array_t* arr = static_cast<zircon_dart_byte_array_t*>(
      malloc(sizeof(zircon_dart_byte_array_t)));
  arr->length = size;
  arr->data = static_cast<uint8_t*>(malloc(size * sizeof(uint8_t)));
  return arr;
}

void zircon_dart_byte_array_set_value(zircon_dart_byte_array_t* arr,
                                      uint32_t index,
                                      uint8_t value) {
  FML_CHECK(arr);
  FML_CHECK(arr->length > index);
  arr->data[index] = value;
}

void zircon_dart_byte_array_free(zircon_dart_byte_array_t* arr) {
  FML_CHECK(arr);
  free(arr->data);
  free(arr);
}
