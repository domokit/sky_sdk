// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "export.h"

#include "third_party/skia/include/core/SkData.h"

SKWASM_EXPORT SkData* skData_create(size_t size) {
  return SkData::MakeUninitialized(size).release();
}

SKWASM_EXPORT void* skData_getPointer(SkData* data) {
  return data->writable_data();
}

SKWASM_EXPORT const void* skData_getConstPointer(SkData* data) {
  return data->data();
}

SKWASM_EXPORT size_t skData_getSize(SkData* data) {
  return data->size();
}

SKWASM_EXPORT void skData_dispose(SkData* data) {
  return data->unref();
}
