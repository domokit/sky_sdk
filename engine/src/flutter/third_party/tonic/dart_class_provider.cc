// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "tonic/dart_class_provider.h"

#include "tonic/converter/dart_converter.h"
#include "tonic/dart_state.h"
#include "tonic/logging/dart_error.h"

namespace tonic {

DartClassProvider::DartClassProvider(DartState* dart_state,
                                     const char* class_name) {
  library_.Set(dart_state, Dart_LookupLibrary(ToDart(class_name)));
}

DartClassProvider::~DartClassProvider() {}

Dart_Handle DartClassProvider::GetClassByName(const char* class_name) {
  Dart_Handle name_handle = ToDart(class_name);
  Dart_Handle class_handle =
      Dart_GetNonNullableType(library_.value(), name_handle, 0, nullptr);
  TONIC_DCHECK(!Dart_IsError(class_handle));
  return class_handle;
}

}  // namespace tonic
