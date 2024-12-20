// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef LIB_TONIC_LOGGING_DART_INVOKE_H_
#define LIB_TONIC_LOGGING_DART_INVOKE_H_

#include <initializer_list>

#include "third_party/dart/runtime/include/dart_api.h"

namespace tonic {

Dart_Handle DartInvokeField(Dart_Handle target,
                            const char* name,
                            std::initializer_list<Dart_Handle> args);

Dart_Handle DartInvoke(Dart_Handle closure,
                       std::initializer_list<Dart_Handle> args);
Dart_Handle DartInvokeVoid(Dart_Handle closure);

}  // namespace tonic

#endif  // LIB_TONIC_LOGGING_DART_INVOKE_H_
