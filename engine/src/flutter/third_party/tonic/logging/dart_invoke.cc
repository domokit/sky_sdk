// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "tonic/logging/dart_invoke.h"

#include "tonic/common/macros.h"
#include "tonic/logging/dart_error.h"

namespace tonic {

Dart_Handle DartInvokeField(Dart_Handle target,
                            const char* name,
                            std::initializer_list<Dart_Handle> args) {
  Dart_Handle field = Dart_NewStringFromCString(name);
  return Dart_Invoke(target, field, args.size(),
                     const_cast<Dart_Handle*>(args.begin()));
}

Dart_Handle DartInvoke(Dart_Handle closure,
                       std::initializer_list<Dart_Handle> args) {
  int argc = args.size();
  Dart_Handle* argv = const_cast<Dart_Handle*>(args.begin());
  Dart_Handle handle = Dart_InvokeClosure(closure, argc, argv);
  CheckAndHandleError(handle);
  return handle;
}

Dart_Handle DartInvokeVoid(Dart_Handle closure) {
  Dart_Handle handle = Dart_InvokeClosure(closure, 0, nullptr);
  CheckAndHandleError(handle);
  return handle;
}

}  // namespace tonic
