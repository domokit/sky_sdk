// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef LIB_TONIC_SCOPES_DART_ISOLATE_SCOPE_H_
#define LIB_TONIC_SCOPES_DART_ISOLATE_SCOPE_H_

#include "third_party/dart/runtime/include/dart_api.h"
#include "tonic/common/macros.h"

namespace tonic {

// DartIsolateScope is a helper class for entering and exiting a given isolate.
class DartIsolateScope {
 public:
  explicit DartIsolateScope(Dart_Isolate isolate);
  ~DartIsolateScope();

 private:
  Dart_Isolate isolate_;
  Dart_Isolate previous_;

  TONIC_DISALLOW_COPY_AND_ASSIGN(DartIsolateScope);
};

}  // namespace tonic

#endif  // LIB_TONIC_SCOPES_DART_ISOLATE_SCOPE_H_
