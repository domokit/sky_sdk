// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef LIB_TONIC_DART_CLASS_PROVIDER_H_
#define LIB_TONIC_DART_CLASS_PROVIDER_H_

#include "third_party/dart/runtime/include/dart_api.h"
#include "tonic/dart_persistent_value.h"

namespace tonic {
class DartState;

class DartClassProvider {
 public:
  DartClassProvider(DartState* dart_state, const char* library_name);
  ~DartClassProvider();

  Dart_Handle GetClassByName(const char* class_name);

 private:
  DartPersistentValue library_;

  TONIC_DISALLOW_COPY_AND_ASSIGN(DartClassProvider);
};

}  // namespace tonic

#endif  // LIB_TONIC_DART_CLASS_PROVIDER_H_
