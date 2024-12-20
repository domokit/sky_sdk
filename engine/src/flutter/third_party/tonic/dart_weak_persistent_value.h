// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef LIB_TONIC_DART_WEAK_PERSISTENT_VALUE_H_
#define LIB_TONIC_DART_WEAK_PERSISTENT_VALUE_H_

#include <memory>

#include "third_party/dart/runtime/include/dart_api.h"
#include "tonic/common/macros.h"

namespace tonic {
class DartState;

// DartWeakPersistentValue is a bookkeeping class to help pair calls to
// Dart_NewWeakPersistentHandle with Dart_DeleteWeakPersistentHandle even in
// the case if IsolateGroup shutdown. Consider using this class instead of
// holding a Dart_PersistentHandle directly so that you don't leak the
// Dart_WeakPersistentHandle.
class DartWeakPersistentValue {
 public:
  DartWeakPersistentValue();
  ~DartWeakPersistentValue();

  Dart_WeakPersistentHandle value() const { return handle_; }
  bool is_empty() const { return handle_ == nullptr; }

  void Set(DartState* dart_state,
           Dart_Handle object,
           void* peer,
           intptr_t external_allocation_size,
           Dart_HandleFinalizer callback);
  void Clear();
  Dart_Handle Get();

  const std::weak_ptr<DartState>& dart_state() const { return dart_state_; }

 private:
  std::weak_ptr<DartState> dart_state_;
  Dart_WeakPersistentHandle handle_;

  TONIC_DISALLOW_COPY_AND_ASSIGN(DartWeakPersistentValue);
};

}  // namespace tonic

#endif  // LIB_TONIC_DART_WEAK_PERSISTENT_VALUE_H_
