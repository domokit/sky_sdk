// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "tonic/dart_weak_persistent_value.h"

#include "tonic/dart_state.h"
#include "tonic/scopes/dart_isolate_scope.h"

namespace tonic {

DartWeakPersistentValue::DartWeakPersistentValue() : handle_(nullptr) {}

DartWeakPersistentValue::~DartWeakPersistentValue() {
  Clear();
}

void DartWeakPersistentValue::Set(DartState* dart_state,
                                  Dart_Handle object,
                                  void* peer,
                                  intptr_t external_allocation_size,
                                  Dart_HandleFinalizer callback) {
  TONIC_DCHECK(is_empty());
  dart_state_ = dart_state->GetWeakPtr();
  handle_ = Dart_NewWeakPersistentHandle(object, peer, external_allocation_size,
                                         callback);
}

void DartWeakPersistentValue::Clear() {
  if (!handle_) {
    return;
  }

  auto dart_state = dart_state_.lock();
  if (!dart_state) {
    // The DartVM that the handle used to belong to has been shut down and that
    // handle has already been deleted.
    handle_ = nullptr;
    return;
  }

  // The DartVM frees the handles during shutdown and calls the finalizers.
  // Freeing handles during shutdown would fail.
  if (!dart_state->IsShuttingDown()) {
    if (Dart_CurrentIsolateGroup()) {
      Dart_DeleteWeakPersistentHandle(handle_);
    } else {
      // If we are not on the mutator thread, this will fail. The caller must
      // ensure to be on the mutator thread.
      DartIsolateScope scope(dart_state->isolate());
      Dart_DeleteWeakPersistentHandle(handle_);
    }
  }
  // If it's shutting down, the handle will be deleted already.

  dart_state_.reset();
  handle_ = nullptr;
}

Dart_Handle DartWeakPersistentValue::Get() {
  auto dart_state = dart_state_.lock();
  TONIC_DCHECK(dart_state);
  TONIC_DCHECK(!dart_state->IsShuttingDown());
  if (!handle_) {
    return nullptr;
  }
  return Dart_HandleFromWeakPersistent(handle_);
}

}  // namespace tonic
