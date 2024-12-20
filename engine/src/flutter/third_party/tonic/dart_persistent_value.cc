// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "tonic/dart_persistent_value.h"

#include "tonic/dart_state.h"
#include "tonic/scopes/dart_isolate_scope.h"

namespace tonic {

DartPersistentValue::DartPersistentValue() : value_(nullptr) {}

DartPersistentValue::DartPersistentValue(DartPersistentValue&& other)
    : dart_state_(other.dart_state_), value_(other.value_) {
  other.dart_state_.reset();
  other.value_ = nullptr;
}

DartPersistentValue::DartPersistentValue(DartState* dart_state,
                                         Dart_Handle value)
    : value_(nullptr) {
  Set(dart_state, value);
}

DartPersistentValue::~DartPersistentValue() {
  Clear();
}

void DartPersistentValue::Set(DartState* dart_state, Dart_Handle value) {
  TONIC_DCHECK(is_empty());
  dart_state_ = dart_state->GetWeakPtr();
  value_ = Dart_NewPersistentHandle(value);
}

void DartPersistentValue::Clear() {
  if (!value_) {
    return;
  }

  auto dart_state = dart_state_.lock();
  if (!dart_state) {
    // The Dart isolate was collected and the persistent value has been
    // collected with it. value_ is a dangling reference.
    value_ = nullptr;
    return;
  }

  /// TODO(80155): Remove the handle even if the isolate is shutting down.  This
  /// may cause memory to stick around until the isolate group is destroyed.
  /// Without this branch, if DartState::IsShuttingDown == true, this code will
  /// crash when binding the isolate.
  if (!dart_state->IsShuttingDown()) {
    if (Dart_CurrentIsolateGroup()) {
      Dart_DeletePersistentHandle(value_);
    } else {
      DartIsolateScope scope(dart_state->isolate());
      Dart_DeletePersistentHandle(value_);
    }
  }

  dart_state_.reset();
  value_ = nullptr;
}

Dart_Handle DartPersistentValue::Get() {
  if (!value_)
    return nullptr;
  return Dart_HandleFromPersistent(value_);
}

Dart_Handle DartPersistentValue::Release() {
  Dart_Handle local = Get();
  Clear();
  return local;
}
}  // namespace tonic
