// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/fml/memory/weak_ptr_internal.h"

#include "flutter/fml/logging.h"

namespace fml {
namespace internal {

WeakPtrFlag::WeakPtrFlag() : is_valid_(true) {}

WeakPtrFlag::~WeakPtrFlag() {
  // Should be invalidated before destruction.
  FML_DCHECK(!is_valid_);
}

void WeakPtrFlag::Invalidate() {
  // Invalidation should happen exactly once.
  FML_DCHECK(is_valid_);
  is_valid_ = false;
}

}  // namespace internal
}  // namespace fml
