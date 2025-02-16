// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/toolkit/interop/backend/gles/reactor_worker_gles.h"

namespace impeller::interop {

ReactorWorkerGLES::ReactorWorkerGLES()
    : thread_id_(std::this_thread::get_id()) {}

ReactorWorkerGLES::~ReactorWorkerGLES() = default;

bool ReactorWorkerGLES::CanReactorReactOnCurrentThreadNow(
    const ReactorGLES& reactor) const {
  return thread_id_ == std::this_thread::get_id();
}

}  // namespace impeller::interop
