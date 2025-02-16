// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_TOOLKIT_INTEROP_BACKEND_GLES_REACTOR_WORKER_GLES_H_
#define FLUTTER_IMPELLER_TOOLKIT_INTEROP_BACKEND_GLES_REACTOR_WORKER_GLES_H_

#include "impeller/renderer/backend/gles/reactor_gles.h"

namespace impeller::interop {

class ReactorWorkerGLES final : public ReactorGLES::Worker {
 public:
  ReactorWorkerGLES();

  // |ReactorGLES::Worker|
  ~ReactorWorkerGLES() override;

  // |ReactorGLES::Worker|
  bool CanReactorReactOnCurrentThreadNow(
      const ReactorGLES& reactor) const override;

 private:
  std::thread::id thread_id_;

  ReactorWorkerGLES(const ReactorWorkerGLES&) = delete;

  ReactorWorkerGLES& operator=(const ReactorWorkerGLES&) = delete;
};

}  // namespace impeller::interop

#endif  // FLUTTER_IMPELLER_TOOLKIT_INTEROP_BACKEND_GLES_REACTOR_WORKER_GLES_H_
