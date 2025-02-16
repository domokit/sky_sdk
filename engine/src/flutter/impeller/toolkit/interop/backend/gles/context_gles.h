// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_TOOLKIT_INTEROP_BACKEND_GLES_CONTEXT_GLES_H_
#define FLUTTER_IMPELLER_TOOLKIT_INTEROP_BACKEND_GLES_CONTEXT_GLES_H_

#include <functional>
#include <memory>

#include "impeller/toolkit/interop/backend/gles/reactor_worker_gles.h"
#include "impeller/toolkit/interop/context.h"

namespace impeller::interop {

class ContextGLES final : public Context {
 public:
  static ScopedObject<Context> Create(
      std::function<void*(const char* gl_proc_name)> proc_address_callback);

  static ScopedObject<Context> Create(
      std::shared_ptr<impeller::Context> context,
      std::shared_ptr<ReactorWorkerGLES> worker = nullptr);

  ContextGLES();

  // |Context|
  ~ContextGLES() override;

  ContextGLES(const ContextGLES&) = delete;

  ContextGLES& operator=(const ContextGLES&) = delete;

 private:
  std::shared_ptr<ReactorWorkerGLES> worker_;

  ContextGLES(std::shared_ptr<impeller::Context> context,
              std::shared_ptr<ReactorWorkerGLES> worker);
};

}  // namespace impeller::interop

#endif  // FLUTTER_IMPELLER_TOOLKIT_INTEROP_BACKEND_GLES_CONTEXT_GLES_H_
