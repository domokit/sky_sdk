// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BACKEND_GLES_UNIQUE_HANDLE_GLES_H_
#define FLUTTER_IMPELLER_RENDERER_BACKEND_GLES_UNIQUE_HANDLE_GLES_H_

#include "impeller/renderer/backend/gles/handle_gles.h"
#include "impeller/renderer/backend/gles/reactor_gles.h"

namespace impeller {

//------------------------------------------------------------------------------
/// @brief      A unique handle to an OpenGL object. The collection of this
///             handle scheduled the destruction of the associated OpenGL object
///             in the reactor.
///
class UniqueHandleGLES {
 public:
  UniqueHandleGLES(std::shared_ptr<ReactorGLES> reactor, HandleType type);

  static UniqueHandleGLES MakeUntracked(std::shared_ptr<ReactorGLES> reactor,
                                        HandleType type);

  UniqueHandleGLES(std::shared_ptr<ReactorGLES> reactor, HandleGLES handle);

  ~UniqueHandleGLES();

  UniqueHandleGLES(UniqueHandleGLES&&);

  UniqueHandleGLES(const UniqueHandleGLES&) = delete;

  UniqueHandleGLES& operator=(const UniqueHandleGLES&) = delete;

  const HandleGLES& Get() const;

  bool IsValid() const;

 private:
  std::shared_ptr<ReactorGLES> reactor_ = nullptr;
  HandleGLES handle_ = HandleGLES::DeadHandle();
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_BACKEND_GLES_UNIQUE_HANDLE_GLES_H_
