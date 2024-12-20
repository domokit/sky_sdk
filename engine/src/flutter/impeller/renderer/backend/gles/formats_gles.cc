// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/backend/gles/formats_gles.h"

namespace impeller {

std::string DebugToFramebufferError(int status) {
  switch (status) {
    case GL_FRAMEBUFFER_UNDEFINED:
      return "GL_FRAMEBUFFER_UNDEFINED";
    case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT:
      return "GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT";
    case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT:
      return "GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT";
    case GL_FRAMEBUFFER_UNSUPPORTED:
      return "GL_FRAMEBUFFER_UNSUPPORTED";
    case GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE:
      return "GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE";
    default:
      return "Unknown error code: " + std::to_string(status);
  }
}

}  // namespace impeller
