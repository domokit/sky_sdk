// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/backend/gles/handle_gles.h"

#include "flutter/fml/logging.h"

namespace impeller {

std::string HandleTypeToString(HandleType type) {
  switch (type) {
    case HandleType::kUnknown:
      return "Unknown";
    case HandleType::kTexture:
      return "Texture";
    case HandleType::kBuffer:
      return "Buffer";
    case HandleType::kProgram:
      return "Program";
    case HandleType::kRenderBuffer:
      return "RenderBuffer";
    case HandleType::kFrameBuffer:
      return "Framebuffer";
    case HandleType::kFence:
      return "Fence";
  }
  FML_UNREACHABLE();
}

}  // namespace impeller
