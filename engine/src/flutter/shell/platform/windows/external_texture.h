// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_WINDOWS_EXTERNAL_TEXTURE_H_
#define FLUTTER_SHELL_PLATFORM_WINDOWS_EXTERNAL_TEXTURE_H_

#include "flutter/shell/platform/embedder/embedder.h"

#include <GLES2/gl2.h>
#include <GLES2/gl2ext.h>

namespace flutter {

// Abstract external texture.
class ExternalTexture {
 public:
  virtual ~ExternalTexture() = default;

  // Returns the unique id of this texture.
  int64_t texture_id() const { return reinterpret_cast<int64_t>(this); };

  // Attempts to populate the specified |opengl_texture| with texture details
  // such as the name, width, height and the pixel format.
  // Returns true on success.
  virtual bool PopulateTexture(size_t width,
                               size_t height,
                               FlutterOpenGLTexture* opengl_texture) = 0;
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_WINDOWS_EXTERNAL_TEXTURE_H_
