// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_TOOLKIT_GLES_TEXTURE_H_
#define FLUTTER_IMPELLER_TOOLKIT_GLES_TEXTURE_H_

#include "flutter/fml/unique_object.h"
#include "flutter/impeller/toolkit/gles/gles.h"

namespace impeller {

// Simple holder of an GLTexture and the owning EGLDisplay.
struct GLTexture {
  GLuint texture_name;

  constexpr bool operator==(const GLTexture& other) const {
    return texture_name == other.texture_name;
  }

  constexpr bool operator!=(const GLTexture& other) const {
    return !(*this == other);
  }
};

struct GLTextureTraits {
  static GLTexture InvalidValue() { return {0}; }

  static bool IsValid(const GLTexture& value) {
    return value != InvalidValue();
  }

  static void Free(GLTexture image) {
    glDeleteTextures(1, &image.texture_name);
  }
};

using UniqueGLTexture = fml::UniqueObject<GLTexture, GLTextureTraits>;

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_TOOLKIT_GLES_TEXTURE_H_
