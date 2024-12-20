// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_TOOLKIT_EGL_IMAGE_H_
#define FLUTTER_IMPELLER_TOOLKIT_EGL_IMAGE_H_

#include "flutter/fml/unique_object.h"
#include "flutter/impeller/toolkit/egl/egl.h"

namespace impeller {

// Simple holder of an EGLImage and the owning EGLDisplay.
struct EGLImageWithDisplay {
  EGLImage image = EGL_NO_IMAGE;
  EGLDisplay display = EGL_NO_DISPLAY;

  constexpr bool operator==(const EGLImageWithDisplay& other) const {
    return image == other.image && display == other.display;
  }

  constexpr bool operator!=(const EGLImageWithDisplay& other) const {
    return !(*this == other);
  }
};

struct EGLImageWithDisplayTraits {
  static EGLImageWithDisplay InvalidValue() {
    return {EGL_NO_IMAGE, EGL_NO_DISPLAY};
  }

  static bool IsValid(const EGLImageWithDisplay& value) {
    return value != InvalidValue();
  }

  static void Free(EGLImageWithDisplay image) {
    eglDestroyImage(image.display, image.image);
  }
};

using UniqueEGLImage =
    fml::UniqueObject<EGLImageWithDisplay, EGLImageWithDisplayTraits>;

// Simple holder of an EGLImageKHR and the owning EGLDisplay.
struct EGLImageKHRWithDisplay {
  EGLImageKHR image = EGL_NO_IMAGE_KHR;
  EGLDisplay display = EGL_NO_DISPLAY;

  constexpr bool operator==(const EGLImageKHRWithDisplay& other) const {
    return image == other.image && display == other.display;
  }

  constexpr bool operator!=(const EGLImageKHRWithDisplay& other) const {
    return !(*this == other);
  }
};

struct EGLImageKHRWithDisplayTraits {
  static EGLImageKHRWithDisplay InvalidValue() {
    return {EGL_NO_IMAGE_KHR, EGL_NO_DISPLAY};
  }

  static bool IsValid(const EGLImageKHRWithDisplay& value) {
    return value != InvalidValue();
  }

  static void Free(EGLImageKHRWithDisplay image) {
    eglDestroyImageKHR(image.display, image.image);
  }
};

using UniqueEGLImageKHR =
    fml::UniqueObject<EGLImageKHRWithDisplay, EGLImageKHRWithDisplayTraits>;

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_TOOLKIT_EGL_IMAGE_H_
