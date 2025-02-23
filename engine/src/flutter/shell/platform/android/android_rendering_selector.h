// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_ANDROID_ANDROID_RENDERING_SELECTOR_H_
#define FLUTTER_SHELL_PLATFORM_ANDROID_ANDROID_RENDERING_SELECTOR_H_

namespace flutter {

// The combination of targeted graphics API and Impeller support.
enum class AndroidRenderingAPI {
  kSoftware,
  kImpellerOpenGLES,
  kImpellerVulkan,
  kSkiaOpenGLES
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_ANDROID_ANDROID_RENDERING_SELECTOR_H_
