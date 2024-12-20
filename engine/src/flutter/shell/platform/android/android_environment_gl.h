// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_ANDROID_ANDROID_ENVIRONMENT_GL_H_
#define FLUTTER_SHELL_PLATFORM_ANDROID_ANDROID_ENVIRONMENT_GL_H_

#include "flutter/fml/macros.h"
#include "flutter/fml/memory/ref_counted.h"

#include <EGL/egl.h>

namespace flutter {

class AndroidEnvironmentGL
    : public fml::RefCountedThreadSafe<AndroidEnvironmentGL> {
 private:
  // MakeRefCounted
  AndroidEnvironmentGL();

  // MakeRefCounted
  ~AndroidEnvironmentGL();

 public:
  bool IsValid() const;

  EGLDisplay Display() const;

 private:
  EGLDisplay display_;
  bool valid_ = false;

  FML_FRIEND_MAKE_REF_COUNTED(AndroidEnvironmentGL);
  FML_FRIEND_REF_COUNTED_THREAD_SAFE(AndroidEnvironmentGL);
  FML_DISALLOW_COPY_AND_ASSIGN(AndroidEnvironmentGL);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_ANDROID_ANDROID_ENVIRONMENT_GL_H_
