// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_ANDROID_ANDROID_CONTEXT_GL_IMPELLER_H_
#define FLUTTER_SHELL_PLATFORM_ANDROID_ANDROID_CONTEXT_GL_IMPELLER_H_

#include "flutter/fml/macros.h"
#include "flutter/impeller/toolkit/egl/display.h"
#include "flutter/shell/platform/android/context/android_context.h"

namespace flutter {

class AndroidContextGLImpeller : public AndroidContext {
 public:
  AndroidContextGLImpeller(std::unique_ptr<impeller::egl::Display> display,
                           bool enable_gpu_tracing);

  ~AndroidContextGLImpeller();

  // |AndroidContext|
  bool IsValid() const override;

  bool ResourceContextMakeCurrent(impeller::egl::Surface* offscreen_surface);
  bool ResourceContextClearCurrent();
  std::unique_ptr<impeller::egl::Surface> CreateOffscreenSurface();
  bool OnscreenContextMakeCurrent(impeller::egl::Surface* onscreen_surface);
  bool OnscreenContextClearCurrent();
  std::unique_ptr<impeller::egl::Surface> CreateOnscreenSurface(
      EGLNativeWindowType window);

 private:
  class ReactorWorker;

  std::shared_ptr<ReactorWorker> reactor_worker_;
  std::unique_ptr<impeller::egl::Display> display_;
  std::unique_ptr<impeller::egl::Config> onscreen_config_;
  std::unique_ptr<impeller::egl::Config> offscreen_config_;
  std::unique_ptr<impeller::egl::Context> onscreen_context_;
  std::unique_ptr<impeller::egl::Context> offscreen_context_;
  bool is_valid_ = false;

  FML_DISALLOW_COPY_AND_ASSIGN(AndroidContextGLImpeller);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_ANDROID_ANDROID_CONTEXT_GL_IMPELLER_H_
