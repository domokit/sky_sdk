// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/android/android_context_gl_skia.h"

#include <utility>

#include "flutter/fml/trace_event.h"
#include "flutter/shell/platform/android/android_egl_surface.h"

namespace flutter {

template <class T>
using EGLResult = std::pair<bool, T>;

static EGLResult<EGLContext> CreateContext(EGLDisplay display,
                                           EGLConfig config,
                                           EGLContext share = EGL_NO_CONTEXT) {
  EGLint attributes[] = {EGL_CONTEXT_CLIENT_VERSION, 2, EGL_NONE};

  EGLContext context = eglCreateContext(display, config, share, attributes);

  return {context != EGL_NO_CONTEXT, context};
}

static EGLResult<EGLConfig> ChooseEGLConfiguration(EGLDisplay display) {
  EGLint attributes[] = {
      // clang-format off
      EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT,
      EGL_SURFACE_TYPE,    EGL_WINDOW_BIT,
      EGL_RED_SIZE,        8,
      EGL_GREEN_SIZE,      8,
      EGL_BLUE_SIZE,       8,
      EGL_ALPHA_SIZE,      8,
      EGL_DEPTH_SIZE,      0,
      EGL_STENCIL_SIZE,    0,
      EGL_NONE,            // termination sentinel
      // clang-format on
  };

  EGLint config_count = 0;
  EGLConfig egl_config = nullptr;

  if (eglChooseConfig(display, attributes, &egl_config, 1, &config_count) !=
      EGL_TRUE) {
    return {false, nullptr};
  }

  bool success = config_count > 0 && egl_config != nullptr;

  return {success, success ? egl_config : nullptr};
}

static bool TeardownContext(EGLDisplay display, EGLContext context) {
  if (context != EGL_NO_CONTEXT) {
    return eglDestroyContext(display, context) == EGL_TRUE;
  }

  return true;
}

AndroidContextGLSkia::AndroidContextGLSkia(
    fml::RefPtr<AndroidEnvironmentGL> environment,
    const TaskRunners& task_runners)
    : AndroidContext(AndroidRenderingAPI::kSkiaOpenGLES),
      environment_(std::move(environment)),
      task_runners_(task_runners) {
  if (!environment_->IsValid()) {
    FML_LOG(ERROR) << "Could not create an Android GL environment.";
    return;
  }

  bool success = false;

  // Choose a valid configuration.
  std::tie(success, config_) = ChooseEGLConfiguration(environment_->Display());
  if (!success) {
    FML_LOG(ERROR) << "Could not choose an EGL configuration.";
    LogLastEGLError();
    return;
  }

  // Create a context for the configuration.
  std::tie(success, context_) =
      CreateContext(environment_->Display(), config_, EGL_NO_CONTEXT);
  if (!success) {
    FML_LOG(ERROR) << "Could not create an EGL context";
    LogLastEGLError();
    return;
  }

  std::tie(success, resource_context_) =
      CreateContext(environment_->Display(), config_, context_);
  if (!success) {
    FML_LOG(ERROR) << "Could not create an EGL resource context";
    LogLastEGLError();
    return;
  }

  // All done!
  valid_ = true;
}

AndroidContextGLSkia::~AndroidContextGLSkia() {
  FML_DCHECK(task_runners_.GetPlatformTaskRunner()->RunsTasksOnCurrentThread());
  sk_sp<GrDirectContext> main_context = GetMainSkiaContext();
  SetMainSkiaContext(nullptr);
  fml::AutoResetWaitableEvent latch;
  // This context needs to be deallocated from the raster thread in order to
  // keep a coherent usage of egl from a single thread.
  fml::TaskRunner::RunNowOrPostTask(task_runners_.GetRasterTaskRunner(), [&] {
    if (main_context) {
      std::unique_ptr<AndroidEGLSurface> pbuffer_surface =
          CreatePbufferSurface();
      auto status = pbuffer_surface->MakeCurrent();
      if (status != AndroidEGLSurfaceMakeCurrentStatus::kFailure) {
        main_context->releaseResourcesAndAbandonContext();
        main_context.reset();
        ClearCurrent();
      }
    }
    latch.Signal();
  });
  latch.Wait();

  if (!TeardownContext(environment_->Display(), context_)) {
    FML_LOG(ERROR)
        << "Could not tear down the EGL context. Possible resource leak.";
    LogLastEGLError();
  }

  if (!TeardownContext(environment_->Display(), resource_context_)) {
    FML_LOG(ERROR) << "Could not tear down the EGL resource context. Possible "
                      "resource leak.";
    LogLastEGLError();
  }
}

std::unique_ptr<AndroidEGLSurface> AndroidContextGLSkia::CreateOnscreenSurface(
    const fml::RefPtr<AndroidNativeWindow>& window) const {
  if (window->IsFakeWindow()) {
    return CreatePbufferSurface();
  } else {
    EGLDisplay display = environment_->Display();

    const EGLint attribs[] = {EGL_NONE};

    EGLSurface surface = eglCreateWindowSurface(
        display, config_,
        reinterpret_cast<EGLNativeWindowType>(window->handle()), attribs);
    return std::make_unique<AndroidEGLSurface>(surface, display, context_);
  }
}

std::unique_ptr<AndroidEGLSurface>
AndroidContextGLSkia::CreateOffscreenSurface() const {
  // We only ever create pbuffer surfaces for background resource loading
  // contexts. We never bind the pbuffer to anything.
  EGLDisplay display = environment_->Display();

  const EGLint attribs[] = {EGL_WIDTH, 1, EGL_HEIGHT, 1, EGL_NONE};

  EGLSurface surface = eglCreatePbufferSurface(display, config_, attribs);
  return std::make_unique<AndroidEGLSurface>(surface, display,
                                             resource_context_);
}

std::unique_ptr<AndroidEGLSurface> AndroidContextGLSkia::CreatePbufferSurface()
    const {
  EGLDisplay display = environment_->Display();

  const EGLint attribs[] = {EGL_WIDTH, 1, EGL_HEIGHT, 1, EGL_NONE};

  EGLSurface surface = eglCreatePbufferSurface(display, config_, attribs);
  return std::make_unique<AndroidEGLSurface>(surface, display, context_);
}

fml::RefPtr<AndroidEnvironmentGL> AndroidContextGLSkia::Environment() const {
  return environment_;
}

bool AndroidContextGLSkia::IsValid() const {
  return valid_;
}

bool AndroidContextGLSkia::ClearCurrent() const {
  if (eglGetCurrentContext() != context_) {
    return true;
  }
  if (eglMakeCurrent(environment_->Display(), EGL_NO_SURFACE, EGL_NO_SURFACE,
                     EGL_NO_CONTEXT) != EGL_TRUE) {
    FML_LOG(ERROR) << "Could not clear the current context";
    LogLastEGLError();
    return false;
  }
  return true;
}

EGLContext AndroidContextGLSkia::GetEGLContext() const {
  return context_;
}

EGLDisplay AndroidContextGLSkia::GetEGLDisplay() const {
  return environment_->Display();
}

EGLContext AndroidContextGLSkia::CreateNewContext() const {
  bool success;
  EGLContext context;
  std::tie(success, context) =
      CreateContext(environment_->Display(), config_, EGL_NO_CONTEXT);
  return success ? context : EGL_NO_CONTEXT;
}

}  // namespace flutter
