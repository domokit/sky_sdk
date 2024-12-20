// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_ANDROID_ANDROID_CONTEXT_GL_SKIA_H_
#define FLUTTER_SHELL_PLATFORM_ANDROID_ANDROID_CONTEXT_GL_SKIA_H_

#include "flutter/fml/macros.h"
#include "flutter/fml/memory/ref_counted.h"
#include "flutter/fml/memory/ref_ptr.h"
#include "flutter/shell/common/platform_view.h"
#include "flutter/shell/platform/android/android_environment_gl.h"
#include "flutter/shell/platform/android/context/android_context.h"
#include "flutter/shell/platform/android/surface/android_native_window.h"
#include "third_party/skia/include/core/SkSize.h"

namespace flutter {

class AndroidEGLSurface;

//------------------------------------------------------------------------------
/// The Android context is used by `AndroidSurfaceGL` to create and manage
/// EGL surfaces.
///
/// This context binds `EGLContext` to the current rendering thread and to the
/// draw and read `EGLSurface`s.
///
class AndroidContextGLSkia : public AndroidContext {
 public:
  AndroidContextGLSkia(fml::RefPtr<AndroidEnvironmentGL> environment,
                       const TaskRunners& taskRunners);

  ~AndroidContextGLSkia();

  //----------------------------------------------------------------------------
  /// @brief      Allocates an new EGL window surface that is used for on-screen
  ///             pixels.
  ///
  /// @return     The window surface.
  ///
  std::unique_ptr<AndroidEGLSurface> CreateOnscreenSurface(
      const fml::RefPtr<AndroidNativeWindow>& window) const;

  //----------------------------------------------------------------------------
  /// @brief      Allocates an 1x1 pbuffer surface that is used for making the
  ///             offscreen current for texture uploads.
  ///
  /// @return     The pbuffer surface.
  ///
  std::unique_ptr<AndroidEGLSurface> CreateOffscreenSurface() const;

  //----------------------------------------------------------------------------
  /// @brief      Allocates an 1x1 pbuffer surface that is used for making the
  ///             onscreen context current for snapshotting.
  ///
  /// @return     The pbuffer surface.
  ///
  std::unique_ptr<AndroidEGLSurface> CreatePbufferSurface() const;

  //----------------------------------------------------------------------------
  /// @return     The Android environment that contains a reference to the
  /// display.
  ///
  fml::RefPtr<AndroidEnvironmentGL> Environment() const;

  //----------------------------------------------------------------------------
  /// @return     Whether the current context is valid. That is, if the EGL
  ///             contexts were successfully created.
  ///
  bool IsValid() const override;

  //----------------------------------------------------------------------------
  /// @return     Whether the current context was successfully clear.
  ///
  bool ClearCurrent() const;

  //----------------------------------------------------------------------------
  /// @brief      Returns the EGLContext.
  ///
  /// @return     EGLContext.
  ///
  EGLContext GetEGLContext() const;

  //----------------------------------------------------------------------------
  /// @brief      Returns the EGLDisplay.
  ///
  /// @return     EGLDisplay.
  ///
  EGLDisplay GetEGLDisplay() const;

  //----------------------------------------------------------------------------
  /// @brief      Create a new EGLContext using the same EGLConfig.
  ///
  /// @return     The EGLContext.
  ///
  EGLContext CreateNewContext() const;

  //----------------------------------------------------------------------------
  /// @brief      The EGLConfig for this context.
  ///
  EGLConfig Config() const { return config_; }

 private:
  fml::RefPtr<AndroidEnvironmentGL> environment_;
  EGLConfig config_;
  EGLContext context_;
  EGLContext resource_context_;
  bool valid_ = false;
  TaskRunners task_runners_;

  FML_DISALLOW_COPY_AND_ASSIGN(AndroidContextGLSkia);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_ANDROID_ANDROID_CONTEXT_GL_SKIA_H_
