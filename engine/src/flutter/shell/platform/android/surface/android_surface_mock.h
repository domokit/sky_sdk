// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_ANDROID_SURFACE_ANDROID_SURFACE_MOCK_H_
#define FLUTTER_SHELL_PLATFORM_ANDROID_SURFACE_ANDROID_SURFACE_MOCK_H_

#include "flutter/shell/gpu/gpu_surface_gl_skia.h"
#include "flutter/shell/platform/android/surface/android_surface.h"
#include "gmock/gmock.h"

namespace flutter {

//------------------------------------------------------------------------------
/// Mock for |AndroidSurface|. This implementation can be used in unit
/// tests without requiring the Android toolchain.
///
class AndroidSurfaceMock final : public GPUSurfaceGLDelegate,
                                 public AndroidSurface {
 public:
  MOCK_METHOD(bool, IsValid, (), (const, override));

  MOCK_METHOD(void, TeardownOnScreenContext, (), (override));

  MOCK_METHOD(std::unique_ptr<Surface>,
              CreateGPUSurface,
              (GrDirectContext * gr_context),
              (override));

  MOCK_METHOD(bool, OnScreenSurfaceResize, (const SkISize& size), (override));

  MOCK_METHOD(bool, ResourceContextMakeCurrent, (), (override));

  MOCK_METHOD(bool, ResourceContextClearCurrent, (), (override));

  MOCK_METHOD(bool,
              SetNativeWindow,
              (fml::RefPtr<AndroidNativeWindow> window,
               const std::shared_ptr<PlatformViewAndroidJNI>& jni_facade),
              (override));

  // |GPUSurfaceGLDelegate|
  std::unique_ptr<GLContextResult> GLContextMakeCurrent() override;

  // |GPUSurfaceGLDelegate|
  bool GLContextClearCurrent() override;

  // |GPUSurfaceGLDelegate|
  bool GLContextPresent(const GLPresentInfo& present_info) override;

  // |GPUSurfaceGLDelegate|
  GLFBOInfo GLContextFBO(GLFrameInfo frame_info) const override;
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_ANDROID_SURFACE_ANDROID_SURFACE_MOCK_H_
