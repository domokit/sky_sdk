// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_ANDROID_ANDROID_SURFACE_SOFTWARE_H_
#define FLUTTER_SHELL_PLATFORM_ANDROID_ANDROID_SURFACE_SOFTWARE_H_

#include "flutter/fml/macros.h"
#include "flutter/fml/platform/android/jni_weak_ref.h"
#include "flutter/fml/platform/android/scoped_java_ref.h"
#include "flutter/shell/gpu/gpu_surface_software.h"
#include "flutter/shell/platform/android/jni/platform_view_android_jni.h"
#include "flutter/shell/platform/android/surface/android_surface.h"

#include "third_party/skia/include/core/SkSurface.h"

namespace flutter {

class AndroidSurfaceSoftware final : public AndroidSurface,
                                     public GPUSurfaceSoftwareDelegate {
 public:
  AndroidSurfaceSoftware();

  ~AndroidSurfaceSoftware() override;

  // |AndroidSurface|
  bool IsValid() const override;

  // |AndroidSurface|
  bool ResourceContextMakeCurrent() override;

  // |AndroidSurface|
  bool ResourceContextClearCurrent() override;

  // |AndroidSurface|
  std::unique_ptr<Surface> CreateGPUSurface(
      GrDirectContext* gr_context) override;

  // |AndroidSurface|
  void TeardownOnScreenContext() override;

  // |AndroidSurface|
  bool OnScreenSurfaceResize(const SkISize& size) override;

  // |AndroidSurface|
  bool SetNativeWindow(fml::RefPtr<AndroidNativeWindow> window) override;

  // |GPUSurfaceSoftwareDelegate|
  sk_sp<SkSurface> AcquireBackingStore(const SkISize& size) override;

  // |GPUSurfaceSoftwareDelegate|
  bool PresentBackingStore(sk_sp<SkSurface> backing_store) override;

 private:
  sk_sp<SkSurface> sk_surface_;
  fml::RefPtr<AndroidNativeWindow> native_window_;
  SkColorType target_color_type_;
  SkAlphaType target_alpha_type_;

  FML_DISALLOW_COPY_AND_ASSIGN(AndroidSurfaceSoftware);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_ANDROID_ANDROID_SURFACE_SOFTWARE_H_
