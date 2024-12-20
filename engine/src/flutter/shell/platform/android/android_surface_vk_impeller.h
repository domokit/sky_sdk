// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_ANDROID_ANDROID_SURFACE_VK_IMPELLER_H_
#define FLUTTER_SHELL_PLATFORM_ANDROID_ANDROID_SURFACE_VK_IMPELLER_H_

#include "flutter/fml/concurrent_message_loop.h"
#include "flutter/fml/macros.h"
#include "flutter/impeller/renderer/backend/vulkan/surface_context_vk.h"
#include "flutter/shell/platform/android/android_context_vk_impeller.h"
#include "flutter/shell/platform/android/surface/android_native_window.h"
#include "flutter/shell/platform/android/surface/android_surface.h"
#include "shell/gpu/gpu_surface_vulkan_impeller.h"

namespace flutter {

class AndroidSurfaceVKImpeller : public AndroidSurface {
 public:
  explicit AndroidSurfaceVKImpeller(
      const std::shared_ptr<AndroidContextVKImpeller>& android_context);

  ~AndroidSurfaceVKImpeller() override;

  // |AndroidSurface|
  bool IsValid() const override;

  // |AndroidSurface|
  std::unique_ptr<Surface> CreateGPUSurface(
      GrDirectContext* gr_context) override;

  // |AndroidSurface|
  void TeardownOnScreenContext() override;

  // |AndroidSurface|
  bool OnScreenSurfaceResize(const SkISize& size) override;

  // |AndroidSurface|
  bool ResourceContextMakeCurrent() override;

  // |AndroidSurface|
  bool ResourceContextClearCurrent() override;

  // |AndroidSurface|
  std::shared_ptr<impeller::Context> GetImpellerContext() override;

  // |AndroidSurface|
  bool SetNativeWindow(fml::RefPtr<AndroidNativeWindow> window) override;

 private:
  std::shared_ptr<impeller::SurfaceContextVK> surface_context_vk_;
  fml::RefPtr<AndroidNativeWindow> native_window_;
  // The first GPU Surface is initialized as soon as the
  // AndroidSurfaceVulkanImpeller is created. This ensures that the pipelines
  // are bootstrapped as early as possible.
  std::unique_ptr<GPUSurfaceVulkanImpeller> eager_gpu_surface_;

  bool is_valid_ = false;

  FML_DISALLOW_COPY_AND_ASSIGN(AndroidSurfaceVKImpeller);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_ANDROID_ANDROID_SURFACE_VK_IMPELLER_H_
