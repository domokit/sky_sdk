// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_VULKAN_VULKAN_NATIVE_SURFACE_ANDROID_H_
#define FLUTTER_VULKAN_VULKAN_NATIVE_SURFACE_ANDROID_H_

#include "flutter/fml/macros.h"
#include "vulkan_native_surface.h"

struct ANativeWindow;
typedef struct ANativeWindow ANativeWindow;

namespace vulkan {

class VulkanNativeSurfaceAndroid : public VulkanNativeSurface {
 public:
  /// Create a native surface from the valid ANativeWindow reference. Ownership
  /// of the ANativeWindow is assumed by this instance.
  explicit VulkanNativeSurfaceAndroid(ANativeWindow* native_window);

  ~VulkanNativeSurfaceAndroid();

  const char* GetExtensionName() const override;

  VkSurfaceKHR CreateSurfaceHandle(
      VulkanProcTable& vk,
      const VulkanHandle<VkInstance>& instance) const override;

  bool IsValid() const override;

  SkISize GetSize() const override;

 private:
  ANativeWindow* native_window_;

  FML_DISALLOW_COPY_AND_ASSIGN(VulkanNativeSurfaceAndroid);
};

}  // namespace vulkan

#endif  // FLUTTER_VULKAN_VULKAN_NATIVE_SURFACE_ANDROID_H_
