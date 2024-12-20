// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_VULKAN_VULKAN_NATIVE_SURFACE_H_
#define FLUTTER_VULKAN_VULKAN_NATIVE_SURFACE_H_

#include "flutter/fml/macros.h"
#include "flutter/vulkan/procs/vulkan_handle.h"
#include "flutter/vulkan/procs/vulkan_proc_table.h"
#include "third_party/skia/include/core/SkSize.h"

namespace vulkan {

class VulkanNativeSurface {
 public:
  virtual ~VulkanNativeSurface() = default;

  virtual const char* GetExtensionName() const = 0;

  virtual VkSurfaceKHR CreateSurfaceHandle(
      VulkanProcTable& vk,
      const VulkanHandle<VkInstance>& instance) const = 0;

  virtual bool IsValid() const = 0;

  virtual SkISize GetSize() const = 0;
};

}  // namespace vulkan

#endif  // FLUTTER_VULKAN_VULKAN_NATIVE_SURFACE_H_
