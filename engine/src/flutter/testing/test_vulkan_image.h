// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_TESTING_TEST_VULKAN_IMAGE_H_
#define FLUTTER_TESTING_TEST_VULKAN_IMAGE_H_

#include "flutter/fml/macros.h"

#include "flutter/fml/memory/ref_ptr.h"
#include "flutter/vulkan/procs/vulkan_handle.h"
#include "third_party/skia/include/core/SkSize.h"

namespace flutter::testing {

class TestVulkanContext;

/// Captures the lifetime of a test VkImage along with its bound memory.
class TestVulkanImage {
 public:
  TestVulkanImage(TestVulkanImage&& other);
  TestVulkanImage& operator=(TestVulkanImage&& other);

  ~TestVulkanImage();

  VkImage GetImage();

 private:
  TestVulkanImage();

  // The lifetime of the Vulkan state must exceed memory/image handles.
  fml::RefPtr<TestVulkanContext> context_;

  vulkan::VulkanHandle<VkImage> image_;
  vulkan::VulkanHandle<VkDeviceMemory> memory_;

  FML_DISALLOW_COPY_AND_ASSIGN(TestVulkanImage);

  friend TestVulkanContext;
};

}  // namespace flutter::testing

#endif  // FLUTTER_TESTING_TEST_VULKAN_IMAGE_H_
