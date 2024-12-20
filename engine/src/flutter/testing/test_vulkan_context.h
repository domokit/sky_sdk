// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_TESTING_TEST_VULKAN_CONTEXT_H_
#define FLUTTER_TESTING_TEST_VULKAN_CONTEXT_H_

#include "flutter/fml/macros.h"
#include "flutter/fml/memory/ref_ptr.h"
#include "flutter/testing/test_vulkan_image.h"
#include "flutter/vulkan/procs/vulkan_proc_table.h"
#include "flutter/vulkan/vulkan_application.h"
#include "flutter/vulkan/vulkan_device.h"

#include "third_party/skia/include/core/SkSize.h"
#include "third_party/skia/include/gpu/ganesh/GrDirectContext.h"

namespace flutter::testing {

class TestVulkanContext : public fml::RefCountedThreadSafe<TestVulkanContext> {
 public:
  TestVulkanContext();
  ~TestVulkanContext();

  std::optional<TestVulkanImage> CreateImage(const SkISize& size) const;

  sk_sp<GrDirectContext> GetGrDirectContext() const;

 private:
  fml::RefPtr<vulkan::VulkanProcTable> vk_;
  std::unique_ptr<vulkan::VulkanApplication> application_;
  std::unique_ptr<vulkan::VulkanDevice> device_;

  sk_sp<GrDirectContext> context_;

  friend class EmbedderTestContextVulkan;
  friend class EmbedderConfigBuilder;

  FML_FRIEND_MAKE_REF_COUNTED(TestVulkanContext);
  FML_FRIEND_REF_COUNTED_THREAD_SAFE(TestVulkanContext);
  FML_DISALLOW_COPY_AND_ASSIGN(TestVulkanContext);
};

}  // namespace flutter::testing

#endif  // FLUTTER_TESTING_TEST_VULKAN_CONTEXT_H_
