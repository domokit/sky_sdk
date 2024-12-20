// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_TESTING_TEST_VULKAN_SURFACE_H_
#define FLUTTER_TESTING_TEST_VULKAN_SURFACE_H_

#include <memory>
#include "flutter/testing/test_vulkan_context.h"

#include "third_party/skia/include/core/SkRefCnt.h"
#include "third_party/skia/include/core/SkSize.h"
#include "third_party/skia/include/core/SkSurface.h"
#include "third_party/skia/include/gpu/ganesh/GrDirectContext.h"

namespace flutter::testing {

class TestVulkanSurface {
 public:
  static std::unique_ptr<TestVulkanSurface> Create(
      const TestVulkanContext& context,
      const SkISize& surface_size);

  bool IsValid() const;

  sk_sp<SkImage> GetSurfaceSnapshot() const;

  VkImage GetImage();

 private:
  explicit TestVulkanSurface(TestVulkanImage&& image);

  TestVulkanImage image_;
  sk_sp<SkSurface> surface_;
};

}  // namespace flutter::testing

#endif  // FLUTTER_TESTING_TEST_VULKAN_SURFACE_H_
