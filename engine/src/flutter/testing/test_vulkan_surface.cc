// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/testing/test_vulkan_surface.h"
#include <memory>
#include "flutter/fml/logging.h"
#include "flutter/testing/test_vulkan_context.h"

#include "third_party/skia/include/core/SkColorSpace.h"
#include "third_party/skia/include/core/SkColorType.h"
#include "third_party/skia/include/core/SkSurface.h"
#include "third_party/skia/include/core/SkSurfaceProps.h"
#include "third_party/skia/include/gpu/ganesh/GrBackendSurface.h"
#include "third_party/skia/include/gpu/ganesh/SkSurfaceGanesh.h"
#include "third_party/skia/include/gpu/ganesh/vk/GrVkBackendSurface.h"
#include "third_party/skia/include/gpu/ganesh/vk/GrVkTypes.h"

namespace flutter::testing {

TestVulkanSurface::TestVulkanSurface(TestVulkanImage&& image)
    : image_(std::move(image)){};

std::unique_ptr<TestVulkanSurface> TestVulkanSurface::Create(
    const TestVulkanContext& context,
    const SkISize& surface_size) {
  auto image_result = context.CreateImage(surface_size);

  if (!image_result.has_value()) {
    FML_LOG(ERROR) << "Could not create VkImage.";
    return nullptr;
  }

  GrVkImageInfo image_info = {
      .fImage = image_result.value().GetImage(),
      .fImageTiling = VK_IMAGE_TILING_OPTIMAL,
      .fImageLayout = VK_IMAGE_LAYOUT_UNDEFINED,
      .fFormat = VK_FORMAT_R8G8B8A8_UNORM,
      .fImageUsageFlags = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT |
                          VK_IMAGE_USAGE_TRANSFER_SRC_BIT |
                          VK_IMAGE_USAGE_TRANSFER_DST_BIT |
                          VK_IMAGE_USAGE_SAMPLED_BIT,
      .fSampleCount = 1,
      .fLevelCount = 1,
  };
  auto backend_texture = GrBackendTextures::MakeVk(
      surface_size.width(), surface_size.height(), image_info);

  SkSurfaceProps surface_properties(0, kUnknown_SkPixelGeometry);

  auto result = std::unique_ptr<TestVulkanSurface>(
      new TestVulkanSurface(std::move(image_result.value())));
  result->surface_ = SkSurfaces::WrapBackendTexture(
      context.GetGrDirectContext().get(),  // context
      backend_texture,                     // back-end texture
      kTopLeft_GrSurfaceOrigin,            // surface origin
      1,                                   // sample count
      kRGBA_8888_SkColorType,              // color type
      SkColorSpace::MakeSRGB(),            // color space
      &surface_properties,                 // surface properties
      nullptr,                             // release proc
      nullptr                              // release context
  );

  if (!result->surface_) {
    FML_LOG(ERROR)
        << "Could not wrap VkImage as an SkSurface Vulkan render texture.";
    return nullptr;
  }

  return result;
}

bool TestVulkanSurface::IsValid() const {
  return surface_ != nullptr;
}

sk_sp<SkImage> TestVulkanSurface::GetSurfaceSnapshot() const {
  if (!IsValid()) {
    return nullptr;
  }

  if (!surface_) {
    FML_LOG(ERROR) << "Aborting snapshot because of on-screen surface "
                      "acquisition failure.";
    return nullptr;
  }

  auto device_snapshot = surface_->makeImageSnapshot();

  if (!device_snapshot) {
    FML_LOG(ERROR) << "Could not create the device snapshot while attempting "
                      "to snapshot the Vulkan surface.";
    return nullptr;
  }

  auto host_snapshot = device_snapshot->makeRasterImage();

  if (!host_snapshot) {
    FML_LOG(ERROR) << "Could not create the host snapshot while attempting to "
                      "snapshot the Vulkan surface.";
    return nullptr;
  }

  return host_snapshot;
}

VkImage TestVulkanSurface::GetImage() {
  return image_.GetImage();
}

}  // namespace flutter::testing
