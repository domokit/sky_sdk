// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/embedder/tests/embedder_test_backingstore_producer_vulkan.h"

#include "flutter/fml/logging.h"
#include "third_party/skia/include/core/SkColorSpace.h"
#include "third_party/skia/include/gpu/ganesh/GrBackendSurface.h"
#include "third_party/skia/include/gpu/ganesh/SkSurfaceGanesh.h"
#include "third_party/skia/include/gpu/ganesh/vk/GrVkBackendSurface.h"
#include "third_party/skia/include/gpu/ganesh/vk/GrVkTypes.h"

namespace flutter::testing {

namespace {
struct UserData {
  sk_sp<SkSurface> surface;
  FlutterVulkanImage* image;
};
}  // namespace

EmbedderTestBackingStoreProducerVulkan::EmbedderTestBackingStoreProducerVulkan(
    sk_sp<GrDirectContext> context,
    RenderTargetType type)
    : EmbedderTestBackingStoreProducer(std::move(context), type),
      test_vulkan_context_(nullptr) {}

EmbedderTestBackingStoreProducerVulkan::
    ~EmbedderTestBackingStoreProducerVulkan() = default;

bool EmbedderTestBackingStoreProducerVulkan::Create(
    const FlutterBackingStoreConfig* config,
    FlutterBackingStore* backing_store_out) {
  if (!test_vulkan_context_) {
    test_vulkan_context_ = fml::MakeRefCounted<TestVulkanContext>();
  }

  auto surface_size = SkISize::Make(config->size.width, config->size.height);
  auto optional_image = test_vulkan_context_->CreateImage(surface_size);
  if (!optional_image.has_value()) {
    FML_LOG(ERROR) << "Could not create Vulkan image.";
    return false;
  }
  TestVulkanImage* test_image = new TestVulkanImage(std::move(*optional_image));

  GrVkImageInfo image_info = {
      .fImage = test_image->GetImage(),
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

  SkSurfaces::TextureReleaseProc release_vktexture = [](void* user_data) {
    delete reinterpret_cast<TestVulkanImage*>(user_data);
  };

  sk_sp<SkSurface> surface = SkSurfaces::WrapBackendTexture(
      context_.get(),            // context
      backend_texture,           // back-end texture
      kTopLeft_GrSurfaceOrigin,  // surface origin
      1,                         // sample count
      kRGBA_8888_SkColorType,    // color type
      SkColorSpace::MakeSRGB(),  // color space
      &surface_properties,       // surface properties
      release_vktexture,         // texture release proc
      test_image                 // release context
  );

  if (!surface) {
    FML_LOG(ERROR) << "Could not create Skia surface from Vulkan image.";
    return false;
  }
  backing_store_out->type = kFlutterBackingStoreTypeVulkan;

  FlutterVulkanImage* image = new FlutterVulkanImage();
  image->image = reinterpret_cast<uint64_t>(image_info.fImage);
  image->format = VK_FORMAT_R8G8B8A8_UNORM;
  backing_store_out->vulkan.image = image;

  // Collect all allocated resources in the destruction_callback.
  {
    auto user_data = new UserData{.surface = surface, .image = image};
    backing_store_out->user_data = user_data;
    backing_store_out->vulkan.user_data = user_data;
    backing_store_out->vulkan.destruction_callback = [](void* user_data) {
      UserData* d = reinterpret_cast<UserData*>(user_data);
      delete d->image;
      delete d;
    };
  }

  return true;
}

sk_sp<SkSurface> EmbedderTestBackingStoreProducerVulkan::GetSurface(
    const FlutterBackingStore* backing_store) const {
  UserData* user_data = reinterpret_cast<UserData*>(backing_store->user_data);
  return user_data->surface;
}

sk_sp<SkImage> EmbedderTestBackingStoreProducerVulkan::MakeImageSnapshot(
    const FlutterBackingStore* backing_store) const {
  UserData* user_data = reinterpret_cast<UserData*>(backing_store->user_data);
  return user_data->surface->makeImageSnapshot();
}

}  // namespace flutter::testing
