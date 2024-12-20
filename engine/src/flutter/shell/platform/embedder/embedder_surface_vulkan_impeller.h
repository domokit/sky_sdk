// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_EMBEDDER_EMBEDDER_SURFACE_VULKAN_IMPELLER_H_
#define FLUTTER_SHELL_PLATFORM_EMBEDDER_EMBEDDER_SURFACE_VULKAN_IMPELLER_H_

#include "flutter/shell/common/context_options.h"
#include "flutter/shell/gpu/gpu_surface_vulkan.h"
#include "flutter/shell/gpu/gpu_surface_vulkan_delegate.h"
#include "flutter/shell/platform/embedder/embedder.h"
#include "flutter/shell/platform/embedder/embedder_external_view_embedder.h"
#include "flutter/shell/platform/embedder/embedder_surface.h"
#include "flutter/vulkan/procs/vulkan_proc_table.h"
#include "impeller/renderer/backend/vulkan/context_vk.h"

namespace flutter {

class EmbedderSurfaceVulkanImpeller final : public EmbedderSurface,
                                            public GPUSurfaceVulkanDelegate {
 public:
  struct VulkanDispatchTable {
    PFN_vkGetInstanceProcAddr get_instance_proc_address;  // required
    std::function<FlutterVulkanImage(const SkISize& frame_size)>
        get_next_image;  // required
    std::function<bool(VkImage image, VkFormat format)>
        present_image;  // required
  };

  EmbedderSurfaceVulkanImpeller(
      uint32_t version,
      VkInstance instance,
      size_t instance_extension_count,
      const char** instance_extensions,
      size_t device_extension_count,
      const char** device_extensions,
      VkPhysicalDevice physical_device,
      VkDevice device,
      uint32_t queue_family_index,
      VkQueue queue,
      const VulkanDispatchTable& vulkan_dispatch_table,
      std::shared_ptr<EmbedderExternalViewEmbedder> external_view_embedder);

  ~EmbedderSurfaceVulkanImpeller() override;

  // |GPUSurfaceVulkanDelegate|
  const vulkan::VulkanProcTable& vk() override;

  // |GPUSurfaceVulkanDelegate|
  FlutterVulkanImage AcquireImage(const SkISize& size) override;

  // |GPUSurfaceVulkanDelegate|
  bool PresentImage(VkImage image, VkFormat format) override;

  // |GPUSurfaceVulkanDelegate|
  std::shared_ptr<impeller::Context> CreateImpellerContext() const override;

 private:
  bool valid_ = false;
  fml::RefPtr<vulkan::VulkanProcTable> vk_;
  VulkanDispatchTable vulkan_dispatch_table_;
  std::shared_ptr<EmbedderExternalViewEmbedder> external_view_embedder_;
  std::shared_ptr<impeller::ContextVK> context_;

  // |EmbedderSurface|
  bool IsValid() const override;

  // |EmbedderSurface|
  std::unique_ptr<Surface> CreateGPUSurface() override;

  // |EmbedderSurface|
  sk_sp<GrDirectContext> CreateResourceContext() const override;

  EmbedderSurfaceVulkanImpeller(const EmbedderSurfaceVulkanImpeller&) = delete;
  EmbedderSurfaceVulkanImpeller& operator=(
      const EmbedderSurfaceVulkanImpeller&) = delete;
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_EMBEDDER_EMBEDDER_SURFACE_VULKAN_IMPELLER_H_
