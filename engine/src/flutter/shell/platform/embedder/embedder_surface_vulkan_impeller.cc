// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/embedder/embedder_surface_vulkan_impeller.h"

#include <utility>

#include "flutter/impeller/entity/vk/entity_shaders_vk.h"
#include "flutter/impeller/entity/vk/framebuffer_blend_shaders_vk.h"
#include "flutter/impeller/entity/vk/modern_shaders_vk.h"
#include "flutter/shell/gpu/gpu_surface_vulkan.h"
#include "impeller/renderer/backend/vulkan/context_vk.h"
#include "include/gpu/ganesh/GrDirectContext.h"
#include "shell/gpu/gpu_surface_vulkan_impeller.h"

namespace flutter {

EmbedderSurfaceVulkanImpeller::EmbedderSurfaceVulkanImpeller(
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
    std::shared_ptr<EmbedderExternalViewEmbedder> external_view_embedder)
    : vk_(fml::MakeRefCounted<vulkan::VulkanProcTable>(
          vulkan_dispatch_table.get_instance_proc_address)),
      vulkan_dispatch_table_(vulkan_dispatch_table),
      external_view_embedder_(std::move(external_view_embedder)) {
  // Make sure all required members of the dispatch table are checked.
  if (!vulkan_dispatch_table_.get_instance_proc_address ||
      !vulkan_dispatch_table_.get_next_image ||
      !vulkan_dispatch_table_.present_image) {
    return;
  }

  std::vector<std::shared_ptr<fml::Mapping>> shader_mappings = {
      std::make_shared<fml::NonOwnedMapping>(impeller_entity_shaders_vk_data,
                                             impeller_entity_shaders_vk_length),
      std::make_shared<fml::NonOwnedMapping>(impeller_modern_shaders_vk_data,
                                             impeller_modern_shaders_vk_length),
      std::make_shared<fml::NonOwnedMapping>(
          impeller_framebuffer_blend_shaders_vk_data,
          impeller_framebuffer_blend_shaders_vk_length),
  };
  impeller::ContextVK::Settings settings;
  settings.shader_libraries_data = shader_mappings;
  settings.proc_address_callback =
      vulkan_dispatch_table.get_instance_proc_address;

  impeller::ContextVK::EmbedderData data;
  data.instance = instance;
  data.physical_device = physical_device;
  data.device = device;
  data.queue = queue;
  data.queue_family_index = queue_family_index;
  data.instance_extensions.reserve(instance_extension_count);
  for (auto i = 0u; i < instance_extension_count; i++) {
    data.instance_extensions.push_back(std::string{instance_extensions[i]});
  }
  data.device_extensions.reserve(device_extension_count);
  for (auto i = 0u; i < device_extension_count; i++) {
    data.device_extensions.push_back(std::string{device_extensions[i]});
  }
  settings.embedder_data = data;

  context_ = impeller::ContextVK::Create(std::move(settings));
  if (!context_) {
    FML_LOG(ERROR) << "Failed to initialize Vulkan Context.";
    return;
  }

  FML_LOG(IMPORTANT) << "Using the Impeller rendering backend (Vulkan).";

  valid_ = true;
}

EmbedderSurfaceVulkanImpeller::~EmbedderSurfaceVulkanImpeller() {}

std::shared_ptr<impeller::Context>
EmbedderSurfaceVulkanImpeller::CreateImpellerContext() const {
  return context_;
}

// |GPUSurfaceVulkanDelegate|
const vulkan::VulkanProcTable& EmbedderSurfaceVulkanImpeller::vk() {
  return *vk_;
}

// |GPUSurfaceVulkanDelegate|
FlutterVulkanImage EmbedderSurfaceVulkanImpeller::AcquireImage(
    const SkISize& size) {
  return vulkan_dispatch_table_.get_next_image(size);
}

// |GPUSurfaceVulkanDelegate|
bool EmbedderSurfaceVulkanImpeller::PresentImage(VkImage image,
                                                 VkFormat format) {
  return vulkan_dispatch_table_.present_image(image, format);
}

// |EmbedderSurface|
bool EmbedderSurfaceVulkanImpeller::IsValid() const {
  return valid_;
}

// |EmbedderSurface|
std::unique_ptr<Surface> EmbedderSurfaceVulkanImpeller::CreateGPUSurface() {
  return std::make_unique<GPUSurfaceVulkanImpeller>(this, context_);
}

// |EmbedderSurface|
sk_sp<GrDirectContext> EmbedderSurfaceVulkanImpeller::CreateResourceContext()
    const {
  return nullptr;
}

}  // namespace flutter
