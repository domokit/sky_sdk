// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/toolkit/interop/backend/vulkan/context_vk.h"

#include "flutter/fml/paths.h"
#include "impeller/entity/vk/entity_shaders_vk.h"
#include "impeller/entity/vk/framebuffer_blend_shaders_vk.h"
#include "impeller/entity/vk/modern_shaders_vk.h"
#include "impeller/renderer/backend/vulkan/context_vk.h"
#include "impeller/renderer/vk/compute_shaders_vk.h"

namespace impeller::interop {

static std::vector<std::shared_ptr<fml::Mapping>>
CreateShaderLibraryMappings() {
  return {
      std::make_shared<fml::NonOwnedMapping>(impeller_entity_shaders_vk_data,
                                             impeller_entity_shaders_vk_length),
      std::make_shared<fml::NonOwnedMapping>(impeller_modern_shaders_vk_data,
                                             impeller_modern_shaders_vk_length),
      std::make_shared<fml::NonOwnedMapping>(
          impeller_framebuffer_blend_shaders_vk_data,
          impeller_framebuffer_blend_shaders_vk_length),
      std::make_shared<fml::NonOwnedMapping>(
          impeller_compute_shaders_vk_data, impeller_compute_shaders_vk_length),
  };
}
// This bit is complicated by the fact that impeller::ContextVK::Settings takes
// a raw function pointer to the callback.
thread_local std::function<PFN_vkVoidFunction(VkInstance instance,
                                              const char* proc_name)>
    sContextVKProcAddressCallback;

VKAPI_ATTR PFN_vkVoidFunction VKAPI_CALL ContextVKGetInstanceProcAddress(
    VkInstance instance,
    const char* proc_name) {
  if (sContextVKProcAddressCallback) {
    return sContextVKProcAddressCallback(instance, proc_name);
  }
  return nullptr;
}

ScopedObject<Context> ContextVK::Create(const Settings& settings) {
  if (!settings.IsValid()) {
    VALIDATION_LOG << "Invalid settings for Vulkan context creation.";
    return {};
  }
  impeller::ContextVK::Settings impeller_settings;
  impeller_settings.shader_libraries_data = CreateShaderLibraryMappings();
  impeller_settings.cache_directory = fml::paths::GetCachesDirectory();
  impeller_settings.enable_validation = true;
  sContextVKProcAddressCallback = settings.instance_proc_address_callback;
  impeller_settings.proc_address_callback = ContextVKGetInstanceProcAddress;
  auto impeller_context =
      impeller::ContextVK::Create(std::move(impeller_settings));
  sContextVKProcAddressCallback = nullptr;
  if (!impeller_context) {
    VALIDATION_LOG << "Could not create Impeller context.";
    return {};
  }
  return Create(std::move(impeller_context));
}

ScopedObject<Context> ContextVK::Create(
    std::shared_ptr<impeller::Context> impeller_context) {
  // Can't call Create because of private constructor. Adopt the raw pointer
  // instead.
  auto context = Adopt<Context>(new ContextVK(std::move(impeller_context)));
  if (!context->IsValid()) {
    VALIDATION_LOG << " Could not create valid context.";
    return {};
  }
  return context;
}

ContextVK::ContextVK(std::shared_ptr<impeller::Context> context)
    : Context(std::move(context)) {}

ContextVK::~ContextVK() = default;

ContextVK::Settings::Settings(const ImpellerContextVulkanSettings& settings)
    : enable_validation(settings.enable_vulkan_validation) {
  instance_proc_address_callback =
      [&settings](VkInstance instance,
                  const char* proc_name) -> PFN_vkVoidFunction {
    if (settings.proc_address_callback) {
      return reinterpret_cast<PFN_vkVoidFunction>(
          settings.proc_address_callback(instance, proc_name,
                                         settings.user_data));
    }
    return nullptr;
  };
}

bool ContextVK::GetInfo(ImpellerContextVulkanInfo& info) const {
  if (!IsValid()) {
    return false;
  }
  const auto& context = impeller::ContextVK::Cast(*GetContext());
  // NOLINTBEGIN(google-readability-casting)
  info.vk_instance = reinterpret_cast<void*>(VkInstance(context.GetInstance()));
  info.vk_physical_device =
      reinterpret_cast<void*>(VkPhysicalDevice(context.GetPhysicalDevice()));
  info.vk_logical_device =
      reinterpret_cast<void*>(VkDevice(context.GetDevice()));
  // NOLINTEND(google-readability-casting)
  info.graphics_queue_family_index =
      context.GetGraphicsQueue()->GetIndex().family;
  info.graphics_queue_index = context.GetGraphicsQueue()->GetIndex().index;
  return true;
}

bool ContextVK::Settings::IsValid() const {
  return !!instance_proc_address_callback;
}

}  // namespace impeller::interop
