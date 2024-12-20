// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/playground/backend/vulkan/playground_impl_vk.h"

#include "flutter/fml/paths.h"
#include "impeller/renderer/backend/vulkan/vk.h"

#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

#include "flutter/fml/logging.h"
#include "flutter/fml/mapping.h"
#include "impeller/entity/vk/entity_shaders_vk.h"
#include "impeller/entity/vk/framebuffer_blend_shaders_vk.h"
#include "impeller/entity/vk/modern_shaders_vk.h"
#include "impeller/fixtures/vk/fixtures_shaders_vk.h"
#include "impeller/fixtures/vk/modern_fixtures_shaders_vk.h"
#include "impeller/playground/imgui/vk/imgui_shaders_vk.h"
#include "impeller/renderer/backend/vulkan/context_vk.h"
#include "impeller/renderer/backend/vulkan/formats_vk.h"
#include "impeller/renderer/backend/vulkan/surface_context_vk.h"
#include "impeller/renderer/backend/vulkan/swapchain/surface_vk.h"
#include "impeller/renderer/backend/vulkan/texture_vk.h"
#include "impeller/renderer/vk/compute_shaders_vk.h"

namespace impeller {

static std::vector<std::shared_ptr<fml::Mapping>>
ShaderLibraryMappingsForPlayground() {
  return {
      std::make_shared<fml::NonOwnedMapping>(impeller_entity_shaders_vk_data,
                                             impeller_entity_shaders_vk_length),
      std::make_shared<fml::NonOwnedMapping>(impeller_modern_shaders_vk_data,
                                             impeller_modern_shaders_vk_length),
      std::make_shared<fml::NonOwnedMapping>(
          impeller_framebuffer_blend_shaders_vk_data,
          impeller_framebuffer_blend_shaders_vk_length),
      std::make_shared<fml::NonOwnedMapping>(
          impeller_fixtures_shaders_vk_data,
          impeller_fixtures_shaders_vk_length),
      std::make_shared<fml::NonOwnedMapping>(
          impeller_modern_fixtures_shaders_vk_data,
          impeller_modern_fixtures_shaders_vk_length),
      std::make_shared<fml::NonOwnedMapping>(impeller_imgui_shaders_vk_data,
                                             impeller_imgui_shaders_vk_length),
      std::make_shared<fml::NonOwnedMapping>(
          impeller_compute_shaders_vk_data, impeller_compute_shaders_vk_length),
  };
}

vk::UniqueInstance PlaygroundImplVK::global_instance_;

void PlaygroundImplVK::DestroyWindowHandle(WindowHandle handle) {
  if (!handle) {
    return;
  }
  ::glfwDestroyWindow(reinterpret_cast<GLFWwindow*>(handle));
}

PlaygroundImplVK::PlaygroundImplVK(PlaygroundSwitches switches)
    : PlaygroundImpl(switches), handle_(nullptr, &DestroyWindowHandle) {
  FML_CHECK(IsVulkanDriverPresent());

  InitGlobalVulkanInstance();

  ::glfwDefaultWindowHints();
  ::glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
  ::glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);

  auto window = ::glfwCreateWindow(1, 1, "Test", nullptr, nullptr);
  if (!window) {
    VALIDATION_LOG << "Unable to create glfw window";
    return;
  }

  int width = 0;
  int height = 0;
  ::glfwGetWindowSize(window, &width, &height);
  size_ = ISize{width, height};

  handle_.reset(window);

  ContextVK::Settings context_settings;
  context_settings.proc_address_callback =
      reinterpret_cast<PFN_vkGetInstanceProcAddr>(
          &::glfwGetInstanceProcAddress);
  context_settings.shader_libraries_data = ShaderLibraryMappingsForPlayground();
  context_settings.cache_directory = fml::paths::GetCachesDirectory();
  context_settings.enable_validation = switches_.enable_vulkan_validation;
  context_settings.fatal_missing_validations =
      switches_.enable_vulkan_validation;
  ;

  auto context_vk = ContextVK::Create(std::move(context_settings));
  if (!context_vk || !context_vk->IsValid()) {
    VALIDATION_LOG << "Could not create Vulkan context in the playground.";
    return;
  }

  VkSurfaceKHR vk_surface;
  auto res = vk::Result{::glfwCreateWindowSurface(
      context_vk->GetInstance(),  // instance
      window,                     // window
      nullptr,                    // allocator
      &vk_surface                 // surface
      )};
  if (res != vk::Result::eSuccess) {
    VALIDATION_LOG << "Could not create surface for GLFW window: "
                   << vk::to_string(res);
    return;
  }

  vk::UniqueSurfaceKHR surface{vk_surface, context_vk->GetInstance()};
  auto context = context_vk->CreateSurfaceContext();
  if (!context->SetWindowSurface(std::move(surface), size_)) {
    VALIDATION_LOG << "Could not set up surface for context.";
    return;
  }

  context_ = std::move(context);
}

PlaygroundImplVK::~PlaygroundImplVK() = default;

// |PlaygroundImpl|
std::shared_ptr<Context> PlaygroundImplVK::GetContext() const {
  return context_;
}

// |PlaygroundImpl|
PlaygroundImpl::WindowHandle PlaygroundImplVK::GetWindowHandle() const {
  return handle_.get();
}

// |PlaygroundImpl|
std::unique_ptr<Surface> PlaygroundImplVK::AcquireSurfaceFrame(
    std::shared_ptr<Context> context) {
  SurfaceContextVK* surface_context_vk =
      reinterpret_cast<SurfaceContextVK*>(context_.get());

  int width = 0;
  int height = 0;
  ::glfwGetFramebufferSize(reinterpret_cast<GLFWwindow*>(handle_.get()), &width,
                           &height);
  size_ = ISize{width, height};
  surface_context_vk->UpdateSurfaceSize(ISize{width, height});

  return surface_context_vk->AcquireNextSurface();
}

// Create a global instance of Vulkan in order to prevent unloading of the
// Vulkan library.
// A test suite may repeatedly create and destroy PlaygroundImplVK instances,
// and if the PlaygroundImplVK's Vulkan instance is the only one in the
// process then the Vulkan library will be unloaded when the instance is
// destroyed.  Repeated loading and unloading of SwiftShader was leaking
// resources, so this will work around that leak.
// (see https://github.com/flutter/flutter/issues/138028)
void PlaygroundImplVK::InitGlobalVulkanInstance() {
  if (global_instance_) {
    return;
  }

  VULKAN_HPP_DEFAULT_DISPATCHER.init(::glfwGetInstanceProcAddress);

  vk::ApplicationInfo application_info;
  application_info.setApplicationVersion(VK_API_VERSION_1_0);
  application_info.setApiVersion(VK_API_VERSION_1_1);
  application_info.setEngineVersion(VK_API_VERSION_1_0);
  application_info.setPEngineName("PlaygroundImplVK");
  application_info.setPApplicationName("PlaygroundImplVK");

  auto caps = std::shared_ptr<CapabilitiesVK>(
      new CapabilitiesVK(/*enable_validations=*/true));
  FML_DCHECK(caps->IsValid());

  std::optional<std::vector<std::string>> enabled_layers =
      caps->GetEnabledLayers();
  std::optional<std::vector<std::string>> enabled_extensions =
      caps->GetEnabledInstanceExtensions();
  FML_DCHECK(enabled_layers.has_value() && enabled_extensions.has_value());

  std::vector<const char*> enabled_layers_c;
  std::vector<const char*> enabled_extensions_c;

  if (enabled_layers.has_value()) {
    for (const auto& layer : enabled_layers.value()) {
      enabled_layers_c.push_back(layer.c_str());
    }
  }

  if (enabled_extensions.has_value()) {
    for (const auto& ext : enabled_extensions.value()) {
      enabled_extensions_c.push_back(ext.c_str());
    }
  }

  vk::InstanceCreateFlags instance_flags = {};
  instance_flags |= vk::InstanceCreateFlagBits::eEnumeratePortabilityKHR;
  vk::InstanceCreateInfo instance_info;
  instance_info.setPEnabledLayerNames(enabled_layers_c);
  instance_info.setPEnabledExtensionNames(enabled_extensions_c);
  instance_info.setPApplicationInfo(&application_info);
  instance_info.setFlags(instance_flags);
  auto instance_result = vk::createInstanceUnique(instance_info);
  FML_CHECK(instance_result.result == vk::Result::eSuccess)
      << "Unable to initialize global Vulkan instance";
  global_instance_ = std::move(instance_result.value);
}

fml::Status PlaygroundImplVK::SetCapabilities(
    const std::shared_ptr<Capabilities>& capabilities) {
  return fml::Status(
      fml::StatusCode::kUnimplemented,
      "PlaygroundImplVK doesn't support setting the capabilities.");
}

bool PlaygroundImplVK::IsVulkanDriverPresent() {
  if (::glfwVulkanSupported()) {
    return true;
  }
#ifdef TARGET_OS_MAC
  FML_LOG(ERROR) << "Attempting to initialize a Vulkan playground on macOS "
                    "where Vulkan cannot be found. It can be installed via "
                    "MoltenVK and make sure to install it globally so "
                    "dlopen can find it.";
#else   // TARGET_OS_MAC
  FML_LOG(ERROR) << "Attempting to initialize a Vulkan playground on a system "
                    "that does not support Vulkan.";
#endif  // TARGET_OS_MAC
  return false;
}

}  // namespace impeller
