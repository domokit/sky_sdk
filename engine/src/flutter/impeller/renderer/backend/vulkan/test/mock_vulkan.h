// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_TEST_MOCK_VULKAN_H_
#define FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_TEST_MOCK_VULKAN_H_

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "impeller/base/thread.h"
#include "impeller/renderer/backend/vulkan/context_vk.h"
#include "vulkan/vulkan_core.h"
#include "vulkan/vulkan_enums.hpp"

namespace impeller {
namespace testing {

std::shared_ptr<std::vector<std::string>> GetMockVulkanFunctions(
    VkDevice device);

// A test-controlled version of |vk::Fence|.
class MockFence final {
 public:
  MockFence() = default;

  // Returns the result that was set in the constructor or |SetStatus|.
  VkResult GetStatus() { return static_cast<VkResult>(result_.load()); }

  // Sets the result that will be returned by `GetFenceStatus`.
  void SetStatus(vk::Result result) { result_ = result; }

  // Sets the result that will be returned by `GetFenceStatus`.
  static void SetStatus(vk::UniqueFence& fence, vk::Result result) {
    // Cast the fence to a MockFence and set the result.
    VkFence raw_fence = fence.get();
    MockFence* mock_fence = reinterpret_cast<MockFence*>(raw_fence);
    mock_fence->SetStatus(result);
  }

  // Gets a raw pointer to manipulate the fence after it's been moved.
  static MockFence* GetRawPointer(vk::UniqueFence& fence) {
    // Cast the fence to a MockFence and get the result.
    VkFence raw_fence = fence.get();
    MockFence* mock_fence = reinterpret_cast<MockFence*>(raw_fence);
    return mock_fence;
  }

 private:
  std::atomic<vk::Result> result_ = vk::Result::eSuccess;

  MockFence(const MockFence&) = delete;

  MockFence& operator=(const MockFence&) = delete;
};

class MockVulkanContextBuilder {
 public:
  MockVulkanContextBuilder();

  //------------------------------------------------------------------------------
  /// @brief      Create a Vulkan context with Vulkan functions mocked. The
  ///             caller is given a chance to tinker on the settings right
  ///             before a context is created.
  ///
  /// @return     A context if one can be created.
  ///
  std::shared_ptr<ContextVK> Build();

  /// A callback that allows the modification of the ContextVK::Settings before
  /// the context is made.
  MockVulkanContextBuilder& SetSettingsCallback(
      const std::function<void(ContextVK::Settings&)>& settings_callback) {
    settings_callback_ = settings_callback;
    return *this;
  }

  MockVulkanContextBuilder& SetInstanceExtensions(
      const std::vector<std::string>& instance_extensions) {
    instance_extensions_ = instance_extensions;
    return *this;
  }

  MockVulkanContextBuilder& SetInstanceLayers(
      const std::vector<std::string>& instance_layers) {
    instance_layers_ = instance_layers;
    return *this;
  }

  /// Set the behavior of vkGetPhysicalDeviceFormatProperties, which needs to
  /// respond differently for different formats.
  MockVulkanContextBuilder& SetPhysicalDeviceFormatPropertiesCallback(
      std::function<void(VkPhysicalDevice physicalDevice,
                         VkFormat format,
                         VkFormatProperties* pFormatProperties)>
          format_properties_callback) {
    format_properties_callback_ = std::move(format_properties_callback);
    return *this;
  }

  MockVulkanContextBuilder& SetPhysicalPropertiesCallback(
      std::function<void(VkPhysicalDevice device,
                         VkPhysicalDeviceProperties* physicalProperties)>
          physical_properties_callback) {
    physical_properties_callback_ = std::move(physical_properties_callback);
    return *this;
  }

  MockVulkanContextBuilder SetEmbedderData(
      const ContextVK::EmbedderData& embedder_data) {
    embedder_data_ = embedder_data;
    return *this;
  }

 private:
  std::function<void(ContextVK::Settings&)> settings_callback_;
  std::vector<std::string> instance_extensions_;
  std::vector<std::string> instance_layers_;
  std::optional<ContextVK::EmbedderData> embedder_data_;
  std::function<void(VkPhysicalDevice physicalDevice,
                     VkFormat format,
                     VkFormatProperties* pFormatProperties)>
      format_properties_callback_;
  std::function<void(VkPhysicalDevice device,
                     VkPhysicalDeviceProperties* physicalProperties)>
      physical_properties_callback_;
};

/// @brief Override the image size returned by all swapchain images.
void SetSwapchainImageSize(ISize size);

}  // namespace testing
}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_TEST_MOCK_VULKAN_H_
