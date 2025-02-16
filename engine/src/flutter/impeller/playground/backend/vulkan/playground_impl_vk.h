// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_PLAYGROUND_BACKEND_VULKAN_PLAYGROUND_IMPL_VK_H_
#define FLUTTER_IMPELLER_PLAYGROUND_BACKEND_VULKAN_PLAYGROUND_IMPL_VK_H_

#include "impeller/playground/playground_impl.h"
#include "impeller/renderer/backend/vulkan/vk.h"

namespace impeller {

class PlaygroundImplVK final : public PlaygroundImpl {
 public:
  static bool IsVulkanDriverPresent();

  explicit PlaygroundImplVK(PlaygroundSwitches switches);

  ~PlaygroundImplVK();

  fml::Status SetCapabilities(
      const std::shared_ptr<Capabilities>& capabilities) override;

 private:
  std::shared_ptr<Context> context_;

  // Windows management.
  static void DestroyWindowHandle(WindowHandle handle);
  using UniqueHandle = std::unique_ptr<void, decltype(&DestroyWindowHandle)>;
  UniqueHandle handle_;
  ISize size_ = {1, 1};

  // A global Vulkan instance which ensures that the Vulkan library will remain
  // loaded throughout the lifetime of the process.
  static VkInstance global_instance_;

  // |PlaygroundImpl|
  std::shared_ptr<Context> GetContext() const override;

  // |PlaygroundImpl|
  WindowHandle GetWindowHandle() const override;

  // |PlaygroundImpl|
  std::unique_ptr<Surface> AcquireSurfaceFrame(
      std::shared_ptr<Context> context) override;

  // |PlaygroundImpl|
  Playground::VKProcAddressResolver CreateVKProcAddressResolver()
      const override;

  PlaygroundImplVK(const PlaygroundImplVK&) = delete;

  PlaygroundImplVK& operator=(const PlaygroundImplVK&) = delete;

  static void InitGlobalVulkanInstance();
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_PLAYGROUND_BACKEND_VULKAN_PLAYGROUND_IMPL_VK_H_
