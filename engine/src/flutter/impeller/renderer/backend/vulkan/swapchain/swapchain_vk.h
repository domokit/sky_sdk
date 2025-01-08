// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_SWAPCHAIN_SWAPCHAIN_VK_H_
#define FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_SWAPCHAIN_SWAPCHAIN_VK_H_

#include <memory>

#include "flutter/fml/build_config.h"
#include "impeller/geometry/size.h"
#include "impeller/renderer/backend/vulkan/vk.h"
#include "impeller/renderer/command_buffer.h"
#include "impeller/renderer/context.h"
#include "impeller/renderer/surface.h"

#if FML_OS_ANDROID
#include "impeller/toolkit/android/native_window.h"
#endif  // FML_OS_ANDROID

namespace impeller {

//------------------------------------------------------------------------------
/// @brief      A swapchain that adapts to the underlying surface going out of
///             date. If the caller cannot acquire the next drawable, it is due
///             to an unrecoverable error and the swapchain must be recreated
///             with a new surface.
///
class SwapchainVK {
 public:
  static std::shared_ptr<SwapchainVK> Create(
      const std::shared_ptr<Context>& context,
      vk::UniqueSurfaceKHR surface,
      const ISize& size,
      bool enable_msaa = true);

#if FML_OS_ANDROID
  static std::shared_ptr<SwapchainVK> Create(
      const std::shared_ptr<Context>& context,
      ANativeWindow* window,
      bool enable_msaa = true);
#endif  // FML_OS_ANDROID

  virtual ~SwapchainVK();

  SwapchainVK(const SwapchainVK&) = delete;

  SwapchainVK& operator=(const SwapchainVK&) = delete;

  virtual bool IsValid() const = 0;

  virtual std::unique_ptr<Surface> AcquireNextDrawable() = 0;

  virtual vk::Format GetSurfaceFormat() const = 0;

  virtual void AddFinalCommandBuffer(
      std::shared_ptr<CommandBuffer> cmd_buffer) const = 0;

  /// @brief Mark the current swapchain configuration as dirty, forcing it to be
  ///        recreated on the next frame.
  virtual void UpdateSurfaceSize(const ISize& size) = 0;

 protected:
  SwapchainVK();
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_SWAPCHAIN_SWAPCHAIN_VK_H_
