// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_SWAPCHAIN_KHR_KHR_SWAPCHAIN_VK_H_
#define FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_SWAPCHAIN_KHR_KHR_SWAPCHAIN_VK_H_

#include <memory>

#include "impeller/geometry/size.h"
#include "impeller/renderer/backend/vulkan/swapchain/swapchain_vk.h"
#include "impeller/renderer/backend/vulkan/vk.h"
#include "impeller/renderer/context.h"
#include "impeller/renderer/surface.h"

namespace impeller {

class KHRSwapchainImplVK;

//------------------------------------------------------------------------------
/// @brief      A swapchain implemented backed by VK_KHR_swapchain and
///             VK_KHR_surface.
///
class KHRSwapchainVK final : public SwapchainVK {
 public:
  ~KHRSwapchainVK();

  // |SwapchainVK|
  bool IsValid() const override;

  // |SwapchainVK|
  std::unique_ptr<Surface> AcquireNextDrawable() override;

  // |SwapchainVK|
  vk::Format GetSurfaceFormat() const override;

  // |SwapchainVK|
  void UpdateSurfaceSize(const ISize& size) override;

 private:
  friend class SwapchainVK;

  std::shared_ptr<KHRSwapchainImplVK> impl_;
  ISize size_;
  const bool enable_msaa_;

  KHRSwapchainVK(const std::shared_ptr<Context>& context,
                 vk::UniqueSurfaceKHR surface,
                 const ISize& size,
                 bool enable_msaa);

  KHRSwapchainVK(const KHRSwapchainVK&) = delete;

  KHRSwapchainVK& operator=(const KHRSwapchainVK&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_SWAPCHAIN_KHR_KHR_SWAPCHAIN_VK_H_
