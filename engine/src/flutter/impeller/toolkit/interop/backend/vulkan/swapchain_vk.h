// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_TOOLKIT_INTEROP_BACKEND_VULKAN_SWAPCHAIN_VK_H_
#define FLUTTER_IMPELLER_TOOLKIT_INTEROP_BACKEND_VULKAN_SWAPCHAIN_VK_H_

#include "impeller/renderer/backend/vulkan/swapchain/swapchain_vk.h"
#include "impeller/toolkit/interop/backend/vulkan/surface_vk.h"
#include "impeller/toolkit/interop/context.h"
#include "impeller/toolkit/interop/impeller.h"
#include "impeller/toolkit/interop/object.h"

namespace impeller::interop {

class SwapchainVK final
    : public Object<SwapchainVK,
                    IMPELLER_INTERNAL_HANDLE_NAME(ImpellerVulkanSwapchain)> {
 public:
  SwapchainVK(Context& context, VkSurfaceKHR surface);

  ~SwapchainVK();

  bool IsValid() const;

  SwapchainVK(const SwapchainVK&) = delete;

  SwapchainVK& operator=(const SwapchainVK&) = delete;

  ScopedObject<SurfaceVK> AcquireNextSurface();

 private:
  ScopedObject<Context> context_;
  std::shared_ptr<impeller::SwapchainVK> swapchain_;
};

}  // namespace impeller::interop

#endif  // FLUTTER_IMPELLER_TOOLKIT_INTEROP_BACKEND_VULKAN_SWAPCHAIN_VK_H_
