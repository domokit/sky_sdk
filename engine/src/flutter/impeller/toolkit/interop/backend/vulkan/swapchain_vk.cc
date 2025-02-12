// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/toolkit/interop/backend/vulkan/swapchain_vk.h"

#include "impeller/renderer/backend/vulkan/context_vk.h"

namespace impeller::interop {

SwapchainVK::SwapchainVK(Context& context, VkSurfaceKHR c_surface)
    : context_(Ref(&context)) {
  if (!context.IsVulkan()) {
    VALIDATION_LOG << "Context is not Vulkan.";
    return;
  }

  if (!c_surface) {
    VALIDATION_LOG << "Invalid surface.";
    return;
  }

  // Creating a unique object from a raw handle requires fetching the owner
  // manually.
  auto surface = vk::UniqueSurfaceKHR(
      vk::SurfaceKHR{c_surface},
      impeller::ContextVK::Cast(*context_->GetContext()).GetInstance());
  auto swapchain = impeller::SwapchainVK::Create(context.GetContext(),  //
                                                 std::move(surface),    //
                                                 ISize::MakeWH(1, 1)    //
  );
  if (!swapchain) {
    VALIDATION_LOG << "Could not create Vulkan swapchain.";
    return;
  }
  swapchain_ = std::move(swapchain);
}

SwapchainVK::~SwapchainVK() = default;

bool SwapchainVK::IsValid() const {
  return swapchain_ && swapchain_->IsValid();
}

ScopedObject<SurfaceVK> SwapchainVK::AcquireNextSurface() {
  if (!IsValid()) {
    return nullptr;
  }

  auto impeller_surface = swapchain_->AcquireNextDrawable();
  if (!impeller_surface) {
    VALIDATION_LOG << "Could not acquire next drawable.";
    return nullptr;
  }

  auto surface = Create<SurfaceVK>(*context_, std::move(impeller_surface));
  if (!surface || !surface->IsValid()) {
    VALIDATION_LOG << "Could not create valid surface.";
    return nullptr;
  }

  return surface;
}

}  // namespace impeller::interop
