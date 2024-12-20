// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/android/android_surface_vk_impeller.h"

#include <memory>
#include <utility>

#include "flutter/fml/concurrent_message_loop.h"
#include "flutter/fml/logging.h"
#include "flutter/fml/memory/ref_ptr.h"
#include "flutter/impeller/renderer/backend/vulkan/context_vk.h"
#include "flutter/impeller/renderer/backend/vulkan/swapchain/swapchain_vk.h"
#include "flutter/shell/gpu/gpu_surface_vulkan_impeller.h"
#include "flutter/vulkan/vulkan_native_surface_android.h"

namespace flutter {

AndroidSurfaceVKImpeller::AndroidSurfaceVKImpeller(
    const std::shared_ptr<AndroidContextVKImpeller>& android_context) {
  is_valid_ = android_context->IsValid();

  auto& context_vk =
      impeller::ContextVK::Cast(*android_context->GetImpellerContext());
  surface_context_vk_ = context_vk.CreateSurfaceContext();
  eager_gpu_surface_ =
      std::make_unique<GPUSurfaceVulkanImpeller>(nullptr, surface_context_vk_);
}

AndroidSurfaceVKImpeller::~AndroidSurfaceVKImpeller() = default;

bool AndroidSurfaceVKImpeller::IsValid() const {
  return is_valid_;
}

void AndroidSurfaceVKImpeller::TeardownOnScreenContext() {
  // Nothing to do.
}

std::unique_ptr<Surface> AndroidSurfaceVKImpeller::CreateGPUSurface(
    GrDirectContext* gr_context) {
  if (!IsValid()) {
    return nullptr;
  }

  if (!native_window_ || !native_window_->IsValid()) {
    return nullptr;
  }

  if (eager_gpu_surface_) {
    auto gpu_surface = std::move(eager_gpu_surface_);
    if (!gpu_surface->IsValid()) {
      return nullptr;
    }
    return gpu_surface;
  }

  std::unique_ptr<GPUSurfaceVulkanImpeller> gpu_surface =
      std::make_unique<GPUSurfaceVulkanImpeller>(nullptr, surface_context_vk_);

  if (!gpu_surface->IsValid()) {
    return nullptr;
  }

  return gpu_surface;
}

bool AndroidSurfaceVKImpeller::OnScreenSurfaceResize(const SkISize& size) {
  surface_context_vk_->UpdateSurfaceSize(
      impeller::ISize{size.width(), size.height()});
  return true;
}

bool AndroidSurfaceVKImpeller::ResourceContextMakeCurrent() {
  return true;
}

bool AndroidSurfaceVKImpeller::ResourceContextClearCurrent() {
  return true;
}

bool AndroidSurfaceVKImpeller::SetNativeWindow(
    fml::RefPtr<AndroidNativeWindow> window) {
  if (window && (native_window_ == window)) {
    return OnScreenSurfaceResize(window->GetSize());
  }

  native_window_ = nullptr;

  if (!window || !window->IsValid()) {
    return false;
  }

  auto swapchain = impeller::SwapchainVK::Create(
      std::reinterpret_pointer_cast<impeller::Context>(
          surface_context_vk_->GetParent()),
      window->handle());

  if (surface_context_vk_->SetSwapchain(std::move(swapchain))) {
    native_window_ = std::move(window);
    return true;
  }

  return false;
}

std::shared_ptr<impeller::Context>
AndroidSurfaceVKImpeller::GetImpellerContext() {
  return surface_context_vk_;
}

}  // namespace flutter
