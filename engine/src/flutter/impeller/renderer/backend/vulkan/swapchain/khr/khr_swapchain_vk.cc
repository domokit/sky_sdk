// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/backend/vulkan/swapchain/khr/khr_swapchain_vk.h"

#include "flutter/fml/build_config.h"
#include "flutter/fml/trace_event.h"
#include "impeller/base/validation.h"
#include "impeller/renderer/backend/vulkan/swapchain/khr/khr_swapchain_impl_vk.h"

namespace impeller {

KHRSwapchainVK::KHRSwapchainVK(const std::shared_ptr<Context>& context,
                               vk::UniqueSurfaceKHR surface,
                               const ISize& size,
                               bool enable_msaa)
    : size_(size), enable_msaa_(enable_msaa) {
  auto impl = KHRSwapchainImplVK::Create(context,             //
                                         std::move(surface),  //
                                         size_,               //
                                         enable_msaa_         //
  );
  if (!impl || !impl->IsValid()) {
    VALIDATION_LOG << "Failed to create SwapchainVK implementation.";
    return;
  }
  impl_ = std::move(impl);
}

KHRSwapchainVK::~KHRSwapchainVK() = default;

bool KHRSwapchainVK::IsValid() const {
  return impl_ ? impl_->IsValid() : false;
}

void KHRSwapchainVK::UpdateSurfaceSize(const ISize& size) {
  // Update the size of the swapchain. On the next acquired drawable,
  // the sizes may no longer match, forcing the swapchain to be recreated.
  size_ = size;
}

void KHRSwapchainVK::AddFinalCommandBuffer(
    std::shared_ptr<CommandBuffer> cmd_buffer) const {
  impl_->AddFinalCommandBuffer(std::move(cmd_buffer));
}

std::unique_ptr<Surface> KHRSwapchainVK::AcquireNextDrawable() {
  return AcquireNextDrawable(0u);
}

std::unique_ptr<Surface> KHRSwapchainVK::AcquireNextDrawable(
    size_t resize_retry_count) {
  if (!IsValid()) {
    return nullptr;
  }

  TRACE_EVENT0("impeller", __FUNCTION__);

  auto result = impl_->AcquireNextDrawable();
  if (!result.out_of_date && size_ == impl_->GetSize()) {
    return std::move(result.surface);
  }

// When the swapchain says its out-of-date, we attempt to read the underlying
// surface size and re-create the swapchain at that size automatically (subject
// to a specific number of retries). However, on some platforms, the surface
// size reported by the Vulkan API may be stale for several frames. Those
// platforms must explicitly set the swapchain size using out-of-band (to
// Vulkan) APIs.
//
// TODO(163070): Expose the API to set surface size in impeller.h
#if !FML_OS_ANDROID
  constexpr const size_t kMaxResizeAttempts = 3u;
  if (resize_retry_count == kMaxResizeAttempts) {
    VALIDATION_LOG << "Attempted to resize the swapchain" << kMaxResizeAttempts
                   << " time unsuccessfully. This platform likely doesn't "
                      "support returning the current swapchain extents and "
                      "must recreate the swapchain using the actual size.";
    return nullptr;
  }

  size_ = impl_->GetCurrentUnderlyingSurfaceSize().value_or(size_);
#endif  // !FML_OS_ANDROID

  TRACE_EVENT0("impeller", "RecreateSwapchain");

  // This swapchain implementation indicates that it is out of date. Tear it
  // down and make a new one.
  auto context = impl_->GetContext();
  auto [surface, old_swapchain] = impl_->DestroySwapchain();

  auto new_impl = KHRSwapchainImplVK::Create(context,             //
                                             std::move(surface),  //
                                             size_,               //
                                             enable_msaa_,        //
                                             *old_swapchain       //
  );
  if (!new_impl || !new_impl->IsValid()) {
    VALIDATION_LOG << "Could not update swapchain.";
    // The old swapchain is dead because we took its surface. This is
    // unrecoverable.
    impl_.reset();
    return nullptr;
  }
  impl_ = std::move(new_impl);

  //----------------------------------------------------------------------------
  /// We managed to recreate the swapchain in the new configuration. Try again.
  ///
  return AcquireNextDrawable(resize_retry_count + 1);
}

vk::Format KHRSwapchainVK::GetSurfaceFormat() const {
  return IsValid() ? impl_->GetSurfaceFormat() : vk::Format::eUndefined;
}

}  // namespace impeller
