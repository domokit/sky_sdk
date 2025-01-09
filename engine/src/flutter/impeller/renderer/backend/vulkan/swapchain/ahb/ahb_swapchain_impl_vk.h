// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_SWAPCHAIN_AHB_AHB_SWAPCHAIN_IMPL_VK_H_
#define FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_SWAPCHAIN_AHB_AHB_SWAPCHAIN_IMPL_VK_H_

#include <memory>

#include "flutter/fml/closure.h"
#include "flutter/fml/synchronization/semaphore.h"
#include "impeller/base/thread.h"
#include "impeller/renderer/backend/vulkan/android/ahb_texture_source_vk.h"
#include "impeller/renderer/backend/vulkan/swapchain/ahb/ahb_texture_pool_vk.h"
#include "impeller/renderer/backend/vulkan/swapchain/ahb/external_fence_vk.h"
#include "impeller/renderer/backend/vulkan/swapchain/swapchain_transients_vk.h"
#include "impeller/renderer/surface.h"
#include "impeller/toolkit/android/hardware_buffer.h"
#include "impeller/toolkit/android/surface_control.h"

namespace impeller {

//------------------------------------------------------------------------------
/// @brief      The implementation of a swapchain at a specific size. Resizes to
///             the surface will cause the instance of the swapchain impl at
///             that size to be discarded along with all its caches and
///             transients.
///
class AHBSwapchainImplVK final
    : public std::enable_shared_from_this<AHBSwapchainImplVK> {
 public:
  //----------------------------------------------------------------------------
  /// @brief      Create a swapchain of a specific size whose images will be
  ///             presented to the provided surface control.
  ///
  /// @param[in]  context          The context whose allocators will be used to
  ///                              create swapchain image resources.
  /// @param[in]  surface_control  The surface control to which the swapchain
  ///                              images will be presented.
  /// @param[in]  size             The size of the swapchain images. This is
  ///                              constant for the lifecycle of the swapchain
  ///                              impl.
  /// @param[in]  enable_msaa      If the swapchain images will be presented
  ///                              using a render target that enables MSAA. This
  ///                              allows for additional caching of transients.
  ///
  /// @return     A valid swapchain impl if one can be created. `nullptr`
  ///             otherwise.
  ///
  static std::shared_ptr<AHBSwapchainImplVK> Create(
      const std::weak_ptr<Context>& context,
      std::weak_ptr<android::SurfaceControl> surface_control,
      const ISize& size,
      bool enable_msaa,
      size_t swapchain_image_count);

  ~AHBSwapchainImplVK();

  AHBSwapchainImplVK(const AHBSwapchainImplVK&) = delete;

  AHBSwapchainImplVK& operator=(const AHBSwapchainImplVK&) = delete;

  //----------------------------------------------------------------------------
  /// @return     The size of the swapchain images that will be displayed on the
  ///             surface control.
  ///
  const ISize& GetSize() const;

  //----------------------------------------------------------------------------
  /// @return     If the swapchain impl is valid. If it is not, the instance
  ///             must be discarded. There is no error recovery.
  ///
  bool IsValid() const;

  //----------------------------------------------------------------------------
  /// @brief      Get the descriptor used to create the hardware buffers that
  ///             will be displayed on the surface control.
  ///
  /// @return     The descriptor.
  ///
  const android::HardwareBufferDescriptor& GetDescriptor() const;

  //----------------------------------------------------------------------------
  /// @brief      Acquire the next surface that can be used to present to the
  ///             swapchain.
  ///
  /// @return     A surface if one can be created. If one cannot be created, it
  ///             is likely due to resource exhaustion.
  ///
  std::unique_ptr<Surface> AcquireNextDrawable();

  void AddFinalCommandBuffer(std::shared_ptr<CommandBuffer> cmd_buffer);

 private:
  using AutoSemaSignaler = std::shared_ptr<fml::ScopedCleanupClosure>;

  std::weak_ptr<android::SurfaceControl> surface_control_;
  android::HardwareBufferDescriptor desc_;
  std::shared_ptr<AHBTexturePoolVK> pool_;
  std::shared_ptr<SwapchainTransientsVK> transients_;
  // In C++20, this mutex can be replaced by the shared pointer specialization
  // of std::atomic.
  Mutex currently_displayed_texture_mutex_;
  std::shared_ptr<AHBTextureSourceVK> currently_displayed_texture_
      IPLR_GUARDED_BY(currently_displayed_texture_mutex_);
  std::shared_ptr<fml::Semaphore> pending_presents_;
  std::shared_ptr<CommandBuffer> pending_cmd_buffer_;
  bool is_valid_ = false;

  explicit AHBSwapchainImplVK(
      const std::weak_ptr<Context>& context,
      std::weak_ptr<android::SurfaceControl> surface_control,
      const ISize& size,
      bool enable_msaa,
      size_t swapchain_image_count);

  bool Present(const AutoSemaSignaler& signaler,
               const std::shared_ptr<AHBTextureSourceVK>& texture);

  vk::UniqueFence CreateRenderReadyFence(
      const std::shared_ptr<fml::UniqueFD>& fd) const;

  bool SubmitWaitForRenderReady(
      const std::shared_ptr<fml::UniqueFD>& render_ready_fence,
      const std::shared_ptr<AHBTextureSourceVK>& texture) const;

  std::shared_ptr<ExternalFenceVK> SubmitSignalForPresentReady(
      const std::shared_ptr<AHBTextureSourceVK>& texture) const;

  void OnTextureUpdatedOnSurfaceControl(
      const AutoSemaSignaler& signaler,
      std::shared_ptr<AHBTextureSourceVK> texture,
      ASurfaceTransactionStats* stats);
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_SWAPCHAIN_AHB_AHB_SWAPCHAIN_IMPL_VK_H_
