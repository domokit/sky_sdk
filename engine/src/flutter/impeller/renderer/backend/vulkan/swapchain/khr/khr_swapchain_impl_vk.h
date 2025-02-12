// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_SWAPCHAIN_KHR_KHR_SWAPCHAIN_IMPL_VK_H_
#define FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_SWAPCHAIN_KHR_KHR_SWAPCHAIN_IMPL_VK_H_

#include <cstdint>
#include <memory>

#include "impeller/geometry/size.h"
#include "impeller/renderer/backend/vulkan/swapchain/swapchain_transients_vk.h"
#include "impeller/renderer/backend/vulkan/vk.h"

namespace impeller {

class Context;
class KHRSwapchainImageVK;
class Surface;
struct KHRFrameSynchronizerVK;

//------------------------------------------------------------------------------
/// @brief      An instance of a swapchain that does NOT adapt to going out of
///             date with the underlying surface. Errors will be indicated when
///             the next drawable is acquired from this implementation of the
///             swapchain. If the error is due the swapchain going out of date,
///             the caller must recreate another instance by optionally
///             stealing this implementations guts.
///
class KHRSwapchainImplVK final
    : public std::enable_shared_from_this<KHRSwapchainImplVK> {
 public:
  static std::shared_ptr<KHRSwapchainImplVK> Create(
      const std::shared_ptr<Context>& context,
      vk::UniqueSurfaceKHR surface,
      const ISize& size,
      bool enable_msaa = true,
      vk::SwapchainKHR old_swapchain = VK_NULL_HANDLE);

  ~KHRSwapchainImplVK();

  bool IsValid() const;

  struct AcquireResult {
    std::unique_ptr<Surface> surface;
    bool out_of_date = false;

    explicit AcquireResult(bool p_out_of_date = false)
        : out_of_date(p_out_of_date) {}

    explicit AcquireResult(std::unique_ptr<Surface> p_surface)
        : surface(std::move(p_surface)) {}
  };

  AcquireResult AcquireNextDrawable();

  vk::Format GetSurfaceFormat() const;

  std::shared_ptr<Context> GetContext() const;

  std::pair<vk::UniqueSurfaceKHR, vk::UniqueSwapchainKHR> DestroySwapchain();

  const ISize& GetSize() const;

  void AddFinalCommandBuffer(std::shared_ptr<CommandBuffer> cmd_buffer);

  std::optional<ISize> GetCurrentUnderlyingSurfaceSize() const;

 private:
  std::weak_ptr<Context> context_;
  vk::UniqueSurfaceKHR surface_;
  vk::Format surface_format_ = vk::Format::eUndefined;
  vk::UniqueSwapchainKHR swapchain_;
  std::shared_ptr<SwapchainTransientsVK> transients_;
  std::vector<std::shared_ptr<KHRSwapchainImageVK>> images_;
  std::vector<std::unique_ptr<KHRFrameSynchronizerVK>> synchronizers_;
  size_t current_frame_ = 0u;
  ISize size_;
  bool enable_msaa_ = true;
  bool is_valid_ = false;

  KHRSwapchainImplVK(const std::shared_ptr<Context>& context,
                     vk::UniqueSurfaceKHR surface,
                     const ISize& size,
                     bool enable_msaa,
                     vk::SwapchainKHR old_swapchain);

  bool Present(const std::shared_ptr<KHRSwapchainImageVK>& image,
               uint32_t index);

  void WaitIdle() const;

  KHRSwapchainImplVK(const KHRSwapchainImplVK&) = delete;

  KHRSwapchainImplVK& operator=(const KHRSwapchainImplVK&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_SWAPCHAIN_KHR_KHR_SWAPCHAIN_IMPL_VK_H_
