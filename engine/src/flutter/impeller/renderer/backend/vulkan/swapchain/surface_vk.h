// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_SWAPCHAIN_SURFACE_VK_H_
#define FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_SWAPCHAIN_SURFACE_VK_H_

#include <memory>

#include "impeller/renderer/backend/vulkan/context_vk.h"
#include "impeller/renderer/backend/vulkan/swapchain/swapchain_transients_vk.h"
#include "impeller/renderer/backend/vulkan/texture_source_vk.h"
#include "impeller/renderer/surface.h"

namespace impeller {

class SurfaceVK final : public Surface {
 public:
  using SwapCallback = std::function<bool(void)>;

  /// @brief Wrap the swapchain image in a Surface, which provides the
  ///        additional configuration required for usage as on onscreen render
  ///        target by Impeller.
  ///
  ///        This creates the associated MSAA and depth+stencil texture.
  static std::unique_ptr<SurfaceVK> WrapSwapchainImage(
      const std::shared_ptr<SwapchainTransientsVK>& transients,
      const std::shared_ptr<TextureSourceVK>& swapchain_image,
      SwapCallback swap_callback);

  // |Surface|
  ~SurfaceVK() override;

 private:
  SwapCallback swap_callback_;

  SurfaceVK(const RenderTarget& target, SwapCallback swap_callback);

  // |Surface|
  bool Present() const override;

  SurfaceVK(const SurfaceVK&) = delete;

  SurfaceVK& operator=(const SurfaceVK&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_SWAPCHAIN_SURFACE_VK_H_
