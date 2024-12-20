// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_GPU_GPU_SURFACE_VULKAN_H_
#define FLUTTER_SHELL_GPU_GPU_SURFACE_VULKAN_H_

#include <memory>

#include "flutter/flow/surface.h"
#include "flutter/fml/macros.h"
#include "flutter/fml/memory/weak_ptr.h"
#include "flutter/shell/gpu/gpu_surface_vulkan_delegate.h"
#include "flutter/vulkan/vulkan_backbuffer.h"
#include "flutter/vulkan/vulkan_native_surface.h"
#include "flutter/vulkan/vulkan_window.h"

#include "third_party/skia/include/core/SkRefCnt.h"
#include "third_party/skia/include/core/SkSurface.h"

namespace flutter {

//------------------------------------------------------------------------------
/// @brief  A GPU surface backed by VkImages provided by a
///         GPUSurfaceVulkanDelegate.
///
class GPUSurfaceVulkan : public Surface {
 public:
  //------------------------------------------------------------------------------
  /// @brief      Create a GPUSurfaceVulkan while letting it reuse an existing
  ///             GrDirectContext.
  ///
  GPUSurfaceVulkan(GPUSurfaceVulkanDelegate* delegate,
                   const sk_sp<GrDirectContext>& context,
                   bool render_to_surface);

  ~GPUSurfaceVulkan() override;

  // |Surface|
  bool IsValid() override;

  // |Surface|
  std::unique_ptr<SurfaceFrame> AcquireFrame(const SkISize& size) override;

  // |Surface|
  SkMatrix GetRootTransformation() const override;

  // |Surface|
  GrDirectContext* GetContext() override;

  static SkColorType ColorTypeFromFormat(const VkFormat format);

 private:
  GPUSurfaceVulkanDelegate* delegate_;
  sk_sp<GrDirectContext> skia_context_;
  bool render_to_surface_;

  fml::WeakPtrFactory<GPUSurfaceVulkan> weak_factory_;

  sk_sp<SkSurface> CreateSurfaceFromVulkanImage(const VkImage image,
                                                const VkFormat format,
                                                const SkISize& size);

  FML_DISALLOW_COPY_AND_ASSIGN(GPUSurfaceVulkan);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_GPU_GPU_SURFACE_VULKAN_H_
