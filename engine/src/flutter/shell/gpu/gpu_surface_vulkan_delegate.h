// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_GPU_GPU_SURFACE_VULKAN_DELEGATE_H_
#define FLUTTER_SHELL_GPU_GPU_SURFACE_VULKAN_DELEGATE_H_

#include "flutter/fml/memory/ref_ptr.h"
#include "flutter/shell/platform/embedder/embedder.h"
#include "flutter/vulkan/procs/vulkan_proc_table.h"
#include "flutter/vulkan/vulkan_device.h"
#include "flutter/vulkan/vulkan_image.h"
#include "third_party/skia/include/core/SkSize.h"

namespace flutter {

//------------------------------------------------------------------------------
/// @brief      Interface implemented by all platform surfaces that can present
///             a Vulkan backing store to the "screen". The GPU surface
///             abstraction (which abstracts the client rendering API) uses this
///             delegation pattern to tell the platform surface (which abstracts
///             how backing stores fulfilled by the selected client rendering
///             API end up on the "screen" on a particular platform) when the
///             rasterizer needs to allocate and present the Vulkan backing
///             store.
///
/// @see        |EmbedderSurfaceVulkan|.
///
class GPUSurfaceVulkanDelegate {
 public:
  virtual ~GPUSurfaceVulkanDelegate();

  /// @brief  Obtain a reference to the Vulkan implementation's proc table.
  ///
  virtual const vulkan::VulkanProcTable& vk() = 0;

  /// @brief  Called by the engine to fetch a VkImage for writing the next
  ///         frame.
  ///
  virtual FlutterVulkanImage AcquireImage(const SkISize& size) = 0;

  /// @brief  Called by the engine once a frame has been rendered to the image
  ///         and it's ready to be bound for further reading/writing.
  ///
  virtual bool PresentImage(VkImage image, VkFormat format) = 0;
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_GPU_GPU_SURFACE_VULKAN_DELEGATE_H_
