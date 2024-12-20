// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_SWAPCHAIN_AHB_EXTERNAL_FENCE_VK_H_
#define FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_SWAPCHAIN_AHB_EXTERNAL_FENCE_VK_H_

#include "flutter/fml/unique_fd.h"
#include "impeller/renderer/backend/vulkan/shared_object_vk.h"
#include "impeller/renderer/backend/vulkan/vk.h"
#include "impeller/renderer/context.h"

namespace impeller {

//------------------------------------------------------------------------------
/// @brief      A Vulkan fence that can be exported as a platform specific file
///             descriptor.
///
///             The fences are exported as sync file descriptors.
///
/// @warning    Only fences that have been signaled or have a single operation
///             pending can be exported. Make sure to submit a fence signalling
///             operation to a queue before attempted to obtain a file
///             descriptor for the fence. See
///             VUID-VkFenceGetFdInfoKHR-handleType-01454 for additional details
///             on the implementation.
///
class ExternalFenceVK {
 public:
  //----------------------------------------------------------------------------
  /// @brief      Create a new un-signaled fence that can be exported as a sync
  ///             file descriptor.
  ///
  /// @param[in]  context  The device context.
  ///
  explicit ExternalFenceVK(const std::shared_ptr<Context>& context);

  ~ExternalFenceVK();

  ExternalFenceVK(const ExternalFenceVK&) = delete;

  ExternalFenceVK& operator=(const ExternalFenceVK&) = delete;

  //----------------------------------------------------------------------------
  /// @brief      If a valid fence could be created.
  ///
  /// @return     True if valid, False otherwise.
  ///
  bool IsValid() const;

  //----------------------------------------------------------------------------
  /// @brief      Create a new sync file descriptor for the underlying fence.
  ///             The fence must already be signaled or have a signal operation
  ///             pending in a queue. There are no checks for this in the
  ///             implementation and only Vulkan validation will catch such a
  ///             misuse and undefined behavior.
  ///
  /// @warning    Implementations are also allowed to return invalid file
  ///             descriptors in case a fence has already been signaled. So it
  ///             is not necessary an error to obtain an invalid descriptor from
  ///             this call. For APIs that are meant to consume such
  ///             descriptors, pass -1 as the file handle.
  ///
  ///             Since this call can return an invalid FD even in case of
  ///             success, make sure to make the `IsValid` check before
  ///             attempting to export a FD.
  ///
  /// @return     A (potentially invalid even in case of success) file
  ///             descriptor.
  ///
  fml::UniqueFD CreateFD() const;

  const vk::Fence& GetHandle() const;

  const SharedHandleVK<vk::Fence>& GetSharedHandle() const;

 private:
  SharedHandleVK<vk::Fence> fence_;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_SWAPCHAIN_AHB_EXTERNAL_FENCE_VK_H_
