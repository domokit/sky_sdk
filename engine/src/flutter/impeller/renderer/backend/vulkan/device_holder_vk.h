// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_DEVICE_HOLDER_VK_H_
#define FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_DEVICE_HOLDER_VK_H_

#include "impeller/renderer/backend/vulkan/vk.h"  // IWYU pragma: keep.

namespace impeller {

//------------------------------------------------------------------------------
/// @brief      Holds a strong reference to the underlying logical Vulkan
///             device. This comes in handy when the context is being torn down
///             and the various components on different threads may need to
///             orchestrate safe shutdown.
///
class DeviceHolderVK {
 public:
  virtual ~DeviceHolderVK() = default;

  virtual const vk::Device& GetDevice() const = 0;

  virtual const vk::PhysicalDevice& GetPhysicalDevice() const = 0;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_DEVICE_HOLDER_VK_H_
