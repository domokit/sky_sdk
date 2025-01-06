// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_WORKAROUNDS_VK_H_
#define FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_WORKAROUNDS_VK_H_

#include "impeller/renderer/backend/vulkan/driver_info_vk.h"

namespace impeller {

/// A non-exhaustive set of driver specific workarounds.
struct WorkaroundsVK {
  // Adreno GPUs exhibit terrible performance when primitive
  // restart is used. This was confirmed up to Adreno 640 (Pixel 4).
  // Because this feature is fairly marginal, we disable it for _all_
  // Adreno GPUs until we have an upper bound for this bug.
  bool slow_primitive_restart_performance = false;

  /// Early 600 series Adreno drivers would deadlock if a command
  /// buffer submission had too much work attached to it, this
  /// requires the renderer to split up command buffers that could
  /// be logically combined.
  bool batch_submit_command_buffer_timeout = false;
};
WorkaroundsVK GetWorkarounds(DriverInfoVK& driver_info);

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_WORKAROUNDS_VK_H_
