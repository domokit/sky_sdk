// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_VULKAN_VULKAN_SKIA_PROC_TABLE_H_
#define FLUTTER_VULKAN_VULKAN_SKIA_PROC_TABLE_H_

#include "flutter/vulkan/procs/vulkan_proc_table.h"

#include "third_party/skia/include/gpu/vk/VulkanTypes.h"

namespace vulkan {

skgpu::VulkanGetProc CreateSkiaGetProc(
    const fml::RefPtr<vulkan::VulkanProcTable>& vk);

}  // namespace vulkan

#endif  // FLUTTER_VULKAN_VULKAN_SKIA_PROC_TABLE_H_
