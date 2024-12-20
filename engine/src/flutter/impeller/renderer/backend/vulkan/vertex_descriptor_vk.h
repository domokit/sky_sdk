// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_VERTEX_DESCRIPTOR_VK_H_
#define FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_VERTEX_DESCRIPTOR_VK_H_

#include "impeller/core/shader_types.h"
#include "impeller/renderer/backend/vulkan/vk.h"

namespace impeller {

vk::Format ToVertexDescriptorFormat(const ShaderStageIOSlot& input);

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_VERTEX_DESCRIPTOR_VK_H_
