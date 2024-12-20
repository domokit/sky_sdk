// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/backend/vulkan/formats_vk.h"

namespace impeller {

vk::PipelineDepthStencilStateCreateInfo ToVKPipelineDepthStencilStateCreateInfo(
    std::optional<DepthAttachmentDescriptor> depth,
    std::optional<StencilAttachmentDescriptor> front,
    std::optional<StencilAttachmentDescriptor> back) {
  vk::PipelineDepthStencilStateCreateInfo info;

  if (depth.has_value()) {
    info.depthTestEnable = true;
    info.depthWriteEnable = depth->depth_write_enabled;
    info.depthCompareOp = ToVKCompareOp(depth->depth_compare);
    info.minDepthBounds = 0.0f;
    info.maxDepthBounds = 1.0f;
  }

  if (front.has_value()) {
    info.stencilTestEnable = true;
    info.front = ToVKStencilOpState(*front);
  }

  if (back.has_value()) {
    info.stencilTestEnable = true;
    info.back = ToVKStencilOpState(*back);
  }

  return info;
}

}  // namespace impeller
