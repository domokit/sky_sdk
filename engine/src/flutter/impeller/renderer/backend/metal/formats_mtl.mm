// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/backend/metal/formats_mtl.h"
#include <Metal/Metal.h>

#include <memory>

#include "impeller/renderer/render_pass.h"

namespace impeller {

MTLRenderPipelineColorAttachmentDescriptor*
ToMTLRenderPipelineColorAttachmentDescriptor(
    ColorAttachmentDescriptor descriptor) {
  auto des = [[MTLRenderPipelineColorAttachmentDescriptor alloc] init];
  des.pixelFormat = ToMTLPixelFormat(descriptor.format);

  des.blendingEnabled = descriptor.blending_enabled;

  des.sourceRGBBlendFactor =
      ToMTLBlendFactor(descriptor.src_color_blend_factor);
  des.rgbBlendOperation = ToMTLBlendOperation(descriptor.color_blend_op);
  des.destinationRGBBlendFactor =
      ToMTLBlendFactor(descriptor.dst_color_blend_factor);

  des.sourceAlphaBlendFactor =
      ToMTLBlendFactor(descriptor.src_alpha_blend_factor);
  des.alphaBlendOperation = ToMTLBlendOperation(descriptor.alpha_blend_op);
  des.destinationAlphaBlendFactor =
      ToMTLBlendFactor(descriptor.dst_alpha_blend_factor);

  des.writeMask = ToMTLColorWriteMask(descriptor.write_mask);
  return des;
}

MTLStencilDescriptor* ToMTLStencilDescriptor(
    const StencilAttachmentDescriptor& descriptor) {
  auto des = [[MTLStencilDescriptor alloc] init];
  des.stencilCompareFunction = ToMTLCompareFunction(descriptor.stencil_compare);
  des.stencilFailureOperation =
      ToMTLStencilOperation(descriptor.stencil_failure);
  des.depthFailureOperation = ToMTLStencilOperation(descriptor.depth_failure);
  des.depthStencilPassOperation =
      ToMTLStencilOperation(descriptor.depth_stencil_pass);

  des.readMask = descriptor.read_mask;
  des.writeMask = descriptor.write_mask;

  return des;
}

MTLDepthStencilDescriptor* ToMTLDepthStencilDescriptor(
    std::optional<DepthAttachmentDescriptor> depth,
    std::optional<StencilAttachmentDescriptor> front,
    std::optional<StencilAttachmentDescriptor> back) {
  if (!depth) {
    depth = DepthAttachmentDescriptor{
        // Always pass the depth test.
        .depth_compare = CompareFunction::kAlways,
        .depth_write_enabled = false,
    };
  }

  auto des = [[MTLDepthStencilDescriptor alloc] init];

  // These temporary variables are necessary for clang-tidy (Fuchsia LLVM
  // version 17.0.0git) to not crash.
  auto compare_function = ToMTLCompareFunction(depth->depth_compare);
  auto depth_write_enabled = depth->depth_write_enabled;

  des.depthCompareFunction = compare_function;
  des.depthWriteEnabled = depth_write_enabled;

  if (front.has_value()) {
    des.frontFaceStencil = ToMTLStencilDescriptor(front.value());
  }
  if (back.has_value()) {
    des.backFaceStencil = ToMTLStencilDescriptor(back.value());
  }

  return des;
}

MTLTextureDescriptor* ToMTLTextureDescriptor(const TextureDescriptor& desc) {
  if (!desc.IsValid()) {
    return nil;
  }
  auto mtl_desc = [[MTLTextureDescriptor alloc] init];
  mtl_desc.textureType = ToMTLTextureType(desc.type);
  mtl_desc.pixelFormat = ToMTLPixelFormat(desc.format);
  mtl_desc.sampleCount = static_cast<NSUInteger>(desc.sample_count);
  mtl_desc.width = desc.size.width;
  mtl_desc.height = desc.size.height;
  mtl_desc.mipmapLevelCount = desc.mip_count;
  mtl_desc.usage = MTLTextureUsageUnknown;
  if (desc.usage & TextureUsage::kUnknown) {
    mtl_desc.usage |= MTLTextureUsageUnknown;
  }
  if (desc.usage & TextureUsage::kShaderRead) {
    mtl_desc.usage |= MTLTextureUsageShaderRead;
  }
  if (desc.usage & TextureUsage::kShaderWrite) {
    mtl_desc.usage |= MTLTextureUsageShaderWrite;
  }
  if (desc.usage & TextureUsage::kRenderTarget) {
    mtl_desc.usage |= MTLTextureUsageRenderTarget;
  }
  return mtl_desc;
}

MTLPixelFormat SafeMTLPixelFormatDepth24Unorm_Stencil8() {
#if !FML_OS_IOS
  if (@available(macOS 10.11, *)) {
    return MTLPixelFormatDepth24Unorm_Stencil8;
  }
#endif  // FML_OS_IOS
  return MTLPixelFormatInvalid;
}

MTLPixelFormat SafeMTLPixelFormatBGR10_XR_sRGB() {
  if (@available(iOS 11, macOS 11.0, *)) {
    return MTLPixelFormatBGR10_XR_sRGB;
  } else {
    return MTLPixelFormatInvalid;
  }
}

MTLPixelFormat SafeMTLPixelFormatBGR10_XR() {
  if (@available(iOS 10, macOS 11.0, *)) {
    return MTLPixelFormatBGR10_XR;
  } else {
    return MTLPixelFormatInvalid;
  }
}

MTLPixelFormat SafeMTLPixelFormatBGRA10_XR() {
  if (@available(iOS 10, macOS 11.0, *)) {
    return MTLPixelFormatBGRA10_XR;
  } else {
    return MTLPixelFormatInvalid;
  }
}

}  // namespace impeller
