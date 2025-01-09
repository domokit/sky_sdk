// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/backend/metal/blit_pass_mtl.h"

#include <Metal/Metal.h>
#import <MetalPerformanceShaders/MetalPerformanceShaders.h>
#include <cstdint>
#include <memory>
#include <utility>
#include <variant>

#include "flutter/fml/closure.h"
#include "flutter/fml/logging.h"
#include "flutter/fml/trace_event.h"
#include "impeller/base/backend_cast.h"
#include "impeller/core/formats.h"
#include "impeller/core/host_buffer.h"
#include "impeller/core/shader_types.h"
#include "impeller/renderer/backend/metal/device_buffer_mtl.h"
#include "impeller/renderer/backend/metal/formats_mtl.h"
#include "impeller/renderer/backend/metal/pipeline_mtl.h"
#include "impeller/renderer/backend/metal/sampler_mtl.h"
#include "impeller/renderer/backend/metal/texture_mtl.h"

#include "impeller/renderer/blit_command.h"

namespace impeller {

BlitPassMTL::BlitPassMTL(id<MTLCommandBuffer> buffer, id<MTLDevice> device)
    : buffer_(buffer), device_(device) {
  if (!buffer_) {
    return;
  }
  encoder_ = [buffer_ blitCommandEncoder];
#ifdef IMPELLER_DEBUG
  is_metal_trace_active_ =
      [[MTLCaptureManager sharedCaptureManager] isCapturing];
#endif  // IMPELLER_DEBUG
  is_valid_ = true;
}

BlitPassMTL::~BlitPassMTL() {
  if (!did_finish_encoding_) {
    [encoder_ endEncoding];
  }
}

bool BlitPassMTL::IsValid() const {
  return is_valid_;
}

void BlitPassMTL::OnSetLabel(std::string_view label) {
  if (label.empty()) {
    return;
  }
  [encoder_ setLabel:@(label.data())];
}

bool BlitPassMTL::EncodeCommands() const {
  [encoder_ endEncoding];
  did_finish_encoding_ = true;
  return true;
}

// |BlitPass|
bool BlitPassMTL::OnCopyTextureToTextureCommand(
    std::shared_ptr<Texture> source,
    std::shared_ptr<Texture> destination,
    IRect source_region,
    IPoint destination_origin,
    std::string_view label) {
  auto source_mtl = TextureMTL::Cast(*source).GetMTLTexture();
  if (!source_mtl) {
    return false;
  }

  auto destination_mtl = TextureMTL::Cast(*destination).GetMTLTexture();
  if (!destination_mtl) {
    return false;
  }

  auto source_origin_mtl =
      MTLOriginMake(source_region.GetX(), source_region.GetY(), 0);
  auto source_size_mtl =
      MTLSizeMake(source_region.GetWidth(), source_region.GetHeight(), 1);
  auto destination_origin_mtl =
      MTLOriginMake(destination_origin.x, destination_origin.y, 0);

#ifdef IMPELLER_DEBUG
  if (is_metal_trace_active_) {
    [encoder_ pushDebugGroup:@(label.data())];
  }
#endif  // IMPELLER_DEBUG
  [encoder_ copyFromTexture:source_mtl
                sourceSlice:0
                sourceLevel:0
               sourceOrigin:source_origin_mtl
                 sourceSize:source_size_mtl
                  toTexture:destination_mtl
           destinationSlice:0
           destinationLevel:0
          destinationOrigin:destination_origin_mtl];

#ifdef IMPELLER_DEBUG
  if (is_metal_trace_active_) {
    [encoder_ popDebugGroup];
  }
#endif  // IMPELLER_DEBUG
  return true;
}

// |BlitPass|
bool BlitPassMTL::ResizeTexture(const std::shared_ptr<Texture>& source,
                                const std::shared_ptr<Texture>& destination) {
  auto source_mtl = TextureMTL::Cast(*source).GetMTLTexture();
  if (!source_mtl) {
    return false;
  }

  auto destination_mtl = TextureMTL::Cast(*destination).GetMTLTexture();
  if (!destination_mtl) {
    return false;
  }

  [encoder_ endEncoding];
  auto filter = [[MPSImageBilinearScale alloc] initWithDevice:device_];
  [filter encodeToCommandBuffer:buffer_
                  sourceTexture:source_mtl
             destinationTexture:destination_mtl];
  encoder_ = [buffer_ blitCommandEncoder];
  return true;
}

// |BlitPass|
bool BlitPassMTL::OnCopyTextureToBufferCommand(
    std::shared_ptr<Texture> source,
    std::shared_ptr<DeviceBuffer> destination,
    IRect source_region,
    size_t destination_offset,
    std::string_view label) {
  auto source_mtl = TextureMTL::Cast(*source).GetMTLTexture();
  if (!source_mtl) {
    return false;
  }

  auto destination_mtl = DeviceBufferMTL::Cast(*destination).GetMTLBuffer();
  if (!destination_mtl) {
    return false;
  }

  auto source_origin_mtl =
      MTLOriginMake(source_region.GetX(), source_region.GetY(), 0);
  auto source_size_mtl =
      MTLSizeMake(source_region.GetWidth(), source_region.GetHeight(), 1);

  auto destination_bytes_per_pixel =
      BytesPerPixelForPixelFormat(source->GetTextureDescriptor().format);
  auto destination_bytes_per_row =
      source_size_mtl.width * destination_bytes_per_pixel;
  auto destination_bytes_per_image =
      source_size_mtl.height * destination_bytes_per_row;

#ifdef IMPELLER_DEBUG
  if (is_metal_trace_active_) {
    [encoder_ pushDebugGroup:@(label.data())];
  }
#endif  // IMPELLER_DEBUG
  [encoder_ copyFromTexture:source_mtl
                   sourceSlice:0
                   sourceLevel:0
                  sourceOrigin:source_origin_mtl
                    sourceSize:source_size_mtl
                      toBuffer:destination_mtl
             destinationOffset:destination_offset
        destinationBytesPerRow:destination_bytes_per_row
      destinationBytesPerImage:destination_bytes_per_image];

#ifdef IMPELLER_DEBUG
  if (is_metal_trace_active_) {
    [encoder_ popDebugGroup];
  }
#endif  // IMPELLER_DEBUG
  return true;
}

bool BlitPassMTL::OnCopyBufferToTextureCommand(
    BufferView source,
    std::shared_ptr<Texture> destination,
    IRect destination_region,
    std::string_view label,
    uint32_t mip_level,
    uint32_t slice,
    bool convert_to_read) {
  auto source_mtl = DeviceBufferMTL::Cast(*source.GetBuffer()).GetMTLBuffer();
  if (!source_mtl) {
    return false;
  }

  auto destination_mtl = TextureMTL::Cast(*destination).GetMTLTexture();
  if (!destination_mtl) {
    return false;
  }

  auto destination_origin_mtl =
      MTLOriginMake(destination_region.GetX(), destination_region.GetY(), 0);
  auto source_size_mtl = MTLSizeMake(destination_region.GetWidth(),
                                     destination_region.GetHeight(), 1);

  auto destination_bytes_per_pixel =
      BytesPerPixelForPixelFormat(destination->GetTextureDescriptor().format);
  auto source_bytes_per_row =
      destination_region.GetWidth() * destination_bytes_per_pixel;

#ifdef IMPELLER_DEBUG
  if (is_metal_trace_active_) {
    [encoder_ pushDebugGroup:@(label.data())];
  }
#endif  // IMPELLER_DEBUG
  [encoder_
           copyFromBuffer:source_mtl
             sourceOffset:source.GetRange().offset
        sourceBytesPerRow:source_bytes_per_row
      sourceBytesPerImage:
          0  // 0 for 2D textures according to
             // https://developer.apple.com/documentation/metal/mtlblitcommandencoder/1400752-copyfrombuffer
               sourceSize:source_size_mtl
                toTexture:destination_mtl
         destinationSlice:slice
         destinationLevel:mip_level
        destinationOrigin:destination_origin_mtl];

#ifdef IMPELLER_DEBUG
  if (is_metal_trace_active_) {
    [encoder_ popDebugGroup];
  }
#endif  // IMPELLER_DEBUG
  return true;
}

// |BlitPass|
bool BlitPassMTL::OnGenerateMipmapCommand(std::shared_ptr<Texture> texture,
                                          std::string_view label) {
#ifdef IMPELLER_DEBUG
  if (is_metal_trace_active_) {
    [encoder_ pushDebugGroup:@(label.data())];
  }
#endif  // IMPELLER_DEBUG
  auto result = TextureMTL::Cast(*texture).GenerateMipmap(encoder_);
#ifdef IMPELLER_DEBUG
  if (is_metal_trace_active_) {
    [encoder_ popDebugGroup];
  }
#endif  // IMPELLER_DEBUG
  return result;
}

}  // namespace impeller
