// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/core/allocator.h"

#include "impeller/base/validation.h"
#include "impeller/core/device_buffer.h"
#include "impeller/core/formats.h"
#include "impeller/core/range.h"

namespace impeller {

Allocator::Allocator() = default;

Allocator::~Allocator() = default;

std::shared_ptr<DeviceBuffer> Allocator::CreateBufferWithCopy(
    const uint8_t* buffer,
    size_t length) {
  DeviceBufferDescriptor desc;
  desc.size = length;
  desc.storage_mode = StorageMode::kHostVisible;
  auto new_buffer = CreateBuffer(desc);

  if (!new_buffer) {
    return nullptr;
  }

  auto entire_range = Range{0, length};

  if (!new_buffer->CopyHostBuffer(buffer, entire_range)) {
    return nullptr;
  }

  return new_buffer;
}

std::shared_ptr<DeviceBuffer> Allocator::CreateBufferWithCopy(
    const fml::Mapping& mapping) {
  return CreateBufferWithCopy(mapping.GetMapping(), mapping.GetSize());
}

std::shared_ptr<DeviceBuffer> Allocator::CreateBuffer(
    const DeviceBufferDescriptor& desc) {
  return OnCreateBuffer(desc);
}

std::shared_ptr<Texture> Allocator::CreateTexture(
    const TextureDescriptor& desc) {
  const auto max_size = GetMaxTextureSizeSupported();
  if (desc.size.width > max_size.width || desc.size.height > max_size.height) {
    VALIDATION_LOG << "Requested texture size " << desc.size
                   << " exceeds maximum supported size of " << max_size;
    return nullptr;
  }

  if (desc.mip_count > desc.size.MipCount()) {
    VALIDATION_LOG << "Requested mip_count " << desc.mip_count
                   << " exceeds maximum supported for size " << desc.size;
    TextureDescriptor corrected_desc = desc;
    corrected_desc.mip_count = desc.size.MipCount();
    return OnCreateTexture(corrected_desc);
  }

  return OnCreateTexture(desc);
}

uint16_t Allocator::MinimumBytesPerRow(PixelFormat format) const {
  return BytesPerPixelForPixelFormat(format);
}

}  // namespace impeller
