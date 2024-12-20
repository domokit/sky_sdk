// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_CORE_TEXTURE_DESCRIPTOR_H_
#define FLUTTER_IMPELLER_CORE_TEXTURE_DESCRIPTOR_H_

#include <cstdint>
#include "impeller/core/formats.h"
#include "impeller/geometry/size.h"

namespace impeller {

//------------------------------------------------------------------------------
/// @brief      Additional compression to apply to a texture. This value is
///             ignored on platforms which do not support it.
///
///             Lossy compression is only supported on iOS 15+ on A15 chips.
enum class CompressionType {
  kLossless,
  kLossy,
};

constexpr const char* CompressionTypeToString(CompressionType type) {
  switch (type) {
    case CompressionType::kLossless:
      return "Lossless";
    case CompressionType::kLossy:
      return "Lossy";
  }
  FML_UNREACHABLE();
}

//------------------------------------------------------------------------------
/// @brief      A lightweight object that describes the attributes of a texture
///             that can then used an allocator to create that texture.
///
struct TextureDescriptor {
  StorageMode storage_mode = StorageMode::kDeviceTransient;
  TextureType type = TextureType::kTexture2D;
  PixelFormat format = PixelFormat::kUnknown;
  ISize size;
  size_t mip_count = 1u;  // Size::MipCount is usually appropriate.
  TextureUsageMask usage = TextureUsage::kShaderRead;
  SampleCount sample_count = SampleCount::kCount1;
  CompressionType compression_type = CompressionType::kLossless;

  constexpr size_t GetByteSizeOfBaseMipLevel() const {
    if (!IsValid()) {
      return 0u;
    }
    return size.Area() * BytesPerPixelForPixelFormat(format);
  }

  constexpr size_t GetByteSizeOfAllMipLevels() const {
    if (!IsValid()) {
      return 0u;
    }
    size_t result = 0u;
    int64_t width = size.width;
    int64_t height = size.height;
    for (auto i = 0u; i < mip_count; i++) {
      result +=
          ISize(width, height).Area() * BytesPerPixelForPixelFormat(format);
      width /= 2;
      height /= 2;
    }
    return result;
  }

  constexpr size_t GetBytesPerRow() const {
    if (!IsValid()) {
      return 0u;
    }
    return size.width * BytesPerPixelForPixelFormat(format);
  }

  constexpr bool SamplingOptionsAreValid() const {
    const auto count = static_cast<uint64_t>(sample_count);
    return IsMultisampleCapable(type) ? count > 1 : count == 1;
  }

  constexpr bool operator==(const TextureDescriptor& other) const {
    return size == other.size &&                          //
           storage_mode == other.storage_mode &&          //
           format == other.format &&                      //
           usage == other.usage &&                        //
           sample_count == other.sample_count &&          //
           type == other.type &&                          //
           compression_type == other.compression_type &&  //
           mip_count == other.mip_count;
  }

  constexpr bool operator!=(const TextureDescriptor& other) const {
    return !(*this == other);
  }

  constexpr bool IsValid() const {
    return format != PixelFormat::kUnknown &&  //
           !size.IsEmpty() &&                  //
           mip_count >= 1u &&                  //
           SamplingOptionsAreValid();
  }
};

std::string TextureDescriptorToString(const TextureDescriptor& desc);

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_CORE_TEXTURE_DESCRIPTOR_H_
