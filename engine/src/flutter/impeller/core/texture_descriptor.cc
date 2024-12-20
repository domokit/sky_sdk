// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/core/texture_descriptor.h"

#include <sstream>

namespace impeller {

std::string TextureDescriptorToString(const TextureDescriptor& desc) {
  std::stringstream stream;
  stream << "StorageMode=" << StorageModeToString(desc.storage_mode) << ",";
  stream << "Type=" << TextureTypeToString(desc.type) << ",";
  stream << "Format=" << PixelFormatToString(desc.format) << ",";
  stream << "Size=" << desc.size << ",";
  stream << "MipCount=" << desc.mip_count << ",";
  stream << "SampleCount=" << static_cast<size_t>(desc.sample_count) << ",";
  stream << "Compression=" << CompressionTypeToString(desc.compression_type);
  return stream.str();
}

}  // namespace impeller
