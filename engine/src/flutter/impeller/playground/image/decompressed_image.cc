// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/playground/image/decompressed_image.h"

#include <limits>

#include "flutter/fml/mapping.h"
#include "impeller/base/allocation.h"

namespace impeller {

DecompressedImage::DecompressedImage() = default;

DecompressedImage::DecompressedImage(
    ISize size,
    Format format,
    std::shared_ptr<const fml::Mapping> allocation)
    : size_(size), format_(format), allocation_(std::move(allocation)) {
  if (!allocation_ || size.IsEmpty() || format_ == Format::kInvalid) {
    return;
  }
  is_valid_ = true;
}

DecompressedImage::~DecompressedImage() = default;

bool DecompressedImage::IsValid() const {
  return is_valid_;
}

const ISize& DecompressedImage::GetSize() const {
  return size_;
}

DecompressedImage::Format DecompressedImage::GetFormat() const {
  return format_;
}

const std::shared_ptr<const fml::Mapping>& DecompressedImage::GetAllocation()
    const {
  return allocation_;
}

static size_t GetBytesPerPixel(DecompressedImage::Format format) {
  switch (format) {
    case DecompressedImage::Format::kInvalid:
      return 0u;
    case DecompressedImage::Format::kGrey:
      return 1u;
    case DecompressedImage::Format::kGreyAlpha:
      return 1u;
    case DecompressedImage::Format::kRGB:
      return 3u;
    case DecompressedImage::Format::kRGBA:
      return 4;
  }
  return 0u;
}

DecompressedImage DecompressedImage::ConvertToRGBA() const {
  if (!is_valid_) {
    return {};
  }

  if (format_ == Format::kRGBA) {
    return DecompressedImage{size_, format_, allocation_};
  }

  const auto bpp = GetBytesPerPixel(format_);
  const auto source_byte_size = size_.Area() * bpp;
  if (allocation_->GetSize() < source_byte_size) {
    return {};
  }

  auto rgba_allocation = std::make_shared<Allocation>();
  if (!rgba_allocation->Truncate(Bytes{size_.Area() * 4u}, false)) {
    return {};
  }

  const uint8_t* source = allocation_->GetMapping();
  uint8_t* dest = rgba_allocation->GetBuffer();

  for (size_t i = 0, j = 0; i < source_byte_size; i += bpp, j += 4u) {
    switch (format_) {
      case DecompressedImage::Format::kGrey:
        dest[j + 0] = source[i];
        dest[j + 1] = source[i];
        dest[j + 2] = source[i];
        dest[j + 3] = std::numeric_limits<uint8_t>::max();
        break;
      case DecompressedImage::Format::kGreyAlpha:
        dest[j + 0] = std::numeric_limits<uint8_t>::max();
        dest[j + 1] = std::numeric_limits<uint8_t>::max();
        dest[j + 2] = std::numeric_limits<uint8_t>::max();
        dest[j + 3] = source[i];
        break;
      case DecompressedImage::Format::kRGB:
        dest[j + 0] = source[i + 0];
        dest[j + 1] = source[i + 1];
        dest[j + 2] = source[i + 2];
        dest[j + 3] = std::numeric_limits<uint8_t>::max();
        break;
      case DecompressedImage::Format::kInvalid:
      case DecompressedImage::Format::kRGBA:
        // Should never happen. The necessary checks have already been
        // performed.
        FML_CHECK(false);
        break;
    }
  }

  return DecompressedImage{
      size_, Format::kRGBA,
      std::make_shared<fml::NonOwnedMapping>(
          rgba_allocation->GetBuffer(),                //
          rgba_allocation->GetLength().GetByteSize(),  //
          [rgba_allocation](auto, auto) {})            //
  };
}

}  // namespace impeller
