// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/playground/image/backends/skia/compressed_image_skia.h"

#include <memory>

#include "impeller/base/validation.h"
#include "third_party/skia/include/core/SkBitmap.h"
#include "third_party/skia/include/core/SkData.h"
#include "third_party/skia/include/core/SkImage.h"
#include "third_party/skia/include/core/SkPixmap.h"
#include "third_party/skia/include/core/SkRefCnt.h"

namespace impeller {

std::shared_ptr<CompressedImage> CompressedImageSkia::Create(
    std::shared_ptr<const fml::Mapping> allocation) {
  // There is only one backend today.
  if (!allocation) {
    return nullptr;
  }
  return std::make_shared<CompressedImageSkia>(std::move(allocation));
}

CompressedImageSkia::CompressedImageSkia(
    std::shared_ptr<const fml::Mapping> allocation)
    : CompressedImage(std::move(allocation)) {}

CompressedImageSkia::~CompressedImageSkia() = default;

// |CompressedImage|
DecompressedImage CompressedImageSkia::Decode() const {
  if (!IsValid()) {
    return {};
  }
  if (source_->GetSize() == 0u) {
    return {};
  }

  auto src = new std::shared_ptr<const fml::Mapping>(source_);
  auto sk_data = SkData::MakeWithProc(
      source_->GetMapping(), source_->GetSize(),
      [](const void* ptr, void* context) {
        delete reinterpret_cast<decltype(src)>(context);
      },
      src);

  auto image = SkImages::DeferredFromEncodedData(sk_data);
  if (!image) {
    return {};
  }

  const auto dims = image->imageInfo().dimensions();
  auto info = SkImageInfo::Make(dims.width(), dims.height(),
                                kRGBA_8888_SkColorType, kPremul_SkAlphaType);

  auto bitmap = std::make_shared<SkBitmap>();
  if (!bitmap->tryAllocPixels(info)) {
    VALIDATION_LOG << "Could not allocate arena for decompressing image.";
    return {};
  }

  if (!image->readPixels(nullptr, bitmap->pixmap(), 0, 0)) {
    VALIDATION_LOG << "Could not decompress image into arena.";
    return {};
  }

  auto mapping = std::make_shared<fml::NonOwnedMapping>(
      reinterpret_cast<const uint8_t*>(bitmap->pixmap().addr()),  // data
      bitmap->pixmap().rowBytes() * bitmap->pixmap().height(),    // size
      [bitmap](const uint8_t* data, size_t size) mutable {
        bitmap.reset();
      }  // proc
  );

  return {
      {bitmap->pixmap().dimensions().fWidth,
       bitmap->pixmap().dimensions().fHeight},  // size
      DecompressedImage::Format::kRGBA,         // format
      mapping                                   // allocation
  };
}

}  // namespace impeller
