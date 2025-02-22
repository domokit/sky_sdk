// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/lib/ui/painting/display_list_image_gpu.h"

namespace flutter {

sk_sp<DlImageGPU> DlImageGPU::Make(SkiaGPUObject<SkImage> image) {
  if (!image.skia_object()) {
    return nullptr;
  }
  return sk_sp<DlImageGPU>(new DlImageGPU(std::move(image)));
}

DlImageGPU::DlImageGPU(SkiaGPUObject<SkImage> image)
    : image_(std::move(image)) {}

// |DlImage|
DlImageGPU::~DlImageGPU() {}

// |DlImage|
sk_sp<SkImage> DlImageGPU::skia_image() const {
  return image_.skia_object();
};

// |DlImage|
std::shared_ptr<impeller::Texture> DlImageGPU::impeller_texture() const {
  return nullptr;
}

// |DlImage|
bool DlImageGPU::isOpaque() const {
  if (auto image = skia_image()) {
    return image->isOpaque();
  }
  return false;
}

// |DlImage|
bool DlImageGPU::isTextureBacked() const {
  if (auto image = skia_image()) {
    return image->isTextureBacked();
  }
  return false;
}

// |DlImage|
bool DlImageGPU::isUIThreadSafe() const {
  return true;
}

// |DlImage|
SkISize DlImageGPU::dimensions() const {
  const auto image = skia_image();
  return image ? image->dimensions() : SkISize::MakeEmpty();
}

// |DlImage|
DlISize DlImageGPU::GetSize() const {
  const auto image = skia_image();
  return image ? ToDlISize(image->dimensions()) : DlISize();
}

// |DlImage|
size_t DlImageGPU::GetApproximateByteSize() const {
  auto size = sizeof(*this);
  if (auto image = skia_image()) {
    const auto& info = image->imageInfo();
    const auto kMipmapOverhead = image->hasMipmaps() ? 4.0 / 3.0 : 1.0;
    const size_t image_byte_size = info.computeMinByteSize() * kMipmapOverhead;
    size += image_byte_size;
  }
  return size;
}

}  // namespace flutter
