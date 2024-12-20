// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/fml/logging.h"
#include "third_party/skia/include/core/SkColorSpace.h"
#include "third_party/skia/include/core/SkImage.h"
#include "third_party/skia/include/core/SkImageInfo.h"
#include "third_party/skia/include/core/SkStream.h"
#include "third_party/skia/include/core/SkTypeface.h"

namespace flutter {

sk_sp<SkData> SerializeTypefaceWithoutData(SkTypeface* typeface, void* ctx) {
  return SkData::MakeEmpty();
}

sk_sp<SkData> SerializeTypefaceWithData(SkTypeface* typeface, void* ctx) {
  return typeface->serialize(SkTypeface::SerializeBehavior::kDoIncludeData);
}

sk_sp<SkTypeface> DeserializeTypefaceWithoutData(const void* data,
                                                 size_t length,
                                                 void* ctx) {
  return nullptr;
}

struct ImageMetaData {
  int32_t width;
  int32_t height;
  uint32_t color_type;
  uint32_t alpha_type;
  bool has_color_space;
} __attribute__((packed));

sk_sp<SkData> SerializeImageWithoutData(SkImage* image, void* ctx) {
  const auto& info = image->imageInfo();
  SkDynamicMemoryWStream stream;

  ImageMetaData metadata = {info.width(), info.height(),
                            static_cast<uint32_t>(info.colorType()),
                            static_cast<uint32_t>(info.alphaType()),
                            static_cast<bool>(info.colorSpace())};
  stream.write(&metadata, sizeof(ImageMetaData));

  if (info.colorSpace()) {
    auto color_space_data = info.colorSpace()->serialize();
    FML_CHECK(color_space_data);
    SkMemoryStream color_space_stream(color_space_data);
    stream.writeStream(&color_space_stream, color_space_data->size());
  }

  return stream.detachAsData();
};

sk_sp<SkImage> DeserializeImageWithoutData(const void* data,
                                           size_t length,
                                           void* ctx) {
  FML_CHECK(length >= sizeof(ImageMetaData));
  auto metadata = static_cast<const ImageMetaData*>(data);
  sk_sp<SkColorSpace> color_space = nullptr;
  if (metadata->has_color_space) {
    color_space = SkColorSpace::Deserialize(
        static_cast<const uint8_t*>(data) + sizeof(ImageMetaData),
        length - sizeof(ImageMetaData));
  }

  auto image_size = SkISize::Make(metadata->width, metadata->height);
  auto info = SkImageInfo::Make(
      image_size, static_cast<SkColorType>(metadata->color_type),
      static_cast<SkAlphaType>(metadata->alpha_type), color_space);
  sk_sp<SkData> image_data =
      SkData::MakeUninitialized(image_size.width() * image_size.height() * 4);
  memset(image_data->writable_data(), 0x0f, image_data->size());
  sk_sp<SkImage> image =
      SkImages::RasterFromData(info, image_data, image_size.width() * 4);

  return image;
};

}  // namespace flutter
