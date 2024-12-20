// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_COMMON_SERIALIZATION_CALLBACKS_H_
#define FLUTTER_SHELL_COMMON_SERIALIZATION_CALLBACKS_H_

#include "flutter/fml/logging.h"
#include "third_party/skia/include/core/SkImage.h"
#include "third_party/skia/include/core/SkTypeface.h"

namespace flutter {

sk_sp<SkData> SerializeTypefaceWithoutData(SkTypeface* typeface, void* ctx);
sk_sp<SkData> SerializeTypefaceWithData(SkTypeface* typeface, void* ctx);
sk_sp<SkTypeface> DeserializeTypefaceWithoutData(const void* data,
                                                 size_t length,
                                                 void* ctx);

// Serializes only the metadata of the image and not the underlying pixel data.
sk_sp<SkData> SerializeImageWithoutData(SkImage* image, void* ctx);
sk_sp<SkImage> DeserializeImageWithoutData(const void* data,
                                           size_t length,
                                           void* ctx);

}  // namespace flutter

#endif  // FLUTTER_SHELL_COMMON_SERIALIZATION_CALLBACKS_H_
