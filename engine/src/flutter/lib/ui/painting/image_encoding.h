// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_UI_PAINTING_IMAGE_ENCODING_H_
#define FLUTTER_LIB_UI_PAINTING_IMAGE_ENCODING_H_

#include "fml/status_or.h"
#include "third_party/skia/include/core/SkImage.h"
#include "third_party/tonic/dart_library_natives.h"

namespace flutter {

class CanvasImage;

// This must be kept in sync with the enum in painting.dart
enum ImageByteFormat {
  kRawRGBA,
  kRawStraightRGBA,
  kRawUnmodified,
  kRawExtendedRgba128,
  kPNG,
};

Dart_Handle EncodeImage(CanvasImage* canvas_image,
                        int format,
                        Dart_Handle callback_handle);

fml::StatusOr<sk_sp<SkData>> EncodeImage(const sk_sp<SkImage>& raster_image,
                                         ImageByteFormat format);

}  // namespace flutter

#endif  // FLUTTER_LIB_UI_PAINTING_IMAGE_ENCODING_H_
