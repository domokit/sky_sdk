// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/embedder/pixel_formats.h"
#include "flutter/shell/platform/embedder/embedder.h"

#include "third_party/skia/include/core/SkColorSpace.h"
#include "third_party/skia/include/core/SkColorType.h"
#include "third_party/skia/include/core/SkImageInfo.h"

std::optional<SkColorType> getSkColorType(FlutterSoftwarePixelFormat pixfmt) {
  switch (pixfmt) {
    case kFlutterSoftwarePixelFormatGray8:
      return kGray_8_SkColorType;
    case kFlutterSoftwarePixelFormatRGB565:
      return kRGB_565_SkColorType;
    case kFlutterSoftwarePixelFormatRGBA4444:
      return kARGB_4444_SkColorType;
    case kFlutterSoftwarePixelFormatRGBA8888:
      return kRGBA_8888_SkColorType;
    case kFlutterSoftwarePixelFormatRGBX8888:
      return kRGB_888x_SkColorType;
    case kFlutterSoftwarePixelFormatBGRA8888:
      return kBGRA_8888_SkColorType;
    case kFlutterSoftwarePixelFormatNative32:
      return kN32_SkColorType;
    default:
      FML_LOG(ERROR) << "Invalid software rendering pixel format";
      return std::nullopt;
  }
}

std::optional<SkColorInfo> getSkColorInfo(FlutterSoftwarePixelFormat pixfmt) {
  auto ct = getSkColorType(pixfmt);
  if (!ct) {
    return std::nullopt;
  }

  auto at = SkColorTypeIsAlwaysOpaque(*ct) ? kOpaque_SkAlphaType
                                           : kPremul_SkAlphaType;

  return SkColorInfo(*ct, at, SkColorSpace::MakeSRGB());
}
