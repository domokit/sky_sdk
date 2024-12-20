// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_EMBEDDER_PIXEL_FORMATS_H_
#define FLUTTER_SHELL_PLATFORM_EMBEDDER_PIXEL_FORMATS_H_

#include <optional>
#include "flutter/shell/common/rasterizer.h"
#include "flutter/shell/platform/embedder/embedder.h"

std::optional<SkColorType> getSkColorType(FlutterSoftwarePixelFormat pixfmt);

std::optional<SkColorInfo> getSkColorInfo(FlutterSoftwarePixelFormat pixfmt);

#endif  // FLUTTER_SHELL_PLATFORM_EMBEDDER_PIXEL_FORMATS_H_
