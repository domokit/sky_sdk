// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_RUNTIME_TEST_FONT_DATA_H_
#define FLUTTER_RUNTIME_TEST_FONT_DATA_H_

#include <memory>
#include <string>
#include <vector>

#include "third_party/skia/include/core/SkStream.h"
#include "third_party/skia/include/core/SkTypeface.h"

namespace flutter {

std::vector<sk_sp<SkTypeface>> GetTestFontData();
std::vector<std::string> GetTestFontFamilyNames();

}  // namespace flutter

#endif  // FLUTTER_RUNTIME_TEST_FONT_DATA_H_
