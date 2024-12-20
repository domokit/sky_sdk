// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef TXT_PLATFORM_H_
#define TXT_PLATFORM_H_

#include <string>
#include <vector>

#include "flutter/fml/macros.h"
#include "third_party/skia/include/core/SkFontMgr.h"

namespace txt {

std::vector<std::string> GetDefaultFontFamilies();

sk_sp<SkFontMgr> GetDefaultFontManager(uint32_t font_initialization_data = 0);

}  // namespace txt

#endif  // TXT_PLATFORM_H_
