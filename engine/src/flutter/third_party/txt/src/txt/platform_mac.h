// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef TXT_PLATFORM_MAC_H_
#define TXT_PLATFORM_MAC_H_

#include "txt/asset_font_manager.h"

namespace txt {

// Register additional system font for MacOS and iOS.
void RegisterSystemFonts(const DynamicFontManager& dynamic_font_manager);

}  // namespace txt

#endif  // TXT_PLATFORM_MAC_H_
