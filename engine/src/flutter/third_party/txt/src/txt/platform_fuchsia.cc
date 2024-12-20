// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <lib/zx/channel.h>

#include "third_party/skia/include/ports/SkFontMgr_fuchsia.h"
#include "txt/platform.h"

#if defined(SK_FONTMGR_FREETYPE_EMPTY_AVAILABLE)
#include "third_party/skia/include/ports/SkFontMgr_empty.h"
#endif

namespace txt {

std::vector<std::string> GetDefaultFontFamilies() {
  return {"Roboto"};
}

sk_sp<SkFontMgr> GetDefaultFontManager(uint32_t font_initialization_data) {
  if (font_initialization_data) {
    fuchsia::fonts::ProviderSyncPtr sync_font_provider;
    sync_font_provider.Bind(zx::channel(font_initialization_data));
    return SkFontMgr_New_Fuchsia(std::move(sync_font_provider));
  } else {
#if defined(SK_FONTMGR_FREETYPE_EMPTY_AVAILABLE)
    static sk_sp<SkFontMgr> mgr = SkFontMgr_New_Custom_Empty();
#else
    static sk_sp<SkFontMgr> mgr = SkFontMgr::RefEmpty();
#endif
    return mgr;
  }
}

}  // namespace txt
