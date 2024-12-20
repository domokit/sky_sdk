// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_WEB_UI_SKWASM_WRAPPERS_H_
#define FLUTTER_LIB_WEB_UI_SKWASM_WRAPPERS_H_

#include <emscripten/html5_webgl.h>
#include "third_party/skia/include/core/SkCanvas.h"
#include "third_party/skia/include/core/SkSurface.h"
#include "third_party/skia/modules/skparagraph/include/FontCollection.h"
#include "third_party/skia/modules/skparagraph/include/TypefaceFontProvider.h"

namespace Skwasm {

using SkwasmObject = __externref_t;

struct SurfaceWrapper {
  EMSCRIPTEN_WEBGL_CONTEXT_HANDLE context;
  sk_sp<GrDirectContext> grContext;
  sk_sp<SkSurface> surface;
};

inline void makeCurrent(EMSCRIPTEN_WEBGL_CONTEXT_HANDLE handle) {
  if (!handle)
    return;

  int result = emscripten_webgl_make_context_current(handle);
  if (result != EMSCRIPTEN_RESULT_SUCCESS) {
    printf("make_context failed: %d", result);
  }
}

struct FlutterFontCollection {
  sk_sp<skia::textlayout::FontCollection> collection;
  sk_sp<skia::textlayout::TypefaceFontProvider> provider;
};

}  // namespace Skwasm

#endif  // FLUTTER_LIB_WEB_UI_SKWASM_WRAPPERS_H_
