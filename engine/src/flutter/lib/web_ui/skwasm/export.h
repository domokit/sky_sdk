// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_WEB_UI_SKWASM_EXPORT_H_
#define FLUTTER_LIB_WEB_UI_SKWASM_EXPORT_H_

#include <emscripten.h>

#define SKWASM_EXPORT extern "C" EMSCRIPTEN_KEEPALIVE

#endif  // FLUTTER_LIB_WEB_UI_SKWASM_EXPORT_H_
