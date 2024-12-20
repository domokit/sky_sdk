// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// FLUTTER_NOLINT: https://github.com/flutter/flutter/issues/68332

#include "flutter/shell/version/version.h"

namespace flutter {

const char* GetFlutterEngineVersion() {
  return FLUTTER_ENGINE_VERSION;
}

const char* GetSkiaVersion() {
  return SKIA_VERSION;
}

const char* GetDartVersion() {
  return DART_VERSION;
}

}  // namespace flutter
