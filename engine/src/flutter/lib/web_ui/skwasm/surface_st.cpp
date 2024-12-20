// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "surface.h"

using namespace Skwasm;

Surface::Surface() : _thread(0) {
  _init();
}

SKWASM_EXPORT bool skwasm_isMultiThreaded() {
  return false;
}
