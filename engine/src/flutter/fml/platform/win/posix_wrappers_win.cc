// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/fml/posix_wrappers.h"

#include <cstring>

namespace fml {

char* strdup(const char* str1) {
  return _strdup(str1);
}

}  // namespace fml
