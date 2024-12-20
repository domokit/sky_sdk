// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FML_PLATFORM_DARWIN_PLATFORM_VERSION_H_
#define FLUTTER_FML_PLATFORM_DARWIN_PLATFORM_VERSION_H_

#include <sys/types.h>

#include "flutter/fml/macros.h"

namespace fml {

bool IsPlatformVersionAtLeast(size_t major, size_t minor = 0, size_t patch = 0);

}  // namespace fml

#endif  // FLUTTER_FML_PLATFORM_DARWIN_PLATFORM_VERSION_H_
