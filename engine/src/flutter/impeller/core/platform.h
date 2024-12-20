// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_CORE_PLATFORM_H_
#define FLUTTER_IMPELLER_CORE_PLATFORM_H_

#include <cstddef>

#include "flutter/fml/build_config.h"

namespace impeller {

constexpr size_t DefaultUniformAlignment() {
#if FML_OS_IOS && !TARGET_OS_SIMULATOR
  return 16u;
#else
  return 256u;
#endif
}

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_CORE_PLATFORM_H_
