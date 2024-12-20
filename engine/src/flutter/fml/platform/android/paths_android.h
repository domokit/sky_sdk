// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FML_PLATFORM_ANDROID_PATHS_ANDROID_H_
#define FLUTTER_FML_PLATFORM_ANDROID_PATHS_ANDROID_H_

#include "flutter/fml/macros.h"
#include "flutter/fml/paths.h"

namespace fml {
namespace paths {

void InitializeAndroidCachesPath(std::string caches_path);

}  // namespace paths
}  // namespace fml

#endif  // FLUTTER_FML_PLATFORM_ANDROID_PATHS_ANDROID_H_
