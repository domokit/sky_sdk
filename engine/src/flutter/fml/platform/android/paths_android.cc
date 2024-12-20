// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/fml/platform/android/paths_android.h"

#include "flutter/fml/file.h"

namespace fml {
namespace paths {

std::pair<bool, std::string> GetExecutablePath() {
  return {false, ""};
}

static std::string gCachesPath;

void InitializeAndroidCachesPath(std::string caches_path) {
  gCachesPath = std::move(caches_path);
}

fml::UniqueFD GetCachesDirectory() {
  // If the caches path is not initialized, the FD will be invalid and caching
  // will be disabled throughout the system.
  return OpenDirectory(gCachesPath.c_str(), false, fml::FilePermission::kRead);
}

}  // namespace paths
}  // namespace fml
