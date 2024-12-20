// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/fml/file.h"
#include "flutter/fml/paths.h"

namespace fml {
namespace paths {

std::pair<bool, std::string> GetExecutablePath() {
  return {false, ""};
}

fml::UniqueFD GetCachesDirectory() {
  return OpenDirectory("/cache", false, fml::FilePermission::kRead);
}

}  // namespace paths
}  // namespace fml
