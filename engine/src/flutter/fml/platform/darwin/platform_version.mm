// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/fml/platform/darwin/platform_version.h"

#include <Foundation/NSProcessInfo.h>

namespace fml {

bool IsPlatformVersionAtLeast(size_t major, size_t minor, size_t patch) {
  const NSOperatingSystemVersion version = {
      .majorVersion = static_cast<NSInteger>(major),
      .minorVersion = static_cast<NSInteger>(minor),
      .patchVersion = static_cast<NSInteger>(patch),
  };
  return [[NSProcessInfo processInfo] isOperatingSystemAtLeastVersion:version];
}

}  // namespace fml
