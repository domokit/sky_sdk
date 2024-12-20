// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/fml/paths.h"

#include <Foundation/Foundation.h>

#include "flutter/fml/file.h"

namespace fml {
namespace paths {

std::pair<bool, std::string> GetExecutablePath() {
  @autoreleasepool {
    return {true, [NSBundle mainBundle].executablePath.UTF8String};
  }
}

fml::UniqueFD GetCachesDirectory() {
  @autoreleasepool {
    auto items = [[NSFileManager defaultManager] URLsForDirectory:NSCachesDirectory
                                                        inDomains:NSUserDomainMask];
    if (items.count == 0) {
      return {};
    }

    return OpenDirectory(items[0].fileSystemRepresentation, false, FilePermission::kRead);
  }
}

}  // namespace paths
}  // namespace fml
