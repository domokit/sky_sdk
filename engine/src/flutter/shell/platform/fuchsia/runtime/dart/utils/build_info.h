// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_FUCHSIA_RUNTIME_DART_UTILS_BUILD_INFO_H_
#define FLUTTER_SHELL_PLATFORM_FUCHSIA_RUNTIME_DART_UTILS_BUILD_INFO_H_

#include "root_inspect_node.h"

namespace dart_utils {

class BuildInfo {
 public:
  static const char* DartSdkGitRevision();
  static const char* DartSdkSemanticVersion();
  static const char* FlutterEngineGitRevision();
  static const char* FuchsiaSdkVersion();

  /// Appends the above properties to the specified node on the inspect tree for
  /// the duration of the node's lifetime.
  static void Dump(inspect::Node& node);
};

}  // namespace dart_utils

#endif  // FLUTTER_SHELL_PLATFORM_FUCHSIA_RUNTIME_DART_UTILS_BUILD_INFO_H_
