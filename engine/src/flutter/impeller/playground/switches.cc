// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/playground/switches.h"

#include <cstdlib>

#include "flutter/fml/build_config.h"

namespace impeller {

PlaygroundSwitches::PlaygroundSwitches() = default;

PlaygroundSwitches::PlaygroundSwitches(const fml::CommandLine& args) {
  enable_playground = args.HasOption("enable_playground");
  std::string timeout_str;
  if (args.GetOptionValue("playground_timeout_ms", &timeout_str)) {
    timeout = std::chrono::milliseconds(atoi(timeout_str.c_str()));
    // Specifying a playground timeout implies you want to enable playgrounds.
    enable_playground = true;
  }
  enable_vulkan_validation = args.HasOption("enable_vulkan_validation");
  use_swiftshader = args.HasOption("use_swiftshader");
  use_angle = args.HasOption("use_angle");
#if FML_OS_MACOSX
  // OpenGL on macOS is busted and deprecated. Use Angle there by default.
  use_angle = true;
#endif  // FML_OS_MACOSX
}

}  // namespace impeller
