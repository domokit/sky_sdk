// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_PLAYGROUND_SWITCHES_H_
#define FLUTTER_IMPELLER_PLAYGROUND_SWITCHES_H_

#include <chrono>
#include <optional>

#include "flutter/fml/command_line.h"

namespace impeller {

struct PlaygroundSwitches {
  bool enable_playground = false;
  // If specified, the playgrounds will render for at least the duration
  // specified in the timeout. If the timeout is zero, exactly one frame will be
  // rendered in the playground.
  std::optional<std::chrono::milliseconds> timeout;
  bool enable_vulkan_validation = false;
  //----------------------------------------------------------------------------
  /// Seek a SwiftShader library in known locations and use it when running
  /// Vulkan. It is a fatal error to provide this option and not have the test
  /// find a SwiftShader implementation.
  ///
  bool use_swiftshader = false;
  /// Attempt to use Angle on the system instead of the available OpenGL ES
  /// implementation. This is on-by-default on macOS due to the broken-ness in
  /// the deprecated OpenGL implementation. On other platforms, it this opt-in
  /// via the flag with the system OpenGL ES implementation used by fault.
  ///
  bool use_angle = false;

  bool enable_wide_gamut = false;

  PlaygroundSwitches();

  explicit PlaygroundSwitches(const fml::CommandLine& args);
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_PLAYGROUND_SWITCHES_H_
