// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_COMMON_ENGINE_SWITCHES_H_
#define FLUTTER_SHELL_PLATFORM_COMMON_ENGINE_SWITCHES_H_

#include <string>
#include <vector>

namespace flutter {

// Returns an array of engine switches suitable to pass to the embedder API
// in FlutterProjectArgs, based on parsing variables from the environment in
// the form:
//   FLUTTER_ENGINE_SWITCHES=<count>
//   FLUTTER_ENGINE_SWITCH_1=...
//   FLUTTER_ENGINE_SWITCH_2=...
//   ...
// Values should match those in shell/common/switches.h
//
// The returned array does not include the initial dummy argument expected by
// the embedder API, so command_line_argv should not be set directly from it.
//
// In release mode, not all switches from the environment will necessarily be
// returned. See the implementation for details.
std::vector<std::string> GetSwitchesFromEnvironment();

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_COMMON_ENGINE_SWITCHES_H_
