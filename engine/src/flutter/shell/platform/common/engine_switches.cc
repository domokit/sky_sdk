// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/common/engine_switches.h"

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <sstream>

namespace flutter {

std::vector<std::string> GetSwitchesFromEnvironment() {
  std::vector<std::string> switches;
  // Read engine switches from the environment in debug/profile. If release mode
  // support is needed in the future, it should likely use a whitelist.
#ifndef FLUTTER_RELEASE
  const char* switch_count_key = "FLUTTER_ENGINE_SWITCHES";
  const int kMaxSwitchCount = 50;
  const char* switch_count_string = std::getenv(switch_count_key);
  if (!switch_count_string) {
    return switches;
  }
  int switch_count = std::min(kMaxSwitchCount, atoi(switch_count_string));
  for (int i = 1; i <= switch_count; ++i) {
    std::ostringstream switch_key;
    switch_key << "FLUTTER_ENGINE_SWITCH_" << i;
    const char* switch_value = std::getenv(switch_key.str().c_str());
    if (switch_value) {
      std::ostringstream switch_value_as_flag;
      switch_value_as_flag << "--" << switch_value;
      switches.push_back(switch_value_as_flag.str());
    } else {
      std::cerr << switch_count << " keys expected from " << switch_count_key
                << ", but " << switch_key.str() << " is missing." << std::endl;
    }
  }
#endif  // !FLUTTER_RELEASE
  return switches;
}

}  // namespace flutter
