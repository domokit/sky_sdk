// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/fml/log_settings.h"

#include <fcntl.h>

#include <algorithm>
#include <cstring>
#include <iostream>
#include <limits>

#include "flutter/fml/build_config.h"
#include "flutter/fml/log_level.h"
#include "flutter/fml/logging.h"

namespace fml {
namespace state {

// Defined in log_settings_state.cc.
extern LogSettings g_log_settings;

}  // namespace state

void SetLogSettings(const LogSettings& settings) {
  // Validate the new settings as we set them.
  state::g_log_settings.min_log_level =
      std::min(kLogFatal, settings.min_log_level);
}

LogSettings GetLogSettings() {
  return state::g_log_settings;
}

int GetMinLogLevel() {
  return std::min(state::g_log_settings.min_log_level, kLogFatal);
}

ScopedSetLogSettings::ScopedSetLogSettings(const LogSettings& settings) {
  old_settings_ = GetLogSettings();
  SetLogSettings(settings);
}

ScopedSetLogSettings::~ScopedSetLogSettings() {
  SetLogSettings(old_settings_);
}

}  // namespace fml
