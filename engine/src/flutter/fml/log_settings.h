// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FML_LOG_SETTINGS_H_
#define FLUTTER_FML_LOG_SETTINGS_H_

#include <string>

#include "flutter/fml/log_level.h"

namespace fml {

// Settings which control the behavior of FML logging.
struct LogSettings {
  // The minimum logging level.
  // Anything at or above this level will be logged (if applicable).
  // Anything below this level will be silently ignored.
  //
  // The log level defaults to 0 (kLogInfo).
  //
  // Log messages for FML_VLOG(x) (from flutter/fml/logging.h) are logged
  // at level -x, so setting the min log level to negative values enables
  // verbose logging.
  LogSeverity min_log_level = kLogInfo;
};

// Gets the active log settings for the current process.
void SetLogSettings(const LogSettings& settings);

// Sets the active log settings for the current process.
LogSettings GetLogSettings();

// Gets the minimum log level for the current process. Never returs a value
// higher than kLogFatal.
int GetMinLogLevel();

class ScopedSetLogSettings {
 public:
  explicit ScopedSetLogSettings(const LogSettings& settings);
  ~ScopedSetLogSettings();

 private:
  LogSettings old_settings_;
};

}  // namespace fml

#endif  // FLUTTER_FML_LOG_SETTINGS_H_
