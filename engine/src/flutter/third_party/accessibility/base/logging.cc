// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "logging.h"

#include <algorithm>
#include <iostream>

#include "ax_build/build_config.h"
// #include "log_settings.h"

#if defined(OS_ANDROID)
#include <android/log.h>
#elif defined(OS_IOS)
#include <syslog.h>
#endif

namespace base {

namespace {

const char* StripPath(const char* path) {
  auto* p = strrchr(path, '/');
  if (p) {
    return p + 1;
  }
  return path;
}

}  // namespace

LogMessage::LogMessage(const char* file,
                       int line,
                       const char* condition,
                       bool killProcess)
    : file_(file), line_(line), killProcess_(killProcess) {
  stream_ << "[ERROR:" << StripPath(file_) << "(" << line_ << ")] ";

  if (condition) {
    stream_ << "Check failed: " << condition << ". ";
  }
}

LogMessage::~LogMessage() {
  stream_ << std::endl;

#if defined(OS_ANDROID)
  android_LogPriority priority = ANDROID_LOG_ERROR __android_log_write(
      priority, "flutter", stream_.str().c_str());
#elif defined(OS_IOS)
  syslog(LOG_ALERT, "%s", stream_.str().c_str());
#else
  std::cerr << stream_.str();
  std::cerr.flush();
#endif
  if (killProcess_)
    KillProcess();
}

void KillProcess() {
  abort();
}

}  // namespace base
