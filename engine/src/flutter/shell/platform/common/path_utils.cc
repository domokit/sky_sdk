// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/common/path_utils.h"

#if defined(_WIN32)
#include <windows.h>
#elif defined(__linux__)
#include <linux/limits.h>
#include <unistd.h>
#endif

namespace flutter {

std::filesystem::path GetExecutableDirectory() {
#if defined(_WIN32)
  wchar_t buffer[MAX_PATH];
  if (GetModuleFileName(nullptr, buffer, MAX_PATH) == 0) {
    return std::filesystem::path();
  }
  std::filesystem::path executable_path(buffer);
  return executable_path.remove_filename();
#elif defined(__linux__)
  char buffer[PATH_MAX + 1];
  ssize_t length = readlink("/proc/self/exe", buffer, sizeof(buffer));
  if (length > PATH_MAX) {
    return std::filesystem::path();
  }
  std::filesystem::path executable_path(std::string(buffer, length));
  return executable_path.remove_filename();
#else
  return std::filesystem::path();
#endif
}

}  // namespace flutter
