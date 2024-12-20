// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_COMMON_PATH_UTILS_H_
#define FLUTTER_SHELL_PLATFORM_COMMON_PATH_UTILS_H_

#include <filesystem>

namespace flutter {

// Returns the path of the directory containing this executable, or an empty
// path if the directory cannot be found.
std::filesystem::path GetExecutableDirectory();

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_COMMON_PATH_UTILS_H_
