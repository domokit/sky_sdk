// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_GLFW_SYSTEM_UTILS_H_
#define FLUTTER_SHELL_PLATFORM_GLFW_SYSTEM_UTILS_H_

#include <string>
#include <vector>

#include "flutter/shell/platform/embedder/embedder.h"

namespace flutter {

// Components of a system language/locale.
struct LanguageInfo {
  std::string language;
  std::string territory;
  std::string codeset;
  std::string modifier;
};

// Returns the list of user-preferred languages, in preference order,
// parsed into LanguageInfo structures.
std::vector<LanguageInfo> GetPreferredLanguageInfo();

// Converts a  vector of LanguageInfo structs to a vector of FlutterLocale
// structs. |languages| must outlive the returned value, since the returned
// elements have pointers into it.
std::vector<FlutterLocale> ConvertToFlutterLocale(
    const std::vector<LanguageInfo>& languages);

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_GLFW_SYSTEM_UTILS_H_
