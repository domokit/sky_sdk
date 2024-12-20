// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/fml/paths.h"

#include <sstream>

#include "flutter/fml/build_config.h"

namespace fml {
namespace paths {

std::string JoinPaths(std::initializer_list<std::string> components) {
  std::stringstream stream;
  size_t i = 0;
  const size_t size = components.size();
  for (const auto& component : components) {
    i++;
    stream << component;
    if (i != size) {
#if FML_OS_WIN
      stream << "\\";
#else   // FML_OS_WIN
      stream << "/";
#endif  // FML_OS_WIN
    }
  }
  return stream.str();
}

std::string SanitizeURIEscapedCharacters(const std::string& str) {
  std::string result;
  result.reserve(str.size());
  for (std::string::size_type i = 0; i < str.size(); ++i) {
    if (str[i] == '%') {
      if (i > str.size() - 3 || !isxdigit(str[i + 1]) ||
          !isxdigit(str[i + 2])) {
        return "";
      }
      const std::string hex = str.substr(i + 1, 2);
      const unsigned char c = strtoul(hex.c_str(), nullptr, 16);
      if (!c) {
        return "";
      }
      result += c;
      i += 2;
    } else {
      result += str[i];
    }
  }
  return result;
}

std::pair<bool, std::string> GetExecutableDirectoryPath() {
  auto path = GetExecutablePath();
  if (!path.first) {
    return {false, ""};
  }
  return {true, fml::paths::GetDirectoryName(path.second)};
}

}  // namespace paths
}  // namespace fml
