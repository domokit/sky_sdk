// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/base/strings.h"

#include <cstdarg>

namespace impeller {

IMPELLER_PRINTF_FORMAT(1, 2)
std::string SPrintF(const char* format, ...) {
  std::string ret_val;
  va_list list;
  va_list list2;
  va_start(list, format);
  va_copy(list2, list);
  if (auto string_length = ::vsnprintf(nullptr, 0, format, list);
      string_length >= 0) {
    auto buffer = reinterpret_cast<char*>(::malloc(string_length + 1));
    ::vsnprintf(buffer, string_length + 1, format, list2);
    ret_val = std::string{buffer, static_cast<size_t>(string_length)};
    ::free(buffer);
  }
  va_end(list2);
  va_end(list);
  return ret_val;
}

bool HasPrefix(const std::string& string, const std::string& prefix) {
  return string.find(prefix) == 0u;
}

bool HasSuffix(const std::string& string, const std::string& suffix) {
  auto position = string.rfind(suffix);
  if (position == std::string::npos) {
    return false;
  }
  return position == string.size() - suffix.size();
}

std::string StripPrefix(const std::string& string,
                        const std::string& to_strip) {
  if (!HasPrefix(string, to_strip)) {
    return string;
  }
  return string.substr(to_strip.length());
}

}  // namespace impeller
