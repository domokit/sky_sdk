// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// Spec: https://github.com/lrhn/dep-pkgspec/blob/master/DEP-pkgspec.md

#include "tonic/parsers/packages_map.h"

#include <memory>

namespace tonic {
namespace {

bool isLineBreak(char c) {
  return c == '\r' || c == '\n';
}

}  // namespace

PackagesMap::PackagesMap() {}

PackagesMap::~PackagesMap() {}

bool PackagesMap::Parse(const std::string& source, std::string* error) {
  map_.clear();
  const auto end = source.end();
  for (auto it = source.begin(); it != end; ++it) {
    const char c = *it;

    // Skip blank lines.
    if (isLineBreak(c))
      continue;

    // Skip comments.
    if (c == '#') {
      while (it != end && !isLineBreak(*it))
        ++it;
      continue;
    }

    if (c == ':') {
      map_.clear();
      *error = "Packages file contains a line that begins with ':'.";
      return false;
    }

    auto package_name_begin = it;
    auto package_name_end = end;
    bool found_separator = false;
    for (; it != end; ++it) {
      const char c = *it;
      if (c == ':' && !found_separator) {
        found_separator = true;
        package_name_end = it;
        continue;
      }
      if (isLineBreak(c))
        break;
    }

    if (!found_separator) {
      map_.clear();
      *error = "Packages file contains non-comment line that lacks a ':'.";
      return false;
    }

    std::string package_name(package_name_begin, package_name_end);
    std::string package_path(package_name_end + 1, it);

    auto result = map_.emplace(package_name, package_path);
    if (!result.second) {
      map_.clear();
      *error =
          std::string("Packages file contains multiple entries for package '") +
          package_name + "'.";
      return false;
    }
  }

  return true;
}

std::string PackagesMap::Resolve(const std::string& package_name) {
  return map_[package_name];
}

}  // namespace tonic
