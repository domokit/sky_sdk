// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/base/version.h"

#include <sstream>

namespace impeller {

std::optional<Version> Version::FromVector(const std::vector<size_t>& version) {
  if (version.size() == 0) {
    return Version{0, 0, 0};
  }
  if (version.size() == 1) {
    return Version{version[0], 0, 0};
  }
  if (version.size() == 2) {
    return Version{version[0], version[1], 0};
  }
  if (version.size() == 3) {
    return Version{version[0], version[1], version[2]};
  }
  return std::nullopt;
}

std::string Version::ToString() const {
  std::stringstream stream;
  stream << major_version << "." << minor_version << "." << patch_version;
  return stream.str();
}

}  // namespace impeller
