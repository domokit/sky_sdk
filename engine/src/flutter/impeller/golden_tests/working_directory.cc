// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/golden_tests/working_directory.h"

#include "flutter/fml/paths.h"

namespace impeller {
namespace testing {

WorkingDirectory* WorkingDirectory::instance_ = nullptr;

WorkingDirectory::WorkingDirectory() {}

WorkingDirectory* WorkingDirectory::Instance() {
  if (!instance_) {
    instance_ = new WorkingDirectory();
  }
  return instance_;
}

std::string WorkingDirectory::GetFilenamePath(
    const std::string& filename) const {
  return fml::paths::JoinPaths({path_, filename});
}

void WorkingDirectory::SetPath(const std::string& path) {
  FML_CHECK(did_set_ == false);
  path_ = path;
  did_set_ = true;
}

}  // namespace testing
}  // namespace impeller
