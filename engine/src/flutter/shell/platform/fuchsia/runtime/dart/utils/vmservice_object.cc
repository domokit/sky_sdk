// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "vmservice_object.h"

#include <dirent.h>
#include <fuchsia/io/cpp/fidl.h>
#include <zircon/status.h>

#include <cerrno>
#include <string>

#include "flutter/fml/logging.h"

namespace {

bool ReadDirContents(const std::string& path, std::vector<std::string>* out) {
  out->clear();
  DIR* dir = opendir(path.c_str());
  if (!dir) {
    return false;
  }
  struct dirent* de;
  errno = 0;
  while ((de = readdir(dir)) != nullptr) {
    out->push_back(de->d_name);
  }
  closedir(dir);
  return !errno;
}

}  // namespace

namespace dart_utils {

void VMServiceObject::GetContents(LazyEntryVector* out_vector) const {
  // List /tmp/dart.services if it exists, and push its contents as
  // the contents of this directory.
  std::vector<std::string> files;
  if (!ReadDirContents(kPortDir, &files)) {
    FML_LOG(ERROR) << "Failed to read Dart VM service port directory '"
                   << kPortDir << "': " << strerror(errno);
    return;
  }
  for (const auto& file : files) {
    if ((file == ".") || (file == "..")) {
      continue;
    }
    out_vector->push_back({std::stoul(file) + GetStartingId(), file,
                           fuchsia::io::MODE_TYPE_FILE});
  }
}

zx_status_t VMServiceObject::GetFile(Node** out_node,
                                     uint64_t id,
                                     std::string name) const {
  return ZX_ERR_NOT_FOUND;
}

}  // namespace dart_utils
