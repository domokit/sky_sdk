// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "tonic/file_loader/file_loader.h"

#include <dirent.h>
#include <fcntl.h>
#include <sys/types.h>

#include <cerrno>
#include <iostream>
#include <memory>
#include <utility>

#include "tonic/common/macros.h"
#include "tonic/converter/dart_converter.h"
#include "tonic/filesystem/filesystem/file.h"
#include "tonic/filesystem/filesystem/path.h"
#include "tonic/parsers/packages_map.h"

namespace tonic {

const std::string FileLoader::kPathSeparator = "/";
const char FileLoader::kFileURLPrefix[] = "file://";
const size_t FileLoader::kFileURLPrefixLength =
    sizeof(FileLoader::kFileURLPrefix) - 1;

std::string FileLoader::SanitizePath(const std::string& url) {
  return SanitizeURIEscapedCharacters(url);
}

bool FileLoader::ReadFileToString(const std::string& path,
                                  std::string* result) {
  if (dirfd_ == -1)
    return filesystem::ReadFileToString(path, result);
  const char* cpath = path.c_str();
  const int offset = (cpath[0] == '/') ? 1 : 0;
  filesystem::Descriptor fd(openat(dirfd_, &cpath[offset], O_RDONLY));
  return filesystem::ReadFileDescriptorToString(fd.get(), result);
}

std::pair<uint8_t*, intptr_t> FileLoader::ReadFileToBytes(
    const std::string& path) {
  if (dirfd_ == -1)
    return filesystem::ReadFileToBytes(path);
  const char* cpath = path.c_str();
  const int offset = (cpath[0] == '/') ? 1 : 0;
  filesystem::Descriptor fd(openat(dirfd_, &cpath[offset], O_RDONLY));
  return filesystem::ReadFileDescriptorToBytes(fd.get());
}

}  // namespace tonic
