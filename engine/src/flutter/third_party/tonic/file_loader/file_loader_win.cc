// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "tonic/file_loader/file_loader.h"

#include <iostream>
#include <memory>
#include <utility>

#include "tonic/common/macros.h"
#include "tonic/filesystem/filesystem/file.h"
#include "tonic/filesystem/filesystem/path.h"
#include "tonic/parsers/packages_map.h"

namespace tonic {
namespace {

void FindAndReplaceInPlace(std::string& str,
                           const std::string& findStr,
                           const std::string& replaceStr) {
  size_t pos = 0;
  while ((pos = str.find(findStr, pos)) != std::string::npos) {
    str.replace(pos, findStr.length(), replaceStr);
    pos += replaceStr.length();
  }
}

}  // namespace

const char FileLoader::kFileURLPrefix[] = "file:///";
const size_t FileLoader::kFileURLPrefixLength =
    sizeof(FileLoader::kFileURLPrefix) - 1;
const std::string FileLoader::kPathSeparator = "\\";

std::string FileLoader::SanitizePath(const std::string& url) {
  std::string sanitized = url;
  FindAndReplaceInPlace(sanitized, "/", FileLoader::kPathSeparator);
  return SanitizeURIEscapedCharacters(sanitized);
}

bool FileLoader::ReadFileToString(const std::string& path,
                                  std::string* result) {
  TONIC_DCHECK(dirfd_ == -1);
  return filesystem::ReadFileToString(path, result);
}

std::pair<uint8_t*, intptr_t> FileLoader::ReadFileToBytes(
    const std::string& path) {
  TONIC_DCHECK(dirfd_ == -1);
  return filesystem::ReadFileToBytes(path);
}

}  // namespace tonic
