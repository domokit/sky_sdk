// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef LIB_TONIC_FILE_LOADER_FILE_LOADER_H_
#define LIB_TONIC_FILE_LOADER_FILE_LOADER_H_

#include <memory>
#include <set>
#include <string>
#include <vector>

#include "third_party/dart/runtime/include/dart_api.h"
#include "tonic/common/macros.h"
#include "tonic/parsers/packages_map.h"

namespace tonic {

class FileLoader {
 public:
  explicit FileLoader(int dirfd = -1);
  ~FileLoader();

  bool LoadPackagesMap(const std::string& packages);

  // The path to the `.packages` file the packages map was loaded from.
  const std::string& packages() const { return packages_; }

  Dart_Handle HandleLibraryTag(Dart_LibraryTag tag,
                               Dart_Handle library,
                               Dart_Handle url);

  Dart_Handle CanonicalizeURL(Dart_Handle library, Dart_Handle url);
  Dart_Handle Import(Dart_Handle url);
  Dart_Handle Kernel(Dart_Handle url);
  void SetPackagesUrl(Dart_Handle url);

  Dart_Handle FetchBytes(const std::string& url,
                         uint8_t*& buffer,
                         intptr_t& buffer_size);

  static const char kFileURLPrefix[];
  static const size_t kFileURLPrefixLength;
  static const std::string kPathSeparator;

 private:
  static std::string SanitizeURIEscapedCharacters(const std::string& str);
  static std::string SanitizePath(const std::string& path);

  std::string GetFilePathForURL(std::string url);
  std::string GetFilePathForPackageURL(std::string url);
  std::string GetFilePathForFileURL(std::string url);

  std::string GetFileURLForPath(const std::string& path);

  bool ReadFileToString(const std::string& path, std::string* result);
  std::pair<uint8_t*, intptr_t> ReadFileToBytes(const std::string& path);

  int dirfd_;
  std::string packages_;
  std::unique_ptr<PackagesMap> packages_map_;
  std::vector<uint8_t*> kernel_buffers_;

  TONIC_DISALLOW_COPY_AND_ASSIGN(FileLoader);
};

}  // namespace tonic

#endif  // LIB_TONIC_FILE_LOADER_FILE_LOADER_H_
