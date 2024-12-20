// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/fml/paths.h"

#include <windows.h>

#include <algorithm>

#include "flutter/fml/paths.h"
#include "flutter/fml/platform/win/wstring_conversion.h"

namespace fml {
namespace paths {

namespace {

constexpr char kFileURLPrefix[] = "file:///";
constexpr size_t kFileURLPrefixLength = sizeof(kFileURLPrefix) - 1;

size_t RootLength(const std::string& path) {
  if (path.empty())
    return 0;
  if (path[0] == '/')
    return 1;
  if (path[0] == '\\') {
    if (path.size() < 2 || path[1] != '\\')
      return 1;
    // The path is a network share. Search for up to two '\'s, as they are
    // the server and share - and part of the root part.
    size_t index = path.find('\\', 2);
    if (index > 0) {
      index = path.find('\\', index + 1);
      if (index > 0)
        return index;
    }
    return path.size();
  }
  // If the path is of the form 'C:/' or 'C:\', with C being any letter, it's
  // a root part.
  if (path.length() >= 2 && path[1] == ':' &&
      (path[2] == '/' || path[2] == '\\') &&
      ((path[0] >= 'A' && path[0] <= 'Z') ||
       (path[0] >= 'a' && path[0] <= 'z'))) {
    return 3;
  }
  return 0;
}

size_t LastSeparator(const std::string& path) {
  return path.find_last_of("/\\");
}

}  // namespace

std::pair<bool, std::string> GetExecutablePath() {
  HMODULE module = GetModuleHandle(NULL);
  if (module == NULL) {
    return {false, ""};
  }
  wchar_t path[MAX_PATH];
  DWORD read_size = GetModuleFileNameW(module, path, MAX_PATH);
  if (read_size == 0 || read_size == MAX_PATH) {
    return {false, ""};
  }
  return {true, WideStringToUtf8({path, read_size})};
}

std::string AbsolutePath(const std::string& path) {
  char absPath[MAX_PATH];
  _fullpath(absPath, path.c_str(), MAX_PATH);
  return std::string(absPath);
}

std::string GetDirectoryName(const std::string& path) {
  size_t rootLength = RootLength(path);
  size_t separator = LastSeparator(path);
  if (separator < rootLength)
    separator = rootLength;
  if (separator == std::string::npos)
    return std::string();
  return path.substr(0, separator);
}

std::string FromURI(const std::string& uri) {
  if (uri.substr(0, kFileURLPrefixLength) != kFileURLPrefix)
    return uri;

  std::string file_path = uri.substr(kFileURLPrefixLength);
  std::replace(file_path.begin(), file_path.end(), '/', '\\');
  return SanitizeURIEscapedCharacters(file_path);
}

fml::UniqueFD GetCachesDirectory() {
  // Unsupported on this platform.
  return {};
}

}  // namespace paths
}  // namespace fml
