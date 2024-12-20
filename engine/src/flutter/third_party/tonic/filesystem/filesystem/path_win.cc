// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "tonic/filesystem/filesystem/path.h"

#include <windows.h>

#include <direct.h>
#include <shellapi.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <functional>
#include <list>
#include <memory>

namespace filesystem {
namespace {

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

size_t IsSeparator(const char sep) {
  return sep == '/' || sep == '\\';
}

size_t LastSeparator(const std::string& path) {
  return path.find_last_of("/\\");
}

size_t LastSeparator(const std::string& path, size_t pos) {
  return path.find_last_of("/\\", pos);
}

size_t FirstSeparator(const std::string& path, size_t pos) {
  return path.find_first_of("/\\", pos);
}

size_t ResolveParentDirectoryTraversal(const std::string& path,
                                       size_t put,
                                       size_t root_length) {
  if (put <= root_length) {
    return root_length;
  }
  size_t previous_separator = LastSeparator(path, put - 2);
  if (previous_separator != std::string::npos)
    return previous_separator + 1;
  return 0;
}
}  // namespace

std::string SimplifyPath(std::string path) {
  if (path.empty())
    return ".";

  size_t put = 0;
  size_t get = 0;
  size_t traversal_root = 0;
  size_t component_start = 0;

  size_t rootLength = RootLength(path);
  if (rootLength > 0) {
    put = rootLength;
    get = rootLength;
    component_start = rootLength;
  }

  while (get < path.size()) {
    char c = path[get];

    if (c == '.' && (get == component_start || get == component_start + 1)) {
      // We've seen "." or ".." so far in this component. We need to continue
      // searching.
      ++get;
      continue;
    }

    if (IsSeparator(c)) {
      if (get == component_start || get == component_start + 1) {
        // We've found a "/" or a "./", which we can elide.
        ++get;
        component_start = get;
        continue;
      }
      if (get == component_start + 2) {
        // We've found a "../", which means we need to remove the previous
        // component.
        if (put == traversal_root) {
          path[put++] = '.';
          path[put++] = '.';
          path[put++] = '\\';
          traversal_root = put;
        } else {
          put = ResolveParentDirectoryTraversal(path, put, rootLength);
        }
        ++get;
        component_start = get;
        continue;
      }
    }

    size_t next_separator = FirstSeparator(path, get);
    if (next_separator == std::string::npos) {
      // We've reached the last component.
      break;
    }
    size_t next_component_start = next_separator + 1;
    ++next_separator;
    size_t component_size = next_component_start - component_start;
    if (put != component_start && component_size > 0) {
      path.replace(put, component_size,
                   path.substr(component_start, component_size));
    }
    put += component_size;
    get = next_component_start;
    component_start = next_component_start;
  }

  size_t last_component_size = path.size() - component_start;
  if (last_component_size == 1 && path[component_start] == '.') {
    // The last component is ".", which we can elide.
  } else if (last_component_size == 2 && path[component_start] == '.' &&
             path[component_start + 1] == '.') {
    // The last component is "..", which means we need to remove the previous
    // component.
    if (put == traversal_root) {
      path[put++] = '.';
      path[put++] = '.';
      path[put++] = '\\';
      traversal_root = put;
    } else {
      put = ResolveParentDirectoryTraversal(path, put, rootLength);
    }
  } else {
    // Otherwise, we need to copy over the last component.
    if (put != component_start && last_component_size > 0) {
      path.replace(put, last_component_size,
                   path.substr(component_start, last_component_size));
    }
    put += last_component_size;
  }

  if (put >= 2 && IsSeparator(path[put - 1]))
    --put;  // Trim trailing /
  else if (put == 0)
    return ".";  // Use . for otherwise empty paths to treat them as relative.

  path.resize(put);
  std::replace(path.begin(), path.end(), '/', '\\');
  return path;
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

std::string GetBaseName(const std::string& path) {
  size_t separator = LastSeparator(path);
  if (separator == std::string::npos)
    return path;
  return path.substr(separator + 1);
}

std::string GetAbsoluteFilePath(const std::string& path) {
  HANDLE file =
      CreateFileA(path.c_str(), FILE_READ_ATTRIBUTES,
                  FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE, NULL,
                  OPEN_EXISTING, FILE_FLAG_BACKUP_SEMANTICS, NULL);
  if (file == INVALID_HANDLE_VALUE) {
    return std::string();
  }
  char buffer[MAX_PATH];
  DWORD ret =
      GetFinalPathNameByHandleA(file, buffer, MAX_PATH, FILE_NAME_NORMALIZED);
  if (ret == 0 || ret > MAX_PATH) {
    std::string result;
    if (GetLastError() == ERROR_ACCESS_DENIED) {
      // In sandboxed apps, GetFinalPathNameByHandle requires the app to
      // declare appropriate capabilities in the app's package manifest. Some
      // of these capabilities are not permitted in shipping apps on the app
      // store, but may be fine for development/debugging scenarios. If we
      // can't resolve the full path due to insufficient access, but have
      // verified the handle is valid, return AbsolutePath of the original
      // path.
      //
      // https://github.com/flutter/flutter/issues/79609
      result = AbsolutePath(path);
    }
    CloseHandle(file);
    return result;
  }
  std::string result(buffer);
  result.erase(0, strlen("\\\\?\\"));
  CloseHandle(file);
  return result;
}

}  // namespace filesystem
