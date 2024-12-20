// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FML_UNIQUE_FD_H_
#define FLUTTER_FML_UNIQUE_FD_H_

#include "flutter/fml/build_config.h"
#include "flutter/fml/unique_object.h"

#if FML_OS_WIN
#include <windows.h>
#include <map>
#include <mutex>
#include <optional>
#else  // FML_OS_WIN
#include <dirent.h>
#include <unistd.h>
#endif  // FML_OS_WIN

namespace fml {
namespace internal {

#if FML_OS_WIN

namespace os_win {

struct DirCacheEntry {
  std::wstring filename;
  FILE_ID_128 id;
};

// The order of these is important.  Must come before UniqueFDTraits struct
// else linker error.  Embedding in struct also causes linker error.

struct UniqueFDTraits {
  static std::mutex file_map_mutex;
  static std::map<HANDLE, DirCacheEntry> file_map;

  static HANDLE InvalidValue() { return INVALID_HANDLE_VALUE; }
  static bool IsValid(HANDLE value) { return value != InvalidValue(); }
  static void Free_Handle(HANDLE fd);

  static void Free(HANDLE fd) {
    RemoveCacheEntry(fd);

    UniqueFDTraits::Free_Handle(fd);
  }

  static void RemoveCacheEntry(HANDLE fd) {
    const std::lock_guard<std::mutex> lock(file_map_mutex);

    file_map.erase(fd);
  }

  static void StoreCacheEntry(HANDLE fd, DirCacheEntry state) {
    const std::lock_guard<std::mutex> lock(file_map_mutex);
    file_map[fd] = state;
  }

  static std::optional<DirCacheEntry> GetCacheEntry(HANDLE fd) {
    const std::lock_guard<std::mutex> lock(file_map_mutex);
    auto found = file_map.find(fd);
    return found == file_map.end()
               ? std::nullopt
               : std::optional<DirCacheEntry>{found->second};
  }
};

}  // namespace os_win

#else  // FML_OS_WIN

namespace os_unix {

struct UniqueFDTraits {
  static int InvalidValue() { return -1; }
  static bool IsValid(int value) { return value >= 0; }
  static void Free(int fd);
};

struct UniqueDirTraits {
  static DIR* InvalidValue() { return nullptr; }
  static bool IsValid(DIR* value) { return value != nullptr; }
  static void Free(DIR* dir);
};

}  // namespace os_unix

#endif  // FML_OS_WIN

}  // namespace internal

#if FML_OS_WIN

using UniqueFD = UniqueObject<HANDLE, internal::os_win::UniqueFDTraits>;

#else  // FML_OS_WIN

using UniqueFD = UniqueObject<int, internal::os_unix::UniqueFDTraits>;
using UniqueDir = UniqueObject<DIR*, internal::os_unix::UniqueDirTraits>;

#endif  // FML_OS_WIN

}  // namespace fml

#endif  // FLUTTER_FML_UNIQUE_FD_H_
