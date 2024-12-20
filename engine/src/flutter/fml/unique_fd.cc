// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/fml/unique_fd.h"

#include "flutter/fml/eintr_wrapper.h"

namespace fml {
namespace internal {

#if FML_OS_WIN

namespace os_win {

std::mutex UniqueFDTraits::file_map_mutex;
std::map<HANDLE, DirCacheEntry> UniqueFDTraits::file_map;

void UniqueFDTraits::Free_Handle(HANDLE fd) {
  CloseHandle(fd);
}

}  // namespace os_win

#else  // FML_OS_WIN

namespace os_unix {

void UniqueFDTraits::Free(int fd) {
  close(fd);
}

void UniqueDirTraits::Free(DIR* dir) {
  closedir(dir);
}

}  // namespace os_unix

#endif  // FML_OS_WIN

}  // namespace internal
}  // namespace fml
