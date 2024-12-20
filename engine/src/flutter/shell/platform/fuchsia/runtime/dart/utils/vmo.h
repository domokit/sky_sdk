// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_FUCHSIA_RUNTIME_DART_UTILS_VMO_H_
#define FLUTTER_SHELL_PLATFORM_FUCHSIA_RUNTIME_DART_UTILS_VMO_H_

#include <string>

#include <fuchsia/mem/cpp/fidl.h>

namespace dart_utils {

bool VmoFromFilename(const std::string& filename,
                     bool executable,
                     fuchsia::mem::Buffer* buffer);

bool VmoFromFilenameAt(int dirfd,
                       const std::string& filename,
                       bool executable,
                       fuchsia::mem::Buffer* buffer);

zx_status_t IsSizeValid(const fuchsia::mem::Buffer& buffer, bool* is_valid);

}  // namespace dart_utils

#endif  // FLUTTER_SHELL_PLATFORM_FUCHSIA_RUNTIME_DART_UTILS_VMO_H_
