// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_FUCHSIA_RUNTIME_DART_UTILS_TEMPFS_H_
#define FLUTTER_SHELL_PLATFORM_FUCHSIA_RUNTIME_DART_UTILS_TEMPFS_H_

#include <lib/fdio/namespace.h>

namespace dart_utils {

// Take the virtual filesystem mapped into the process-wide namespace for /tmp,
// and map it to /tmp in the given namespace.
void BindTemp(fdio_ns_t* ns);

}  // namespace dart_utils

#endif  // FLUTTER_SHELL_PLATFORM_FUCHSIA_RUNTIME_DART_UTILS_TEMPFS_H_
