// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_UNIQUE_FDIO_NS_H_
#define FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_UNIQUE_FDIO_NS_H_

#include "flutter/fml/logging.h"
#include "flutter/fml/unique_object.h"
#include "lib/fdio/namespace.h"

namespace flutter_runner {

struct UniqueFDIONSTraits {
  static fdio_ns_t* InvalidValue() { return nullptr; }

  static bool IsValid(fdio_ns_t* ns) { return ns != InvalidValue(); }

  static void Free(fdio_ns_t* ns) {
    auto status = fdio_ns_destroy(ns);
    FML_DCHECK(status == ZX_OK);
  }
};

using UniqueFDIONS = fml::UniqueObject<fdio_ns_t*, UniqueFDIONSTraits>;

inline UniqueFDIONS UniqueFDIONSCreate() {
  fdio_ns_t* ns = nullptr;
  if (fdio_ns_create(&ns) == ZX_OK) {
    return UniqueFDIONS{ns};
  }
  return UniqueFDIONS{nullptr};
}

}  // namespace flutter_runner

#endif  // FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_UNIQUE_FDIO_NS_H_
