// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "tempfs.h"

#include <string_view>

#include <zircon/errors.h>
#include <zircon/status.h>
#include <zircon/syscalls.h>

#include "flutter/fml/logging.h"

namespace {

constexpr char kTmpPath[] = "/tmp";

}  // namespace

namespace dart_utils {

void BindTemp(fdio_ns_t* ns) {
  // TODO(zra): Should isolates share a /tmp file system within a process, or
  // should isolates each get their own private file system for /tmp? For now,
  // sharing the process-wide /tmp simplifies hot reload since the hot reload
  // devfs requires sharing between the service isolate and the app isolates.
  fdio_flat_namespace_t* rootns;
  if (zx_status_t status = fdio_ns_export_root(&rootns); status != ZX_OK) {
    FML_LOG(ERROR) << "Failed to export root ns: "
                   << zx_status_get_string(status);
    return;
  }

  zx_handle_t tmp_dir_handle;
  for (size_t i = 0; i < rootns->count; i++) {
    if (std::string_view{rootns->path[i]} == kTmpPath) {
      tmp_dir_handle = std::exchange(rootns->handle[i], ZX_HANDLE_INVALID);
    }
  }
  fdio_ns_free_flat_ns(rootns);

  if (zx_status_t status = fdio_ns_bind(ns, kTmpPath, tmp_dir_handle);
      status != ZX_OK) {
    zx_handle_close(tmp_dir_handle);
    FML_LOG(ERROR) << "Failed to bind /tmp directory into isolate namespace: "
                   << zx_status_get_string(status);
  }
}

}  // namespace dart_utils
