// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "vmo.h"

#include <fcntl.h>
#include <sys/stat.h>

#include <fuchsia/io/cpp/fidl.h>
#include <fuchsia/mem/cpp/fidl.h>
#include <lib/fdio/directory.h>
#include <lib/fdio/io.h>
#include <zircon/status.h>

#include <ios>

#include "flutter/fml/logging.h"

namespace {

bool VmoFromFd(int fd, bool executable, fuchsia::mem::Buffer* buffer) {
  if (!buffer) {
    FML_LOG(FATAL) << "Invalid buffer pointer";
  }

  struct stat stat_struct;
  if (fstat(fd, &stat_struct) == -1) {
    FML_LOG(ERROR) << "fstat failed: " << strerror(errno);
    return false;
  }

  zx_handle_t result = ZX_HANDLE_INVALID;
  zx_status_t status;
  if (executable) {
    status = fdio_get_vmo_exec(fd, &result);
  } else {
    status = fdio_get_vmo_copy(fd, &result);
  }

  if (status != ZX_OK) {
    FML_LOG(ERROR) << (executable ? "fdio_get_vmo_exec" : "fdio_get_vmo_copy")
                   << " failed: " << zx_status_get_string(status);
    return false;
  }

  buffer->vmo = zx::vmo(result);
  buffer->size = stat_struct.st_size;

  return true;
}

}  // namespace

namespace dart_utils {

bool VmoFromFilename(const std::string& filename,
                     bool executable,
                     fuchsia::mem::Buffer* buffer) {
  return VmoFromFilenameAt(AT_FDCWD, filename, executable, buffer);
}

bool VmoFromFilenameAt(int dirfd,
                       const std::string& filename,
                       bool executable,
                       fuchsia::mem::Buffer* buffer) {
  fuchsia::io::Flags flags = fuchsia::io::PERM_READABLE;
  if (executable) {
    flags |= fuchsia::io::PERM_EXECUTABLE;
  }
  // fdio_open3_fd_at only allows relative paths
  const char* path = filename.c_str();
  if (path && path[0] == '/') {
    ++path;
  }
  int fd;
  const zx_status_t status =
      fdio_open3_fd_at(dirfd, path, uint64_t{flags}, &fd);
  if (status != ZX_OK) {
    FML_LOG(ERROR) << "fdio_open3_fd_at(" << dirfd << ", \"" << filename
                   << "\", " << std::hex << uint64_t{flags}
                   << ") failed: " << zx_status_get_string(status);
    return false;
  }
  bool result = VmoFromFd(fd, executable, buffer);
  close(fd);
  return result;
}

zx_status_t IsSizeValid(const fuchsia::mem::Buffer& buffer, bool* is_valid) {
  size_t vmo_size;
  zx_status_t status = buffer.vmo.get_size(&vmo_size);
  if (status == ZX_OK) {
    *is_valid = vmo_size >= buffer.size;
  } else {
    *is_valid = false;
  }
  return status;
}

}  // namespace dart_utils
