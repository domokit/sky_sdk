// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "mapped_resource.h"

#include <dlfcn.h>
#include <fcntl.h>
#include <fuchsia/io/cpp/fidl.h>
#include <fuchsia/mem/cpp/fidl.h>
#include <lib/fdio/directory.h>
#include <lib/fdio/io.h>
#include <lib/trace/event.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <zircon/dlfcn.h>
#include <zircon/status.h>

#include "flutter/fml/logging.h"
#include "third_party/dart/runtime/include/dart_api.h"

#include "inlines.h"
#include "vmo.h"

namespace dart_utils {

static bool OpenVmo(fuchsia::mem::Buffer* resource_vmo,
                    fdio_ns_t* namespc,
                    const std::string& path,
                    bool executable) {
  TRACE_DURATION("dart", "LoadFromNamespace", "path", path);

  if (namespc == nullptr) {
    // Opening a file in the root namespace expects an absolute path.
    FML_CHECK(path[0] == '/');
    if (!VmoFromFilename(path, executable, resource_vmo)) {
      return false;
    }
  } else {
    // openat of a path with a leading '/' ignores the namespace fd.
    // require a relative path.
    FML_CHECK(path[0] != '/');

    auto root_dir = fdio_ns_opendir(namespc);
    if (root_dir < 0) {
      FML_LOG(ERROR) << "Failed to open namespace directory";
      return false;
    }

    bool result =
        dart_utils::VmoFromFilenameAt(root_dir, path, executable, resource_vmo);
    close(root_dir);
    if (!result) {
      return result;
    }
  }

  return true;
}

bool MappedResource::LoadFromNamespace(fdio_ns_t* namespc,
                                       const std::string& path,
                                       MappedResource& resource,
                                       bool executable) {
  fuchsia::mem::Buffer resource_vmo;
  return OpenVmo(&resource_vmo, namespc, path, executable) &&
         LoadFromVmo(path, std::move(resource_vmo), resource, executable);
}

bool MappedResource::LoadFromVmo(const std::string& path,
                                 fuchsia::mem::Buffer resource_vmo,
                                 MappedResource& resource,
                                 bool executable) {
  if (resource_vmo.size == 0) {
    return true;
  }

  uint32_t flags = ZX_VM_PERM_READ;
  if (executable) {
    flags |= ZX_VM_PERM_EXECUTE;
  }
  uintptr_t addr;
  zx_status_t status = zx::vmar::root_self()->map(flags, 0, resource_vmo.vmo, 0,
                                                  resource_vmo.size, &addr);
  if (status != ZX_OK) {
    FML_LOG(ERROR) << "Failed to map: " << zx_status_get_string(status);
    return false;
  }

  resource.address_ = reinterpret_cast<void*>(addr);
  resource.size_ = resource_vmo.size;
  return true;
}

MappedResource::~MappedResource() {
  if (address_ != nullptr) {
    zx::vmar::root_self()->unmap(reinterpret_cast<uintptr_t>(address_), size_);
    address_ = nullptr;
    size_ = 0;
  }
}

static int OpenFdExec(const std::string& path, int dirfd) {
  int fd = -1;
  // fdio_open3_fd_at only allows relative paths
  const char* path_ptr = path.c_str();
  if (path_ptr && path_ptr[0] == '/') {
    ++path_ptr;
  }
  zx_status_t result = fdio_open3_fd_at(
      dirfd, path_ptr,
      uint64_t{fuchsia::io::PERM_READABLE | fuchsia::io::PERM_EXECUTABLE}, &fd);
  if (result != ZX_OK) {
    FML_LOG(ERROR) << "fdio_open3_fd_at(" << path << ") "
                   << "failed: " << zx_status_get_string(result);
    return -1;
  }
  return fd;
}

bool ElfSnapshot::Load(fdio_ns_t* namespc, const std::string& path) {
  int root_dir = -1;
  if (namespc == nullptr) {
    root_dir = AT_FDCWD;
  } else {
    root_dir = fdio_ns_opendir(namespc);
    if (root_dir < 0) {
      FML_LOG(ERROR) << "Failed to open namespace directory";
      return false;
    }
  }
  return Load(root_dir, path);
}

bool ElfSnapshot::Load(int dirfd, const std::string& path) {
  const int fd = OpenFdExec(path, dirfd);
  if (fd < 0) {
    FML_LOG(ERROR) << "Failed to open VMO for " << path << " from dir.";
    return false;
  }
  return Load(fd);
}

bool ElfSnapshot::Load(int fd) {
  const char* error;
  handle_ = Dart_LoadELF_Fd(fd, 0, &error, &vm_data_, &vm_instrs_,
                            &isolate_data_, &isolate_instrs_);
  if (handle_ == nullptr) {
    FML_LOG(ERROR) << "Failed load ELF: " << error;
    return false;
  }
  return true;
}

ElfSnapshot::~ElfSnapshot() {
  Dart_UnloadELF(handle_);
}

}  // namespace dart_utils
