// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "file_in_namespace_buffer.h"

#include <fuchsia/io/cpp/fidl.h>
#include <lib/fdio/directory.h>
#include <zircon/status.h>

#include "flutter/fml/trace_event.h"
#include "runtime/dart/utils/files.h"
#include "runtime/dart/utils/handle_exception.h"
#include "runtime/dart/utils/mapped_resource.h"
#include "runtime/dart/utils/tempfs.h"
#include "runtime/dart/utils/vmo.h"

namespace flutter_runner {

FileInNamespaceBuffer::FileInNamespaceBuffer(int namespace_fd,
                                             const char* path,
                                             bool executable)
    : address_(nullptr), size_(0) {
  fuchsia::mem::Buffer buffer;
  if (!dart_utils::VmoFromFilenameAt(namespace_fd, path, executable, &buffer) ||
      buffer.size == 0) {
    return;
  }

  uint32_t flags = ZX_VM_PERM_READ;
  if (executable) {
    flags |= ZX_VM_PERM_EXECUTE;
  }

  uintptr_t addr;
  const zx_status_t status =
      zx::vmar::root_self()->map(flags, 0, buffer.vmo, 0, buffer.size, &addr);
  if (status != ZX_OK) {
    FML_LOG(FATAL) << "Failed to map " << path << ": "
                   << zx_status_get_string(status);
  }

  address_ = reinterpret_cast<void*>(addr);
  size_ = buffer.size;
}

FileInNamespaceBuffer::~FileInNamespaceBuffer() {
  if (address_ != nullptr) {
    zx::vmar::root_self()->unmap(reinterpret_cast<uintptr_t>(address_), size_);
    address_ = nullptr;
    size_ = 0;
  }
}

const uint8_t* FileInNamespaceBuffer::GetMapping() const {
  return reinterpret_cast<const uint8_t*>(address_);
}

size_t FileInNamespaceBuffer::GetSize() const {
  return size_;
}

bool FileInNamespaceBuffer::IsDontNeedSafe() const {
  return true;
}

std::unique_ptr<fml::Mapping> LoadFile(int namespace_fd,
                                       const char* path,
                                       bool executable) {
  FML_TRACE_EVENT("flutter", "LoadFile", "path", path);
  return std::make_unique<FileInNamespaceBuffer>(namespace_fd, path,
                                                 executable);
}

std::unique_ptr<fml::FileMapping> MakeFileMapping(const char* path,
                                                  bool executable) {
  fuchsia::io::Flags flags = fuchsia::io::PERM_READABLE;
  if (executable) {
    flags |= fuchsia::io::PERM_EXECUTABLE;
  }

  // The returned file descriptor is compatible with standard posix operations
  // such as close, mmap, etc. We only need to treat open/open_at specially.
  int fd;
  const zx_status_t status = fdio_open3_fd(path, uint64_t{flags}, &fd);
  if (status != ZX_OK) {
    return nullptr;
  }

  using Protection = fml::FileMapping::Protection;

  std::initializer_list<Protection> protection_execute = {Protection::kRead,
                                                          Protection::kExecute};
  std::initializer_list<Protection> protection_read = {Protection::kRead};
  auto mapping = std::make_unique<fml::FileMapping>(
      fml::UniqueFD{fd}, executable ? protection_execute : protection_read);

  if (!mapping->IsValid()) {
    return nullptr;
  }
  return mapping;
}

}  // namespace flutter_runner
