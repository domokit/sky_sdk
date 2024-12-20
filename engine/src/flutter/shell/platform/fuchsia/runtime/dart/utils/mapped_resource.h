// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_FUCHSIA_RUNTIME_DART_UTILS_MAPPED_RESOURCE_H_
#define FLUTTER_SHELL_PLATFORM_FUCHSIA_RUNTIME_DART_UTILS_MAPPED_RESOURCE_H_

#include <fuchsia/mem/cpp/fidl.h>
#include <lib/fdio/namespace.h>

#include "third_party/dart/runtime/bin/elf_loader.h"

namespace dart_utils {

class ElfSnapshot {
 public:
  ElfSnapshot() {}
  ~ElfSnapshot();
  ElfSnapshot(ElfSnapshot&& other) : handle_(other.handle_) {
    other.handle_ = nullptr;
  }
  ElfSnapshot& operator=(ElfSnapshot&& other) {
    std::swap(handle_, other.handle_);
    return *this;
  }

  bool Load(fdio_ns_t* namespc, const std::string& path);
  bool Load(int dirfd, const std::string& path);

  const uint8_t* VmData() const { return vm_data_; }
  const uint8_t* VmInstrs() const { return vm_instrs_; }
  const uint8_t* IsolateData() const { return isolate_data_; }
  const uint8_t* IsolateInstrs() const { return isolate_instrs_; }

 private:
  bool Load(int fd);

  Dart_LoadedElf* handle_ = nullptr;

  const uint8_t* vm_data_ = nullptr;
  const uint8_t* vm_instrs_ = nullptr;
  const uint8_t* isolate_data_ = nullptr;
  const uint8_t* isolate_instrs_ = nullptr;

  // Disallow copy and assignment.
  ElfSnapshot(const ElfSnapshot&) = delete;
  ElfSnapshot& operator=(const ElfSnapshot&) = delete;
};

class MappedResource {
 public:
  MappedResource() : address_(nullptr), size_(0) {}
  MappedResource(MappedResource&& other)
      : address_(other.address_), size_(other.size_) {
    other.address_ = nullptr;
    other.size_ = 0;
  }
  MappedResource& operator=(MappedResource&& other) {
    address_ = other.address_;
    size_ = other.size_;
    other.address_ = nullptr;
    other.size_ = 0;
    return *this;
  }
  ~MappedResource();

  // Loads the content of a file from the given namespace and maps it into the
  // current process's address space. If namespace is null, the fdio "global"
  // namespace is used (in which case, ./pkg means the dart_runner's package).
  // The content is unmapped when the MappedResource goes out of scope. Returns
  // true on success.
  static bool LoadFromNamespace(fdio_ns_t* namespc,
                                const std::string& path,
                                MappedResource& resource,
                                bool executable = false);

  // Same as LoadFromNamespace, but takes a file descriptor to an opened
  // directory instead of a namespace.
  static bool LoadFromDir(int dirfd,
                          const std::string& path,
                          MappedResource& resource,
                          bool executable = false);

  // Maps a VMO into the current process's address space. The content is
  // unmapped when the MappedResource goes out of scope. Returns true on
  // success. The path is used only for error messages.
  static bool LoadFromVmo(const std::string& path,
                          fuchsia::mem::Buffer resource_vmo,
                          MappedResource& resource,
                          bool executable = false);

  const uint8_t* address() const {
    return reinterpret_cast<const uint8_t*>(address_);
  }
  size_t size() const { return size_; }

 private:
  void* address_;
  size_t size_;

  // Disallow copy and assignment.
  MappedResource(const MappedResource&) = delete;
  MappedResource& operator=(const MappedResource&) = delete;
};

}  // namespace dart_utils

#endif  // FLUTTER_SHELL_PLATFORM_FUCHSIA_RUNTIME_DART_UTILS_MAPPED_RESOURCE_H_
