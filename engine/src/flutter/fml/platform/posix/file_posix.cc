// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/fml/file.h"

#include <dirent.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cstring>
#include <memory>
#include <sstream>

#include "flutter/fml/eintr_wrapper.h"
#include "flutter/fml/logging.h"
#include "flutter/fml/mapping.h"
#include "flutter/fml/trace_event.h"
#include "flutter/fml/unique_fd.h"

namespace fml {

std::string CreateTemporaryDirectory() {
  char directory_name[] = "/tmp/flutter_XXXXXXXX";
  auto* result = ::mkdtemp(directory_name);
  if (result == nullptr) {
    return "";
  }
  return {result};
}

static int ToPosixAccessFlags(FilePermission permission) {
  int flags = 0;
  switch (permission) {
    case FilePermission::kRead:
      flags |= O_RDONLY;  // read only
      break;
    case FilePermission::kWrite:
      flags |= O_WRONLY;  // write only
      break;
    case FilePermission::kReadWrite:
      flags |= O_RDWR;  // read-write
      break;
  }
  return flags;
}

static int ToPosixCreateModeFlags(FilePermission permission) {
  int mode = 0;
  switch (permission) {
    case FilePermission::kRead:
      mode |= S_IRUSR;
      break;
    case FilePermission::kWrite:
      mode |= S_IWUSR;
      break;
    case FilePermission::kReadWrite:
      mode |= S_IRUSR | S_IWUSR;
      break;
  }
  return mode;
}

fml::UniqueFD OpenFile(const char* path,
                       bool create_if_necessary,
                       FilePermission permission) {
  return OpenFile(fml::UniqueFD{AT_FDCWD}, path, create_if_necessary,
                  permission);
}

fml::UniqueFD OpenFile(const fml::UniqueFD& base_directory,
                       const char* path,
                       bool create_if_necessary,
                       FilePermission permission) {
  TRACE_EVENT0("flutter", "fml::OpenFile");
  if (path == nullptr) {
    return {};
  }

  int flags = 0;
  int mode = 0;

  if (create_if_necessary && !FileExists(base_directory, path)) {
    flags = ToPosixAccessFlags(permission) | O_CREAT | O_TRUNC;
    mode = ToPosixCreateModeFlags(permission);
  } else {
    flags = ToPosixAccessFlags(permission);
    mode = 0;  // Not creating since it already exists.
  }

  return fml::UniqueFD{
      FML_HANDLE_EINTR(::openat(base_directory.get(), path, flags, mode))};
}

fml::UniqueFD OpenDirectory(const char* path,
                            bool create_if_necessary,
                            FilePermission permission) {
  return OpenDirectory(fml::UniqueFD{AT_FDCWD}, path, create_if_necessary,
                       permission);
}

fml::UniqueFD OpenDirectory(const fml::UniqueFD& base_directory,
                            const char* path,
                            bool create_if_necessary,
                            FilePermission permission) {
  if (path == nullptr) {
    return {};
  }

  if (create_if_necessary && !FileExists(base_directory, path)) {
    if (::mkdirat(base_directory.get(), path,
                  ToPosixCreateModeFlags(permission) | S_IXUSR) != 0) {
      return {};
    }
  }

  return fml::UniqueFD{FML_HANDLE_EINTR(
      ::openat(base_directory.get(), path, O_RDONLY | O_DIRECTORY))};
}

fml::UniqueFD Duplicate(fml::UniqueFD::element_type descriptor) {
  return fml::UniqueFD{FML_HANDLE_EINTR(::dup(descriptor))};
}

bool IsDirectory(const fml::UniqueFD& directory) {
  if (!directory.is_valid()) {
    return false;
  }

  struct stat stat_result = {};

  if (::fstat(directory.get(), &stat_result) != 0) {
    return false;
  }

  return S_ISDIR(stat_result.st_mode);
}

bool IsDirectory(const fml::UniqueFD& base_directory, const char* path) {
  UniqueFD file = OpenFileReadOnly(base_directory, path);
  return (file.is_valid() && IsDirectory(file));
}

bool IsFile(const std::string& path) {
  struct stat buf;
  if (stat(path.c_str(), &buf) != 0) {
    return false;
  }

  return S_ISREG(buf.st_mode);
}

bool TruncateFile(const fml::UniqueFD& file, size_t size) {
  if (!file.is_valid()) {
    return false;
  }

  return ::ftruncate(file.get(), size) == 0;
}

bool UnlinkDirectory(const char* path) {
  return UnlinkDirectory(fml::UniqueFD{AT_FDCWD}, path);
}

bool UnlinkDirectory(const fml::UniqueFD& base_directory, const char* path) {
  return ::unlinkat(base_directory.get(), path, AT_REMOVEDIR) == 0;
}

bool UnlinkFile(const char* path) {
  return UnlinkFile(fml::UniqueFD{AT_FDCWD}, path);
}

bool UnlinkFile(const fml::UniqueFD& base_directory, const char* path) {
  int code = ::unlinkat(base_directory.get(), path, 0);
  if (code != 0) {
    FML_DLOG(ERROR) << strerror(errno);
  }
  return code == 0;
}

bool FileExists(const fml::UniqueFD& base_directory, const char* path) {
  if (!base_directory.is_valid()) {
    return false;
  }

  return ::faccessat(base_directory.get(), path, F_OK, 0) == 0;
}

bool WriteAtomically(const fml::UniqueFD& base_directory,
                     const char* file_name,
                     const Mapping& data) {
  if (file_name == nullptr || data.GetMapping() == nullptr) {
    return false;
  }

  std::stringstream stream;
  stream << file_name << ".temp";
  const auto temp_file_name = stream.str();

  auto temp_file = OpenFile(base_directory, temp_file_name.c_str(), true,
                            FilePermission::kReadWrite);
  if (!temp_file.is_valid()) {
    return false;
  }

  if (!TruncateFile(temp_file, data.GetSize())) {
    return false;
  }

  ssize_t remaining = data.GetSize();
  ssize_t written = 0;
  ssize_t offset = 0;

  while (remaining > 0) {
    written = FML_HANDLE_EINTR(
        ::write(temp_file.get(), data.GetMapping() + offset, remaining));

    if (written == -1) {
      return false;
    }

    remaining -= written;
    offset += written;
  }

  if (::fsync(temp_file.get()) != 0) {
    return false;
  }

  return ::renameat(base_directory.get(), temp_file_name.c_str(),
                    base_directory.get(), file_name) == 0;
}

bool VisitFiles(const fml::UniqueFD& directory, const FileVisitor& visitor) {
  fml::UniqueFD dup_fd(dup(directory.get()));
  if (!dup_fd.is_valid()) {
    FML_DLOG(ERROR) << "Can't dup the directory fd. Error: " << strerror(errno);
    return true;  // continue to visit other files
  }

  fml::UniqueDir dir(::fdopendir(dup_fd.get()));
  if (!dir.is_valid()) {
    FML_DLOG(ERROR) << "Can't open the directory. Error: " << strerror(errno);
    return true;  // continue to visit other files
  }

  // The directory fd will be closed by `closedir`.
  (void)dup_fd.release();

  // Without `rewinddir`, `readir` will directly return NULL (end of dir is
  // reached) after a previuos `VisitFiles` call for the same `const
  // fml::UniqueFd& directory`.
  rewinddir(dir.get());
  while (dirent* ent = readdir(dir.get())) {
    std::string filename = ent->d_name;
    if (filename != "." && filename != "..") {
      if (!visitor(directory, filename)) {
        return false;
      }
    }
  }

  return true;
}

}  // namespace fml
