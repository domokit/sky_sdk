// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "files.h"

#include <fcntl.h>
#include <unistd.h>

#include <cstdint>

#include "flutter/fml/logging.h"

namespace dart_utils {

namespace {

bool ReadFileDescriptor(int fd, std::string* result) {
  FML_DCHECK(result);
  result->clear();

  if (fd < 0) {
    return false;
  }

  constexpr size_t kBufferSize = 1 << 16;
  size_t offset = 0;
  ssize_t bytes_read = 0;
  do {
    offset += bytes_read;
    result->resize(offset + kBufferSize);
    bytes_read = read(fd, &(*result)[offset], kBufferSize);
  } while (bytes_read > 0);

  if (bytes_read < 0) {
    result->clear();
    return false;
  }

  result->resize(offset + bytes_read);
  return true;
}

bool WriteFileDescriptor(int fd, const char* data, ssize_t size) {
  ssize_t total = 0;
  for (ssize_t partial = 0; total < size; total += partial) {
    partial = write(fd, data + total, size - total);
    if (partial < 0)
      return false;
  }
  return true;
}

}  // namespace

bool ReadFileToString(const std::string& path, std::string* result) {
  return ReadFileToStringAt(AT_FDCWD, path, result);
}

bool ReadFileToStringAt(int dirfd,
                        const std::string& path,
                        std::string* result) {
  int fd = openat(dirfd, path.c_str(), O_RDONLY);
  bool status = ReadFileDescriptor(fd, result);
  close(fd);
  return status;
}

bool WriteFile(const std::string& path, const char* data, ssize_t size) {
  int fd = openat(AT_FDCWD, path.c_str(), O_CREAT | O_TRUNC | O_WRONLY, 0666);
  if (fd < 0) {
    return false;
  }
  bool status = WriteFileDescriptor(fd, data, size);
  close(fd);
  return status;
}

}  // namespace dart_utils
