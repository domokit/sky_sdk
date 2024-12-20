// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FILESYSTEM_FILE_H_
#define FILESYSTEM_FILE_H_

#include <cstdint>
#include <string>
#include <vector>

#include "tonic/filesystem/filesystem/eintr_wrapper.h"
#include "tonic/filesystem/filesystem/portable_unistd.h"

namespace filesystem {

class Descriptor {
 public:
  using Handle = int;

  Descriptor(Handle handle) : handle_(handle) {}

  ~Descriptor() {
    if (is_valid()) {
      IGNORE_EINTR(::close(handle_));
    }
  }

  bool is_valid() { return handle_ >= 0; }

  Handle get() { return handle_; }

 private:
  Handle handle_ = -1;

  Descriptor(Descriptor&) = delete;

  void operator=(const Descriptor&) = delete;
};

// Reads the contents of the file at the given path or file descriptor and
// stores the data in result. Returns true if the file was read successfully,
// otherwise returns false. If this function returns false, |result| will be
// the empty string.
bool ReadFileToString(const std::string& path, std::string* result);
bool ReadFileDescriptorToString(int fd, std::string* result);

// Reads the contents of the file at the given path and if successful, returns
// pair of read allocated bytes with data and size of the data if successful.
// pair of <nullptr, -1> if read failed.
std::pair<uint8_t*, intptr_t> ReadFileToBytes(const std::string& path);
std::pair<uint8_t*, intptr_t> ReadFileDescriptorToBytes(int fd);

}  // namespace filesystem

#endif  // FILESYSTEM_FILE_H_
