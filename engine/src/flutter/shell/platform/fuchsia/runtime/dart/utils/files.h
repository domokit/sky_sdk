// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_FUCHSIA_RUNTIME_DART_UTILS_FILES_H_
#define FLUTTER_SHELL_PLATFORM_FUCHSIA_RUNTIME_DART_UTILS_FILES_H_

#include <string>

namespace dart_utils {

// Reads the contents of the file at the given path or file descriptor and
// stores the data in result. Returns true if the file was read successfully,
// otherwise returns false. If this function returns false, |result| will be
// the empty string.
bool ReadFileToString(const std::string& path, std::string* result);
bool ReadFileToStringAt(int dirfd,
                        const std::string& path,
                        std::string* result);

// Writes the given data to the file at the given path. Returns true if the data
// was successfully written, otherwise returns false.
bool WriteFile(const std::string& path, const char* data, ssize_t size);

}  // namespace dart_utils

#endif  // FLUTTER_SHELL_PLATFORM_FUCHSIA_RUNTIME_DART_UTILS_FILES_H_
