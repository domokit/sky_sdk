// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.
#ifndef FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_FILE_IN_NAMESPACE_BUFFER_H_
#define FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_FILE_IN_NAMESPACE_BUFFER_H_

#include "flutter/fml/mapping.h"

namespace flutter_runner {

/// A mapping to a buffer for a file that has been loaded into a namespace.
class FileInNamespaceBuffer final : public fml::Mapping {
 public:
  /// Loads the file at |path| into the namespace |namespace_fd|, creating
  /// a mapping to the loaded buffer.
  ///
  /// The file will be loaded with the readable permission. If |executable| is
  /// true, the file will also be loaded with the executable permission.
  FileInNamespaceBuffer(int namespace_fd, const char* path, bool executable);
  ~FileInNamespaceBuffer();

  // |fml::Mapping|
  const uint8_t* GetMapping() const override;

  // |fml::Mapping|
  size_t GetSize() const override;

  // |fml::Mapping|
  bool IsDontNeedSafe() const override;

 private:
  /// The address that was mapped to the buffer.
  void* address_;

  /// The size of the buffer.
  size_t size_;

  FML_DISALLOW_COPY_AND_ASSIGN(FileInNamespaceBuffer);
};

/// Loads a file from |file_path| into the namespace |namespace_fd|, returning
/// the mapping for that file.
///
/// The file will be loaded with the readable permission. If |executable| is
/// true, the file will be also be loaded with the executable permission.
std::unique_ptr<fml::Mapping> LoadFile(int namespace_fd,
                                       const char* path,
                                       bool executable);

/// Opens the file at |path| and creates a file mapping for the file.
///
/// The file will be opened with the readable permission. If |executable| is
/// true, the file will also be opened with the executable permission.
std::unique_ptr<fml::FileMapping> MakeFileMapping(const char* path,
                                                  bool executable);

}  // namespace flutter_runner

#endif  // FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_FILE_IN_NAMESPACE_BUFFER_H_
