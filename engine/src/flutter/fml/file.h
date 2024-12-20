// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FML_FILE_H_
#define FLUTTER_FML_FILE_H_

#include <functional>
#include <initializer_list>
#include <string>
#include <vector>

#include "flutter/fml/macros.h"
#include "flutter/fml/unique_fd.h"

#ifdef ERROR
#undef ERROR
#endif

namespace fml {

class Mapping;

enum class FilePermission {
  kRead,
  kWrite,
  kReadWrite,
};

std::string CreateTemporaryDirectory();

/// This can open a directory on POSIX, but not on Windows.
fml::UniqueFD OpenFile(const char* path,
                       bool create_if_necessary,
                       FilePermission permission);

/// This can open a directory on POSIX, but not on Windows.
fml::UniqueFD OpenFile(const fml::UniqueFD& base_directory,
                       const char* path,
                       bool create_if_necessary,
                       FilePermission permission);

/// Helper method that calls `OpenFile` with create_if_necessary = false
/// and permission = kRead.
///
/// This can open a directory on POSIX, but not on Windows.
fml::UniqueFD OpenFileReadOnly(const fml::UniqueFD& base_directory,
                               const char* path);

fml::UniqueFD OpenDirectory(const char* path,
                            bool create_if_necessary,
                            FilePermission permission);

fml::UniqueFD OpenDirectory(const fml::UniqueFD& base_directory,
                            const char* path,
                            bool create_if_necessary,
                            FilePermission permission);

/// Helper method that calls `OpenDirectory` with create_if_necessary = false
/// and permission = kRead.
fml::UniqueFD OpenDirectoryReadOnly(const fml::UniqueFD& base_directory,
                                    const char* path);

fml::UniqueFD Duplicate(fml::UniqueFD::element_type descriptor);

bool IsDirectory(const fml::UniqueFD& directory);

bool IsDirectory(const fml::UniqueFD& base_directory, const char* path);

// Returns whether the given path is a file.
bool IsFile(const std::string& path);

bool TruncateFile(const fml::UniqueFD& file, size_t size);

bool FileExists(const fml::UniqueFD& base_directory, const char* path);

bool UnlinkDirectory(const char* path);

bool UnlinkDirectory(const fml::UniqueFD& base_directory, const char* path);

bool UnlinkFile(const char* path);

bool UnlinkFile(const fml::UniqueFD& base_directory, const char* path);

fml::UniqueFD CreateDirectory(const fml::UniqueFD& base_directory,
                              const std::vector<std::string>& components,
                              FilePermission permission);

bool WriteAtomically(const fml::UniqueFD& base_directory,
                     const char* file_name,
                     const Mapping& mapping);

/// Signature of a callback on a file in `directory` with `filename` (relative
/// to `directory`). The returned bool should be false if and only if further
/// traversal should be stopped. For example, a file-search visitor may return
/// false when the file is found so no more visiting is needed.
using FileVisitor = std::function<bool(const fml::UniqueFD& directory,
                                       const std::string& filename)>;

/// Call `visitor` on all files inside the `directory` non-recursively. The
/// trivial file "." and ".." will not be visited.
///
/// Return false if and only if the visitor returns false during the
/// traversal.
///
/// If recursive visiting is needed, call `VisitFiles` inside the `visitor`, or
/// use our helper method `VisitFilesRecursively`.
///
/// @see `VisitFilesRecursively`.
/// @note Procedure doesn't copy all closures.
bool VisitFiles(const fml::UniqueFD& directory, const FileVisitor& visitor);

/// Recursively call `visitor` on all files inside the `directory`. Return false
/// if and only if the visitor returns false during the traversal.
///
/// This is a helper method that wraps the general `VisitFiles` method. The
/// `VisitFiles` is strictly more powerful as it has the access of the recursion
/// stack to the file. For example, `VisitFiles` may be able to maintain a
/// vector of directory names that lead to a file. That could be useful to
/// compute the relative path between the root directory and the visited file.
///
/// @see `VisitFiles`.
/// @note Procedure doesn't copy all closures.
bool VisitFilesRecursively(const fml::UniqueFD& directory,
                           const FileVisitor& visitor);

/// Helper method to recursively remove files and subdirectories inside the
/// directory. The directory itself will not be removed.
///
/// Return true if and only if all files have been successfully removed.
bool RemoveFilesInDirectory(const fml::UniqueFD& directory);

/// Helper method to recursively remove files and subdirectories inside the
/// directory. The directory itself will also be removed.
///
/// Return true if and only if all files have been successfully removed.
bool RemoveDirectoryRecursively(const fml::UniqueFD& parent,
                                const char* directory_name);

class ScopedTemporaryDirectory {
 public:
  ScopedTemporaryDirectory();

  ~ScopedTemporaryDirectory();

  const std::string& path() const { return path_; }
  const UniqueFD& fd() { return dir_fd_; }

 private:
  std::string path_;
  UniqueFD dir_fd_;
};

}  // namespace fml

#endif  // FLUTTER_FML_FILE_H_
