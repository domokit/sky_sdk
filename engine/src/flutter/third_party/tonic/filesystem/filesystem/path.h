// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FILESYSTEM_PATH_H_
#define FILESYSTEM_PATH_H_

#include <string>

namespace filesystem {

// Resolves ".." and "." components of the path syntactically without consulting
// the file system.
std::string SimplifyPath(std::string path);

// Returns the absolute path of a possibly relative path.
// It doesn't consult the filesystem or simplify the path.
std::string AbsolutePath(const std::string& path);

// Returns the directory name component of the given path.
std::string GetDirectoryName(const std::string& path);

// Returns the basename component of the given path by stripping everything up
// to and including the last slash.
std::string GetBaseName(const std::string& path);

// Returns the real path for the given path by unwinding symbolic links and
// directory traversals.
std::string GetAbsoluteFilePath(const std::string& path);

}  // namespace filesystem

#endif  // FILESYSTEM_PATH_H_
