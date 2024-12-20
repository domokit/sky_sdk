// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FILESYSTEM_PORTABLE_UNISTD_H_
#define FILESYSTEM_PORTABLE_UNISTD_H_

#include "tonic/common/build_config.h"

#if defined(OS_WIN)
#include <direct.h>
#include <io.h>
#include <stdlib.h>

#define STDERR_FILENO _fileno(stderr)
#define PATH_MAX _MAX_PATH

#define S_ISDIR(m) (((m)&S_IFMT) == S_IFDIR)
#define S_ISREG(m) (((m)&S_IFMT) == S_IFREG)
#define R_OK 4

#define mkdir(path, mode) _mkdir(path)

#else
#include <unistd.h>
#endif

#endif  // FILESYSTEM_PORTABLE_UNISTD_H_
