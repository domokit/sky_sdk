// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/fml/file.h"

#include <Fileapi.h>

#include "flutter/fml/platform/win/wstring_conversion.h"
#include "gtest/gtest.h"

namespace fml {
namespace testing {

TEST(FileWin, OpenDirectoryShare) {
  fml::ScopedTemporaryDirectory temp_dir;
  auto temp_path = Utf8ToWideString({temp_dir.path()});
  fml::UniqueFD handle(
      CreateFile(temp_path.c_str(), GENERIC_WRITE,
                 FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr, OPEN_EXISTING,
                 FILE_ATTRIBUTE_NORMAL | FILE_FLAG_BACKUP_SEMANTICS, nullptr));
  ASSERT_TRUE(handle.is_valid());

  // Check that fml::OpenDirectory(FilePermission::kRead) works with a
  // directory that has also been opened for writing.
  auto dir = fml::OpenDirectory(temp_dir.path().c_str(), false,
                                fml::FilePermission::kRead);
  ASSERT_TRUE(dir.is_valid());
}

}  // namespace testing
}  // namespace fml