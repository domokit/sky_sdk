// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "filesystem/directory.h"
#include "filesystem/path.h"
#include "filesystem/scoped_temp_dir.h"
#include "gtest/gtest.h"

namespace filesystem {

TEST(Directory, CreateDirectory) {
  std::string cwd = GetCurrentDirectory();

  ScopedTempDir dir;
  EXPECT_TRUE(IsDirectory(dir.path()));
  EXPECT_EQ(0, chdir(dir.path().c_str()));

  EXPECT_TRUE(CreateDirectory("foo/bar"));
  EXPECT_TRUE(IsDirectory("foo"));
  EXPECT_TRUE(IsDirectory("foo/bar"));
  EXPECT_FALSE(IsDirectory("foo/bar/baz"));

  EXPECT_TRUE(CreateDirectory("foo/bar/baz"));
  EXPECT_TRUE(IsDirectory("foo/bar/baz"));

  EXPECT_TRUE(CreateDirectory("qux"));
  EXPECT_TRUE(IsDirectory("qux"));

  EXPECT_EQ(0, chdir(cwd.c_str()));

  std::string abs_path = dir.path() + "/another/one";
  EXPECT_TRUE(CreateDirectory(abs_path));
  EXPECT_TRUE(IsDirectory(abs_path));
}

}  // namespace filesystem
