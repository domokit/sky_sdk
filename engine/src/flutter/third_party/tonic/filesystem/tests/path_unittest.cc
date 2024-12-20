// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "filesystem/path.h"
#include "filesystem/directory.h"
#include "filesystem/scoped_temp_dir.h"
#include "gtest/gtest.h"
#include "tonic/common/build_config.h"

namespace filesystem {

void ExpectPlatformPath(std::string expected, std::string actual) {
#if defined(OS_WIN)
  std::replace(expected.begin(), expected.end(), '/', '\\');
#endif
  EXPECT_EQ(expected, actual);
}

TEST(Path, SimplifyPath) {
  ExpectPlatformPath(".", SimplifyPath(""));
  ExpectPlatformPath(".", SimplifyPath("."));
  ExpectPlatformPath("..", SimplifyPath(".."));
  ExpectPlatformPath("...", SimplifyPath("..."));

  ExpectPlatformPath("/", SimplifyPath("/"));
  ExpectPlatformPath("/", SimplifyPath("/."));
  ExpectPlatformPath("/", SimplifyPath("/.."));
  ExpectPlatformPath("/...", SimplifyPath("/..."));

  ExpectPlatformPath("foo", SimplifyPath("foo"));
  ExpectPlatformPath("foo", SimplifyPath("foo/"));
  ExpectPlatformPath("foo", SimplifyPath("foo/."));
  ExpectPlatformPath("foo", SimplifyPath("foo/./"));
  ExpectPlatformPath(".", SimplifyPath("foo/.."));
  ExpectPlatformPath(".", SimplifyPath("foo/../"));
  ExpectPlatformPath("foo/...", SimplifyPath("foo/..."));
  ExpectPlatformPath("foo/...", SimplifyPath("foo/.../"));
  ExpectPlatformPath("foo/.b", SimplifyPath("foo/.b"));
  ExpectPlatformPath("foo/.b", SimplifyPath("foo/.b/"));

  ExpectPlatformPath("/foo", SimplifyPath("/foo"));
  ExpectPlatformPath("/foo", SimplifyPath("/foo/"));
  ExpectPlatformPath("/foo", SimplifyPath("/foo/."));
  ExpectPlatformPath("/foo", SimplifyPath("/foo/./"));
  ExpectPlatformPath("/", SimplifyPath("/foo/.."));
  ExpectPlatformPath("/", SimplifyPath("/foo/../"));
  ExpectPlatformPath("/foo/...", SimplifyPath("/foo/..."));
  ExpectPlatformPath("/foo/...", SimplifyPath("/foo/.../"));
  ExpectPlatformPath("/foo/.b", SimplifyPath("/foo/.b"));
  ExpectPlatformPath("/foo/.b", SimplifyPath("/foo/.b/"));

  ExpectPlatformPath("foo/bar", SimplifyPath("foo/bar"));
  ExpectPlatformPath("foo/bar", SimplifyPath("foo/bar/"));
  ExpectPlatformPath("foo/bar", SimplifyPath("foo/./bar"));
  ExpectPlatformPath("foo/bar", SimplifyPath("foo/./bar/"));
  ExpectPlatformPath("bar", SimplifyPath("foo/../bar"));
  ExpectPlatformPath("bar", SimplifyPath("foo/baz/../../bar"));
  ExpectPlatformPath("bar", SimplifyPath("foo/../bar/"));
  ExpectPlatformPath("foo/.../bar", SimplifyPath("foo/.../bar"));
  ExpectPlatformPath("foo/.../bar", SimplifyPath("foo/.../bar/"));
  ExpectPlatformPath("foo/.b/bar", SimplifyPath("foo/.b/bar"));
  ExpectPlatformPath("foo/.b/bar", SimplifyPath("foo/.b/bar/"));

  ExpectPlatformPath("/foo/bar", SimplifyPath("/foo/bar"));
  ExpectPlatformPath("/foo/bar", SimplifyPath("/foo/bar/"));
  ExpectPlatformPath("/foo/bar", SimplifyPath("/foo/./bar"));
  ExpectPlatformPath("/foo/bar", SimplifyPath("/foo/./bar/"));
  ExpectPlatformPath("/bar", SimplifyPath("/foo/../bar"));
  ExpectPlatformPath("/bar", SimplifyPath("/foo/../bar/"));
  ExpectPlatformPath("/foo/.../bar", SimplifyPath("/foo/.../bar"));
  ExpectPlatformPath("/foo/.../bar", SimplifyPath("/foo/.../bar/"));
  ExpectPlatformPath("/foo/.b/bar", SimplifyPath("/foo/.b/bar"));
  ExpectPlatformPath("/foo/.b/bar", SimplifyPath("/foo/.b/bar/"));

  ExpectPlatformPath("../foo", SimplifyPath("../foo"));
  ExpectPlatformPath("../../bar", SimplifyPath("../foo/../../bar"));
  ExpectPlatformPath("/bar", SimplifyPath("/foo/../../bar"));

  // Already clean
  ExpectPlatformPath(".", SimplifyPath(""));
  ExpectPlatformPath("abc", SimplifyPath("abc"));
  ExpectPlatformPath("abc/def", SimplifyPath("abc/def"));
  ExpectPlatformPath("a/b/c", SimplifyPath("a/b/c"));
  ExpectPlatformPath(".", SimplifyPath("."));
  ExpectPlatformPath("..", SimplifyPath(".."));
  ExpectPlatformPath("../..", SimplifyPath("../.."));
  ExpectPlatformPath("../../abc", SimplifyPath("../../abc"));
  ExpectPlatformPath("/abc", SimplifyPath("/abc"));
  ExpectPlatformPath("/", SimplifyPath("/"));

  // Remove trailing slash
  ExpectPlatformPath("abc", SimplifyPath("abc/"));
  ExpectPlatformPath("abc/def", SimplifyPath("abc/def/"));
  ExpectPlatformPath("a/b/c", SimplifyPath("a/b/c/"));
  ExpectPlatformPath(".", SimplifyPath("./"));
  ExpectPlatformPath("..", SimplifyPath("../"));
  ExpectPlatformPath("../..", SimplifyPath("../../"));
  ExpectPlatformPath("/abc", SimplifyPath("/abc/"));

  // Remove doubled slash
  ExpectPlatformPath("abc/def/ghi", SimplifyPath("abc//def//ghi"));
  ExpectPlatformPath("/abc", SimplifyPath("//abc"));
  ExpectPlatformPath("/abc", SimplifyPath("///abc"));
  ExpectPlatformPath("/abc", SimplifyPath("//abc//"));
  ExpectPlatformPath("abc", SimplifyPath("abc//"));

  // Remove . elements
  ExpectPlatformPath("abc/def", SimplifyPath("abc/./def"));
  ExpectPlatformPath("/abc/def", SimplifyPath("/./abc/def"));
  ExpectPlatformPath("abc", SimplifyPath("abc/."));

  // Remove .. elements
  ExpectPlatformPath("abc/def/jkl", SimplifyPath("abc/def/ghi/../jkl"));
  ExpectPlatformPath("abc/jkl", SimplifyPath("abc/def/../ghi/../jkl"));
  ExpectPlatformPath("abc", SimplifyPath("abc/def/.."));
  ExpectPlatformPath(".", SimplifyPath("abc/def/../.."));
  ExpectPlatformPath("/", SimplifyPath("/abc/def/../.."));
  ExpectPlatformPath("..", SimplifyPath("abc/def/../../.."));
  ExpectPlatformPath("/", SimplifyPath("/abc/def/../../.."));
  ExpectPlatformPath("../../mno",
                     SimplifyPath("abc/def/../../../ghi/jkl/../../../mno"));
  ExpectPlatformPath("/mno", SimplifyPath("/../mno"));

  // Combinations
  ExpectPlatformPath("def", SimplifyPath("abc/./../def"));
  ExpectPlatformPath("def", SimplifyPath("abc//./../def"));
  ExpectPlatformPath("../../def", SimplifyPath("abc/../../././../def"));

#if defined(OS_WIN)
  ExpectPlatformPath("a\\c", SimplifyPath("a\\b\\..\\c"));
  ExpectPlatformPath("X:\\a\\c", SimplifyPath("X:/a/b/../c"));
  ExpectPlatformPath("X:\\a\\b\\c", SimplifyPath("X:/a/b/./c"));
  ExpectPlatformPath("X:\\c", SimplifyPath("X:/../../c"));
#endif
}

TEST(Path, AbsolutePath) {
#if defined(OS_WIN)
  // We cut out the drive letter as it can be different on every system.
  EXPECT_EQ(":\\foo\\bar", AbsolutePath("\\foo\\bar").substr(1));
  EXPECT_EQ(":\\foo\\bar", AbsolutePath("/foo/bar").substr(1));
  EXPECT_EQ(":\\foo\\bar\\", AbsolutePath("\\foo\\bar\\").substr(1));
  EXPECT_EQ(":\\foo\\bar\\", AbsolutePath("/foo/bar/").substr(1));
  EXPECT_EQ("C:\\foo\\bar\\", AbsolutePath("C:\\foo\\bar\\"));
  EXPECT_EQ(GetCurrentDirectory() + "\\foo", AbsolutePath("foo"));
#else
  EXPECT_EQ("/foo/bar", AbsolutePath("/foo/bar"));
  EXPECT_EQ("/foo/bar/", AbsolutePath("/foo/bar/"));
  EXPECT_EQ(GetCurrentDirectory() + "/foo", AbsolutePath("foo"));
#endif
  EXPECT_EQ(GetCurrentDirectory(), AbsolutePath(""));
}

TEST(Path, GetDirectoryName) {
  EXPECT_EQ("foo", GetDirectoryName("foo/"));
  EXPECT_EQ("foo/bar", GetDirectoryName("foo/bar/"));
  EXPECT_EQ("foo", GetDirectoryName("foo/bar"));
  EXPECT_EQ("foo/bar", GetDirectoryName("foo/bar/.."));
  EXPECT_EQ("foo/bar/..", GetDirectoryName("foo/bar/../.."));
  EXPECT_EQ("", GetDirectoryName("foo"));
  EXPECT_EQ("/", GetDirectoryName("/"));
  EXPECT_EQ("", GetDirectoryName("a"));
  EXPECT_EQ("/", GetDirectoryName("/a"));
  EXPECT_EQ("/a", GetDirectoryName("/a/"));
  EXPECT_EQ("a", GetDirectoryName("a/"));
#if defined(OS_WIN)
  EXPECT_EQ("C:\\", GetDirectoryName("C:\\"));
  EXPECT_EQ("C:\\foo", GetDirectoryName("C:\\foo\\"));
  EXPECT_EQ("C:\\foo", GetDirectoryName("C:\\foo\\bar"));
  EXPECT_EQ("foo\\bar", GetDirectoryName("foo\\bar\\"));
  EXPECT_EQ("foo", GetDirectoryName("foo\\bar"));
  EXPECT_EQ("\\", GetDirectoryName("\\"));
  EXPECT_EQ("\\", GetDirectoryName("\\a"));
#endif
}

TEST(Path, GetBaseName) {
  EXPECT_EQ("", GetBaseName("foo/"));
  EXPECT_EQ("", GetBaseName("foo/bar/"));
  EXPECT_EQ("bar", GetBaseName("foo/bar"));
  EXPECT_EQ("..", GetBaseName("foo/bar/.."));
  EXPECT_EQ("..", GetBaseName("foo/bar/../.."));
  EXPECT_EQ("foo", GetBaseName("foo"));
  EXPECT_EQ("", GetBaseName("/"));
  EXPECT_EQ("a", GetBaseName("a"));
  EXPECT_EQ("a", GetBaseName("/a"));
  EXPECT_EQ("", GetBaseName("/a/"));
  EXPECT_EQ("", GetBaseName("a/"));
#if defined(OS_WIN)
  EXPECT_EQ("", GetBaseName("C:\\"));
  EXPECT_EQ("", GetBaseName("C:\\foo\\"));
  EXPECT_EQ("bar", GetBaseName("C:\\foo\\bar"));
  EXPECT_EQ("", GetBaseName("foo\\bar\\"));
  EXPECT_EQ("bar", GetBaseName("foo\\bar"));
  EXPECT_EQ("", GetBaseName("\\"));
  EXPECT_EQ("a", GetBaseName("\\a"));
#endif
}

TEST(Path, DeletePath) {
  ScopedTempDir dir;

  std::string sub_dir = dir.path() + "/dir";
  CreateDirectory(sub_dir);
  EXPECT_TRUE(IsDirectory(sub_dir));
  EXPECT_TRUE(DeletePath(sub_dir, false));
  EXPECT_FALSE(IsDirectory(sub_dir));
}

TEST(Path, DeletePathRecursively) {
  ScopedTempDir dir;

  std::string sub_dir = dir.path() + "/dir";
  CreateDirectory(sub_dir);
  EXPECT_TRUE(IsDirectory(sub_dir));

  std::string sub_sub_dir1 = sub_dir + "/dir1";
  CreateDirectory(sub_sub_dir1);
  EXPECT_TRUE(IsDirectory(sub_sub_dir1));
  std::string sub_sub_dir2 = sub_dir + "/dir2";
  CreateDirectory(sub_sub_dir2);
  EXPECT_TRUE(IsDirectory(sub_sub_dir2));

  EXPECT_FALSE(DeletePath(sub_dir, false));
  EXPECT_TRUE(IsDirectory(sub_dir));
  EXPECT_TRUE(IsDirectory(sub_sub_dir1));

  EXPECT_TRUE(DeletePath(sub_dir, true));
  EXPECT_FALSE(IsDirectory(sub_dir));
  EXPECT_FALSE(IsDirectory(sub_sub_dir1));
}

}  // namespace filesystem
