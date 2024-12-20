// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <cstring>
#include <memory>
#include <vector>

#include "flutter/fml/build_config.h"
#include "flutter/fml/file.h"
#include "flutter/fml/mapping.h"
#include "flutter/fml/paths.h"
#include "flutter/fml/unique_fd.h"
#include "gtest/gtest.h"

static bool WriteStringToFile(const fml::UniqueFD& fd,
                              const std::string& contents) {
  if (!fml::TruncateFile(fd, contents.size())) {
    return false;
  }

  fml::FileMapping mapping(fd, {fml::FileMapping::Protection::kWrite});
  if (mapping.GetSize() != contents.size()) {
    return false;
  }

  if (mapping.GetMutableMapping() == nullptr) {
    return false;
  }

  ::memmove(mapping.GetMutableMapping(), contents.data(), contents.size());
  return true;
}

static std::string ReadStringFromFile(const fml::UniqueFD& fd) {
  fml::FileMapping mapping(fd);

  if (mapping.GetMapping() == nullptr) {
    return nullptr;
  }

  return {reinterpret_cast<const char*>(mapping.GetMapping()),
          mapping.GetSize()};
}

TEST(FileTest, CreateTemporaryAndUnlink) {
  auto dir_name = fml::CreateTemporaryDirectory();
  ASSERT_NE(dir_name, "");
  auto dir =
      fml::OpenDirectory(dir_name.c_str(), false, fml::FilePermission::kRead);
  ASSERT_TRUE(dir.is_valid());
  dir.reset();
  ASSERT_TRUE(fml::UnlinkDirectory(dir_name.c_str()));
}

TEST(FileTest, ScopedTempDirIsValid) {
  fml::ScopedTemporaryDirectory dir;
  ASSERT_TRUE(dir.fd().is_valid());
}

TEST(FileTest, CanOpenFileForWriting) {
  fml::ScopedTemporaryDirectory dir;
  ASSERT_TRUE(dir.fd().is_valid());

  auto fd =
      fml::OpenFile(dir.fd(), "some.txt", true, fml::FilePermission::kWrite);
  ASSERT_TRUE(fd.is_valid());
  fd.reset();
  ASSERT_TRUE(fml::UnlinkFile(dir.fd(), "some.txt"));
}

TEST(FileTest, CanTruncateAndWrite) {
  fml::ScopedTemporaryDirectory dir;
  ASSERT_TRUE(dir.fd().is_valid());

  std::string contents = "some contents here";

  // On the first iteration, this tests writing and then reading a file that
  // didn't exist yet. On the second iteration it tests truncating, writing,
  // and reading a file that already existed.
  for (int i = 0; i < 2; i++) {
    {
      auto fd = fml::OpenFile(dir.fd(), "some.txt", true,
                              fml::FilePermission::kReadWrite);
      ASSERT_TRUE(fd.is_valid());

      ASSERT_TRUE(fml::TruncateFile(fd, contents.size()));

      fml::FileMapping mapping(fd, {fml::FileMapping::Protection::kWrite});
      ASSERT_EQ(mapping.GetSize(), contents.size());
      ASSERT_NE(mapping.GetMutableMapping(), nullptr);

      ::memcpy(mapping.GetMutableMapping(), contents.data(), contents.size());
    }

    {
      auto fd = fml::OpenFile(dir.fd(), "some.txt", false,
                              fml::FilePermission::kRead);
      ASSERT_TRUE(fd.is_valid());

      fml::FileMapping mapping(fd);
      ASSERT_EQ(mapping.GetSize(), contents.size());

      ASSERT_EQ(
          0, ::memcmp(mapping.GetMapping(), contents.data(), contents.size()));
    }
  }

  fml::UnlinkFile(dir.fd(), "some.txt");
}

TEST(FileTest, CreateDirectoryStructure) {
  fml::ScopedTemporaryDirectory dir;

  std::string contents = "These are my contents";
  {
    auto sub = fml::CreateDirectory(dir.fd(), {"a", "b", "c"},
                                    fml::FilePermission::kReadWrite);
    ASSERT_TRUE(sub.is_valid());
    auto file = fml::OpenFile(sub, "my_contents", true,
                              fml::FilePermission::kReadWrite);
    ASSERT_TRUE(file.is_valid());
    ASSERT_TRUE(WriteStringToFile(file, contents));
  }

  const char* file_path = "a/b/c/my_contents";

  {
    auto contents_file =
        fml::OpenFile(dir.fd(), file_path, false, fml::FilePermission::kRead);
    ASSERT_EQ(ReadStringFromFile(contents_file), contents);
  }

  // Cleanup.
  ASSERT_TRUE(fml::UnlinkFile(dir.fd(), file_path));
  ASSERT_TRUE(fml::UnlinkDirectory(dir.fd(), "a/b/c"));
  ASSERT_TRUE(fml::UnlinkDirectory(dir.fd(), "a/b"));
  ASSERT_TRUE(fml::UnlinkDirectory(dir.fd(), "a"));
}

TEST(FileTest, VisitFilesCanBeCalledTwice) {
  fml::ScopedTemporaryDirectory dir;

  {
    auto file = fml::OpenFile(dir.fd(), "my_contents", true,
                              fml::FilePermission::kReadWrite);
    ASSERT_TRUE(file.is_valid());
  }

  int count;
  fml::FileVisitor count_visitor = [&count](const fml::UniqueFD& directory,
                                            const std::string& filename) {
    count += 1;
    return true;
  };
  count = 0;
  fml::VisitFiles(dir.fd(), count_visitor);
  ASSERT_EQ(count, 1);

  // Without `rewinddir` in `VisitFiles`, the following check would fail.
  count = 0;
  fml::VisitFiles(dir.fd(), count_visitor);
  ASSERT_EQ(count, 1);

  ASSERT_TRUE(fml::UnlinkFile(dir.fd(), "my_contents"));
}

TEST(FileTest, CanListFilesRecursively) {
  fml::ScopedTemporaryDirectory dir;

  {
    auto c = fml::CreateDirectory(dir.fd(), {"a", "b", "c"},
                                  fml::FilePermission::kReadWrite);
    ASSERT_TRUE(c.is_valid());
    auto file1 =
        fml::OpenFile(c, "file1", true, fml::FilePermission::kReadWrite);
    auto file2 =
        fml::OpenFile(c, "file2", true, fml::FilePermission::kReadWrite);
    auto d = fml::CreateDirectory(c, {"d"}, fml::FilePermission::kReadWrite);
    ASSERT_TRUE(d.is_valid());
    auto file3 =
        fml::OpenFile(d, "file3", true, fml::FilePermission::kReadWrite);
    ASSERT_TRUE(file1.is_valid());
    ASSERT_TRUE(file2.is_valid());
    ASSERT_TRUE(file3.is_valid());
  }

  std::set<std::string> names;
  fml::FileVisitor visitor = [&names](const fml::UniqueFD& directory,
                                      const std::string& filename) {
    names.insert(filename);
    return true;
  };

  fml::VisitFilesRecursively(dir.fd(), visitor);
  ASSERT_EQ(names, std::set<std::string>(
                       {"a", "b", "c", "d", "file1", "file2", "file3"}));

  // Cleanup.
  ASSERT_TRUE(fml::UnlinkFile(dir.fd(), "a/b/c/d/file3"));
  ASSERT_TRUE(fml::UnlinkFile(dir.fd(), "a/b/c/file1"));
  ASSERT_TRUE(fml::UnlinkFile(dir.fd(), "a/b/c/file2"));
  ASSERT_TRUE(fml::UnlinkDirectory(dir.fd(), "a/b/c/d"));
  ASSERT_TRUE(fml::UnlinkDirectory(dir.fd(), "a/b/c"));
  ASSERT_TRUE(fml::UnlinkDirectory(dir.fd(), "a/b"));
  ASSERT_TRUE(fml::UnlinkDirectory(dir.fd(), "a"));
}

TEST(FileTest, CanStopVisitEarly) {
  fml::ScopedTemporaryDirectory dir;

  {
    auto d = fml::CreateDirectory(dir.fd(), {"a", "b", "c", "d"},
                                  fml::FilePermission::kReadWrite);
    ASSERT_TRUE(d.is_valid());
  }

  std::set<std::string> names;
  fml::FileVisitor visitor = [&names](const fml::UniqueFD& directory,
                                      const std::string& filename) {
    names.insert(filename);
    return filename == "c" ? false : true;  // stop if c is found
  };

  // Check the d is not visited as we stop at c.
  ASSERT_FALSE(fml::VisitFilesRecursively(dir.fd(), visitor));
  ASSERT_EQ(names, std::set<std::string>({"a", "b", "c"}));

  // Cleanup.
  ASSERT_TRUE(fml::UnlinkDirectory(dir.fd(), "a/b/c/d"));
  ASSERT_TRUE(fml::UnlinkDirectory(dir.fd(), "a/b/c"));
  ASSERT_TRUE(fml::UnlinkDirectory(dir.fd(), "a/b"));
  ASSERT_TRUE(fml::UnlinkDirectory(dir.fd(), "a"));
}

TEST(FileTest, AtomicWriteTest) {
  fml::ScopedTemporaryDirectory dir;

  const std::string contents = "These are my contents.";

  auto data = std::make_unique<fml::DataMapping>(
      std::vector<uint8_t>{contents.begin(), contents.end()});

  // Write.
  ASSERT_TRUE(fml::WriteAtomically(dir.fd(), "precious_data", *data));

  // Read and verify.
  ASSERT_EQ(contents,
            ReadStringFromFile(fml::OpenFile(dir.fd(), "precious_data", false,
                                             fml::FilePermission::kRead)));

  // Cleanup.
  ASSERT_TRUE(fml::UnlinkFile(dir.fd(), "precious_data"));
}

TEST(FileTest, IgnoreBaseDirWhenPathIsAbsolute) {
  fml::ScopedTemporaryDirectory dir;

  // Make an absolute path.
  std::string filename = "filename.txt";
  std::string full_path =
      fml::paths::AbsolutePath(fml::paths::JoinPaths({dir.path(), filename}));

  const std::string contents = "These are my contents.";
  auto data = std::make_unique<fml::DataMapping>(
      std::vector<uint8_t>{contents.begin(), contents.end()});

  // Write.
  ASSERT_TRUE(fml::WriteAtomically(dir.fd(), full_path.c_str(), *data));

  // Test existence.
  ASSERT_TRUE(fml::FileExists(dir.fd(), full_path.c_str()));

  // Read and verify.
  ASSERT_EQ(contents,
            ReadStringFromFile(fml::OpenFile(dir.fd(), full_path.c_str(), false,
                                             fml::FilePermission::kRead)));

  // Cleanup.
  ASSERT_TRUE(fml::UnlinkFile(dir.fd(), full_path.c_str()));
}

TEST(FileTest, EmptyMappingTest) {
  fml::ScopedTemporaryDirectory dir;

  {
    auto file = fml::OpenFile(dir.fd(), "my_contents", true,
                              fml::FilePermission::kReadWrite);

    fml::FileMapping mapping(file);
    ASSERT_TRUE(mapping.IsValid());
    ASSERT_EQ(mapping.GetSize(), 0ul);
    ASSERT_EQ(mapping.GetMapping(), nullptr);
  }

  ASSERT_TRUE(fml::UnlinkFile(dir.fd(), "my_contents"));
}

TEST(FileTest, MappingDontNeedSafeTest) {
  fml::ScopedTemporaryDirectory dir;

  {
    auto file = fml::OpenFile(dir.fd(), "my_contents", true,
                              fml::FilePermission::kReadWrite);
    WriteStringToFile(file, "some content");
  }

  {
    auto file = fml::OpenFile(dir.fd(), "my_contents", false,
                              fml::FilePermission::kRead);
    fml::FileMapping mapping(file);
    ASSERT_TRUE(mapping.IsValid());
    ASSERT_EQ(mapping.GetMutableMapping(), nullptr);
    ASSERT_TRUE(mapping.IsDontNeedSafe());
  }

  {
    auto file = fml::OpenFile(dir.fd(), "my_contents", false,
                              fml::FilePermission::kReadWrite);
    fml::FileMapping mapping(file, {fml::FileMapping::Protection::kRead,
                                    fml::FileMapping::Protection::kWrite});
    ASSERT_TRUE(mapping.IsValid());
    ASSERT_NE(mapping.GetMutableMapping(), nullptr);
    ASSERT_FALSE(mapping.IsDontNeedSafe());
  }
  ASSERT_TRUE(fml::UnlinkFile(dir.fd(), "my_contents"));
}

TEST(FileTest, FileTestsWork) {
  fml::ScopedTemporaryDirectory dir;
  ASSERT_TRUE(dir.fd().is_valid());
  const char* filename = "some.txt";
  auto fd =
      fml::OpenFile(dir.fd(), filename, true, fml::FilePermission::kWrite);
  ASSERT_TRUE(fd.is_valid());
  fd.reset();
  ASSERT_TRUE(fml::FileExists(dir.fd(), filename));
  ASSERT_TRUE(
      fml::IsFile(fml::paths::JoinPaths({dir.path(), filename}).c_str()));
  ASSERT_TRUE(fml::UnlinkFile(dir.fd(), filename));
}

TEST(FileTest, FileTestsSupportsUnicode) {
  fml::ScopedTemporaryDirectory dir;
  ASSERT_TRUE(dir.fd().is_valid());
  const char* filename = u8"äëïöüテスト☃";
  auto fd =
      fml::OpenFile(dir.fd(), filename, true, fml::FilePermission::kWrite);
  ASSERT_TRUE(fd.is_valid());
  fd.reset();
  ASSERT_TRUE(fml::FileExists(dir.fd(), filename));
  ASSERT_TRUE(
      fml::IsFile(fml::paths::JoinPaths({dir.path(), filename}).c_str()));
  ASSERT_TRUE(fml::UnlinkFile(dir.fd(), filename));
}
