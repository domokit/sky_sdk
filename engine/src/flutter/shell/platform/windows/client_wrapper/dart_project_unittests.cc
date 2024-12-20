// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <memory>
#include <string>

#include "flutter/shell/platform/windows/client_wrapper/include/flutter/dart_project.h"
#include "gtest/gtest.h"

namespace flutter {

class DartProjectTest : public ::testing::Test {
 protected:
  // Wrapper for accessing private icu_data_path.
  std::wstring GetProjectIcuDataPath(const DartProject& project) {
    return project.icu_data_path();
  }

  // Wrapper for accessing private assets_path.
  std::wstring GetProjectAssetsPath(const DartProject& project) {
    return project.assets_path();
  }

  // Wrapper for accessing private aot_library_path_.
  std::wstring GetProjectAotLibraryPath(const DartProject& project) {
    return project.aot_library_path();
  }
};

TEST_F(DartProjectTest, StandardProjectFormat) {
  DartProject project(L"test");
  EXPECT_EQ(GetProjectIcuDataPath(project), L"test\\icudtl.dat");
  EXPECT_EQ(GetProjectAssetsPath(project), L"test\\flutter_assets");
  EXPECT_EQ(GetProjectAotLibraryPath(project), L"test\\app.so");
}

TEST_F(DartProjectTest, ProjectWithCustomPaths) {
  DartProject project(L"data\\assets", L"icu\\icudtl.dat", L"lib\\file.so");
  EXPECT_EQ(GetProjectIcuDataPath(project), L"icu\\icudtl.dat");
  EXPECT_EQ(GetProjectAssetsPath(project), L"data\\assets");
  EXPECT_EQ(GetProjectAotLibraryPath(project), L"lib\\file.so");
}

TEST_F(DartProjectTest, DartEntrypointArguments) {
  DartProject project(L"test");

  std::vector<std::string> test_arguments = {"arg1", "arg2", "arg3"};
  project.set_dart_entrypoint_arguments(test_arguments);

  auto returned_arguments = project.dart_entrypoint_arguments();
  EXPECT_EQ(returned_arguments.size(), 3U);
  EXPECT_EQ(returned_arguments[0], "arg1");
  EXPECT_EQ(returned_arguments[1], "arg2");
  EXPECT_EQ(returned_arguments[2], "arg3");
}

}  // namespace flutter
