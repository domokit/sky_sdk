// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/windows/flutter_project_bundle.h"
#include "gtest/gtest.h"

namespace flutter {
namespace testing {

TEST(FlutterProjectBundle, BasicPropertiesAbsolutePaths) {
  FlutterDesktopEngineProperties properties = {};
  properties.assets_path = L"C:\\foo\\flutter_assets";
  properties.icu_data_path = L"C:\\foo\\icudtl.dat";

  FlutterProjectBundle project(properties);

  EXPECT_TRUE(project.HasValidPaths());
  EXPECT_EQ(project.assets_path().string(), "C:\\foo\\flutter_assets");
  EXPECT_EQ(project.icu_path().string(), "C:\\foo\\icudtl.dat");
}

TEST(FlutterProjectBundle, BasicPropertiesRelativePaths) {
  FlutterDesktopEngineProperties properties = {};
  properties.assets_path = L"foo\\flutter_assets";
  properties.icu_data_path = L"foo\\icudtl.dat";

  FlutterProjectBundle project(properties);

  EXPECT_TRUE(project.HasValidPaths());
  EXPECT_TRUE(project.assets_path().is_absolute());
  EXPECT_EQ(project.assets_path().filename().string(), "flutter_assets");
  EXPECT_TRUE(project.icu_path().is_absolute());
  EXPECT_EQ(project.icu_path().filename().string(), "icudtl.dat");
}

TEST(FlutterProjectBundle, SwitchesEmpty) {
  FlutterDesktopEngineProperties properties = {};
  properties.assets_path = L"foo\\flutter_assets";
  properties.icu_data_path = L"foo\\icudtl.dat";

  // Clear the main environment variable, since test order is not guaranteed.
  _putenv_s("FLUTTER_ENGINE_SWITCHES", "");

  FlutterProjectBundle project(properties);

  EXPECT_EQ(project.GetSwitches().size(), 0);
}

TEST(FlutterProjectBundle, DartEntrypointArguments) {
  FlutterDesktopEngineProperties properties = {};
  properties.assets_path = L"foo\\flutter_assets";
  properties.icu_data_path = L"foo\\icudtl.dat";

  std::vector<const char*> test_arguments = {"arg1", "arg2"};
  properties.dart_entrypoint_argc = test_arguments.size();
  properties.dart_entrypoint_argv = test_arguments.data();

  FlutterProjectBundle project(properties);

  std::vector<std::string> retrieved_arguments =
      project.dart_entrypoint_arguments();
  EXPECT_EQ(retrieved_arguments.size(), 2U);
  EXPECT_EQ(retrieved_arguments[0], "arg1");
  EXPECT_EQ(retrieved_arguments[1], "arg2");
}

#ifndef FLUTTER_RELEASE
TEST(FlutterProjectBundle, Switches) {
  FlutterDesktopEngineProperties properties = {};
  properties.assets_path = L"foo\\flutter_assets";
  properties.icu_data_path = L"foo\\icudtl.dat";

  _putenv_s("FLUTTER_ENGINE_SWITCHES", "2");
  _putenv_s("FLUTTER_ENGINE_SWITCH_1", "abc");
  _putenv_s("FLUTTER_ENGINE_SWITCH_2", "foo=\"bar, baz\"");

  FlutterProjectBundle project(properties);

  std::vector<std::string> switches = project.GetSwitches();
  EXPECT_EQ(switches.size(), 2);
  EXPECT_EQ(switches[0], "--abc");
  EXPECT_EQ(switches[1], "--foo=\"bar, baz\"");
}
#endif  // !FLUTTER_RELEASE

}  // namespace testing
}  // namespace flutter
