// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/common/engine_switches.h"

#include "gtest/gtest.h"

namespace flutter {

namespace {
// Sets |key=value| in the environment of this process.
void SetEnvironmentVariable(const char* key, const char* value) {
#ifdef _WIN32
  _putenv_s(key, value);
#else
  setenv(key, value, 1);
#endif
}

// Removes |key| from the environment of this process, if present.
void ClearEnvironmentVariable(const char* key) {
#ifdef _WIN32
  _putenv_s(key, "");
#else
  unsetenv(key);
#endif
}
}  // namespace

TEST(FlutterProjectBundle, SwitchesEmpty) {
  // Clear the main environment variable, since test order is not guaranteed.
  ClearEnvironmentVariable("FLUTTER_ENGINE_SWITCHES");

  EXPECT_EQ(GetSwitchesFromEnvironment().size(), 0U);
}

#ifdef FLUTTER_RELEASE
TEST(FlutterProjectBundle, SwitchesIgnoredInRelease) {
  SetEnvironmentVariable("FLUTTER_ENGINE_SWITCHES", "2");
  SetEnvironmentVariable("FLUTTER_ENGINE_SWITCH_1", "abc");
  SetEnvironmentVariable("FLUTTER_ENGINE_SWITCH_2", "foo=\"bar, baz\"");

  std::vector<std::string> switches = GetSwitchesFromEnvironment();
  EXPECT_EQ(switches.size(), 0U);
}
#endif  // FLUTTER_RELEASE

#ifndef FLUTTER_RELEASE
TEST(FlutterProjectBundle, Switches) {
  SetEnvironmentVariable("FLUTTER_ENGINE_SWITCHES", "2");
  SetEnvironmentVariable("FLUTTER_ENGINE_SWITCH_1", "abc");
  SetEnvironmentVariable("FLUTTER_ENGINE_SWITCH_2", "foo=\"bar, baz\"");

  std::vector<std::string> switches = GetSwitchesFromEnvironment();
  EXPECT_EQ(switches.size(), 2U);
  EXPECT_EQ(switches[0], "--abc");
  EXPECT_EQ(switches[1], "--foo=\"bar, baz\"");
}

TEST(FlutterProjectBundle, SwitchesExtraValues) {
  SetEnvironmentVariable("FLUTTER_ENGINE_SWITCHES", "1");
  SetEnvironmentVariable("FLUTTER_ENGINE_SWITCH_1", "abc");
  SetEnvironmentVariable("FLUTTER_ENGINE_SWITCH_2", "foo=\"bar, baz\"");

  std::vector<std::string> switches = GetSwitchesFromEnvironment();
  EXPECT_EQ(switches.size(), 1U);
  EXPECT_EQ(switches[0], "--abc");
}

TEST(FlutterProjectBundle, SwitchesMissingValues) {
  SetEnvironmentVariable("FLUTTER_ENGINE_SWITCHES", "4");
  SetEnvironmentVariable("FLUTTER_ENGINE_SWITCH_1", "abc");
  SetEnvironmentVariable("FLUTTER_ENGINE_SWITCH_2", "foo=\"bar, baz\"");
  ClearEnvironmentVariable("FLUTTER_ENGINE_SWITCH_3");
  SetEnvironmentVariable("FLUTTER_ENGINE_SWITCH_4", "oops");

  std::vector<std::string> switches = GetSwitchesFromEnvironment();
  EXPECT_EQ(switches.size(), 3U);
  EXPECT_EQ(switches[0], "--abc");
  EXPECT_EQ(switches[1], "--foo=\"bar, baz\"");
  // The missing switch should be skipped, leaving SWITCH_4 as the third
  // switch in the array.
  EXPECT_EQ(switches[2], "--oops");
}
#endif  // !FLUTTER_RELEASE

}  // namespace flutter
