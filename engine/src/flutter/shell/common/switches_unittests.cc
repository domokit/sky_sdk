// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <initializer_list>

#include "flutter/common/settings.h"
#include "flutter/fml/command_line.h"
#include "flutter/shell/common/switches.h"

#include "gtest/gtest.h"

// TODO(zanderso): https://github.com/flutter/flutter/issues/127701
// NOLINTBEGIN(bugprone-unchecked-optional-access)

namespace flutter {
namespace testing {

TEST(SwitchesTest, SkiaTraceAllowlistFlag) {
  fml::CommandLine command_line =
      fml::CommandLineFromInitializerList({"command"});
  Settings settings = SettingsFromCommandLine(command_line);
#if !FLUTTER_RELEASE
  EXPECT_TRUE(settings.trace_skia);
  EXPECT_TRUE(settings.trace_skia_allowlist.has_value());
  EXPECT_EQ(settings.trace_skia_allowlist->size(), 1ul);
#else
  EXPECT_FALSE(settings.trace_skia);
#endif

  command_line =
      fml::CommandLineFromInitializerList({"command", "--trace-skia"});
  settings = SettingsFromCommandLine(command_line);
#if !FLUTTER_RELEASE
  EXPECT_TRUE(settings.trace_skia);
  EXPECT_FALSE(settings.trace_skia_allowlist.has_value());
#else
  EXPECT_FALSE(settings.trace_skia);
#endif

  command_line = fml::CommandLineFromInitializerList(
      {"command", "--trace-skia-allowlist=aaa,bbb,ccc"});
  settings = SettingsFromCommandLine(command_line);
#if !FLUTTER_RELEASE
  EXPECT_TRUE(settings.trace_skia);
  EXPECT_TRUE(settings.trace_skia_allowlist.has_value());
  EXPECT_EQ(settings.trace_skia_allowlist->size(), 3ul);
  EXPECT_EQ(settings.trace_skia_allowlist->back(), "ccc");
#else
  EXPECT_FALSE(settings.trace_skia);
#endif
}

TEST(SwitchesTest, TraceToFile) {
  fml::CommandLine command_line = fml::CommandLineFromInitializerList(
      {"command", "--trace-to-file=trace.binpb"});
  EXPECT_TRUE(command_line.HasOption("trace-to-file"));
  Settings settings = SettingsFromCommandLine(command_line);
  EXPECT_EQ(settings.trace_to_file, "trace.binpb");
}

TEST(SwitchesTest, RouteParsedFlag) {
  fml::CommandLine command_line =
      fml::CommandLineFromInitializerList({"command", "--route=/animation"});
  Settings settings = SettingsFromCommandLine(command_line);
  EXPECT_EQ(settings.route, "/animation");
  command_line = fml::CommandLineFromInitializerList({"command", "--route"});
  settings = SettingsFromCommandLine(command_line);
  EXPECT_TRUE(settings.route.empty());
}

TEST(SwitchesTest, EnableEmbedderAPI) {
  {
    // enable
    fml::CommandLine command_line = fml::CommandLineFromInitializerList(
        {"command", "--enable-embedder-api"});
    Settings settings = SettingsFromCommandLine(command_line);
    EXPECT_EQ(settings.enable_embedder_api, true);
  }
  {
    // default
    fml::CommandLine command_line =
        fml::CommandLineFromInitializerList({"command"});
    Settings settings = SettingsFromCommandLine(command_line);
    EXPECT_EQ(settings.enable_embedder_api, false);
  }
}

TEST(SwitchesTest, NoEnableImpeller) {
  {
    // enable
    fml::CommandLine command_line =
        fml::CommandLineFromInitializerList({"command", "--enable-impeller"});
    EXPECT_TRUE(command_line.HasOption("enable-impeller"));
    Settings settings = SettingsFromCommandLine(command_line);
    EXPECT_EQ(settings.enable_impeller, true);
  }
  {
    // disable
    fml::CommandLine command_line = fml::CommandLineFromInitializerList(
        {"command", "--enable-impeller=false"});
    EXPECT_TRUE(command_line.HasOption("enable-impeller"));
    Settings settings = SettingsFromCommandLine(command_line);
    EXPECT_EQ(settings.enable_impeller, false);
  }
}

#if !FLUTTER_RELEASE
TEST(SwitchesTest, EnableAsserts) {
  fml::CommandLine command_line = fml::CommandLineFromInitializerList(
      {"command", "--dart-flags=--enable-asserts"});
  Settings settings = SettingsFromCommandLine(command_line);
  ASSERT_EQ(settings.dart_flags.size(), 1ul);
  EXPECT_EQ(settings.dart_flags[0], "--enable-asserts");
}
#endif

}  // namespace testing
}  // namespace flutter

// NOLINTEND(bugprone-unchecked-optional-access)
