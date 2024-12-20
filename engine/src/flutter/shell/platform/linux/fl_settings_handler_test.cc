// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/fl_settings_handler.h"
#include "flutter/shell/platform/embedder/embedder.h"
#include "flutter/shell/platform/embedder/test_utils/proc_table_replacement.h"
#include "flutter/shell/platform/linux/fl_binary_messenger_private.h"
#include "flutter/shell/platform/linux/fl_engine_private.h"
#include "flutter/shell/platform/linux/testing/fl_mock_binary_messenger.h"
#include "flutter/shell/platform/linux/testing/mock_settings.h"
#include "flutter/testing/testing.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

TEST(FlSettingsHandlerTest, AlwaysUse24HourFormat) {
  ::testing::NiceMock<flutter::testing::MockSettings> settings;

  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();
  g_autoptr(FlEngine) engine =
      FL_ENGINE(g_object_new(fl_engine_get_type(), "binary-messenger",
                             FL_BINARY_MESSENGER(messenger), nullptr));
  g_autoptr(FlSettingsHandler) handler = fl_settings_handler_new(engine);

  EXPECT_CALL(settings, fl_settings_get_clock_format(
                            ::testing::Eq<FlSettings*>(settings)))
      .WillOnce(::testing::Return(FL_CLOCK_FORMAT_12H))
      .WillOnce(::testing::Return(FL_CLOCK_FORMAT_24H));

  gboolean called = FALSE;
  fl_mock_binary_messenger_set_json_message_channel(
      messenger, "flutter/settings",
      [](FlMockBinaryMessenger* messenger, FlValue* message,
         gpointer user_data) {
        gboolean* called = static_cast<gboolean*>(user_data);
        *called = TRUE;

        EXPECT_EQ(fl_value_get_type(message), FL_VALUE_TYPE_MAP);
        FlValue* value =
            fl_value_lookup_string(message, "alwaysUse24HourFormat");
        EXPECT_NE(value, nullptr);
        EXPECT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_BOOL);
        EXPECT_FALSE(fl_value_get_bool(value));

        return fl_value_new_null();
      },
      &called);
  fl_settings_handler_start(handler, settings);
  EXPECT_TRUE(called);

  called = FALSE;
  fl_mock_binary_messenger_set_json_message_channel(
      messenger, "flutter/settings",
      [](FlMockBinaryMessenger* messenger, FlValue* message,
         gpointer user_data) {
        gboolean* called = static_cast<gboolean*>(user_data);
        *called = TRUE;

        EXPECT_EQ(fl_value_get_type(message), FL_VALUE_TYPE_MAP);
        FlValue* value =
            fl_value_lookup_string(message, "alwaysUse24HourFormat");
        EXPECT_NE(value, nullptr);
        EXPECT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_BOOL);
        EXPECT_TRUE(fl_value_get_bool(value));

        return fl_value_new_null();
      },
      &called);
  fl_settings_emit_changed(settings);
  EXPECT_TRUE(called);

  fl_binary_messenger_shutdown(FL_BINARY_MESSENGER(messenger));
}

TEST(FlSettingsHandlerTest, PlatformBrightness) {
  ::testing::NiceMock<flutter::testing::MockSettings> settings;

  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();
  g_autoptr(FlEngine) engine =
      FL_ENGINE(g_object_new(fl_engine_get_type(), "binary-messenger",
                             FL_BINARY_MESSENGER(messenger), nullptr));
  g_autoptr(FlSettingsHandler) handler = fl_settings_handler_new(engine);

  EXPECT_CALL(settings, fl_settings_get_color_scheme(
                            ::testing::Eq<FlSettings*>(settings)))
      .WillOnce(::testing::Return(FL_COLOR_SCHEME_LIGHT))
      .WillOnce(::testing::Return(FL_COLOR_SCHEME_DARK));

  gboolean called = FALSE;
  fl_mock_binary_messenger_set_json_message_channel(
      messenger, "flutter/settings",
      [](FlMockBinaryMessenger* messenger, FlValue* message,
         gpointer user_data) {
        gboolean* called = static_cast<gboolean*>(user_data);
        *called = TRUE;

        EXPECT_EQ(fl_value_get_type(message), FL_VALUE_TYPE_MAP);
        FlValue* value = fl_value_lookup_string(message, "platformBrightness");
        EXPECT_NE(value, nullptr);
        EXPECT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_STRING);
        EXPECT_STREQ(fl_value_get_string(value), "light");

        return fl_value_new_null();
      },
      &called);
  fl_settings_handler_start(handler, settings);
  EXPECT_TRUE(called);

  called = FALSE;
  fl_mock_binary_messenger_set_json_message_channel(
      messenger, "flutter/settings",
      [](FlMockBinaryMessenger* messenger, FlValue* message,
         gpointer user_data) {
        gboolean* called = static_cast<gboolean*>(user_data);
        *called = TRUE;

        EXPECT_EQ(fl_value_get_type(message), FL_VALUE_TYPE_MAP);
        FlValue* value = fl_value_lookup_string(message, "platformBrightness");
        EXPECT_NE(value, nullptr);
        EXPECT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_STRING);
        EXPECT_STREQ(fl_value_get_string(value), "dark");

        return fl_value_new_null();
      },
      &called);
  fl_settings_emit_changed(settings);
  EXPECT_TRUE(called);

  fl_binary_messenger_shutdown(FL_BINARY_MESSENGER(messenger));
}

TEST(FlSettingsHandlerTest, TextScaleFactor) {
  ::testing::NiceMock<flutter::testing::MockSettings> settings;

  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();
  g_autoptr(FlEngine) engine =
      FL_ENGINE(g_object_new(fl_engine_get_type(), "binary-messenger",
                             FL_BINARY_MESSENGER(messenger), nullptr));
  g_autoptr(FlSettingsHandler) handler = fl_settings_handler_new(engine);

  EXPECT_CALL(settings, fl_settings_get_text_scaling_factor(
                            ::testing::Eq<FlSettings*>(settings)))
      .WillOnce(::testing::Return(1.0))
      .WillOnce(::testing::Return(2.0));

  gboolean called = FALSE;
  fl_mock_binary_messenger_set_json_message_channel(
      messenger, "flutter/settings",
      [](FlMockBinaryMessenger* messenger, FlValue* message,
         gpointer user_data) {
        gboolean* called = static_cast<gboolean*>(user_data);
        *called = TRUE;

        EXPECT_EQ(fl_value_get_type(message), FL_VALUE_TYPE_MAP);
        FlValue* value = fl_value_lookup_string(message, "textScaleFactor");
        EXPECT_NE(value, nullptr);
        EXPECT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_FLOAT);
        EXPECT_EQ(fl_value_get_float(value), 1.0);

        return fl_value_new_null();
      },
      &called);
  fl_settings_handler_start(handler, settings);
  EXPECT_TRUE(called);

  called = FALSE;
  fl_mock_binary_messenger_set_json_message_channel(
      messenger, "flutter/settings",
      [](FlMockBinaryMessenger* messenger, FlValue* message,
         gpointer user_data) {
        gboolean* called = static_cast<gboolean*>(user_data);
        *called = TRUE;

        EXPECT_EQ(fl_value_get_type(message), FL_VALUE_TYPE_MAP);
        FlValue* value = fl_value_lookup_string(message, "textScaleFactor");
        EXPECT_NE(value, nullptr);
        EXPECT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_FLOAT);
        EXPECT_EQ(fl_value_get_float(value), 2.0);

        return fl_value_new_null();
      },
      &called);
  fl_settings_emit_changed(settings);
  EXPECT_TRUE(called);

  fl_binary_messenger_shutdown(FL_BINARY_MESSENGER(messenger));
}

// MOCK_ENGINE_PROC is leaky by design
// NOLINTBEGIN(clang-analyzer-core.StackAddressEscape)
TEST(FlSettingsHandlerTest, AccessibilityFeatures) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);

  g_autoptr(GError) error = nullptr;
  EXPECT_TRUE(fl_engine_start(engine, &error));
  EXPECT_EQ(error, nullptr);

  std::vector<FlutterAccessibilityFeature> calls;
  fl_engine_get_embedder_api(engine)->UpdateAccessibilityFeatures =
      MOCK_ENGINE_PROC(
          UpdateAccessibilityFeatures,
          ([&calls](auto engine, FlutterAccessibilityFeature features) {
            calls.push_back(features);
            return kSuccess;
          }));

  g_autoptr(FlSettingsHandler) handler = fl_settings_handler_new(engine);

  ::testing::NiceMock<flutter::testing::MockSettings> settings;

  EXPECT_CALL(settings, fl_settings_get_enable_animations(
                            ::testing::Eq<FlSettings*>(settings)))
      .WillOnce(::testing::Return(false))
      .WillOnce(::testing::Return(true))
      .WillOnce(::testing::Return(false))
      .WillOnce(::testing::Return(true));

  EXPECT_CALL(settings, fl_settings_get_high_contrast(
                            ::testing::Eq<FlSettings*>(settings)))
      .WillOnce(::testing::Return(true))
      .WillOnce(::testing::Return(false))
      .WillOnce(::testing::Return(false))
      .WillOnce(::testing::Return(true));

  fl_settings_handler_start(handler, settings);
  EXPECT_THAT(calls, ::testing::SizeIs(1));
  EXPECT_EQ(calls.back(), static_cast<FlutterAccessibilityFeature>(
                              kFlutterAccessibilityFeatureDisableAnimations |
                              kFlutterAccessibilityFeatureHighContrast));

  fl_settings_emit_changed(settings);
  EXPECT_THAT(calls, ::testing::SizeIs(2));
  EXPECT_EQ(calls.back(), static_cast<FlutterAccessibilityFeature>(0));

  fl_settings_emit_changed(settings);
  EXPECT_THAT(calls, ::testing::SizeIs(3));
  EXPECT_EQ(calls.back(), static_cast<FlutterAccessibilityFeature>(
                              kFlutterAccessibilityFeatureDisableAnimations));

  fl_settings_emit_changed(settings);
  EXPECT_THAT(calls, ::testing::SizeIs(4));
  EXPECT_EQ(calls.back(), static_cast<FlutterAccessibilityFeature>(
                              kFlutterAccessibilityFeatureHighContrast));
}
// NOLINTEND(clang-analyzer-core.StackAddressEscape)
