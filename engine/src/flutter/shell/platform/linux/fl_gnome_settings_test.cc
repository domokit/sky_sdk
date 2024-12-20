// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/fl_gnome_settings.h"
#include "flutter/shell/platform/linux/testing/fl_test.h"
#include "flutter/shell/platform/linux/testing/mock_settings.h"
#include "flutter/shell/platform/linux/testing/mock_signal_handler.h"
#include "flutter/testing/testing.h"

#include <gio/gio.h>
#define G_SETTINGS_ENABLE_BACKEND
#include <gio/gsettingsbackend.h>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

class FlGnomeSettingsTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // force _g_io_modules_ensure_extension_points_registered() to get called
    g_settings_backend_get_default();
  }
};

static GSettings* create_settings(const gchar* name, const gchar* schema_id) {
  g_autofree gchar* path =
      g_build_filename(flutter::testing::GetFixturesPath(), name, nullptr);
  g_autoptr(GSettingsSchemaSource) source =
      g_settings_schema_source_new_from_directory(path, nullptr, false,
                                                  nullptr);
  g_autoptr(GSettingsSchema) schema =
      g_settings_schema_source_lookup(source, schema_id, false);
  g_autoptr(GSettingsBackend) backend = g_memory_settings_backend_new();
  return g_settings_new_full(schema, backend, nullptr);
}

TEST_F(FlGnomeSettingsTest, ClockFormat) {
  g_autoptr(GSettings) interface_settings =
      create_settings("ubuntu-20.04", "org.gnome.desktop.interface");
  g_settings_set_string(interface_settings, "clock-format", "24h");

  g_autoptr(FlSettings) settings = FL_SETTINGS(
      g_object_new(fl_gnome_settings_get_type(), "interface_settings",
                   interface_settings, nullptr));
  EXPECT_EQ(fl_settings_get_clock_format(settings), FL_CLOCK_FORMAT_24H);

  flutter::testing::MockSignalHandler settings_changed(settings, "changed");
  EXPECT_SIGNAL(settings_changed).Times(1);

  g_settings_set_string(interface_settings, "clock-format", "12h");
  EXPECT_EQ(fl_settings_get_clock_format(settings), FL_CLOCK_FORMAT_12H);
}

TEST_F(FlGnomeSettingsTest, GtkTheme) {
  g_autoptr(GSettings) interface_settings =
      create_settings("ubuntu-20.04", "org.gnome.desktop.interface");
  g_settings_set_string(interface_settings, "gtk-theme", "Yaru");

  g_autoptr(FlSettings) settings = FL_SETTINGS(
      g_object_new(fl_gnome_settings_get_type(), "interface_settings",
                   interface_settings, nullptr));
  EXPECT_EQ(fl_settings_get_color_scheme(settings), FL_COLOR_SCHEME_LIGHT);

  flutter::testing::MockSignalHandler settings_changed(settings, "changed");
  EXPECT_SIGNAL(settings_changed).Times(1);

  g_settings_set_string(interface_settings, "gtk-theme", "Yaru-dark");
  EXPECT_EQ(fl_settings_get_color_scheme(settings), FL_COLOR_SCHEME_DARK);
}

TEST_F(FlGnomeSettingsTest, EnableAnimations) {
  g_autoptr(FlSettings) settings = fl_gnome_settings_new();
  EXPECT_TRUE(fl_settings_get_enable_animations(settings));
}

TEST_F(FlGnomeSettingsTest, HighContrast) {
  g_autoptr(FlSettings) settings = fl_gnome_settings_new();
  EXPECT_FALSE(fl_settings_get_high_contrast(settings));
}

TEST_F(FlGnomeSettingsTest, TextScalingFactor) {
  g_autoptr(GSettings) interface_settings =
      create_settings("ubuntu-20.04", "org.gnome.desktop.interface");
  g_settings_set_double(interface_settings, "text-scaling-factor", 1.0);

  g_autoptr(FlSettings) settings = FL_SETTINGS(
      g_object_new(fl_gnome_settings_get_type(), "interface_settings",
                   interface_settings, nullptr));
  EXPECT_EQ(fl_settings_get_text_scaling_factor(settings), 1.0);

  flutter::testing::MockSignalHandler settings_changed(settings, "changed");
  EXPECT_SIGNAL(settings_changed).Times(1);

  g_settings_set_double(interface_settings, "text-scaling-factor", 1.5);
  EXPECT_EQ(fl_settings_get_text_scaling_factor(settings), 1.5);
}

TEST_F(FlGnomeSettingsTest, SignalHandlers) {
  g_autoptr(GSettings) interface_settings =
      create_settings("ubuntu-20.04", "org.gnome.desktop.interface");

  g_autoptr(FlSettings) settings = FL_SETTINGS(
      g_object_new(fl_gnome_settings_get_type(), "interface_settings",
                   interface_settings, nullptr));
  flutter::testing::MockSignalHandler settings_changed(settings, "changed");

  EXPECT_SIGNAL(settings_changed).Times(3);

  g_settings_set_string(interface_settings, "clock-format", "12h");
  g_settings_set_string(interface_settings, "gtk-theme", "Yaru-dark");
  g_settings_set_double(interface_settings, "text-scaling-factor", 1.5);

  EXPECT_SIGNAL(settings_changed).Times(0);

  g_clear_object(&settings);

  // destroyed FlSettings object must have disconnected its signal handlers
  g_settings_set_string(interface_settings, "clock-format", "24h");
  g_settings_set_string(interface_settings, "gtk-theme", "Yaru");
  g_settings_set_double(interface_settings, "text-scaling-factor", 2.0);
}
