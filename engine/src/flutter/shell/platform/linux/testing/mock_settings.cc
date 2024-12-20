// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/testing/mock_settings.h"

using namespace flutter::testing;

G_DECLARE_FINAL_TYPE(FlMockSettings,
                     fl_mock_settings,
                     FL,
                     MOCK_SETTINGS,
                     GObject)

struct _FlMockSettings {
  GObject parent_instance;
  MockSettings* mock;
};

static void fl_mock_settings_iface_init(FlSettingsInterface* iface);

#define FL_UNUSED(x) (void)x;

G_DEFINE_TYPE_WITH_CODE(FlMockSettings,
                        fl_mock_settings,
                        G_TYPE_OBJECT,
                        G_IMPLEMENT_INTERFACE(fl_settings_get_type(),
                                              fl_mock_settings_iface_init)
                            FL_UNUSED(FL_IS_MOCK_SETTINGS))

static void fl_mock_settings_class_init(FlMockSettingsClass* klass) {}

static FlClockFormat fl_mock_settings_get_clock_format(FlSettings* settings) {
  FlMockSettings* self = FL_MOCK_SETTINGS(settings);
  return self->mock->fl_settings_get_clock_format(settings);
}

static FlColorScheme fl_mock_settings_get_color_scheme(FlSettings* settings) {
  FlMockSettings* self = FL_MOCK_SETTINGS(settings);
  return self->mock->fl_settings_get_color_scheme(settings);
}

static gboolean fl_mock_settings_get_enable_animations(FlSettings* settings) {
  FlMockSettings* self = FL_MOCK_SETTINGS(settings);
  return self->mock->fl_settings_get_enable_animations(settings);
}

static gboolean fl_mock_settings_get_high_contrast(FlSettings* settings) {
  FlMockSettings* self = FL_MOCK_SETTINGS(settings);
  return self->mock->fl_settings_get_high_contrast(settings);
}

static gdouble fl_mock_settings_get_text_scaling_factor(FlSettings* settings) {
  FlMockSettings* self = FL_MOCK_SETTINGS(settings);
  return self->mock->fl_settings_get_text_scaling_factor(settings);
}

static void fl_mock_settings_iface_init(FlSettingsInterface* iface) {
  iface->get_clock_format = fl_mock_settings_get_clock_format;
  iface->get_color_scheme = fl_mock_settings_get_color_scheme;
  iface->get_enable_animations = fl_mock_settings_get_enable_animations;
  iface->get_high_contrast = fl_mock_settings_get_high_contrast;
  iface->get_text_scaling_factor = fl_mock_settings_get_text_scaling_factor;
}

static void fl_mock_settings_init(FlMockSettings* self) {}

MockSettings::MockSettings()
    : instance_(
          FL_SETTINGS(g_object_new(fl_mock_settings_get_type(), nullptr))) {
  FL_MOCK_SETTINGS(instance_)->mock = this;
}

MockSettings::~MockSettings() {
  if (instance_ != nullptr) {
    g_clear_object(&instance_);
  }
}

MockSettings::operator FlSettings*() {
  return instance_;
}
