// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/fl_settings_handler.h"

#include <gmodule.h>

#include "flutter/shell/platform/embedder/embedder.h"
#include "flutter/shell/platform/linux/fl_engine_private.h"
#include "flutter/shell/platform/linux/fl_settings_channel.h"

struct _FlSettingsHandler {
  GObject parent_instance;

  FlSettingsChannel* channel;
  GWeakRef engine;
  FlSettings* settings;
};

G_DEFINE_TYPE(FlSettingsHandler, fl_settings_handler, G_TYPE_OBJECT)

static FlSettingsChannelPlatformBrightness to_platform_brightness(
    FlColorScheme color_scheme) {
  switch (color_scheme) {
    case FL_COLOR_SCHEME_LIGHT:
      return FL_SETTINGS_CHANNEL_PLATFORM_BRIGHTNESS_LIGHT;
    case FL_COLOR_SCHEME_DARK:
      return FL_SETTINGS_CHANNEL_PLATFORM_BRIGHTNESS_DARK;
    default:
      g_assert_not_reached();
  }
}

// Sends the current settings to the Flutter engine.
static void update_settings(FlSettingsHandler* self) {
  FlClockFormat clock_format = fl_settings_get_clock_format(self->settings);
  FlColorScheme color_scheme = fl_settings_get_color_scheme(self->settings);
  gdouble scaling_factor = fl_settings_get_text_scaling_factor(self->settings);

  fl_settings_channel_send(self->channel, scaling_factor,
                           clock_format == FL_CLOCK_FORMAT_24H,
                           to_platform_brightness(color_scheme));

  g_autoptr(FlEngine) engine = FL_ENGINE(g_weak_ref_get(&self->engine));
  if (engine != nullptr) {
    int32_t flags = 0;
    if (!fl_settings_get_enable_animations(self->settings)) {
      flags |= kFlutterAccessibilityFeatureDisableAnimations;
    }
    if (fl_settings_get_high_contrast(self->settings)) {
      flags |= kFlutterAccessibilityFeatureHighContrast;
    }
    fl_engine_update_accessibility_features(engine, flags);
  }
}

static void fl_settings_handler_dispose(GObject* object) {
  FlSettingsHandler* self = FL_SETTINGS_HANDLER(object);

  g_clear_object(&self->channel);
  g_clear_object(&self->settings);
  g_weak_ref_clear(&self->engine);

  G_OBJECT_CLASS(fl_settings_handler_parent_class)->dispose(object);
}

static void fl_settings_handler_class_init(FlSettingsHandlerClass* klass) {
  G_OBJECT_CLASS(klass)->dispose = fl_settings_handler_dispose;
}

static void fl_settings_handler_init(FlSettingsHandler* self) {}

FlSettingsHandler* fl_settings_handler_new(FlEngine* engine) {
  g_return_val_if_fail(FL_IS_ENGINE(engine), nullptr);

  FlSettingsHandler* self = FL_SETTINGS_HANDLER(
      g_object_new(fl_settings_handler_get_type(), nullptr));

  g_weak_ref_init(&self->engine, engine);

  FlBinaryMessenger* messenger = fl_engine_get_binary_messenger(engine);
  self->channel = fl_settings_channel_new(messenger);

  return self;
}

void fl_settings_handler_start(FlSettingsHandler* self, FlSettings* settings) {
  g_return_if_fail(FL_IS_SETTINGS_HANDLER(self));
  g_return_if_fail(FL_IS_SETTINGS(settings));

  self->settings = FL_SETTINGS(g_object_ref(settings));
  g_signal_connect_object(settings, "changed", G_CALLBACK(update_settings),
                          self, G_CONNECT_SWAPPED);

  update_settings(self);
}
