// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/fl_settings_channel.h"

#include "flutter/shell/platform/linux/public/flutter_linux/fl_basic_message_channel.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_json_message_codec.h"

static constexpr char kChannelName[] = "flutter/settings";
static constexpr char kTextScaleFactorKey[] = "textScaleFactor";
static constexpr char kAlwaysUse24HourFormatKey[] = "alwaysUse24HourFormat";
static constexpr char kPlatformBrightnessKey[] = "platformBrightness";
static constexpr char kPlatformBrightnessLight[] = "light";
static constexpr char kPlatformBrightnessDark[] = "dark";

struct _FlSettingsChannel {
  GObject parent_instance;

  FlBasicMessageChannel* channel;
};

G_DEFINE_TYPE(FlSettingsChannel, fl_settings_channel, G_TYPE_OBJECT)

static void fl_settings_channel_dispose(GObject* object) {
  FlSettingsChannel* self = FL_SETTINGS_CHANNEL(object);

  g_clear_object(&self->channel);

  G_OBJECT_CLASS(fl_settings_channel_parent_class)->dispose(object);
}

static void fl_settings_channel_class_init(FlSettingsChannelClass* klass) {
  G_OBJECT_CLASS(klass)->dispose = fl_settings_channel_dispose;
}

static void fl_settings_channel_init(FlSettingsChannel* self) {}

FlSettingsChannel* fl_settings_channel_new(FlBinaryMessenger* messenger) {
  FlSettingsChannel* self = FL_SETTINGS_CHANNEL(
      g_object_new(fl_settings_channel_get_type(), nullptr));

  g_autoptr(FlJsonMessageCodec) codec = fl_json_message_codec_new();
  self->channel = fl_basic_message_channel_new(messenger, kChannelName,
                                               FL_MESSAGE_CODEC(codec));

  return self;
}

void fl_settings_channel_send(
    FlSettingsChannel* self,
    double text_scale_factor,
    gboolean always_use_24_hour_format,
    FlSettingsChannelPlatformBrightness platform_brightness) {
  g_return_if_fail(FL_IS_SETTINGS_CHANNEL(self));

  g_autoptr(FlValue) message = fl_value_new_map();
  fl_value_set_string_take(message, kTextScaleFactorKey,
                           fl_value_new_float(text_scale_factor));
  fl_value_set_string_take(message, kAlwaysUse24HourFormatKey,
                           fl_value_new_bool(always_use_24_hour_format));
  const gchar* platform_brightness_string;
  switch (platform_brightness) {
    case FL_SETTINGS_CHANNEL_PLATFORM_BRIGHTNESS_LIGHT:
      platform_brightness_string = kPlatformBrightnessLight;
      break;
    case FL_SETTINGS_CHANNEL_PLATFORM_BRIGHTNESS_DARK:
      platform_brightness_string = kPlatformBrightnessDark;
      break;
    default:
      g_assert_not_reached();
  }
  fl_value_set_string_take(message, kPlatformBrightnessKey,
                           fl_value_new_string(platform_brightness_string));
  fl_basic_message_channel_send(self->channel, message, nullptr, nullptr,
                                nullptr);
}
