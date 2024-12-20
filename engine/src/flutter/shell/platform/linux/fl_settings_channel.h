// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_FL_SETTINGS_CHANNEL_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_FL_SETTINGS_CHANNEL_H_

#include "flutter/shell/platform/linux/public/flutter_linux/fl_binary_messenger.h"

G_BEGIN_DECLS

typedef enum {
  FL_SETTINGS_CHANNEL_PLATFORM_BRIGHTNESS_LIGHT,
  FL_SETTINGS_CHANNEL_PLATFORM_BRIGHTNESS_DARK
} FlSettingsChannelPlatformBrightness;

G_DECLARE_FINAL_TYPE(FlSettingsChannel,
                     fl_settings_channel,
                     FL,
                     SETTINGS_CHANNEL,
                     GObject);

/**
 * FlSettingsChannel:
 *
 * #FlSettingsChannel is a channel that implements the Flutter user settings
 * channel.
 */

/**
 * fl_settings_channel_new:
 * @messenger: an #FlBinaryMessenger
 *
 * Creates a new channel that sends user settings to the platform.
 *
 * Returns: a new #FlSettingsChannel
 */
FlSettingsChannel* fl_settings_channel_new(FlBinaryMessenger* messenger);

/*
 * fl_settings_channel_send:
 * @channel: an #FlSettingsChannel.
 * @text_scale_factor: scale factor for text.
 * @always_use_24_hour_format: TRUE if time should always be shown in 24 hour
 * format.
 * @platform_brightness: The brightness theme in use.
 *
 * Send a settings update to the platform.
 */
void fl_settings_channel_send(
    FlSettingsChannel* channel,
    double text_scale_factor,
    gboolean always_use_24_hour_format,
    FlSettingsChannelPlatformBrightness platform_brightness);

G_END_DECLS

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_FL_SETTINGS_CHANNEL_H_
