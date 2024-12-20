// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_FL_KEYBOARD_CHANNEL_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_FL_KEYBOARD_CHANNEL_H_

#include "flutter/shell/platform/linux/public/flutter_linux/fl_binary_messenger.h"

G_BEGIN_DECLS

G_DECLARE_FINAL_TYPE(FlKeyboardChannel,
                     fl_keyboard_channel,
                     FL,
                     KEYBOARD_CHANNEL,
                     GObject);

/**
 * FlKeyboardChannel:
 *
 * #FlKeyboardChannel is a channel that implements the shell side
 * of SystemChannels.keyboard from the Flutter services library.
 */

typedef struct {
  FlValue* (*get_keyboard_state)(gpointer user_data);
} FlKeyboardChannelVTable;

/**
 * fl_keyboard_channel_new:
 * @messenger: an #FlBinaryMessenger
 * @vtable: callbacks for incoming method calls.
 * @user_data: data to pass in callbacks.
 *
 * Creates a new channel that implements SystemChannels.keyboard from the
 * Flutter services library.
 *
 * Returns: a new #FlKeyboardChannel
 */
FlKeyboardChannel* fl_keyboard_channel_new(FlBinaryMessenger* messenger,
                                           FlKeyboardChannelVTable* vtable,
                                           gpointer user_data);

G_END_DECLS

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_FL_KEYBOARD_CHANNEL_H_
