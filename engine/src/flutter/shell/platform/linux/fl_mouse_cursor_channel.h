// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_FL_MOUSE_CURSOR_CHANNEL_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_FL_MOUSE_CURSOR_CHANNEL_H_

#include "flutter/shell/platform/linux/public/flutter_linux/fl_binary_messenger.h"

G_BEGIN_DECLS

G_DECLARE_FINAL_TYPE(FlMouseCursorChannel,
                     fl_mouse_cursor_channel,
                     FL,
                     MOUSE_CURSOR_CHANNEL,
                     GObject);

/**
 * FlMouseCursorChannel:
 *
 * #FlMouseCursorChannel is a cursor channel that implements the shell
 * side of SystemChannels.mouseCursor from the Flutter services library.
 */

typedef struct {
  void (*activate_system_cursor)(const gchar* kind, gpointer user_data);
} FlMouseCursorChannelVTable;

/**
 * fl_mouse_cursor_channel_new:
 * @messenger: an #FlBinaryMessenger.
 * @vtable: callbacks for incoming method calls.
 * @user_data: data to pass in callbacks.
 *
 * Creates a new channel that implements SystemChannels.mouseCursor from the
 * Flutter services library.
 *
 * Returns: a new #FlMouseCursorChannel.
 */
FlMouseCursorChannel* fl_mouse_cursor_channel_new(
    FlBinaryMessenger* messenger,
    FlMouseCursorChannelVTable* vtable,
    gpointer user_data);

G_END_DECLS

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_FL_MOUSE_CURSOR_CHANNEL_H_
