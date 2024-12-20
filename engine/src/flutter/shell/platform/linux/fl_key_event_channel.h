// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_FL_KEY_EVENT_CHANNEL_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_FL_KEY_EVENT_CHANNEL_H_

#include "flutter/shell/platform/linux/public/flutter_linux/fl_binary_messenger.h"

G_BEGIN_DECLS

G_DECLARE_FINAL_TYPE(FlKeyEventChannel,
                     fl_key_event_channel,
                     FL,
                     KEY_EVENT_CHANNEL,
                     GObject);

/**
 * FlKeyEventChannel:
 *
 * #FlKeyEventChannel is a channel that implements the shell side
 * of SystemChannels.keyEvent from the Flutter services library.
 */

typedef enum {
  FL_KEY_EVENT_TYPE_KEYUP,
  FL_KEY_EVENT_TYPE_KEYDOWN,
} FlKeyEventType;

/**
 * fl_key_event_channel_new:
 * @messenger: an #FlBinaryMessenger
 *
 * Creates a new channel that implements SystemChannels.keyEvent from the
 * Flutter services library.
 *
 * Returns: a new #FlKeyEventChannel.
 */
FlKeyEventChannel* fl_key_event_channel_new(FlBinaryMessenger* messenger);

/**
 * fl_key_event_channel_send:
 * @channel: an #FlKeyEventChannel
 * @type: event type.
 * @scan_code: scan code.
 * @key_code: key code.
 * @modifiers: modifiers.
 * @unicode_scarlar_values:
 * @specified_logical_key:
 * @cancellable: (allow-none): a #GCancellable or %NULL.
 * @callback: (scope async): a #GAsyncReadyCallback to call when the method
 * returns.
 * @user_data: (closure): user data to pass to @callback.
 *
 * Send a key event to the platform.
 */
void fl_key_event_channel_send(FlKeyEventChannel* channel,
                               FlKeyEventType type,
                               int64_t scan_code,
                               int64_t key_code,
                               int64_t modifiers,
                               int64_t unicode_scarlar_values,
                               int64_t specified_logical_key,
                               GCancellable* cancellable,
                               GAsyncReadyCallback callback,
                               gpointer user_data);

/**
 * fl_key_event_channel_send_finish:
 * @object:
 * @result: a #GAsyncResult.
 * @error: (allow-none): #GError location to store the error occurring, or %NULL
 * to ignore.
 *
 * Completes request started with fl_key_event_channel_send().
 *
 * Returns: %TRUE on success.
 */
gboolean fl_key_event_channel_send_finish(GObject* object,
                                          GAsyncResult* result,
                                          gboolean* handled,
                                          GError** error);

G_END_DECLS

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_FL_KEY_EVENT_CHANNEL_H_
