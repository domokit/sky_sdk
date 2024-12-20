// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_FL_KEY_CHANNEL_RESPONDER_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_FL_KEY_CHANNEL_RESPONDER_H_

#include "flutter/shell/platform/linux/fl_key_event.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_binary_messenger.h"

G_BEGIN_DECLS

G_DECLARE_FINAL_TYPE(FlKeyChannelResponder,
                     fl_key_channel_responder,
                     FL,
                     KEY_CHANNEL_RESPONDER,
                     GObject);

/**
 * FlKeyChannelResponder:
 *
 * A #FlKeyResponder that handles events by sending the raw event data
 * in JSON through the message channel.
 *
 * This class communicates with the RawKeyboard API in the framework.
 */

/**
 * fl_key_channel_responder_new:
 * @messenger: the messenger that the message channel should be built on.
 *
 * Creates a new #FlKeyChannelResponder.
 *
 * Returns: a new #FlKeyChannelResponder.
 */
FlKeyChannelResponder* fl_key_channel_responder_new(
    FlBinaryMessenger* messenger);

/**
 * fl_key_channel_responder_handle_event:
 * @responder: the #FlKeyChannelResponder self.
 * @event: the event to be handled. Must not be null. The object is managed by
 * callee and must not be assumed available after this function.
 * @specified_logical_key:
 * @cancellable: (allow-none): a #GCancellable or %NULL.
 * @callback: (scope async): a #GAsyncReadyCallback to call when the event has
 * been processed.
 * @user_data: (closure): user data to pass to @callback.
 *
 * Let the responder handle an event.
 */
void fl_key_channel_responder_handle_event(FlKeyChannelResponder* responder,
                                           FlKeyEvent* event,
                                           uint64_t specified_logical_key,
                                           GCancellable* cancellable,
                                           GAsyncReadyCallback callback,
                                           gpointer user_data);

/**
 * fl_key_channel_responder_handle_event_finish:
 * @responder: an #FlKeyChannelResponder.
 * @result: a #GAsyncResult.
 * @handled: location to write if this event was handled by the platform.
 * @error: (allow-none): #GError location to store the error occurring, or %NULL
 * to ignore.
 *
 * Completes request started with fl_key_channel_responder_handle_event().
 *
 * Returns %TRUE on success.
 */
gboolean fl_key_channel_responder_handle_event_finish(
    FlKeyChannelResponder* responder,
    GAsyncResult* result,
    gboolean* handled,
    GError** error);

G_END_DECLS

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_FL_KEY_CHANNEL_RESPONDER_H_
