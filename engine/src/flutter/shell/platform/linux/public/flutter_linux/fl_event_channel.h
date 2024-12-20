// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_PUBLIC_FLUTTER_LINUX_FL_EVENT_CHANNEL_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_PUBLIC_FLUTTER_LINUX_FL_EVENT_CHANNEL_H_

#if !defined(__FLUTTER_LINUX_INSIDE__) && !defined(FLUTTER_LINUX_COMPILATION)
#error "Only <flutter_linux/flutter_linux.h> can be included directly."
#endif

#include <gio/gio.h>
#include <glib-object.h>
#include <gmodule.h>

#include "fl_binary_messenger.h"
#include "fl_method_channel.h"
#include "fl_method_response.h"

G_BEGIN_DECLS

G_MODULE_EXPORT
G_DECLARE_FINAL_TYPE(FlEventChannel,
                     fl_event_channel,
                     FL,
                     EVENT_CHANNEL,
                     GObject)

/**
 * FlEventChannel:
 *
 * #FlEventChannel is an object that allows sending
 * an events stream to Dart code over platform channels.
 *
 * The following example shows how to send events on a channel:
 *
 * |[<!-- language="C" -->
 * static FlEventChannel *channel = NULL;
 * static gboolean send_events = FALSE;
 *
 * static void event_occurs_cb (FooEvent *event) {
 *   if (send_events) {
 *     g_autoptr(FlValue) message = foo_event_to_value (event);
 *     g_autoptr(GError) error = NULL;
 *     if (!fl_event_channel_send (channel, message, NULL, &error)) {
 *       g_warning ("Failed to send event: %s", error->message);
 *     }
 *   }
 * }
 *
 * static FlMethodErrorResponse* listen_cb (FlEventChannel* channel,
 *                                          FlValue *args,
 *                                          gpointer user_data) {
 *   send_events = TRUE;
 *   return NULL;
 * }
 *
 * static FlMethodErrorResponse* cancel_cb (GObject *object,
 *                                          FlValue *args,
 *                                          gpointer user_data) {
 *   send_events = FALSE;
 *   return NULL;
 * }
 *
 * static void setup_channel () {
 *   g_autoptr(FlStandardMethodCodec) codec = fl_standard_method_codec_new ();
 *   channel = fl_event_channel_new (messenger, "flutter/foo",
 *                                   FL_METHOD_CODEC (codec));
 *   fl_event_channel_set_stream_handlers (channel, listen_cb, cancel_cb,
 *                                        NULL, NULL);
 * }
 * ]|
 *
 * #FlEventChannel matches the EventChannel class in the Flutter
 * services library.
 */

/**
 * FlEventChannelHandler:
 * @channel: an #FlEventChannel.
 * @args: arguments passed from the Dart end of the channel.
 * @user_data: (closure): data provided when registering this handler.
 *
 * Function called when the stream is listened to or cancelled.
 *
 * Returns: (transfer full): an #FlMethodErrorResponse or %NULL if no error.
 */
typedef FlMethodErrorResponse* (*FlEventChannelHandler)(FlEventChannel* channel,
                                                        FlValue* args,
                                                        gpointer user_data);

/**
 * fl_event_channel_new:
 * @messenger: an #FlBinaryMessenger.
 * @name: a channel name.
 * @codec: the message codec.
 *
 * Creates an event channel. @codec must match the codec used on the Dart
 * end of the channel.
 *
 * Returns: a new #FlEventChannel.
 */
FlEventChannel* fl_event_channel_new(FlBinaryMessenger* messenger,
                                     const gchar* name,
                                     FlMethodCodec* codec);

/**
 * fl_event_channel_set_stream_handlers:
 * @channel: an #FlEventChannel.
 * @listen_handler: (allow-none): function to call when the Dart side of the
 * channel starts listening to the stream.
 * @cancel_handler: (allow-none): function to call when the Dart side of the
 * channel cancels their subscription to the stream.
 * @user_data: (closure): user data to pass to @listen_handler and
 * @cancel_handler.
 * @destroy_notify: (allow-none): a function which gets called to free
 * @user_data, or %NULL.
 *
 * Sets the functions called when the Dart side requests the stream to start and
 * finish.
 *
 * The handlers are removed if the channel is closed or is replaced by another
 * handler, set @destroy_notify if you want to detect this.
 */
void fl_event_channel_set_stream_handlers(FlEventChannel* channel,
                                          FlEventChannelHandler listen_handler,
                                          FlEventChannelHandler cancel_handler,
                                          gpointer user_data,
                                          GDestroyNotify destroy_notify);

/**
 * fl_event_channel_send:
 * @channel: an #FlEventChannel.
 * @event: event to send, must match what the #FlMethodCodec supports.
 * @cancellable: (allow-none): a #GCancellable or %NULL.
 * @error: (allow-none): #GError location to store the error occurring, or %NULL
 * to ignore.
 *
 * Sends an event on the channel.
 * Events should only be sent once the channel is being listened to.
 *
 * Returns: %TRUE if successful.
 */
gboolean fl_event_channel_send(FlEventChannel* channel,
                               FlValue* event,
                               GCancellable* cancellable,
                               GError** error);

/**
 * fl_event_channel_send_error:
 * @channel: an #FlEventChannel.
 * @code: error code to send.
 * @message: error message to send.
 * @details: (allow-none): error details or %NULL.
 * @cancellable: (allow-none): a #GCancellable or %NULL.
 * @error: (allow-none): #GError location to store the error occurring, or %NULL
 * to ignore.
 *
 * Sends an error on the channel.
 * Errors should only be sent once the channel is being listened to.
 *
 * Returns: %TRUE if successful.
 */
gboolean fl_event_channel_send_error(FlEventChannel* channel,
                                     const gchar* code,
                                     const gchar* message,
                                     FlValue* details,
                                     GCancellable* cancellable,
                                     GError** error);

/**
 * fl_event_channel_send_end_of_stream:
 * @channel: an #FlEventChannel.
 * @cancellable: (allow-none): a #GCancellable or %NULL.
 * @error: (allow-none): #GError location to store the error occurring, or %NULL
 * to ignore.
 *
 * Indicates the stream has completed.
 * It is a programmer error to send any more events after calling this.
 *
 * Returns: %TRUE if successful.
 */
gboolean fl_event_channel_send_end_of_stream(FlEventChannel* channel,
                                             GCancellable* cancellable,
                                             GError** error);

G_END_DECLS

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_PUBLIC_FLUTTER_LINUX_FL_EVENT_CHANNEL_H_
