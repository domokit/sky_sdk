// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_PUBLIC_FLUTTER_LINUX_FL_METHOD_CHANNEL_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_PUBLIC_FLUTTER_LINUX_FL_METHOD_CHANNEL_H_

#if !defined(__FLUTTER_LINUX_INSIDE__) && !defined(FLUTTER_LINUX_COMPILATION)
#error "Only <flutter_linux/flutter_linux.h> can be included directly."
#endif

#include <gio/gio.h>
#include <glib-object.h>
#include <gmodule.h>

#include "fl_binary_messenger.h"
#include "fl_method_call.h"
#include "fl_method_codec.h"
#include "fl_method_response.h"

G_BEGIN_DECLS

G_MODULE_EXPORT
G_DECLARE_FINAL_TYPE(FlMethodChannel,
                     fl_method_channel,
                     FL,
                     METHOD_CHANNEL,
                     GObject)

/**
 * FlMethodChannel:
 *
 * #FlMethodChannel is an object that allows method calls to and from Dart code.
 *
 * The following example shows how to call and handle methods on a channel.
 * See #FlMethodResponse for how to handle errors in more detail.
 *
 * |[<!-- language="C" -->
 * static FlMethodChannel *channel = NULL;
 *
 * static void method_call_cb (FlMethodChannel* channel,
 *                             FlMethodCall* method_call,
 *                             gpointer user_data) {
 *   g_autoptr(FlMethodResponse) response = NULL;
 *   if (strcmp (fl_method_call_get_name (method_call), "Foo.bar") == 0) {
 *     g_autoptr(GError) bar_error = NULL;
 *     g_autoptr(FlValue) result =
 *         do_bar (fl_method_call_get_args (method_call), &bar_error);
 *     if (result == NULL) {
 *       response =
 *         FL_METHOD_RESPONSE (fl_method_error_response_new ("bar error",
 *                                                           bar_error->message,
 *                                                           nullptr);
 *     } else {
 *       response =
 *         FL_METHOD_RESPONSE (fl_method_success_response_new (result));
 *     }
 *   } else {
 *     response =
 *       FL_METHOD_RESPONSE (fl_method_not_implemented_response_new ());
 *   }
 *
 *   g_autoptr(GError) error = NULL;
 *   if (!fl_method_call_respond(method_call, response, &error))
 *     g_warning ("Failed to send response: %s", error->message);
 * }
 *
 * static void method_response_cb(GObject *object,
 *                                GAsyncResult *result,
 *                                gpointer user_data) {
 *   g_autoptr(GError) error = NULL;
 *   g_autoptr(FlMethodResponse) response =
 *     fl_method_channel_invoke_method_finish (FL_METHOD_CODEC (object), result,
 *                                             &error);
 *   if (response == NULL) {
 *     g_warning ("Failed to call method: %s", error->message);
 *     return;
 *   }
 *
 *   g_autoptr(FlValue) value =
 *     fl_method_response_get_result (response, &error);
 *   if (response == NULL) {
 *     g_warning ("Method returned error: %s", error->message);
 *     return;
 *   }
 *
 *   use_result (value);
 * }
 *
 * static void call_method () {
 *   g_autoptr(FlStandardMethodCodec) codec = fl_standard_method_codec_new ();
 *   channel =
 *     fl_method_channel_new(messenger, "flutter/foo", FL_METHOD_CODEC (codec));
 *   fl_method_channel_set_method_call_handler (channel, method_call_cb, NULL,
 * NULL);
 *
 *   g_autoptr(FlValue) args = fl_value_new_string ("Hello World");
 *   fl_method_channel_invoke_method (channel, "Foo.foo", args,
 *                                    cancellable, method_response_cb, NULL);
 * }
 * ]|
 *
 * #FlMethodChannel matches the MethodChannel class in the Flutter services
 * library.
 */

/**
 * FlMethodChannelMethodCallHandler:
 * @channel: an #FlMethodChannel.
 * @method_call: an #FlMethodCall.
 * @user_data: (closure): data provided when registering this handler.
 *
 * Function called when a method call is received. Respond to the method call
 * with fl_method_call_respond(). If the response is not occurring in this
 * callback take a reference to @method_call and release that once it has been
 * responded to. Failing to respond before the last reference to @method_call is
 * dropped is a programming error.
 */
typedef void (*FlMethodChannelMethodCallHandler)(FlMethodChannel* channel,
                                                 FlMethodCall* method_call,
                                                 gpointer user_data);

/**
 * fl_method_channel_new:
 * @messenger: an #FlBinaryMessenger.
 * @name: a channel name.
 * @codec: the method codec.
 *
 * Creates a new method channel. @codec must match the codec used on the Dart
 * end of the channel.
 *
 * Returns: a new #FlMethodChannel.
 */
FlMethodChannel* fl_method_channel_new(FlBinaryMessenger* messenger,
                                       const gchar* name,
                                       FlMethodCodec* codec);

/**
 * fl_method_channel_set_method_call_handler:
 * @channel: an #FlMethodChannel.
 * @handler: function to call when a method call is received on this channel.
 * @user_data: (closure): user data to pass to @handler.
 * @destroy_notify: (allow-none): a function which gets called to free
 * @user_data, or %NULL.
 *
 * Sets the function called when a method call is received from the Dart side of
 * the channel. See #FlMethodChannelMethodCallHandler for details on how to
 * respond to method calls.
 *
 * The handler is removed if the channel is closed or is replaced by another
 * handler, set @destroy_notify if you want to detect this.
 */
void fl_method_channel_set_method_call_handler(
    FlMethodChannel* channel,
    FlMethodChannelMethodCallHandler handler,
    gpointer user_data,
    GDestroyNotify destroy_notify);

/**
 * fl_method_channel_invoke_method:
 * @channel: an #FlMethodChannel.
 * @method: the method to call.
 * @args: (allow-none): arguments to the method, must match what the
 * #FlMethodCodec supports.
 * @cancellable: (allow-none): a #GCancellable or %NULL.
 * @callback: (scope async): (allow-none): a #GAsyncReadyCallback to call when
 * the request is satisfied or %NULL to ignore the response.
 * @user_data: (closure): user data to pass to @callback.
 *
 * Calls a method on this channel.
 */
void fl_method_channel_invoke_method(FlMethodChannel* channel,
                                     const gchar* method,
                                     FlValue* args,
                                     GCancellable* cancellable,
                                     GAsyncReadyCallback callback,
                                     gpointer user_data);

/**
 * fl_method_channel_invoke_method_finish:
 * @channel: an #FlMethodChannel.
 * @result:  #GAsyncResult.
 * @error: (allow-none): #GError location to store the error occurring, or %NULL
 * to ignore.
 *
 * Completes request started with fl_method_channel_invoke_method().
 *
 * Returns: (transfer full): an #FlMethodResponse or %NULL on error.
 */
FlMethodResponse* fl_method_channel_invoke_method_finish(
    FlMethodChannel* channel,
    GAsyncResult* result,
    GError** error);

G_END_DECLS

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_PUBLIC_FLUTTER_LINUX_FL_METHOD_CHANNEL_H_
