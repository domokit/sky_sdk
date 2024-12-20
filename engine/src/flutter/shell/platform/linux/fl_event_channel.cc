// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/public/flutter_linux/fl_event_channel.h"

#include <gmodule.h>

#include "flutter/shell/platform/linux/fl_method_codec_private.h"

static constexpr char kListenMethod[] = "listen";
static constexpr char kCancelMethod[] = "cancel";
static constexpr char kEventRequestError[] = "error";

struct _FlEventChannel {
  GObject parent_instance;

  // Messenger to communicate on.
  FlBinaryMessenger* messenger;

  // TRUE if the channel has been closed.
  gboolean channel_closed;

  // Channel name.
  gchar* name;

  // Codec to en/decode messages.
  FlMethodCodec* codec;

  // Function called when the stream is listened to / cancelled.
  FlEventChannelHandler listen_handler;
  FlEventChannelHandler cancel_handler;
  gpointer handler_data;
  GDestroyNotify handler_data_destroy_notify;
};

struct _FlEventChannelResponseHandle {
  GObject parent_instance;

  FlBinaryMessengerResponseHandle* response_handle;
};

G_DEFINE_TYPE(FlEventChannel, fl_event_channel, G_TYPE_OBJECT)

// Handle method calls from the Dart side of the channel.
static FlMethodErrorResponse* handle_method_call(FlEventChannel* self,
                                                 const gchar* name,
                                                 FlValue* args) {
  FlEventChannelHandler handler;
  if (g_strcmp0(name, kListenMethod) == 0) {
    handler = self->listen_handler;
  } else if (g_strcmp0(name, kCancelMethod) == 0) {
    handler = self->cancel_handler;
  } else {
    g_autofree gchar* message =
        g_strdup_printf("Unknown event channel request '%s'", name);
    return fl_method_error_response_new(kEventRequestError, message, nullptr);
  }

  // If not handled, just accept requests.
  if (handler == nullptr) {
    return nullptr;
  }

  return handler(self, args, self->handler_data);
}

// Called when a binary message is received on this channel.
static void message_cb(FlBinaryMessenger* messenger,
                       const gchar* channel,
                       GBytes* message,
                       FlBinaryMessengerResponseHandle* response_handle,
                       gpointer user_data) {
  FlEventChannel* self = FL_EVENT_CHANNEL(user_data);

  g_autofree gchar* name = nullptr;
  g_autoptr(GError) error = nullptr;
  g_autoptr(FlValue) args = nullptr;
  if (!fl_method_codec_decode_method_call(self->codec, message, &name, &args,
                                          &error)) {
    g_warning("Failed to decode message on event channel %s: %s", self->name,
              error->message);
    fl_binary_messenger_send_response(messenger, response_handle, nullptr,
                                      nullptr);
    return;
  }

  g_autoptr(FlMethodErrorResponse) response =
      handle_method_call(self, name, args);

  g_autoptr(GBytes) data = nullptr;
  if (response == nullptr) {
    g_autoptr(GError) codec_error = nullptr;
    data = fl_method_codec_encode_success_envelope(self->codec, nullptr,
                                                   &codec_error);
    if (data == nullptr) {
      g_warning("Failed to encode event channel %s success response: %s",
                self->name, codec_error->message);
    }
  } else {
    g_autoptr(GError) codec_error = nullptr;
    data = fl_method_codec_encode_error_envelope(
        self->codec, fl_method_error_response_get_code(response),
        fl_method_error_response_get_message(response),
        fl_method_error_response_get_details(response), &codec_error);
    if (data == nullptr) {
      g_warning("Failed to encode event channel %s error response: %s",
                self->name, codec_error->message);
    }
  }

  if (!fl_binary_messenger_send_response(messenger, response_handle, data,
                                         &error)) {
    g_warning("Failed to send event channel response: %s", error->message);
  }
}

// Removes handlers and their associated data.
static void remove_handlers(FlEventChannel* self) {
  if (self->handler_data_destroy_notify != nullptr) {
    self->handler_data_destroy_notify(self->handler_data);
  }
  self->listen_handler = nullptr;
  self->cancel_handler = nullptr;
  self->handler_data = nullptr;
  self->handler_data_destroy_notify = nullptr;
}

// Called when the channel handler is closed.
static void channel_closed_cb(gpointer user_data) {
  g_autoptr(FlEventChannel) self = FL_EVENT_CHANNEL(user_data);
  self->channel_closed = TRUE;
  remove_handlers(self);
}

static void fl_event_channel_dispose(GObject* object) {
  FlEventChannel* self = FL_EVENT_CHANNEL(object);

  if (!self->channel_closed) {
    fl_binary_messenger_set_message_handler_on_channel(
        self->messenger, self->name, nullptr, nullptr, nullptr);
  }

  g_clear_object(&self->messenger);
  g_clear_pointer(&self->name, g_free);
  g_clear_object(&self->codec);

  remove_handlers(self);

  G_OBJECT_CLASS(fl_event_channel_parent_class)->dispose(object);
}

static void fl_event_channel_class_init(FlEventChannelClass* klass) {
  G_OBJECT_CLASS(klass)->dispose = fl_event_channel_dispose;
}

static void fl_event_channel_init(FlEventChannel* self) {}

G_MODULE_EXPORT FlEventChannel* fl_event_channel_new(
    FlBinaryMessenger* messenger,
    const gchar* name,
    FlMethodCodec* codec) {
  g_return_val_if_fail(FL_IS_BINARY_MESSENGER(messenger), nullptr);
  g_return_val_if_fail(name != nullptr, nullptr);
  g_return_val_if_fail(FL_IS_METHOD_CODEC(codec), nullptr);

  FlEventChannel* self =
      FL_EVENT_CHANNEL(g_object_new(fl_event_channel_get_type(), nullptr));

  self->messenger = FL_BINARY_MESSENGER(g_object_ref(messenger));
  self->name = g_strdup(name);
  self->codec = FL_METHOD_CODEC(g_object_ref(codec));

  fl_binary_messenger_set_message_handler_on_channel(
      self->messenger, self->name, message_cb, g_object_ref(self),
      channel_closed_cb);

  return self;
}

G_MODULE_EXPORT void fl_event_channel_set_stream_handlers(
    FlEventChannel* self,
    FlEventChannelHandler listen_handler,
    FlEventChannelHandler cancel_handler,
    gpointer user_data,
    GDestroyNotify destroy_notify) {
  g_return_if_fail(FL_IS_EVENT_CHANNEL(self));

  remove_handlers(self);
  self->listen_handler = listen_handler;
  self->cancel_handler = cancel_handler;
  self->handler_data = user_data;
  self->handler_data_destroy_notify = destroy_notify;
}

G_MODULE_EXPORT gboolean fl_event_channel_send(FlEventChannel* self,
                                               FlValue* event,
                                               GCancellable* cancellable,
                                               GError** error) {
  g_return_val_if_fail(FL_IS_EVENT_CHANNEL(self), FALSE);
  g_return_val_if_fail(event != nullptr, FALSE);

  g_autoptr(GBytes) data =
      fl_method_codec_encode_success_envelope(self->codec, event, error);
  if (data == nullptr) {
    return FALSE;
  }

  fl_binary_messenger_send_on_channel(self->messenger, self->name, data,
                                      cancellable, nullptr, nullptr);

  return TRUE;
}

G_MODULE_EXPORT gboolean fl_event_channel_send_error(FlEventChannel* self,
                                                     const gchar* code,
                                                     const gchar* message,
                                                     FlValue* details,
                                                     GCancellable* cancellable,
                                                     GError** error) {
  g_return_val_if_fail(FL_IS_EVENT_CHANNEL(self), FALSE);
  g_return_val_if_fail(code != nullptr, FALSE);
  g_return_val_if_fail(message != nullptr, FALSE);

  g_autoptr(GBytes) data = fl_method_codec_encode_error_envelope(
      self->codec, code, message, details, error);
  if (data == nullptr) {
    return FALSE;
  }

  fl_binary_messenger_send_on_channel(self->messenger, self->name, data,
                                      cancellable, nullptr, nullptr);

  return TRUE;
}

G_MODULE_EXPORT gboolean fl_event_channel_send_end_of_stream(
    FlEventChannel* self,
    GCancellable* cancellable,
    GError** error) {
  g_return_val_if_fail(FL_IS_EVENT_CHANNEL(self), FALSE);
  fl_binary_messenger_send_on_channel(self->messenger, self->name, nullptr,
                                      cancellable, nullptr, nullptr);
  return TRUE;
}
