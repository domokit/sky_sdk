// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/public/flutter_linux/fl_method_channel.h"

#include <gmodule.h>

#include "flutter/shell/platform/linux/fl_method_call_private.h"
#include "flutter/shell/platform/linux/fl_method_channel_private.h"
#include "flutter/shell/platform/linux/fl_method_codec_private.h"

struct _FlMethodChannel {
  GObject parent_instance;

  // Messenger to communicate on.
  FlBinaryMessenger* messenger;

  // TRUE if the channel has been closed.
  gboolean channel_closed;

  // Channel name.
  gchar* name;

  // Codec to en/decode messages.
  FlMethodCodec* codec;

  // Function called when a method call is received.
  FlMethodChannelMethodCallHandler method_call_handler;
  gpointer method_call_handler_data;
  GDestroyNotify method_call_handler_destroy_notify;
};

G_DEFINE_TYPE(FlMethodChannel, fl_method_channel, G_TYPE_OBJECT)

// Called when a binary message is received on this channel.
static void message_cb(FlBinaryMessenger* messenger,
                       const gchar* channel,
                       GBytes* message,
                       FlBinaryMessengerResponseHandle* response_handle,
                       gpointer user_data) {
  FlMethodChannel* self = FL_METHOD_CHANNEL(user_data);

  if (self->method_call_handler == nullptr) {
    return;
  }

  g_autofree gchar* method = nullptr;
  g_autoptr(FlValue) args = nullptr;
  g_autoptr(GError) error = nullptr;
  if (!fl_method_codec_decode_method_call(self->codec, message, &method, &args,
                                          &error)) {
    g_warning("Failed to decode method call: %s", error->message);
    return;
  }

  g_autoptr(FlMethodCall) method_call =
      fl_method_call_new(method, args, self, response_handle);
  self->method_call_handler(self, method_call, self->method_call_handler_data);
}

// Called when a response is received to a sent message.
static void message_response_cb(GObject* object,
                                GAsyncResult* result,
                                gpointer user_data) {
  g_autoptr(GTask) task = G_TASK(user_data);
  g_task_return_pointer(task, g_object_ref(result), g_object_unref);
}

// Called when the channel handler is closed.
static void channel_closed_cb(gpointer user_data) {
  g_autoptr(FlMethodChannel) self = FL_METHOD_CHANNEL(user_data);

  self->channel_closed = TRUE;

  // Disconnect handler.
  if (self->method_call_handler_destroy_notify != nullptr) {
    self->method_call_handler_destroy_notify(self->method_call_handler_data);
  }
  self->method_call_handler = nullptr;
  self->method_call_handler_data = nullptr;
  self->method_call_handler_destroy_notify = nullptr;
}

static void fl_method_channel_dispose(GObject* object) {
  FlMethodChannel* self = FL_METHOD_CHANNEL(object);

  // Note we don't have to clear the handler in messenger as it holds
  // a reference to this object so the following code is only run after
  // the messenger has closed the channel already.

  g_clear_object(&self->messenger);
  g_clear_pointer(&self->name, g_free);
  g_clear_object(&self->codec);

  if (self->method_call_handler_destroy_notify != nullptr) {
    self->method_call_handler_destroy_notify(self->method_call_handler_data);
  }
  self->method_call_handler = nullptr;
  self->method_call_handler_data = nullptr;
  self->method_call_handler_destroy_notify = nullptr;

  G_OBJECT_CLASS(fl_method_channel_parent_class)->dispose(object);
}

static void fl_method_channel_class_init(FlMethodChannelClass* klass) {
  G_OBJECT_CLASS(klass)->dispose = fl_method_channel_dispose;
}

static void fl_method_channel_init(FlMethodChannel* self) {}

G_MODULE_EXPORT FlMethodChannel* fl_method_channel_new(
    FlBinaryMessenger* messenger,
    const gchar* name,
    FlMethodCodec* codec) {
  g_return_val_if_fail(FL_IS_BINARY_MESSENGER(messenger), nullptr);
  g_return_val_if_fail(name != nullptr, nullptr);
  g_return_val_if_fail(FL_IS_METHOD_CODEC(codec), nullptr);

  FlMethodChannel* self =
      FL_METHOD_CHANNEL(g_object_new(fl_method_channel_get_type(), nullptr));

  self->messenger = FL_BINARY_MESSENGER(g_object_ref(messenger));
  self->name = g_strdup(name);
  self->codec = FL_METHOD_CODEC(g_object_ref(codec));

  fl_binary_messenger_set_message_handler_on_channel(
      self->messenger, self->name, message_cb, g_object_ref(self),
      channel_closed_cb);

  return self;
}

G_MODULE_EXPORT void fl_method_channel_set_method_call_handler(
    FlMethodChannel* self,
    FlMethodChannelMethodCallHandler handler,
    gpointer user_data,
    GDestroyNotify destroy_notify) {
  g_return_if_fail(FL_IS_METHOD_CHANNEL(self));

  // Don't set handler if channel closed.
  if (self->channel_closed) {
    if (handler != nullptr) {
      g_warning(
          "Attempted to set method call handler on a closed FlMethodChannel");
    }
    if (destroy_notify != nullptr) {
      destroy_notify(user_data);
    }
    return;
  }

  if (self->method_call_handler_destroy_notify != nullptr) {
    self->method_call_handler_destroy_notify(self->method_call_handler_data);
  }

  self->method_call_handler = handler;
  self->method_call_handler_data = user_data;
  self->method_call_handler_destroy_notify = destroy_notify;
}

G_MODULE_EXPORT void fl_method_channel_invoke_method(
    FlMethodChannel* self,
    const gchar* method,
    FlValue* args,
    GCancellable* cancellable,
    GAsyncReadyCallback callback,
    gpointer user_data) {
  g_return_if_fail(FL_IS_METHOD_CHANNEL(self));
  g_return_if_fail(method != nullptr);

  g_autoptr(GTask) task =
      callback != nullptr ? g_task_new(self, cancellable, callback, user_data)
                          : nullptr;

  g_autoptr(GError) error = nullptr;
  g_autoptr(GBytes) message =
      fl_method_codec_encode_method_call(self->codec, method, args, &error);
  if (message == nullptr) {
    if (task != nullptr) {
      g_task_return_error(task, g_error_copy(error));
    }
    return;
  }

  fl_binary_messenger_send_on_channel(
      self->messenger, self->name, message, cancellable,
      callback != nullptr ? message_response_cb : nullptr,
      g_steal_pointer(&task));
}

G_MODULE_EXPORT FlMethodResponse* fl_method_channel_invoke_method_finish(
    FlMethodChannel* self,
    GAsyncResult* result,
    GError** error) {
  g_return_val_if_fail(FL_IS_METHOD_CHANNEL(self), nullptr);
  g_return_val_if_fail(g_task_is_valid(result, self), nullptr);

  GTask* task = G_TASK(result);
  g_autoptr(GAsyncResult) r =
      G_ASYNC_RESULT(g_task_propagate_pointer(task, error));
  if (r == nullptr) {
    return nullptr;
  }

  g_autoptr(GBytes) response =
      fl_binary_messenger_send_on_channel_finish(self->messenger, r, error);
  if (response == nullptr) {
    return nullptr;
  }

  return fl_method_codec_decode_response(self->codec, response, error);
}

gboolean fl_method_channel_respond(
    FlMethodChannel* self,
    FlBinaryMessengerResponseHandle* response_handle,
    FlMethodResponse* response,
    GError** error) {
  g_return_val_if_fail(FL_IS_METHOD_CHANNEL(self), FALSE);
  g_return_val_if_fail(FL_IS_BINARY_MESSENGER_RESPONSE_HANDLE(response_handle),
                       FALSE);
  g_return_val_if_fail(FL_IS_METHOD_RESPONSE(response), FALSE);

  g_autoptr(GBytes) message =
      fl_method_codec_encode_response(self->codec, response, error);
  if (message == nullptr) {
    return FALSE;
  }
  return fl_binary_messenger_send_response(self->messenger, response_handle,
                                           message, error);
}
