// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/testing/fl_mock_binary_messenger.h"

#include "flutter/shell/platform/linux/fl_binary_messenger_private.h"
#include "flutter/shell/platform/linux/fl_method_codec_private.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_json_message_codec.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_json_method_codec.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_standard_message_codec.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_standard_method_codec.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_string_codec.h"

G_DECLARE_FINAL_TYPE(FlMockBinaryMessengerResponseHandle,
                     fl_mock_binary_messenger_response_handle,
                     FL,
                     MOCK_BINARY_MESSENGER_RESPONSE_HANDLE,
                     FlBinaryMessengerResponseHandle)

struct _FlMockBinaryMessengerResponseHandle {
  FlBinaryMessengerResponseHandle parent_instance;

  FlMockBinaryMessengerCallback callback;
  gpointer user_data;
};

G_DEFINE_TYPE(FlMockBinaryMessengerResponseHandle,
              fl_mock_binary_messenger_response_handle,
              fl_binary_messenger_response_handle_get_type())

static void fl_mock_binary_messenger_response_handle_class_init(
    FlMockBinaryMessengerResponseHandleClass* klass) {}

static void fl_mock_binary_messenger_response_handle_init(
    FlMockBinaryMessengerResponseHandle* self) {}

FlMockBinaryMessengerResponseHandle*
fl_mock_binary_messenger_response_handle_new(
    FlMockBinaryMessengerCallback callback,
    gpointer user_data) {
  FlMockBinaryMessengerResponseHandle* self =
      FL_MOCK_BINARY_MESSENGER_RESPONSE_HANDLE(g_object_new(
          fl_mock_binary_messenger_response_handle_get_type(), nullptr));
  self->callback = callback;
  self->user_data = user_data;
  return self;
}

struct _FlMockBinaryMessenger {
  GObject parent_instance;

  // Handlers the embedder has registered.
  GHashTable* handlers;

  // Mocked Dart channels.
  GHashTable* mock_channels;
  GHashTable* mock_message_channels;
  GHashTable* mock_method_channels;
  GHashTable* mock_event_channels;
  GHashTable* mock_error_channels;
};

typedef struct {
  FlMockBinaryMessengerChannelHandler callback;
  gpointer user_data;
} MockChannel;

static MockChannel* mock_channel_new(
    FlMockBinaryMessengerChannelHandler callback,
    gpointer user_data) {
  MockChannel* channel = g_new0(MockChannel, 1);
  channel->callback = callback;
  channel->user_data = user_data;
  return channel;
}

static void mock_channel_free(MockChannel* channel) {
  g_free(channel);
}

typedef struct {
  FlMessageCodec* codec;
  FlMockBinaryMessengerMessageChannelHandler callback;
  gpointer user_data;
} MockMessageChannel;

static MockMessageChannel* mock_message_channel_new(
    FlMockBinaryMessengerMessageChannelHandler callback,
    FlMessageCodec* codec,
    gpointer user_data) {
  MockMessageChannel* channel = g_new0(MockMessageChannel, 1);
  channel->codec = FL_MESSAGE_CODEC(g_object_ref(codec));
  channel->callback = callback;
  channel->user_data = user_data;
  return channel;
}

static void mock_message_channel_free(MockMessageChannel* channel) {
  g_object_unref(channel->codec);
  g_free(channel);
}

typedef struct {
  FlMethodCodec* codec;
  FlMockBinaryMessengerMethodChannelHandler callback;
  gpointer user_data;
} MockMethodChannel;

static MockMethodChannel* mock_method_channel_new(
    FlMockBinaryMessengerMethodChannelHandler callback,
    FlMethodCodec* codec,
    gpointer user_data) {
  MockMethodChannel* channel = g_new0(MockMethodChannel, 1);
  channel->codec = FL_METHOD_CODEC(g_object_ref(codec));
  channel->callback = callback;
  channel->user_data = user_data;
  return channel;
}

static void mock_method_channel_free(MockMethodChannel* channel) {
  g_object_unref(channel->codec);
  g_free(channel);
}

typedef struct {
  FlMethodCodec* codec;
  FlMockBinaryMessengerEventChannelHandler callback;
  FlMockBinaryMessengerEventChannelErrorHandler error_callback;
  gpointer user_data;
} MockEventChannel;

static MockEventChannel* mock_event_channel_new(
    FlMockBinaryMessengerEventChannelHandler callback,
    FlMockBinaryMessengerEventChannelErrorHandler error_callback,
    FlMethodCodec* codec,
    gpointer user_data) {
  MockEventChannel* channel = g_new0(MockEventChannel, 1);
  channel->codec = FL_METHOD_CODEC(g_object_ref(codec));
  channel->callback = callback;
  channel->error_callback = error_callback;
  channel->user_data = user_data;
  return channel;
}

static void mock_event_channel_free(MockEventChannel* channel) {
  g_object_unref(channel->codec);
  g_free(channel);
}

typedef struct {
  gint code;
  gchar* message;
} MockErrorChannel;

static MockErrorChannel* mock_error_channel_new(gint code,
                                                const gchar* message) {
  MockErrorChannel* channel = g_new0(MockErrorChannel, 1);
  channel->code = code;
  channel->message = g_strdup(message);
  return channel;
}

static void mock_error_channel_free(MockErrorChannel* channel) {
  g_free(channel->message);
  g_free(channel);
}

typedef struct {
  FlBinaryMessengerMessageHandler callback;
  gpointer user_data;
  GDestroyNotify destroy_notify;
} Handler;

static Handler* handler_new(FlBinaryMessengerMessageHandler callback,
                            gpointer user_data,
                            GDestroyNotify destroy_notify) {
  Handler* handler = g_new0(Handler, 1);
  handler->callback = callback;
  handler->user_data = user_data;
  handler->destroy_notify = destroy_notify;
  return handler;
}

static void handler_free(Handler* handler) {
  if (handler->destroy_notify) {
    handler->destroy_notify(handler->user_data);
  }
  g_free(handler);
}

static void fl_mock_binary_messenger_iface_init(
    FlBinaryMessengerInterface* iface);

G_DEFINE_TYPE_WITH_CODE(
    FlMockBinaryMessenger,
    fl_mock_binary_messenger,
    G_TYPE_OBJECT,
    G_IMPLEMENT_INTERFACE(fl_binary_messenger_get_type(),
                          fl_mock_binary_messenger_iface_init))

static void fl_mock_binary_messenger_set_message_handler_on_channel(
    FlBinaryMessenger* messenger,
    const gchar* channel,
    FlBinaryMessengerMessageHandler handler,
    gpointer user_data,
    GDestroyNotify destroy_notify) {
  FlMockBinaryMessenger* self = FL_MOCK_BINARY_MESSENGER(messenger);
  g_hash_table_insert(self->handlers, g_strdup(channel),
                      handler_new(handler, user_data, destroy_notify));
}

static gboolean fl_mock_binary_messenger_send_response(
    FlBinaryMessenger* messenger,
    FlBinaryMessengerResponseHandle* response_handle,
    GBytes* response,
    GError** error) {
  FlMockBinaryMessenger* self = FL_MOCK_BINARY_MESSENGER(messenger);

  g_return_val_if_fail(
      FL_IS_MOCK_BINARY_MESSENGER_RESPONSE_HANDLE(response_handle), FALSE);
  FlMockBinaryMessengerResponseHandle* handle =
      FL_MOCK_BINARY_MESSENGER_RESPONSE_HANDLE(response_handle);

  handle->callback(self, response, handle->user_data);

  return TRUE;
}

static void fl_mock_binary_messenger_send_on_channel(
    FlBinaryMessenger* messenger,
    const gchar* channel,
    GBytes* message,
    GCancellable* cancellable,
    GAsyncReadyCallback callback,
    gpointer user_data) {
  FlMockBinaryMessenger* self = FL_MOCK_BINARY_MESSENGER(messenger);
  g_autoptr(GTask) task = g_task_new(self, cancellable, callback, user_data);

  MockChannel* mock_channel = static_cast<MockChannel*>(
      g_hash_table_lookup(self->mock_channels, channel));
  MockMessageChannel* mock_message_channel = static_cast<MockMessageChannel*>(
      g_hash_table_lookup(self->mock_message_channels, channel));
  MockMethodChannel* mock_method_channel = static_cast<MockMethodChannel*>(
      g_hash_table_lookup(self->mock_method_channels, channel));
  MockEventChannel* mock_event_channel = static_cast<MockEventChannel*>(
      g_hash_table_lookup(self->mock_event_channels, channel));
  MockErrorChannel* mock_error_channel = static_cast<MockErrorChannel*>(
      g_hash_table_lookup(self->mock_error_channels, channel));
  g_autoptr(GBytes) response = nullptr;
  if (mock_channel != nullptr) {
    response = mock_channel->callback(self, message, mock_channel->user_data);
  } else if (mock_message_channel != nullptr) {
    g_autoptr(GError) error = nullptr;
    g_autoptr(FlValue) message_value = fl_message_codec_decode_message(
        mock_message_channel->codec, message, &error);
    if (message_value == nullptr) {
      g_warning("Failed to decode message: %s", error->message);
    } else {
      g_autoptr(FlValue) response_value = mock_message_channel->callback(
          self, message_value, mock_message_channel->user_data);
      response = fl_message_codec_encode_message(mock_message_channel->codec,
                                                 response_value, &error);
      if (response == nullptr) {
        g_warning("Failed to encode message: %s", error->message);
      }
    }
  } else if (mock_method_channel != nullptr) {
    g_autofree gchar* name = nullptr;
    g_autoptr(FlValue) args = nullptr;
    g_autoptr(GError) error = nullptr;
    if (!fl_method_codec_decode_method_call(mock_method_channel->codec, message,
                                            &name, &args, &error)) {
      g_warning("Failed to decode method call: %s", error->message);
    } else {
      g_autoptr(FlMethodResponse) response_value =
          mock_method_channel->callback(self, name, args,
                                        mock_method_channel->user_data);
      response = fl_method_codec_encode_response(mock_method_channel->codec,
                                                 response_value, &error);
      if (response == nullptr) {
        g_warning("Failed to encode method response: %s", error->message);
      }
    }
  } else if (mock_event_channel != nullptr) {
    g_autoptr(GError) error = nullptr;
    g_autoptr(FlMethodResponse) response = fl_method_codec_decode_response(
        mock_event_channel->codec, message, &error);
    if (response == nullptr) {
      g_warning("Failed to decode event response: %s", error->message);
    } else if (FL_IS_METHOD_SUCCESS_RESPONSE(response)) {
      mock_event_channel->callback(self,
                                   fl_method_success_response_get_result(
                                       FL_METHOD_SUCCESS_RESPONSE(response)),
                                   mock_event_channel->user_data);
    } else if (FL_IS_METHOD_ERROR_RESPONSE(response)) {
      mock_event_channel->error_callback(
          self,
          fl_method_error_response_get_code(FL_METHOD_ERROR_RESPONSE(response)),
          fl_method_error_response_get_message(
              FL_METHOD_ERROR_RESPONSE(response)),
          fl_method_error_response_get_details(
              FL_METHOD_ERROR_RESPONSE(response)),
          mock_event_channel->user_data);
    } else {
      g_warning("Unknown event response");
    }
  } else if (mock_error_channel != nullptr) {
    g_task_return_new_error(task, fl_binary_messenger_codec_error_quark(),
                            mock_error_channel->code, "%s",
                            mock_error_channel->message);
    return;
  }

  if (response != nullptr) {
    g_task_return_pointer(task, g_bytes_ref(response),
                          reinterpret_cast<GDestroyNotify>(g_bytes_unref));
  }
}

static GBytes* fl_mock_binary_messenger_send_on_channel_finish(
    FlBinaryMessenger* messenger,
    GAsyncResult* result,
    GError** error) {
  return static_cast<GBytes*>(g_task_propagate_pointer(G_TASK(result), error));
}

static void fl_mock_binary_messenger_resize_channel(
    FlBinaryMessenger* messenger,
    const gchar* channel,
    int64_t new_size) {}

static void fl_mock_binary_messenger_set_warns_on_channel_overflow(
    FlBinaryMessenger* messenger,
    const gchar* channel,
    bool warns) {}

static void fl_mock_binary_messenger_shutdown(FlBinaryMessenger* messenger) {
  FlMockBinaryMessenger* self = FL_MOCK_BINARY_MESSENGER(messenger);
  g_hash_table_remove_all(self->handlers);
}

static void fl_mock_binary_messenger_dispose(GObject* object) {
  FlMockBinaryMessenger* self = FL_MOCK_BINARY_MESSENGER(object);

  g_clear_pointer(&self->mock_channels, g_hash_table_unref);
  g_clear_pointer(&self->mock_message_channels, g_hash_table_unref);
  g_clear_pointer(&self->mock_method_channels, g_hash_table_unref);
  g_clear_pointer(&self->mock_event_channels, g_hash_table_unref);
  g_clear_pointer(&self->mock_error_channels, g_hash_table_unref);

  G_OBJECT_CLASS(fl_mock_binary_messenger_parent_class)->dispose(object);
}

static void fl_mock_binary_messenger_class_init(
    FlMockBinaryMessengerClass* klass) {
  G_OBJECT_CLASS(klass)->dispose = fl_mock_binary_messenger_dispose;
}

static void fl_mock_binary_messenger_iface_init(
    FlBinaryMessengerInterface* iface) {
  iface->set_message_handler_on_channel =
      fl_mock_binary_messenger_set_message_handler_on_channel;
  iface->send_response = fl_mock_binary_messenger_send_response;
  iface->send_on_channel = fl_mock_binary_messenger_send_on_channel;
  iface->send_on_channel_finish =
      fl_mock_binary_messenger_send_on_channel_finish;
  iface->resize_channel = fl_mock_binary_messenger_resize_channel;
  iface->set_warns_on_channel_overflow =
      fl_mock_binary_messenger_set_warns_on_channel_overflow;
  iface->shutdown = fl_mock_binary_messenger_shutdown;
}

static void fl_mock_binary_messenger_init(FlMockBinaryMessenger* self) {
  self->handlers =
      g_hash_table_new_full(g_str_hash, g_str_equal, g_free,
                            reinterpret_cast<GDestroyNotify>(handler_free));

  self->mock_channels = g_hash_table_new_full(
      g_str_hash, g_str_equal, g_free,
      reinterpret_cast<GDestroyNotify>(mock_channel_free));
  self->mock_message_channels = g_hash_table_new_full(
      g_str_hash, g_str_equal, g_free,
      reinterpret_cast<GDestroyNotify>(mock_message_channel_free));
  self->mock_method_channels = g_hash_table_new_full(
      g_str_hash, g_str_equal, g_free,
      reinterpret_cast<GDestroyNotify>(mock_method_channel_free));
  self->mock_event_channels = g_hash_table_new_full(
      g_str_hash, g_str_equal, g_free,
      reinterpret_cast<GDestroyNotify>(mock_event_channel_free));
  self->mock_error_channels = g_hash_table_new_full(
      g_str_hash, g_str_equal, g_free,
      reinterpret_cast<GDestroyNotify>(mock_error_channel_free));
}

FlMockBinaryMessenger* fl_mock_binary_messenger_new() {
  FlMockBinaryMessenger* self = FL_MOCK_BINARY_MESSENGER(
      g_object_new(fl_mock_binary_messenger_get_type(), nullptr));
  return self;
}

gboolean fl_mock_binary_messenger_has_handler(FlMockBinaryMessenger* self,
                                              const gchar* channel) {
  g_return_val_if_fail(FL_IS_MOCK_BINARY_MESSENGER(self), FALSE);
  return g_hash_table_lookup(self->handlers, channel) != nullptr;
}

void fl_mock_binary_messenger_set_channel(
    FlMockBinaryMessenger* self,
    const gchar* channel,
    FlMockBinaryMessengerChannelHandler handler,
    gpointer user_data) {
  g_return_if_fail(FL_IS_MOCK_BINARY_MESSENGER(self));

  g_hash_table_insert(self->mock_channels, g_strdup(channel),
                      mock_channel_new(handler, user_data));
}

void fl_mock_binary_messenger_set_message_channel(
    FlMockBinaryMessenger* self,
    const gchar* channel,
    FlMessageCodec* codec,
    FlMockBinaryMessengerMessageChannelHandler handler,
    gpointer user_data) {
  g_return_if_fail(FL_IS_MOCK_BINARY_MESSENGER(self));

  g_hash_table_insert(self->mock_message_channels, g_strdup(channel),
                      mock_message_channel_new(handler, codec, user_data));
}

void fl_mock_binary_messenger_set_standard_message_channel(
    FlMockBinaryMessenger* self,
    const gchar* channel,
    FlMockBinaryMessengerMessageChannelHandler handler,
    gpointer user_data) {
  g_return_if_fail(FL_IS_MOCK_BINARY_MESSENGER(self));

  g_autoptr(FlStandardMessageCodec) codec = fl_standard_message_codec_new();
  return fl_mock_binary_messenger_set_message_channel(
      self, channel, FL_MESSAGE_CODEC(codec), handler, user_data);
}

void fl_mock_binary_messenger_set_string_message_channel(
    FlMockBinaryMessenger* self,
    const gchar* channel,
    FlMockBinaryMessengerMessageChannelHandler handler,
    gpointer user_data) {
  g_return_if_fail(FL_IS_MOCK_BINARY_MESSENGER(self));

  g_autoptr(FlStringCodec) codec = fl_string_codec_new();
  return fl_mock_binary_messenger_set_message_channel(
      self, channel, FL_MESSAGE_CODEC(codec), handler, user_data);
}

void fl_mock_binary_messenger_set_json_message_channel(
    FlMockBinaryMessenger* self,
    const gchar* channel,
    FlMockBinaryMessengerMessageChannelHandler handler,
    gpointer user_data) {
  g_return_if_fail(FL_IS_MOCK_BINARY_MESSENGER(self));

  g_autoptr(FlJsonMessageCodec) codec = fl_json_message_codec_new();
  return fl_mock_binary_messenger_set_message_channel(
      self, channel, FL_MESSAGE_CODEC(codec), handler, user_data);
}

void fl_mock_binary_messenger_set_method_channel(
    FlMockBinaryMessenger* self,
    const gchar* channel,
    FlMethodCodec* codec,
    FlMockBinaryMessengerMethodChannelHandler handler,
    gpointer user_data) {
  g_return_if_fail(FL_IS_MOCK_BINARY_MESSENGER(self));

  g_hash_table_insert(self->mock_method_channels, g_strdup(channel),
                      mock_method_channel_new(handler, codec, user_data));
}

void fl_mock_binary_messenger_set_standard_method_channel(
    FlMockBinaryMessenger* self,
    const gchar* channel,
    FlMockBinaryMessengerMethodChannelHandler handler,
    gpointer user_data) {
  g_return_if_fail(FL_IS_MOCK_BINARY_MESSENGER(self));
  g_autoptr(FlStandardMethodCodec) codec = fl_standard_method_codec_new();
  fl_mock_binary_messenger_set_method_channel(
      self, channel, FL_METHOD_CODEC(codec), handler, user_data);
}

void fl_mock_binary_messenger_set_json_method_channel(
    FlMockBinaryMessenger* self,
    const gchar* channel,
    FlMockBinaryMessengerMethodChannelHandler handler,
    gpointer user_data) {
  g_return_if_fail(FL_IS_MOCK_BINARY_MESSENGER(self));
  g_autoptr(FlJsonMethodCodec) codec = fl_json_method_codec_new();
  fl_mock_binary_messenger_set_method_channel(
      self, channel, FL_METHOD_CODEC(codec), handler, user_data);
}

void fl_mock_binary_messenger_set_event_channel(
    FlMockBinaryMessenger* self,
    const gchar* channel,
    FlMethodCodec* codec,
    FlMockBinaryMessengerEventChannelHandler handler,
    FlMockBinaryMessengerEventChannelErrorHandler error_handler,
    gpointer user_data) {
  g_return_if_fail(FL_IS_MOCK_BINARY_MESSENGER(self));

  g_hash_table_insert(
      self->mock_event_channels, g_strdup(channel),
      mock_event_channel_new(handler, error_handler, codec, user_data));
}

void fl_mock_binary_messenger_set_standard_event_channel(
    FlMockBinaryMessenger* self,
    const gchar* channel,
    FlMockBinaryMessengerEventChannelHandler handler,
    FlMockBinaryMessengerEventChannelErrorHandler error_handler,
    gpointer user_data) {
  g_return_if_fail(FL_IS_MOCK_BINARY_MESSENGER(self));
  g_autoptr(FlStandardMethodCodec) codec = fl_standard_method_codec_new();
  fl_mock_binary_messenger_set_event_channel(
      self, channel, FL_METHOD_CODEC(codec), handler, error_handler, user_data);
}

void fl_mock_binary_messenger_set_json_event_channel(
    FlMockBinaryMessenger* self,
    const gchar* channel,
    FlMockBinaryMessengerEventChannelHandler handler,
    FlMockBinaryMessengerEventChannelErrorHandler error_handler,
    gpointer user_data) {
  g_return_if_fail(FL_IS_MOCK_BINARY_MESSENGER(self));
  g_autoptr(FlJsonMethodCodec) codec = fl_json_method_codec_new();
  fl_mock_binary_messenger_set_event_channel(
      self, channel, FL_METHOD_CODEC(codec), handler, error_handler, user_data);
}

void fl_mock_binary_messenger_set_error_channel(FlMockBinaryMessenger* self,
                                                const gchar* channel,
                                                gint code,
                                                const gchar* message) {
  g_return_if_fail(FL_IS_MOCK_BINARY_MESSENGER(self));

  g_hash_table_insert(self->mock_error_channels, g_strdup(channel),
                      mock_error_channel_new(code, message));
}

void fl_mock_binary_messenger_send(FlMockBinaryMessenger* self,
                                   const gchar* channel,
                                   GBytes* message,
                                   FlMockBinaryMessengerCallback callback,
                                   gpointer user_data) {
  g_return_if_fail(FL_IS_MOCK_BINARY_MESSENGER(self));

  Handler* handler =
      static_cast<Handler*>(g_hash_table_lookup(self->handlers, channel));
  if (handler == nullptr) {
    return;
  }

  handler->callback(
      FL_BINARY_MESSENGER(self), channel, message,
      FL_BINARY_MESSENGER_RESPONSE_HANDLE(
          fl_mock_binary_messenger_response_handle_new(callback, user_data)),
      handler->user_data);
}

typedef struct {
  FlMessageCodec* codec;
  FlMockBinaryMessengerMessageCallback callback;
  gpointer user_data;
} SendMessageData;

static SendMessageData* send_message_data_new(
    FlMessageCodec* codec,
    FlMockBinaryMessengerMessageCallback callback,
    gpointer user_data) {
  SendMessageData* data = g_new0(SendMessageData, 1);
  data->codec = FL_MESSAGE_CODEC(g_object_ref(codec));
  data->callback = callback;
  data->user_data = user_data;
  return data;
}

static void send_message_data_free(SendMessageData* data) {
  g_object_unref(data->codec);
  free(data);
}

G_DEFINE_AUTOPTR_CLEANUP_FUNC(SendMessageData, send_message_data_free)

static void send_message_cb(FlMockBinaryMessenger* self,
                            GBytes* response,
                            gpointer user_data) {
  g_autoptr(SendMessageData) data = static_cast<SendMessageData*>(user_data);

  g_autoptr(GError) error = nullptr;
  g_autoptr(FlValue) response_value =
      fl_message_codec_decode_message(data->codec, response, &error);
  if (response_value == nullptr) {
    g_warning("Failed to decode message response: %s", error->message);
    return;
  }

  data->callback(self, response_value, data->user_data);
}

void fl_mock_binary_messenger_send_message(
    FlMockBinaryMessenger* self,
    const gchar* channel,
    FlMessageCodec* codec,
    FlValue* message,
    FlMockBinaryMessengerMessageCallback callback,
    gpointer user_data) {
  g_return_if_fail(FL_IS_MOCK_BINARY_MESSENGER(self));

  g_autoptr(GError) error = nullptr;
  g_autoptr(GBytes) encoded_message =
      fl_message_codec_encode_message(codec, message, &error);
  if (encoded_message == nullptr) {
    g_warning("Failed to encode message: %s", error->message);
    return;
  }

  fl_mock_binary_messenger_send(
      self, channel, encoded_message, send_message_cb,
      send_message_data_new(codec, callback, user_data));
}

void fl_mock_binary_messenger_send_standard_message(
    FlMockBinaryMessenger* self,
    const gchar* channel,
    FlValue* message,
    FlMockBinaryMessengerMessageCallback callback,
    gpointer user_data) {
  g_return_if_fail(FL_IS_MOCK_BINARY_MESSENGER(self));
  g_autoptr(FlStandardMessageCodec) codec = fl_standard_message_codec_new();
  fl_mock_binary_messenger_send_message(self, channel, FL_MESSAGE_CODEC(codec),
                                        message, callback, user_data);
}

void fl_mock_binary_messenger_send_json_message(
    FlMockBinaryMessenger* self,
    const gchar* channel,
    FlValue* message,
    FlMockBinaryMessengerMessageCallback callback,
    gpointer user_data) {
  g_return_if_fail(FL_IS_MOCK_BINARY_MESSENGER(self));
  g_autoptr(FlJsonMessageCodec) codec = fl_json_message_codec_new();
  fl_mock_binary_messenger_send_message(self, channel, FL_MESSAGE_CODEC(codec),
                                        message, callback, user_data);
}

typedef struct {
  FlMethodCodec* codec;
  FlMockBinaryMessengerMethodCallback callback;
  gpointer user_data;
} InvokeMethodData;

static InvokeMethodData* invoke_method_data_new(
    FlMethodCodec* codec,
    FlMockBinaryMessengerMethodCallback callback,
    gpointer user_data) {
  InvokeMethodData* data = g_new0(InvokeMethodData, 1);
  data->codec = FL_METHOD_CODEC(g_object_ref(codec));
  data->callback = callback;
  data->user_data = user_data;
  return data;
}

static void invoke_method_data_free(InvokeMethodData* data) {
  g_object_unref(data->codec);
  free(data);
}

G_DEFINE_AUTOPTR_CLEANUP_FUNC(InvokeMethodData, invoke_method_data_free)

static void invoke_method_cb(FlMockBinaryMessenger* self,
                             GBytes* response,
                             gpointer user_data) {
  g_autoptr(InvokeMethodData) data = static_cast<InvokeMethodData*>(user_data);

  g_autoptr(GError) error = nullptr;
  g_autoptr(FlMethodResponse) method_response =
      fl_method_codec_decode_response(data->codec, response, &error);
  if (method_response == nullptr) {
    g_warning("Failed to decode method response: %s", error->message);
    return;
  }

  data->callback(self, method_response, data->user_data);
}

void fl_mock_binary_messenger_invoke_method(
    FlMockBinaryMessenger* self,
    const gchar* channel,
    FlMethodCodec* codec,
    const char* name,
    FlValue* args,
    FlMockBinaryMessengerMethodCallback callback,
    gpointer user_data) {
  g_return_if_fail(FL_IS_MOCK_BINARY_MESSENGER(self));

  g_autoptr(GError) error = nullptr;
  g_autoptr(GBytes) message =
      fl_method_codec_encode_method_call(codec, name, args, &error);
  if (message == nullptr) {
    g_warning("Failed to encode method call: %s", error->message);
    return;
  }

  fl_mock_binary_messenger_send(
      self, channel, message, invoke_method_cb,
      invoke_method_data_new(codec, callback, user_data));
}

void fl_mock_binary_messenger_invoke_standard_method(
    FlMockBinaryMessenger* self,
    const gchar* channel,
    const char* name,
    FlValue* args,
    FlMockBinaryMessengerMethodCallback callback,
    gpointer user_data) {
  g_return_if_fail(FL_IS_MOCK_BINARY_MESSENGER(self));
  g_autoptr(FlStandardMethodCodec) codec = fl_standard_method_codec_new();
  fl_mock_binary_messenger_invoke_method(self, channel, FL_METHOD_CODEC(codec),
                                         name, args, callback, user_data);
}

void fl_mock_binary_messenger_invoke_json_method(
    FlMockBinaryMessenger* self,
    const gchar* channel,
    const char* name,
    FlValue* args,
    FlMockBinaryMessengerMethodCallback callback,
    gpointer user_data) {
  g_return_if_fail(FL_IS_MOCK_BINARY_MESSENGER(self));
  g_autoptr(FlJsonMethodCodec) codec = fl_json_method_codec_new();
  fl_mock_binary_messenger_invoke_method(self, channel, FL_METHOD_CODEC(codec),
                                         name, args, callback, user_data);
}
