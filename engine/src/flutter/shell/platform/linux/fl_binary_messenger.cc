// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/public/flutter_linux/fl_binary_messenger.h"
#include "flutter/shell/platform/linux/fl_binary_messenger_private.h"

#include "flutter/fml/logging.h"
#include "flutter/shell/platform/linux/fl_engine_private.h"
#include "flutter/shell/platform/linux/fl_method_codec_private.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_engine.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_method_channel.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_method_codec.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_standard_method_codec.h"

#include <gmodule.h>

static constexpr char kControlChannelName[] = "dev.flutter/channel-buffers";
static constexpr char kResizeMethod[] = "resize";
static constexpr char kOverflowMethod[] = "overflow";

G_DEFINE_QUARK(fl_binary_messenger_codec_error_quark,
               fl_binary_messenger_codec_error)

G_DECLARE_FINAL_TYPE(FlBinaryMessengerImpl,
                     fl_binary_messenger_impl,
                     FL,
                     BINARY_MESSENGER_IMPL,
                     GObject)

G_DECLARE_FINAL_TYPE(FlBinaryMessengerResponseHandleImpl,
                     fl_binary_messenger_response_handle_impl,
                     FL,
                     BINARY_MESSENGER_RESPONSE_HANDLE_IMPL,
                     FlBinaryMessengerResponseHandle)

G_DEFINE_INTERFACE(FlBinaryMessenger, fl_binary_messenger, G_TYPE_OBJECT)

struct _FlBinaryMessengerImpl {
  GObject parent_instance;

  GWeakRef engine;

  // PlatformMessageHandler keyed by channel name.
  GHashTable* platform_message_handlers;
};

static void fl_binary_messenger_impl_iface_init(
    FlBinaryMessengerInterface* iface);

G_DEFINE_TYPE_WITH_CODE(
    FlBinaryMessengerImpl,
    fl_binary_messenger_impl,
    G_TYPE_OBJECT,
    G_IMPLEMENT_INTERFACE(fl_binary_messenger_get_type(),
                          fl_binary_messenger_impl_iface_init))

static void fl_binary_messenger_response_handle_class_init(
    FlBinaryMessengerResponseHandleClass* klass) {}

G_DEFINE_TYPE(FlBinaryMessengerResponseHandle,
              fl_binary_messenger_response_handle,
              G_TYPE_OBJECT)

static void fl_binary_messenger_response_handle_init(
    FlBinaryMessengerResponseHandle* self) {}

struct _FlBinaryMessengerResponseHandleImpl {
  FlBinaryMessengerResponseHandle parent_instance;

  // Messenger sending response on.
  FlBinaryMessengerImpl* messenger;

  // Handle to send the response with. This is cleared to nullptr when it is
  // used.
  const FlutterPlatformMessageResponseHandle* response_handle;
};

G_DEFINE_TYPE(FlBinaryMessengerResponseHandleImpl,
              fl_binary_messenger_response_handle_impl,
              fl_binary_messenger_response_handle_get_type())

static void fl_binary_messenger_default_init(
    FlBinaryMessengerInterface* iface) {}

static void fl_binary_messenger_response_handle_impl_dispose(GObject* object) {
  FlBinaryMessengerResponseHandleImpl* self =
      FL_BINARY_MESSENGER_RESPONSE_HANDLE_IMPL(object);

  g_autoptr(FlEngine) engine =
      FL_ENGINE(g_weak_ref_get(&self->messenger->engine));
  if (self->response_handle != nullptr && engine != nullptr) {
    g_critical("FlBinaryMessengerResponseHandle was not responded to");
  }

  g_clear_object(&self->messenger);
  self->response_handle = nullptr;

  G_OBJECT_CLASS(fl_binary_messenger_response_handle_impl_parent_class)
      ->dispose(object);
}

static void fl_binary_messenger_response_handle_impl_class_init(
    FlBinaryMessengerResponseHandleImplClass* klass) {
  G_OBJECT_CLASS(klass)->dispose =
      fl_binary_messenger_response_handle_impl_dispose;
}

static void fl_binary_messenger_response_handle_impl_init(
    FlBinaryMessengerResponseHandleImpl* self) {}

static FlBinaryMessengerResponseHandleImpl*
fl_binary_messenger_response_handle_impl_new(
    FlBinaryMessengerImpl* messenger,
    const FlutterPlatformMessageResponseHandle* response_handle) {
  FlBinaryMessengerResponseHandleImpl* self =
      FL_BINARY_MESSENGER_RESPONSE_HANDLE_IMPL(g_object_new(
          fl_binary_messenger_response_handle_impl_get_type(), nullptr));

  self->messenger = FL_BINARY_MESSENGER_IMPL(g_object_ref(messenger));
  self->response_handle = response_handle;

  return self;
}

typedef struct {
  FlBinaryMessengerMessageHandler message_handler;
  gpointer message_handler_data;
  GDestroyNotify message_handler_destroy_notify;
} PlatformMessageHandler;

static PlatformMessageHandler* platform_message_handler_new(
    FlBinaryMessengerMessageHandler handler,
    gpointer user_data,
    GDestroyNotify destroy_notify) {
  PlatformMessageHandler* self = static_cast<PlatformMessageHandler*>(
      g_malloc0(sizeof(PlatformMessageHandler)));
  self->message_handler = handler;
  self->message_handler_data = user_data;
  self->message_handler_destroy_notify = destroy_notify;
  return self;
}

static void platform_message_handler_free(gpointer data) {
  PlatformMessageHandler* self = static_cast<PlatformMessageHandler*>(data);
  if (self->message_handler_destroy_notify) {
    self->message_handler_destroy_notify(self->message_handler_data);
  }
  g_free(self);
}

static gboolean fl_binary_messenger_platform_message_cb(
    FlEngine* engine,
    const gchar* channel,
    GBytes* message,
    const FlutterPlatformMessageResponseHandle* response_handle,
    void* user_data) {
  FlBinaryMessenger* self = FL_BINARY_MESSENGER(user_data);
  return fl_binary_messenger_handle_message(self, channel, message,
                                            response_handle);
}

static void fl_binary_messenger_impl_dispose(GObject* object) {
  FlBinaryMessengerImpl* self = FL_BINARY_MESSENGER_IMPL(object);

  g_weak_ref_clear(&self->engine);

  g_clear_pointer(&self->platform_message_handlers, g_hash_table_unref);

  G_OBJECT_CLASS(fl_binary_messenger_impl_parent_class)->dispose(object);
}

static void set_message_handler_on_channel(
    FlBinaryMessenger* messenger,
    const gchar* channel,
    FlBinaryMessengerMessageHandler handler,
    gpointer user_data,
    GDestroyNotify destroy_notify) {
  FlBinaryMessengerImpl* self = FL_BINARY_MESSENGER_IMPL(messenger);

  // Don't set handlers if engine already gone.
  g_autoptr(FlEngine) engine = FL_ENGINE(g_weak_ref_get(&self->engine));
  if (engine == nullptr) {
    if (handler != nullptr) {
      g_warning(
          "Attempted to set message handler on an FlBinaryMessenger without an "
          "engine");
    }
    if (destroy_notify != nullptr) {
      destroy_notify(user_data);
    }
    return;
  }

  if (handler != nullptr) {
    g_hash_table_replace(
        self->platform_message_handlers, g_strdup(channel),
        platform_message_handler_new(handler, user_data, destroy_notify));
  } else {
    g_hash_table_remove(self->platform_message_handlers, channel);
  }
}

static gboolean do_unref(gpointer value) {
  g_object_unref(value);
  return G_SOURCE_REMOVE;
}

// Note: This function can be called from any thread.
static gboolean send_response(FlBinaryMessenger* messenger,
                              FlBinaryMessengerResponseHandle* response_handle_,
                              GBytes* response,
                              GError** error) {
  FlBinaryMessengerImpl* self = FL_BINARY_MESSENGER_IMPL(messenger);
  g_return_val_if_fail(
      FL_IS_BINARY_MESSENGER_RESPONSE_HANDLE_IMPL(response_handle_), FALSE);
  FlBinaryMessengerResponseHandleImpl* response_handle =
      FL_BINARY_MESSENGER_RESPONSE_HANDLE_IMPL(response_handle_);

  g_return_val_if_fail(response_handle->messenger == self, FALSE);
  g_return_val_if_fail(response_handle->response_handle != nullptr, FALSE);

  FlEngine* engine = FL_ENGINE(g_weak_ref_get(&self->engine));
  if (engine == nullptr) {
    return TRUE;
  }

  gboolean result = false;
  if (response_handle->response_handle == nullptr) {
    g_set_error(
        error, FL_BINARY_MESSENGER_ERROR,
        FL_BINARY_MESSENGER_ERROR_ALREADY_RESPONDED,
        "Attempted to respond to a message that is already responded to");
    result = FALSE;
  } else {
    result = fl_engine_send_platform_message_response(
        engine, response_handle->response_handle, response, error);
    response_handle->response_handle = nullptr;
  }

  // This guarantees that the dispose method for the engine is executed
  // on the platform thread in the rare chance this is the last ref.
  g_idle_add(do_unref, engine);

  return result;
}

static void platform_message_ready_cb(GObject* object,
                                      GAsyncResult* result,
                                      gpointer user_data) {
  g_autoptr(GTask) task = G_TASK(user_data);
  g_task_return_pointer(task, g_object_ref(result), g_object_unref);
}

static void send_on_channel(FlBinaryMessenger* messenger,
                            const gchar* channel,
                            GBytes* message,
                            GCancellable* cancellable,
                            GAsyncReadyCallback callback,
                            gpointer user_data) {
  FlBinaryMessengerImpl* self = FL_BINARY_MESSENGER_IMPL(messenger);

  g_autoptr(FlEngine) engine = FL_ENGINE(g_weak_ref_get(&self->engine));
  if (engine == nullptr) {
    return;
  }

  fl_engine_send_platform_message(
      engine, channel, message, cancellable,
      callback != nullptr ? platform_message_ready_cb : nullptr,
      callback != nullptr ? g_task_new(self, cancellable, callback, user_data)
                          : nullptr);
}

static GBytes* send_on_channel_finish(FlBinaryMessenger* messenger,
                                      GAsyncResult* result,
                                      GError** error) {
  FlBinaryMessengerImpl* self = FL_BINARY_MESSENGER_IMPL(messenger);
  g_return_val_if_fail(g_task_is_valid(result, self), FALSE);

  GTask* task = G_TASK(result);
  g_autoptr(GAsyncResult) r =
      G_ASYNC_RESULT(g_task_propagate_pointer(task, error));
  if (r == nullptr) {
    return nullptr;
  }

  g_autoptr(FlEngine) engine = FL_ENGINE(g_weak_ref_get(&self->engine));
  if (engine == nullptr) {
    return nullptr;
  }

  return fl_engine_send_platform_message_finish(engine, r, error);
}

// Completes method call and returns TRUE if the call was successful.
static gboolean finish_method(GObject* object,
                              GAsyncResult* result,
                              GError** error) {
  g_autoptr(GBytes) response = fl_binary_messenger_send_on_channel_finish(
      FL_BINARY_MESSENGER(object), result, error);
  if (response == nullptr) {
    return FALSE;
  }
  g_autoptr(FlStandardMethodCodec) codec = fl_standard_method_codec_new();
  return fl_method_codec_decode_response(FL_METHOD_CODEC(codec), response,
                                         error) != nullptr;
}

// Called when a response is received for the resize channel message.
static void resize_channel_response_cb(GObject* object,
                                       GAsyncResult* result,
                                       gpointer user_data) {
  g_autoptr(GError) error = nullptr;
  if (!finish_method(object, result, &error)) {
    g_warning("Failed to resize channel: %s", error->message);
  }
}

static void resize_channel(FlBinaryMessenger* messenger,
                           const gchar* channel,
                           int64_t new_size) {
  FML_DCHECK(new_size >= 0);
  g_autoptr(FlStandardMethodCodec) codec = fl_standard_method_codec_new();
  g_autoptr(FlValue) args = fl_value_new_list();
  fl_value_append_take(args, fl_value_new_string(channel));
  fl_value_append_take(args, fl_value_new_int(new_size));
  g_autoptr(GBytes) message = fl_method_codec_encode_method_call(
      FL_METHOD_CODEC(codec), kResizeMethod, args, nullptr);
  fl_binary_messenger_send_on_channel(messenger, kControlChannelName, message,
                                      nullptr, resize_channel_response_cb,
                                      nullptr);
}

// Called when a response is received for the warns on overflow message.
static void set_warns_on_channel_overflow_response_cb(GObject* object,
                                                      GAsyncResult* result,
                                                      gpointer user_data) {
  g_autoptr(GError) error = nullptr;
  if (!finish_method(object, result, &error)) {
    g_warning("Failed to set warns on channel overflow: %s", error->message);
  }
}

static void set_warns_on_channel_overflow(FlBinaryMessenger* messenger,
                                          const gchar* channel,
                                          bool warns) {
  g_autoptr(FlStandardMethodCodec) codec = fl_standard_method_codec_new();
  g_autoptr(FlValue) args = fl_value_new_list();
  fl_value_append_take(args, fl_value_new_string(channel));
  fl_value_append_take(args, fl_value_new_bool(!warns));
  g_autoptr(GBytes) message = fl_method_codec_encode_method_call(
      FL_METHOD_CODEC(codec), kOverflowMethod, args, nullptr);
  fl_binary_messenger_send_on_channel(
      messenger, kControlChannelName, message, nullptr,
      set_warns_on_channel_overflow_response_cb, nullptr);
}

static void shutdown(FlBinaryMessenger* messenger) {
  FlBinaryMessengerImpl* self = FL_BINARY_MESSENGER_IMPL(messenger);

  // Disconnect any handlers.
  // Take the reference in case a handler tries to modify this table.
  g_autoptr(GHashTable) handlers = self->platform_message_handlers;
  self->platform_message_handlers = g_hash_table_new_full(
      g_str_hash, g_str_equal, g_free, platform_message_handler_free);
  g_hash_table_remove_all(handlers);
}

static void fl_binary_messenger_impl_class_init(
    FlBinaryMessengerImplClass* klass) {
  G_OBJECT_CLASS(klass)->dispose = fl_binary_messenger_impl_dispose;
}

static void fl_binary_messenger_impl_iface_init(
    FlBinaryMessengerInterface* iface) {
  iface->set_message_handler_on_channel = set_message_handler_on_channel;
  iface->send_response = send_response;
  iface->send_on_channel = send_on_channel;
  iface->send_on_channel_finish = send_on_channel_finish;
  iface->resize_channel = resize_channel;
  iface->set_warns_on_channel_overflow = set_warns_on_channel_overflow;
  iface->shutdown = shutdown;
}

static void fl_binary_messenger_impl_init(FlBinaryMessengerImpl* self) {
  self->platform_message_handlers = g_hash_table_new_full(
      g_str_hash, g_str_equal, g_free, platform_message_handler_free);
}

FlBinaryMessenger* fl_binary_messenger_new(FlEngine* engine) {
  g_return_val_if_fail(FL_IS_ENGINE(engine), nullptr);

  FlBinaryMessengerImpl* self = FL_BINARY_MESSENGER_IMPL(
      g_object_new(fl_binary_messenger_impl_get_type(), nullptr));

  // Added to stop compiler complaining about an unused function.
  FL_IS_BINARY_MESSENGER_IMPL(self);

  g_weak_ref_init(&self->engine, G_OBJECT(engine));

  fl_engine_set_platform_message_handler(
      engine, fl_binary_messenger_platform_message_cb, self, NULL);

  return FL_BINARY_MESSENGER(self);
}

G_MODULE_EXPORT void fl_binary_messenger_set_message_handler_on_channel(
    FlBinaryMessenger* self,
    const gchar* channel,
    FlBinaryMessengerMessageHandler handler,
    gpointer user_data,
    GDestroyNotify destroy_notify) {
  g_return_if_fail(FL_IS_BINARY_MESSENGER(self));
  g_return_if_fail(channel != nullptr);

  FL_BINARY_MESSENGER_GET_IFACE(self)->set_message_handler_on_channel(
      self, channel, handler, user_data, destroy_notify);
}

// Note: This function can be called from any thread.
G_MODULE_EXPORT gboolean fl_binary_messenger_send_response(
    FlBinaryMessenger* self,
    FlBinaryMessengerResponseHandle* response_handle,
    GBytes* response,
    GError** error) {
  g_return_val_if_fail(FL_IS_BINARY_MESSENGER(self), FALSE);
  g_return_val_if_fail(FL_IS_BINARY_MESSENGER_RESPONSE_HANDLE(response_handle),
                       FALSE);

  return FL_BINARY_MESSENGER_GET_IFACE(self)->send_response(
      self, response_handle, response, error);
}

G_MODULE_EXPORT void fl_binary_messenger_send_on_channel(
    FlBinaryMessenger* self,
    const gchar* channel,
    GBytes* message,
    GCancellable* cancellable,
    GAsyncReadyCallback callback,
    gpointer user_data) {
  g_return_if_fail(FL_IS_BINARY_MESSENGER(self));
  g_return_if_fail(channel != nullptr);

  FL_BINARY_MESSENGER_GET_IFACE(self)->send_on_channel(
      self, channel, message, cancellable, callback, user_data);
}

G_MODULE_EXPORT GBytes* fl_binary_messenger_send_on_channel_finish(
    FlBinaryMessenger* self,
    GAsyncResult* result,
    GError** error) {
  g_return_val_if_fail(FL_IS_BINARY_MESSENGER(self), FALSE);

  return FL_BINARY_MESSENGER_GET_IFACE(self)->send_on_channel_finish(
      self, result, error);
}

G_MODULE_EXPORT void fl_binary_messenger_resize_channel(FlBinaryMessenger* self,
                                                        const gchar* channel,
                                                        int64_t new_size) {
  g_return_if_fail(FL_IS_BINARY_MESSENGER(self));

  return FL_BINARY_MESSENGER_GET_IFACE(self)->resize_channel(self, channel,
                                                             new_size);
}

G_MODULE_EXPORT void fl_binary_messenger_set_warns_on_channel_overflow(
    FlBinaryMessenger* self,
    const gchar* channel,
    bool warns) {
  g_return_if_fail(FL_IS_BINARY_MESSENGER(self));

  return FL_BINARY_MESSENGER_GET_IFACE(self)->set_warns_on_channel_overflow(
      self, channel, warns);
}

gboolean fl_binary_messenger_handle_message(
    FlBinaryMessenger* messenger,
    const gchar* channel,
    GBytes* message,
    const FlutterPlatformMessageResponseHandle* response_handle) {
  FlBinaryMessengerImpl* self = FL_BINARY_MESSENGER_IMPL(messenger);

  PlatformMessageHandler* handler = static_cast<PlatformMessageHandler*>(
      g_hash_table_lookup(self->platform_message_handlers, channel));
  if (handler == nullptr) {
    return FALSE;
  }

  g_autoptr(FlBinaryMessengerResponseHandleImpl) handle =
      fl_binary_messenger_response_handle_impl_new(self, response_handle);
  handler->message_handler(FL_BINARY_MESSENGER(self), channel, message,
                           FL_BINARY_MESSENGER_RESPONSE_HANDLE(handle),
                           handler->message_handler_data);

  return TRUE;
}

void fl_binary_messenger_shutdown(FlBinaryMessenger* self) {
  g_return_if_fail(FL_IS_BINARY_MESSENGER(self));

  return FL_BINARY_MESSENGER_GET_IFACE(self)->shutdown(self);
}
