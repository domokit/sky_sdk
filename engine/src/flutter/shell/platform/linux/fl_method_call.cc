// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/public/flutter_linux/fl_method_call.h"
#include "flutter/shell/platform/linux/fl_method_call_private.h"
#include "flutter/shell/platform/linux/fl_method_channel_private.h"

#include <gmodule.h>

struct _FlMethodCall {
  GObject parent_instance;

  // Name of method being called.
  gchar* name;

  // Arguments provided to method call.
  FlValue* args;

  // Channel to respond on.
  FlMethodChannel* channel;
  FlBinaryMessengerResponseHandle* response_handle;
};

G_DEFINE_TYPE(FlMethodCall, fl_method_call, G_TYPE_OBJECT)

static void fl_method_call_dispose(GObject* object) {
  FlMethodCall* self = FL_METHOD_CALL(object);

  g_clear_pointer(&self->name, g_free);
  g_clear_pointer(&self->args, fl_value_unref);
  g_clear_object(&self->channel);
  g_clear_object(&self->response_handle);

  G_OBJECT_CLASS(fl_method_call_parent_class)->dispose(object);
}

static void fl_method_call_class_init(FlMethodCallClass* klass) {
  G_OBJECT_CLASS(klass)->dispose = fl_method_call_dispose;
}

static void fl_method_call_init(FlMethodCall* self) {}

FlMethodCall* fl_method_call_new(
    const gchar* name,
    FlValue* args,
    FlMethodChannel* channel,
    FlBinaryMessengerResponseHandle* response_handle) {
  g_return_val_if_fail(name != nullptr, nullptr);
  g_return_val_if_fail(args != nullptr, nullptr);
  g_return_val_if_fail(FL_IS_METHOD_CHANNEL(channel), nullptr);
  g_return_val_if_fail(FL_IS_BINARY_MESSENGER_RESPONSE_HANDLE(response_handle),
                       nullptr);

  FlMethodCall* self =
      FL_METHOD_CALL(g_object_new(fl_method_call_get_type(), nullptr));

  self->name = g_strdup(name);
  self->args = fl_value_ref(args);
  self->channel = FL_METHOD_CHANNEL(g_object_ref(channel));
  self->response_handle =
      FL_BINARY_MESSENGER_RESPONSE_HANDLE(g_object_ref(response_handle));

  return self;
}

G_MODULE_EXPORT const gchar* fl_method_call_get_name(FlMethodCall* self) {
  g_return_val_if_fail(FL_IS_METHOD_CALL(self), nullptr);
  return self->name;
}

G_MODULE_EXPORT FlValue* fl_method_call_get_args(FlMethodCall* self) {
  g_return_val_if_fail(FL_IS_METHOD_CALL(self), nullptr);
  return self->args;
}

G_MODULE_EXPORT gboolean fl_method_call_respond(FlMethodCall* self,
                                                FlMethodResponse* response,
                                                GError** error) {
  g_return_val_if_fail(FL_IS_METHOD_CALL(self), FALSE);
  g_return_val_if_fail(FL_IS_METHOD_RESPONSE(response), FALSE);

  g_autoptr(GError) local_error = nullptr;
  if (!fl_method_channel_respond(self->channel, self->response_handle, response,
                                 &local_error)) {
    // If the developer chose not to handle the error then log it so it's not
    // missed.
    if (error == nullptr) {
      g_warning("Failed to send method call response: %s",
                local_error->message);
    }

    g_propagate_error(error, local_error);
    return FALSE;
  }

  return TRUE;
}

G_MODULE_EXPORT gboolean fl_method_call_respond_success(FlMethodCall* self,
                                                        FlValue* result,
                                                        GError** error) {
  g_return_val_if_fail(FL_IS_METHOD_CALL(self), FALSE);

  g_autoptr(FlMethodResponse) response =
      FL_METHOD_RESPONSE(fl_method_success_response_new(result));
  return fl_method_channel_respond(self->channel, self->response_handle,
                                   response, error);
}

G_MODULE_EXPORT gboolean fl_method_call_respond_error(FlMethodCall* self,
                                                      const gchar* code,
                                                      const gchar* message,
                                                      FlValue* details,
                                                      GError** error) {
  g_return_val_if_fail(FL_IS_METHOD_CALL(self), FALSE);
  g_return_val_if_fail(code != nullptr, FALSE);

  g_autoptr(FlMethodResponse) response =
      FL_METHOD_RESPONSE(fl_method_error_response_new(code, message, details));
  return fl_method_channel_respond(self->channel, self->response_handle,
                                   response, error);
}

G_MODULE_EXPORT gboolean fl_method_call_respond_not_implemented(
    FlMethodCall* self,
    GError** error) {
  g_return_val_if_fail(FL_IS_METHOD_CALL(self), FALSE);

  g_autoptr(FlMethodResponse) response =
      FL_METHOD_RESPONSE(fl_method_not_implemented_response_new());
  return fl_method_channel_respond(self->channel, self->response_handle,
                                   response, error);
}
