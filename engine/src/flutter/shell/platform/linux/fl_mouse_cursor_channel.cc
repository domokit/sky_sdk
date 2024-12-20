// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/fl_mouse_cursor_channel.h"

#include <cstring>

#include "flutter/shell/platform/linux/public/flutter_linux/fl_method_channel.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_standard_method_codec.h"

static constexpr char kChannelName[] = "flutter/mousecursor";
static constexpr char kBadArgumentsError[] = "Bad Arguments";
static constexpr char kActivateSystemCursorMethod[] = "activateSystemCursor";
static constexpr char kKindKey[] = "kind";

struct _FlMouseCursorChannel {
  GObject parent_instance;

  FlMethodChannel* channel;

  FlMouseCursorChannelVTable* vtable;

  gpointer user_data;
};

G_DEFINE_TYPE(FlMouseCursorChannel, fl_mouse_cursor_channel, G_TYPE_OBJECT)

// Sets the mouse cursor.
static FlMethodResponse* activate_system_cursor(FlMouseCursorChannel* self,
                                                FlValue* args) {
  if (fl_value_get_type(args) != FL_VALUE_TYPE_MAP) {
    return FL_METHOD_RESPONSE(fl_method_error_response_new(
        kBadArgumentsError, "Argument map missing or malformed", nullptr));
  }

  FlValue* kind_value = fl_value_lookup_string(args, kKindKey);
  const gchar* kind = nullptr;
  if (fl_value_get_type(kind_value) == FL_VALUE_TYPE_STRING) {
    kind = fl_value_get_string(kind_value);
  }

  self->vtable->activate_system_cursor(kind, self->user_data);

  return FL_METHOD_RESPONSE(fl_method_success_response_new(nullptr));
}

// Called when a method call is received from Flutter.
static void method_call_cb(FlMethodChannel* channel,
                           FlMethodCall* method_call,
                           gpointer user_data) {
  FlMouseCursorChannel* self = FL_MOUSE_CURSOR_CHANNEL(user_data);

  const gchar* method = fl_method_call_get_name(method_call);
  FlValue* args = fl_method_call_get_args(method_call);

  g_autoptr(FlMethodResponse) response = nullptr;
  if (strcmp(method, kActivateSystemCursorMethod) == 0) {
    response = activate_system_cursor(self, args);
  } else {
    response = FL_METHOD_RESPONSE(fl_method_not_implemented_response_new());
  }

  g_autoptr(GError) error = nullptr;
  if (!fl_method_call_respond(method_call, response, &error)) {
    g_warning("Failed to send method call response: %s", error->message);
  }
}

static void fl_mouse_cursor_channel_dispose(GObject* object) {
  FlMouseCursorChannel* self = FL_MOUSE_CURSOR_CHANNEL(object);

  g_clear_object(&self->channel);

  G_OBJECT_CLASS(fl_mouse_cursor_channel_parent_class)->dispose(object);
}

static void fl_mouse_cursor_channel_class_init(
    FlMouseCursorChannelClass* klass) {
  G_OBJECT_CLASS(klass)->dispose = fl_mouse_cursor_channel_dispose;
}

static void fl_mouse_cursor_channel_init(FlMouseCursorChannel* self) {}

FlMouseCursorChannel* fl_mouse_cursor_channel_new(
    FlBinaryMessenger* messenger,
    FlMouseCursorChannelVTable* vtable,
    gpointer user_data) {
  g_return_val_if_fail(FL_IS_BINARY_MESSENGER(messenger), nullptr);

  FlMouseCursorChannel* self = FL_MOUSE_CURSOR_CHANNEL(
      g_object_new(fl_mouse_cursor_channel_get_type(), nullptr));

  self->vtable = vtable;
  self->user_data = user_data;

  g_autoptr(FlStandardMethodCodec) codec = fl_standard_method_codec_new();
  self->channel =
      fl_method_channel_new(messenger, kChannelName, FL_METHOD_CODEC(codec));
  fl_method_channel_set_method_call_handler(self->channel, method_call_cb, self,
                                            nullptr);

  return self;
}
