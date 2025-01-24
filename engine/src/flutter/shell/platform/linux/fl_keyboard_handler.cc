// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/fl_keyboard_handler.h"

#include "flutter/shell/platform/linux/fl_keyboard_channel.h"
#include "flutter/shell/platform/linux/key_mapping.h"

struct _FlKeyboardHandler {
  GObject parent_instance;

  FlKeyboardManager* keyboard_manager;

  // The channel used by the framework to query the keyboard pressed state.
  FlKeyboardChannel* channel;
};

G_DEFINE_TYPE(FlKeyboardHandler, fl_keyboard_handler, G_TYPE_OBJECT);

// Returns the keyboard pressed state.
static FlValue* get_keyboard_state(gpointer user_data) {
  FlKeyboardHandler* self = FL_KEYBOARD_HANDLER(user_data);

  FlValue* result = fl_value_new_map();

  GHashTable* pressing_records =
      fl_keyboard_manager_get_pressed_state(self->keyboard_manager);

  g_hash_table_foreach(
      pressing_records,
      [](gpointer key, gpointer value, gpointer user_data) {
        int64_t physical_key = gpointer_to_uint64(key);
        int64_t logical_key = gpointer_to_uint64(value);
        FlValue* fl_value_map = reinterpret_cast<FlValue*>(user_data);

        fl_value_set_take(fl_value_map, fl_value_new_int(physical_key),
                          fl_value_new_int(logical_key));
      },
      result);

  return result;
}

static void fl_keyboard_handler_dispose(GObject* object) {
  FlKeyboardHandler* self = FL_KEYBOARD_HANDLER(object);

  g_clear_object(&self->keyboard_manager);
  g_clear_object(&self->channel);

  G_OBJECT_CLASS(fl_keyboard_handler_parent_class)->dispose(object);
}

static void fl_keyboard_handler_class_init(FlKeyboardHandlerClass* klass) {
  G_OBJECT_CLASS(klass)->dispose = fl_keyboard_handler_dispose;
}

static FlKeyboardChannelVTable keyboard_channel_vtable = {
    .get_keyboard_state = get_keyboard_state};

static void fl_keyboard_handler_init(FlKeyboardHandler* self) {}

FlKeyboardHandler* fl_keyboard_handler_new(
    FlBinaryMessenger* messenger,
    FlKeyboardManager* keyboard_manager) {
  FlKeyboardHandler* self = FL_KEYBOARD_HANDLER(
      g_object_new(fl_keyboard_handler_get_type(), nullptr));

  self->keyboard_manager = FL_KEYBOARD_MANAGER(g_object_ref(keyboard_manager));
  self->channel =
      fl_keyboard_channel_new(messenger, &keyboard_channel_vtable, self);

  return self;
}
