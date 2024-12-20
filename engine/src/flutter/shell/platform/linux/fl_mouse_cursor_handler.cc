// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/fl_mouse_cursor_handler.h"

#include <cstring>

#include "flutter/shell/platform/linux/fl_mouse_cursor_channel.h"

static constexpr char kFallbackCursor[] = "default";

struct _FlMouseCursorHandler {
  GObject parent_instance;

  FlMouseCursorChannel* channel;

  GHashTable* system_cursor_table;

  // The current cursor.
  gchar* cursor_name;
};

enum { SIGNAL_CURSOR_CHANGED, LAST_SIGNAL };

static guint fl_mouse_cursor_handler_signals[LAST_SIGNAL];

G_DEFINE_TYPE(FlMouseCursorHandler, fl_mouse_cursor_handler, G_TYPE_OBJECT)

// Insert a new entry into a hashtable from strings to strings.
//
// Returns whether the newly added value was already in the hash table or not.
static bool define_system_cursor(GHashTable* table,
                                 const gchar* key,
                                 const gchar* value) {
  return g_hash_table_insert(
      table, reinterpret_cast<gpointer>(const_cast<gchar*>(key)),
      reinterpret_cast<gpointer>(const_cast<gchar*>(value)));
}

// Populate the hash table so that it maps from Flutter's cursor kinds to GTK's
// cursor values.
//
// The table must have been created as a hashtable from strings to strings.
static void populate_system_cursor_table(GHashTable* table) {
  // The following mapping must be kept in sync with Flutter framework's
  // mouse_cursor.dart.
  define_system_cursor(table, "alias", "alias");
  define_system_cursor(table, "allScroll", "all-scroll");
  define_system_cursor(table, "basic", "default");
  define_system_cursor(table, "cell", "cell");
  define_system_cursor(table, "click", "pointer");
  define_system_cursor(table, "contextMenu", "context-menu");
  define_system_cursor(table, "copy", "copy");
  define_system_cursor(table, "forbidden", "not-allowed");
  define_system_cursor(table, "grab", "grab");
  define_system_cursor(table, "grabbing", "grabbing");
  define_system_cursor(table, "help", "help");
  define_system_cursor(table, "move", "move");
  define_system_cursor(table, "none", "none");
  define_system_cursor(table, "noDrop", "no-drop");
  define_system_cursor(table, "precise", "crosshair");
  define_system_cursor(table, "progress", "progress");
  define_system_cursor(table, "text", "text");
  define_system_cursor(table, "resizeColumn", "col-resize");
  define_system_cursor(table, "resizeDown", "s-resize");
  define_system_cursor(table, "resizeDownLeft", "sw-resize");
  define_system_cursor(table, "resizeDownRight", "se-resize");
  define_system_cursor(table, "resizeLeft", "w-resize");
  define_system_cursor(table, "resizeLeftRight", "ew-resize");
  define_system_cursor(table, "resizeRight", "e-resize");
  define_system_cursor(table, "resizeRow", "row-resize");
  define_system_cursor(table, "resizeUp", "n-resize");
  define_system_cursor(table, "resizeUpDown", "ns-resize");
  define_system_cursor(table, "resizeUpLeft", "nw-resize");
  define_system_cursor(table, "resizeUpRight", "ne-resize");
  define_system_cursor(table, "resizeUpLeftDownRight", "nwse-resize");
  define_system_cursor(table, "resizeUpRightDownLeft", "nesw-resize");
  define_system_cursor(table, "verticalText", "vertical-text");
  define_system_cursor(table, "wait", "wait");
  define_system_cursor(table, "zoomIn", "zoom-in");
  define_system_cursor(table, "zoomOut", "zoom-out");
}

// Sets the mouse cursor.
static void activate_system_cursor(const gchar* kind, gpointer user_data) {
  FlMouseCursorHandler* self = FL_MOUSE_CURSOR_HANDLER(user_data);

  if (self->system_cursor_table == nullptr) {
    self->system_cursor_table = g_hash_table_new(g_str_hash, g_str_equal);
    populate_system_cursor_table(self->system_cursor_table);
  }

  const gchar* cursor_name = reinterpret_cast<const gchar*>(
      g_hash_table_lookup(self->system_cursor_table, kind));
  if (cursor_name == nullptr) {
    cursor_name = kFallbackCursor;
  }

  g_free(self->cursor_name);
  self->cursor_name = g_strdup(cursor_name);

  g_signal_emit(self, fl_mouse_cursor_handler_signals[SIGNAL_CURSOR_CHANGED],
                0);
}

static void fl_mouse_cursor_handler_dispose(GObject* object) {
  FlMouseCursorHandler* self = FL_MOUSE_CURSOR_HANDLER(object);

  g_clear_object(&self->channel);
  g_clear_pointer(&self->system_cursor_table, g_hash_table_unref);
  g_clear_pointer(&self->cursor_name, g_free);

  G_OBJECT_CLASS(fl_mouse_cursor_handler_parent_class)->dispose(object);
}

static void fl_mouse_cursor_handler_class_init(
    FlMouseCursorHandlerClass* klass) {
  G_OBJECT_CLASS(klass)->dispose = fl_mouse_cursor_handler_dispose;

  fl_mouse_cursor_handler_signals[SIGNAL_CURSOR_CHANGED] =
      g_signal_new("cursor-changed", fl_mouse_cursor_handler_get_type(),
                   G_SIGNAL_RUN_LAST, 0, NULL, NULL, NULL, G_TYPE_NONE, 0);
}

static void fl_mouse_cursor_handler_init(FlMouseCursorHandler* self) {
  self->cursor_name = g_strdup("");
}

static FlMouseCursorChannelVTable mouse_cursor_vtable = {
    .activate_system_cursor = activate_system_cursor,
};

FlMouseCursorHandler* fl_mouse_cursor_handler_new(
    FlBinaryMessenger* messenger) {
  g_return_val_if_fail(FL_IS_BINARY_MESSENGER(messenger), nullptr);

  FlMouseCursorHandler* self = FL_MOUSE_CURSOR_HANDLER(
      g_object_new(fl_mouse_cursor_handler_get_type(), nullptr));

  self->channel =
      fl_mouse_cursor_channel_new(messenger, &mouse_cursor_vtable, self);

  return self;
}

const gchar* fl_mouse_cursor_handler_get_cursor_name(
    FlMouseCursorHandler* self) {
  g_return_val_if_fail(FL_IS_MOUSE_CURSOR_HANDLER(self), nullptr);
  return self->cursor_name;
}
