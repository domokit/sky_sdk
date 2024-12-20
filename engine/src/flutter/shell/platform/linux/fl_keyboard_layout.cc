// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/fl_keyboard_layout.h"

// The maxiumum keycode in a derived layout.
//
// Although X supports higher keycodes, Flutter only cares about standard keys,
// which are below this.
constexpr size_t kLayoutSize = 128;

struct _FlKeyboardLayout {
  GObject parent_instance;

  // Each keycode->logical key mapping per group.
  GHashTable* groups;
};

G_DEFINE_TYPE(FlKeyboardLayout, fl_keyboard_layout, G_TYPE_OBJECT)

static void fl_keyboard_layout_dispose(GObject* object) {
  FlKeyboardLayout* self = FL_KEYBOARD_LAYOUT(object);

  g_clear_pointer(&self->groups, g_hash_table_unref);

  G_OBJECT_CLASS(fl_keyboard_layout_parent_class)->dispose(object);
}

static void fl_keyboard_layout_class_init(FlKeyboardLayoutClass* klass) {
  G_OBJECT_CLASS(klass)->dispose = fl_keyboard_layout_dispose;
}

static void fl_keyboard_layout_init(FlKeyboardLayout* self) {
  self->groups = g_hash_table_new_full(
      g_direct_hash, g_direct_equal, nullptr,
      reinterpret_cast<GDestroyNotify>(g_hash_table_unref));
}

FlKeyboardLayout* fl_keyboard_layout_new() {
  return FL_KEYBOARD_LAYOUT(
      g_object_new(fl_keyboard_layout_get_type(), nullptr));
}

gboolean fl_keyboard_layout_has_group(FlKeyboardLayout* self, uint8_t group) {
  return g_hash_table_lookup(self->groups, GINT_TO_POINTER(group)) != nullptr;
}

void fl_keyboard_layout_set_logical_key(FlKeyboardLayout* self,
                                        uint8_t group,
                                        uint16_t keycode,
                                        uint64_t logical_key) {
  GHashTable* group_layout = static_cast<GHashTable*>(
      g_hash_table_lookup(self->groups, GINT_TO_POINTER(group)));
  if (group_layout == nullptr) {
    group_layout =
        g_hash_table_new_full(g_direct_hash, g_direct_equal, nullptr, nullptr);
    g_hash_table_insert(self->groups, GINT_TO_POINTER(group), group_layout);
  }

  g_hash_table_insert(group_layout, GINT_TO_POINTER(keycode),
                      GINT_TO_POINTER(logical_key));
}

uint64_t fl_keyboard_layout_get_logical_key(FlKeyboardLayout* self,
                                            uint8_t group,
                                            uint16_t keycode) {
  if (keycode >= kLayoutSize) {
    return 0;
  }

  GHashTable* group_layout = static_cast<GHashTable*>(
      g_hash_table_lookup(self->groups, GINT_TO_POINTER(group)));
  if (group_layout == nullptr) {
    return 0;
  }

  return GPOINTER_TO_INT(
      g_hash_table_lookup(group_layout, GINT_TO_POINTER(keycode)));
}
