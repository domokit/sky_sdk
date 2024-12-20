// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/testing/mock_keymap.h"

using namespace flutter::testing;

G_DECLARE_FINAL_TYPE(FlMockKeymap, fl_mock_keymap, FL, MOCK_KEYMAP, GObject)

struct _FlMockKeymap {
  GObject parent_instance;
  MockKeymap* mock;
};

G_DEFINE_TYPE(FlMockKeymap, fl_mock_keymap, G_TYPE_OBJECT)

static void fl_mock_keymap_class_init(FlMockKeymapClass* klass) {
  g_signal_new("keys-changed", fl_mock_keymap_get_type(), G_SIGNAL_RUN_LAST, 0,
               nullptr, nullptr, nullptr, G_TYPE_NONE, 0);
}

static void fl_mock_keymap_init(FlMockKeymap* self) {}

static MockKeymap* mock = nullptr;

MockKeymap::MockKeymap() {
  mock = this;
}

GdkKeymap* gdk_keymap_get_for_display(GdkDisplay* display) {
  FlMockKeymap* keymap =
      FL_MOCK_KEYMAP(g_object_new(fl_mock_keymap_get_type(), nullptr));
  (void)FL_IS_MOCK_KEYMAP(keymap);
  return reinterpret_cast<GdkKeymap*>(keymap);
}

guint gdk_keymap_lookup_key(GdkKeymap* keymap, const GdkKeymapKey* key) {
  return mock->gdk_keymap_lookup_key(keymap, key);
}
