// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/fl_socket_accessible.h"

// This is a copy of GtkSocketAccessible, which requires GTK 3.24.30

struct _FlSocketAccessible {
  GtkContainerAccessible parent;
  AtkObject* accessible_socket;
};

G_DEFINE_TYPE(FlSocketAccessible,
              fl_socket_accessible,
              GTK_TYPE_CONTAINER_ACCESSIBLE)

static AtkObject* fl_socket_accessible_ref_child(AtkObject* object, int i) {
  FlSocketAccessible* self = FL_SOCKET_ACCESSIBLE(object);
  return i == 0 ? ATK_OBJECT(g_object_ref(self->accessible_socket)) : nullptr;
}

static int fl_socket_accessible_get_n_children(AtkObject* object) {
  return 1;
}

static void fl_socket_accessible_dispose(GObject* object) {
  FlSocketAccessible* self = FL_SOCKET_ACCESSIBLE(object);

  g_clear_object(&self->accessible_socket);

  G_OBJECT_CLASS(fl_socket_accessible_parent_class)->dispose(object);
}

static void fl_socket_accessible_initialize(AtkObject* object, gpointer data) {
  FlSocketAccessible* self = FL_SOCKET_ACCESSIBLE(object);

  ATK_OBJECT_CLASS(fl_socket_accessible_parent_class)->initialize(object, data);

  self->accessible_socket = atk_socket_new();
}

static void fl_socket_accessible_class_init(FlSocketAccessibleClass* klass) {
  GObjectClass* object_class = G_OBJECT_CLASS(klass);
  object_class->dispose = fl_socket_accessible_dispose;

  AtkObjectClass* atk_class = ATK_OBJECT_CLASS(klass);
  atk_class->initialize = fl_socket_accessible_initialize;
  atk_class->get_n_children = fl_socket_accessible_get_n_children;
  atk_class->ref_child = fl_socket_accessible_ref_child;
}

static void fl_socket_accessible_init(FlSocketAccessible* self) {}

void fl_socket_accessible_embed(FlSocketAccessible* self, gchar* id) {
  atk_socket_embed(ATK_SOCKET(self->accessible_socket), id);
}
